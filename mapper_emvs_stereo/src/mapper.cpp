#include <mapper_emvs_stereo/mapper.hpp>
// #define TRI
// #define DEBUG_CONF

onlineMapper::onlineMapper(const ros::NodeHandle &nh,
                           const ros::NodeHandle &nh_private):
    it_(nh_),
    pc_(new EMVS::PointCloud),
    pc_global_(new EMVS::PointCloud){

    nh_ = nh;
    pnh_ = nh_private;
    initialized_ = false;
    initialized_tf_ = false;
    state_= IDLE;
    accumulate_pc_ = true;

    current_ts_ = ros::Time(0);
    latest_tf_stamp_ = ros::Time(0);

    nh.param<std::string>("dvs_frame_id", regular_frame_id_, std::string("dvs0"));
    nh.param<std::string>("dvs_bootstrap_frame_id", bootstrap_frame_id_, regular_frame_id_);
    frame_id_ = bootstrap_frame_id_;
    nh.param<std::string>("world_frame_id", world_frame_id_ , "world");
    nh.param<bool>("auto_trigger", auto_trigger_, true);
    nh.param<int>("num_ev_per_map", NUM_EV_PER_MAP, 100000);
    nh.param<bool>("on_demand", on_demand_, true);
    nh.param<float>("max_duration", max_duration_, 1.);
    nh.param<float>("min_duration", min_duration_, .1);
    nh.param<float>("init_wait_time", init_wait_t_, 3.);

    nh.param<bool>("auto_copilot", auto_copilot_, false);

    LOG(INFO) << "On demand mode has been set to " << on_demand_;

    // TODO: pass config via ROS paramaters
    //  nh.param<double>("min_depth", min_depth, 0.3);
    //  nh.param<double>("max_depth", max_depth, 5.0);
    //  nh.param<double>("fov_deg", fov_deg, 0.0);
    //  nh.param<int>("dimX", dimX, 0);
    //  nh.param<int>("dimY", dimY, 0);
    //  nh.param<int>("dimZ", dimZ, 100);

    //  nh.param<int>("adaptive_threshold_kernel_size", adaptive_threshold_kernel_size, 5);
    //  nh.param<double>("adaptive_threshold_c", adaptive_threshold_c, 5.0);
    //  nh.param<int>("median_filter_size", median_filter_size, 5);

    //  nh.param<double>("radius_search", radius_search, 0.05);
    //  nh.param<int>("min_num_negihbors", min_num_negihbors, 3);

    //  nh.param<std::string>("calib_path", calib_path, "./");
    //  nh.param<std::string>("calib_type", calib_type, "yaml");
    //  nh.param<int>("process_method", process_method, 1);
    //  nh.param<int>("num_intervals", num_intervals, 2);
    //  nh.param<int>("stereo_fusion", stereo_fusion, 2);
    //  nh.param<int>("temporal_fusion", temporal_fusion, 4);

    pc_->header.frame_id = world_frame_id_;
    pc_global_->header.frame_id = world_frame_id_;

    // callback functions
    events_left_sub_ = nh_.subscribe<dvs_msgs::EventArray>(FLAGS_event_topic0, 0, boost::bind(&onlineMapper::eventsCallback, this, _1, boost::ref(events_left_), FLAGS_offset0));
    events_right_sub_ = nh_.subscribe<dvs_msgs::EventArray>(FLAGS_event_topic1, 0, boost::bind(&onlineMapper::eventsCallback, this, _1, boost::ref(events_right_), FLAGS_offset1));
#ifdef TRI
    events_tri_sub_ = nh_.subscribe<dvs_msgs::EventArray>(FLAGS_event_topic2, 0, boost::bind(&onlineMapper::eventsCallback, this, _1, boost::ref(events_tri_), FLAGS_offset2));
#endif
    LOG(INFO) << "Subscribing to event topic left: " << FLAGS_event_topic0;
    // TF
    tf_ = std::make_shared<tf::Transformer>(true, ros::Duration(100));
    tf_->setUsingDedicatedThread(true);
    tf_sub_ = nh_.subscribe("tf", 0, &onlineMapper::tfCallback, this);
    copilot_sub_ = nh_.subscribe("/evo/copilot_remote", 0, &onlineMapper::copilotCallback, this);
    remote_key_ = nh_.subscribe("/evo/remote_key", 0,
                                &onlineMapper::remoteKeyCallback, this);

    // result publishers
    proj_pub_ = it_.advertise("projected_pointcloud", 1);
    invDepthMap_pub_ = it_.advertise("Inverse_Depth_Map", 1);
    conf_map_pub_ = it_.advertise("Confidence_Map", 1);

#ifdef DEBUG_CONF
    conf_map0_pub_ = it_.advertise("Confidence_Map0", 1);
    conf_map1_pub_ = it_.advertise("Confidence_Map1", 1);
    conf_map2_pub_ = it_.advertise("Confidence_Map2", 1);
#endif

    ev_img_pub_ = it_.advertise("event_image0", 1);
    pc_pub_ = nh_.advertise<EMVS::PointCloud>("pointcloud_local", 1);
    pc_global_pub_ = nh_.advertise<EMVS::PointCloud>("pointcloud_global", 1);

    calibrate();

    cv::Size full_resolution = cam0.fullResolution();
    event_image0_ = cv::Mat(full_resolution,CV_8UC1);
    event_image1_ = cv::Mat(full_resolution,CV_8UC1);

    std::thread mapperThread(&onlineMapper::mappingLoop, this);
    mapperThread.detach();
}

void onlineMapper::remoteKeyCallback(const std_msgs::String::ConstPtr& msg) {
    std::string command_str = msg->data;
    //    LOG(INFO) << "Received command: " << command_str;
    if (command_str == "update") {
        state_ = MAPPING;
        LOG(INFO) << "state has been updated to ... " << state_;
    }
    if (command_str == "global_pc_switch") {
        accumulate_pc_ = !accumulate_pc_;
        LOG(INFO) << "New command received. Are we accumulating point clouds? " << accumulate_pc_;
    }
}

void onlineMapper::copilotCallback(const std_msgs::Bool& msg) {
    frame_id_ = (msg.data) ? regular_frame_id_ : bootstrap_frame_id_;
    LOG(INFO) << "Mapping switched copilot to " << frame_id_;
}

void onlineMapper::calibrate(){

    // Load calibration from file (not from ROS bag)
    if (FLAGS_calib_type == "eccv18")
        get_camera_calib_ECCV18(cam0, cam1, mat4_1_0, mat4_hand_eye);
    else if(FLAGS_calib_type == "esim")
        get_camera_calib_ESIM(cam0, cam1, mat4_1_0, mat4_hand_eye);
    else if(FLAGS_calib_type == "dvsgen3")
        get_camera_calib_DVS_Gen3(cam0, cam1, mat4_1_0, mat4_hand_eye);
    else if(FLAGS_calib_type == "yaml")
        get_camera_calib_yaml(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "yaml_mvsec")
        get_camera_calib_yaml_mvsec(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "slider")
        get_camera_calib_slider(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "hkust")
        get_camera_calib_hkust(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "evimo2")
        get_camera_calib_evimo2(cam0, cam1, cam2, mat4_1_0, mat4_2_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "evimo2_pleft")
        get_camera_calib_evimo2_pleft(cam0, cam1, cam2, mat4_1_0, mat4_2_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "json")
        get_camera_calib_json(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);
    else if(FLAGS_calib_type == "dsec_yaml")
        get_camera_calib_dsec_yaml(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);
    else if(FLAGS_calib_type == "sl1")
        get_camera_calib_sl1(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);
    else if(FLAGS_calib_type == "sl1_kalibr")
        get_camera_calib_sl1_kalibr(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);
    else if(FLAGS_calib_type == "sony")
        get_camera_calib_sony(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);
    else if(FLAGS_calib_type == "demo")
        get_camera_calib_yaml_demo(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path);
    else if(FLAGS_calib_type == "rip")
        get_camera_calib_rip(cam0, cam1, mat4_1_0, mat4_hand_eye, FLAGS_calib_path, FLAGS_mocap_calib_path);
    LOG(INFO) << "Finished calibrating for type: " << FLAGS_calib_type;

}

void onlineMapper::tfCallback(const tf::tfMessage::ConstPtr& tf_msg) {

    std::chrono::high_resolution_clock::time_point t_start_callback = std::chrono::high_resolution_clock::now();
    //    LOG(INFO) << "Inside TF callback#####################################################################";
    for (const geometry_msgs::TransformStamped& transform_stamped_msg :
         tf_msg->transforms) {

        tf::StampedTransform t;
        tf::transformStampedMsgToTF(transform_stamped_msg, t);
        tf_->setTransform(t);

        //              LOG(INFO) << transform_stamped_msg.child_frame_id << " " << frame_id_
        //                        << " " << world_frame_id_ << " "
        //                        << transform_stamped_msg.header.frame_id;

        if (transform_stamped_msg.child_frame_id == "hand") {

            ros::Time tf_stamp_= transform_stamped_msg.header.stamp;
            tf::Transform T_hand_eye;
            tf::transformEigenToTF(Eigen::Affine3d(mat4_hand_eye), T_hand_eye);
            tf::StampedTransform stamped_T_hand_eye(T_hand_eye, tf_stamp_, "hand", bootstrap_frame_id_);
            //            tf_->setTransform(stamped_T_hand_eye);

            // Broadcast hand eye transform
            broadcaster_.sendTransform(stamped_T_hand_eye);
        }

        if (transform_stamped_msg.child_frame_id == frame_id_ || transform_stamped_msg.child_frame_id == "/"+frame_id_) {

            ros::Time tf_stamp_= transform_stamped_msg.header.stamp;

            // TODO: listener instead of subsribe and setTransform
            tf::Transform T_0_1, T_0_2;
            tf::transformEigenToTF(Eigen::Affine3d(mat4_1_0.inverse()), T_0_1);
            tf::StampedTransform stamped_T_0_1(T_0_1, tf_stamp_, frame_id_, "dvs1");
            tf_->setTransform(stamped_T_0_1);
#ifdef TRI
            tf::transformEigenToTF(Eigen::Affine3d(mat4_2_0.inverse()), T_0_2);
            tf::StampedTransform stamped_T_0_2(T_0_2, tf_stamp_, frame_id_, "dvs2");
            tf_->setTransform(stamped_T_0_2);
#endif

            if (frame_id_ == bootstrap_frame_id_) {
                // keep bootstrap frame also as regular frame for future use
                t.child_frame_id_ = regular_frame_id_;
                tf_->setTransform(t);
            }
        }
    }
    std::chrono::high_resolution_clock::time_point t_end_callback = std::chrono::high_resolution_clock::now();
    //  LOG(INFO) <<  "Time taken to process TF callback: " << std::chrono::duration_cast<std::chrono::microseconds>(t_end_callback - t_start_callback).count();
}

void onlineMapper::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg, std::deque<dvs_msgs::Event>& EQ, const double events_offset){

    std::chrono::high_resolution_clock::time_point t_start_callback = std::chrono::high_resolution_clock::now();
    // Exit if there are no subscribers
    //  if (invDepthMap_pub_.getNumSubscribers() <= 0 && conf_map_pub_.getNumSubscribers() <= 0)
    //    {
    //      return;
    //    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    for(const dvs_msgs::Event& e : msg->events)
    {
        dvs_msgs::Event ev_modified(e);
        ev_modified.ts = ros::Time(ev_modified.ts.toSec() - events_offset);

        if (!initialized_) {
            first_ev_ts = ev_modified.ts;
            initialized_ = true;
            //          current_ts = first_ev_ts;
            LOG(INFO) << "First timestamp is: "<< first_ev_ts;
            current_ts_ = first_ev_ts;
        }
        EQ.push_back(ev_modified);
    }
    clearEventQueue(EQ);

    std::chrono::high_resolution_clock::time_point t_end_callback = std::chrono::high_resolution_clock::now();
    //  LOG(INFO) <<  "Time taken to process event callback: " << std::chrono::duration_cast<std::chrono::microseconds>(t_end_callback - t_start_callback).count();

}

void onlineMapper::clearEventQueue(std::deque<dvs_msgs::Event>& EQ)
{
    static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 50000000;
    if (EQ.size() > MAX_EVENT_QUEUE_LENGTH)
    {
        //        LOG(INFO) << "WILL TRY TO ERASE NOW. IT IS TOO BIG!!";
        size_t NUM_EVENTS_TO_REMOVE = EQ.size() - MAX_EVENT_QUEUE_LENGTH;
        //        std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
        EQ.erase(EQ.begin(), EQ.begin() + NUM_EVENTS_TO_REMOVE);
        //        std::chrono::high_resolution_clock::time_point t_stop = std::chrono::high_resolution_clock::now();
        //        LOG(INFO) <<  "Time taken to erase::::::::::::::::::::::::::::: " << std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start).count();
    }
}

void onlineMapper::mappingLoop(){

    // Initialize the DSI
    CHECK_LE(FLAGS_dimZ, 256) << "Number of depth planes should be <= 256";
    dsi_shape = EMVS::ShapeDSI(FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ,
                               FLAGS_min_depth, FLAGS_max_depth,
                               FLAGS_fov_deg);

    opts_depth_map.max_confidence = FLAGS_max_confidence;
    opts_depth_map.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
    opts_depth_map.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
    opts_depth_map.median_filter_size_ = FLAGS_median_filter_size;
    opts_depth_map.full_sequence = FLAGS_full_seq;
    opts_depth_map.save_conf_stats = FLAGS_save_conf_stats;
    opts_depth_map.save_mono = FLAGS_save_mono;
    opts_depth_map.rv_pos = FLAGS_rv_pos;

    bool map_initialized = false;

    ros::Rate rate(10);
    std::string error_msg;

    while (ros::ok()) {
        if (!on_demand_)
            rate.sleep();
        if (pc_global_pub_.getNumSubscribers()==0)
            pc_global_->clear();

        //      LOG(INFO) << state_;
        tf::StampedTransform latest_tf;
        if(tf_->waitForTransform(world_frame_id_, frame_id_, ros::Time(0), ros::Duration(0), ros::Duration(0.01), &error_msg)){
            tf_->lookupTransform(world_frame_id_, frame_id_, ros::Time(0), latest_tf);
            latest_tf_stamp_ = latest_tf.stamp_;
        }
        else {
            // LOG(WARNING) << error_msg;
            continue;
        }
        //        LOG(INFO) << (latest_tf_stamp_ - current_ts_).toSec();

        if (auto_trigger_ && initialized_ && !map_initialized && (latest_tf_stamp_ - current_ts_).toSec() > init_wait_t_) {
            LOG(INFO) << "GENERATING INITIAL MAP AUTOMATICALLY.";
            state_ = MAPPING;
        }
        // LOG(INFO) << state_;
        if (state_ == MAPPING)
        {
            //   LOG(INFO) << "Mapping";
            //   LOG(INFO) << "Latest tf ts: "<< latest_tf_stamp_;

            std::vector<dvs_msgs::Event> ev_subset_left_, ev_subset_right_, ev_subset_tri_;

            int last_tracked_ev_left, last_tracked_ev_right, last_tracked_ev_tri;

            if (current_ts_ < latest_tf_stamp_) {
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    last_tracked_ev_left = events_left_.size() - 1;
                    while (last_tracked_ev_left>0 && events_left_[last_tracked_ev_left].ts > latest_tf_stamp_){
                        --last_tracked_ev_left;
                    }
                    // LOG(INFO) << "Index of last tracked left event: " << last_tracked_ev_left << " with time " << latest_tf_stamp_;

                    last_tracked_ev_right = events_right_.size() - 1;
                    while (last_tracked_ev_right>0 && events_right_[last_tracked_ev_right].ts > latest_tf_stamp_){
                        --last_tracked_ev_right;
                    }
#ifdef TRI
                    last_tracked_ev_tri = events_tri_.size() - 1;
                    while (last_tracked_ev_tri>0 && events_tri_[last_tracked_ev_tri].ts > latest_tf_stamp_){
                        --last_tracked_ev_tri;
                    }
#endif

                    // LOG(INFO) << "Index of last tracked right event: " << last_tracked_ev_right << " with time " << latest_tf_stamp_;
#ifdef TRI
                    if (last_tracked_ev_left <= NUM_EV_PER_MAP || last_tracked_ev_right <= NUM_EV_PER_MAP || last_tracked_ev_tri <= NUM_EV_PER_MAP) {
                        LOG(INFO) << "Not enough events yet...";
                        continue;
                    }
#else
                    if (last_tracked_ev_left <= NUM_EV_PER_MAP || last_tracked_ev_right <= NUM_EV_PER_MAP) {
                        // LOG(INFO) << "Not enough events yet...";
                        continue;
                    }
#endif

                    LOG(INFO) << "Creating subset of events with size: "<< NUM_EV_PER_MAP;
                    current_ts_ = latest_tf_stamp_;
                    double_t duration = (current_ts_ - events_left_[last_tracked_ev_left - NUM_EV_PER_MAP].ts).toSec();
                    LOG(INFO) << "Duration: "<<duration;
                    //              ros::Time ev_subset_start_ts = (events_left_.end()-NUM_EV_PER_MAP)->ts;
                    if (duration > max_duration_){
                        LOG(INFO) << "Looking too far back in the past. skip";
                        continue;
                    }
                    if (duration < min_duration_){
                        LOG(INFO) << "Time interval is not big enough. There might be flashing events. Skip.";
                        continue;
                    }
                    ev_subset_left_ = std::vector<dvs_msgs::Event>(events_left_.begin()+last_tracked_ev_left-NUM_EV_PER_MAP,events_left_.begin()+last_tracked_ev_left);
                    ev_subset_right_ = std::vector<dvs_msgs::Event>(events_right_.begin()+last_tracked_ev_right-NUM_EV_PER_MAP, events_right_.begin()+last_tracked_ev_right);
#ifdef TRI
                    ev_subset_tri_ = std::vector<dvs_msgs::Event>(events_tri_.begin()+last_tracked_ev_tri-NUM_EV_PER_MAP, events_tri_.begin()+last_tracked_ev_tri);
#endif

                }

                LOG(INFO)<<"------------------- Mapping at time: " << (current_ts_ - first_ev_ts).toSec() << " seconds ----------------------";
                LOG(INFO)<<"------------------- Mapping at time: " << current_ts_ << " ---------------------";
                LOG(INFO)<<"------------------- Pose from: " << frame_id_ << " ---------------------";

                LOG(INFO)<<"Using left events from " << (ev_subset_left_.front().ts - first_ev_ts).toSec() << "-" << (ev_subset_left_.back().ts - first_ev_ts).toSec();
                LOG(INFO)<<"Using left events from " << ev_subset_left_.front().ts << "-" << ev_subset_left_.back().ts;

                LOG(INFO)<<"Using right events from " << (ev_subset_right_.front().ts - first_ev_ts).toSec() << "-" << (ev_subset_right_.back().ts - first_ev_ts).toSec();
                LOG(INFO)<<"Using right events from " << ev_subset_right_.front().ts << "-" << ev_subset_right_.back().ts;

#ifdef TRI
                LOG(INFO)<<"Using third camera events from " << (ev_subset_tri_.front().ts - first_ev_ts).toSec() << "-" << (ev_subset_tri_.back().ts - first_ev_ts).toSec();
                LOG(INFO)<<"Using third camera events from " << ev_subset_tri_.front().ts << "-" << ev_subset_tri_.back().ts;

#endif
                MappingAtTime(current_ts_, ev_subset_left_, ev_subset_right_, ev_subset_tri_, frame_id_);

                if (on_demand_)
                    state_ = IDLE;
                if (auto_copilot_ && !map_initialized)
                    frame_id_ = regular_frame_id_;
                map_initialized = true;
            }
            else {
                // LOG(INFO) << "No more events to process";
            }
        }
    }
}

void onlineMapper::MappingAtTime(ros::Time current_ts, std::vector<dvs_msgs::Event>& events_left_, std::vector<dvs_msgs::Event>& events_right_, std::vector<dvs_msgs::Event>& events_tri_, std::string frame_id) {

    // Use linear interpolation to compute the camera pose for each event

    LOG(INFO)<<"Initializing mapper fused";
    EMVS::MapperEMVS mapper_fused(cam0, dsi_shape); //, mapper_fused_camera_time(cam0, dsi_shape);
    mapper_fused.name = "fused";
    LOG(INFO)<<"Initializing mapper 0";
    EMVS::MapperEMVS mapper0(cam0, dsi_shape);
    mapper0.name="0";
    LOG(INFO)<<"Initializing mapper 1";
    EMVS::MapperEMVS mapper1(cam1, dsi_shape);
    mapper1.name="1";
#ifndef TRI
    cam2 = cam1;
#endif
    LOG(INFO)<<"Initializing mapper 2";
    EMVS::MapperEMVS mapper2(cam2, dsi_shape);
    mapper2.name="2";

    // Build and fuse DSIs using events and camera poses
    switch( FLAGS_process_method )
    {
    // Only fusion across stereo camera, i.e., Alg. 1 of MC-EMVS (Ghosh and Gallego, AISY 2022) is implemented here
    case 1:
        // 1-3. Compute two DSIs (one for each camera) and fuse them
        process_1(world_frame_id_, frame_id, cam0,cam1, cam2, tf_, events_left_, events_right_, events_tri_, opts_depth_map, dsi_shape, mapper_fused, mapper0, mapper1, mapper2, FLAGS_out_path,  current_ts, FLAGS_stereo_fusion, T_rv_w_);

        break;
    default:
        LOG(INFO) << "Incorrect process method selected.. exiting";
        exit(0);
    }

    // Convert DSI to depth maps using argmax and noise filtering
    mapper_fused.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
    //    mapper0.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);

    // Convert semi-dense depth map to point cloud
    EMVS::OptionsPointCloud opts_pc;
    opts_pc.radius_search_ = FLAGS_radius_search;
    opts_pc.min_num_neighbors_ = FLAGS_min_num_neighbors;
    mapper_fused.getPointcloud(depth_map, semidense_mask, opts_pc, pc_, T_rv_w_);
    //    mapper0.getPointcloud(depth_map, semidense_mask, opts_pc, pc_, T_rv_w_);

#ifdef DEBUG_CONF
    cv::Mat dummy;
    mapper0.getDepthMapFromDSI(dummy, confidence_map_0, dummy, opts_depth_map);
    mapper1.getDepthMapFromDSI(dummy, confidence_map_1, dummy, opts_depth_map);
    mapper2.getDepthMapFromDSI(dummy, confidence_map_2, dummy, opts_depth_map);
#endif

    publishMsgs(frame_id);

}

void onlineMapper::publishMsgs(std::string frame_id){

    LOG(INFO) << "Publishing message!!!!!!!!!!!!!!!!!!!!!!!!! ";
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->encoding = "mono8";
    cv_ptr->header.stamp = ros::Time(current_ts_);
    cv_ptr->header.frame_id = frame_id;
    cv_ptr->image = event_image0_;
    ev_img_pub_.publish(cv_ptr->toImageMsg());


    if (invDepthMap_pub_.getNumSubscribers() > 0 || conf_map_pub_.getNumSubscribers() > 0) {

        // Save confidence map as an 8-bit image
        cv::Mat confidence_map_255;
        cv::normalize(confidence_map, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_8UC1);

        // colorize depth maps according to max and min depth
        cv::Mat invdepthmap_8bit, invdepthmap_color;
        cv::Mat invmap = 1./depth_map;
        float mod_max_depth = 1 * dsi_shape.max_depth_;
        cv::Mat invdepth_map_255 = (invmap - 1./mod_max_depth)  / (1./dsi_shape.min_depth_ - 1./mod_max_depth) * 255.;
        invdepth_map_255.convertTo(invdepthmap_8bit, CV_8U);
        cv::applyColorMap(invdepthmap_8bit, invdepthmap_color, cv::COLORMAP_JET);
        cv::Mat invdepth_on_canvas = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC3, cv::Scalar(1,1,1)*0);
        invdepthmap_color.copyTo(invdepth_on_canvas, semidense_mask);
        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(3,3));
        cv::dilate(invdepth_on_canvas, invdepth_on_canvas, element);

        // Change the background from black to white
        for ( int i = 0; i < invdepth_on_canvas.rows; i++ ) {
            for ( int j = 0; j < invdepth_on_canvas.cols; j++ ) {
                if ( invdepth_on_canvas.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0) )
                {
                    invdepth_on_canvas.at<cv::Vec3b>(i, j)[0] = 255;
                    invdepth_on_canvas.at<cv::Vec3b>(i, j)[1] = 255;
                    invdepth_on_canvas.at<cv::Vec3b>(i, j)[2] = 255;
                }
            }
        }

        //      saveDepthMaps(depth_map, confidence_map, semidense_mask, dsi_shape.min_depth_, dsi_shape.max_depth_, std::string("fused"), FLAGS_out_path + std::to_string(ts));

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = ros::Time(current_ts_);
        cv_ptr->header.frame_id = "/map";
        cv_ptr->image = invdepth_on_canvas;
        invDepthMap_pub_.publish(cv_ptr->toImageMsg());

        cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);
        cv_ptr2->encoding = "mono8";
        cv_ptr2->header.stamp = ros::Time(current_ts_);
        cv_ptr2->header.frame_id = "/map";
        cv_ptr2->image = 255 - confidence_map_255;
        conf_map_pub_.publish(cv_ptr2->toImageMsg());

#ifdef DEBUG_CONF

        cv::normalize(confidence_map_0, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_8UC1);
        cv_ptr2->encoding = "mono8";
        cv_ptr2->header.stamp = ros::Time(current_ts_);
        cv_ptr2->header.frame_id = "/map";
        cv_ptr2->image = 255 - confidence_map_255;
        conf_map0_pub_.publish(cv_ptr2->toImageMsg());


        cv::normalize(confidence_map_1, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_8UC1);
        cv_ptr2->encoding = "mono8";
        cv_ptr2->header.stamp = ros::Time(current_ts_);
        cv_ptr2->header.frame_id = "/map";
        cv_ptr2->image = 255 - confidence_map_255;
        conf_map1_pub_.publish(cv_ptr2->toImageMsg());


        cv::normalize(confidence_map_2, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_8UC1);
        cv_ptr2->encoding = "mono8";
        cv_ptr2->header.stamp = ros::Time(current_ts_);
        cv_ptr2->header.frame_id = "/map";
        cv_ptr2->image = 255 - confidence_map_255;
        conf_map2_pub_.publish(cv_ptr2->toImageMsg());
#endif
    }

    if (!pc_->empty() && pc_pub_.getNumSubscribers()>0)
    {
        sensor_msgs::PointCloud2::Ptr pc_to_publish (new sensor_msgs::PointCloud2);
        LOG(INFO) << "<<<<<<<<<(pointcloud)<<<<<<<<" << pc_->size() << " points are published";
        pcl::toROSMsg(*pc_, *pc_to_publish);
        pc_to_publish->header.stamp = current_ts_;
        pc_pub_.publish(pc_to_publish);

        if (pc_global_pub_.getNumSubscribers()>0 && accumulate_pc_){
            // copy the most current pc tp pc_global
            size_t pc_length = pc_->size();
            size_t numAddedPC_threshold_ = 4000;
            size_t numAddedPC = std::min(pc_length, numAddedPC_threshold_) - 1;
            pc_global_->insert(pc_global_->end(), pc_->end() - numAddedPC, pc_->end());
            // publish point cloud
            pcl::toROSMsg(*pc_global_, *pc_to_publish);
            pc_to_publish->header.stamp = current_ts_;
            pc_global_pub_.publish(pc_to_publish);
            t_last_pub_pc_ = current_ts_.toSec();
        }
        else {
            LOG(INFO) << "Clearing global PC!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
            pc_global_->clear();
        }
    }
}
