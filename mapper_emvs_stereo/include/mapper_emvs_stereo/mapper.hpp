/*
* \file mapper.hpp
* \brief header file for the onlineMapper class
* \author (1) Suman Ghosh
* \date 2024-09-29
* \author (2) Valentina Cavinato
* \date 2024-09-29
* \author (3) Guillermo Gallego
* \date 2024-09-29
* Copyright/Rights of Use:
* 2024, Technische Universität Berlin
* Prof. Guillermo Gallego
* Robotic Interactive Perception
* Marchstrasse 23, Sekr. MAR 5-5
* 10587 Berlin, Germany
*/

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <mapper_emvs_stereo/mapper_emvs_stereo.hpp>
#include <mapper_emvs_stereo/utils.hpp>
#include <mapper_emvs_stereo/calib.hpp>
#include <mapper_emvs_stereo/process1_live.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <thread>

#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

// Input parameters

// I/O paths
DECLARE_string(bag_filename);
DECLARE_string(bag_filename_left);
DECLARE_string(bag_filename_right);
DECLARE_string(bag_filename_pose);
DECLARE_string(out_path);

// Calibration
DECLARE_string(calib_type);
DECLARE_string(calib_path);
DECLARE_string(mocap_calib_path);

// ROS topics
DECLARE_string(event_topic0);
DECLARE_string(event_topic1);
DECLARE_string(event_topic2);
DECLARE_string(camera_info_topic0);
DECLARE_string(camera_info_topic1);
DECLARE_string(camera_info_topic2);
DECLARE_string(pose_topic);

DECLARE_double(offset0);
DECLARE_double(offset1);
DECLARE_double(offset2);

DECLARE_double(start_time_s);
DECLARE_double(stop_time_s);

// Disparity Space Image (DSI) parameters
DECLARE_int32(dimX);
DECLARE_int32(dimY);
DECLARE_int32(dimZ);
DECLARE_double(fov_deg);
DECLARE_double(min_depth);
DECLARE_double(max_depth);

// Depth map parameters (selection and noise removal)
DECLARE_int32(adaptive_threshold_kernel_size);
DECLARE_double(adaptive_threshold_c);
DECLARE_int32(median_filter_size);
DECLARE_bool(save_mono);

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DECLARE_double(radius_search);
DECLARE_int32(min_num_neighbors);
DECLARE_bool(late_fusion);

DECLARE_int32(process_method);
DECLARE_int32(num_intervals);
DECLARE_double(ts);
DECLARE_double(rv_pos);
DECLARE_bool(forward_looking);
DECLARE_int32(stereo_fusion);
DECLARE_int32(temporal_fusion);

// Parameters for full sequence processing
DECLARE_bool(full_seq);
DECLARE_bool(save_conf_stats);
DECLARE_double(duration);
DECLARE_double(out_skip);
DECLARE_double(max_confidence);

class onlineMapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  onlineMapper(const ros::NodeHandle& nh,
               const ros::NodeHandle& nh_private);
  virtual ~onlineMapper(){}

  // mapping
  //    void MappingLoop(std::promise<void> prom_mapping, std::future<void> future_reset);
  void MappingAtTime(const ros::Time t, std::vector<dvs_msgs::Event>& ev_left, std::vector<dvs_msgs::Event>& ev_right, std::vector<dvs_msgs::Event>& ev_tri, std::string frame_id);
  bool InitializationAtTime(const ros::Time& t);

  // callback functions
  void tfCallback(const tf::tfMessage::ConstPtr &tf_msg);
  void remoteKeyCallback(const std_msgs::String::ConstPtr& msg);
  void stampedPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void stampedPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr &msg, std::deque<dvs_msgs::Event>& EQ, const double events_offset);
  void clearEventQueue(std::deque<dvs_msgs::Event>& EQ);
  //    void onlineParameterChangeCallback(DVS_MappingStereoConfig &config, uint32_t level);
  void calibrate();
  void mappingLoop();
  void Finish();
  void publishMsgs(std::string frame_id);
  void projectPointCloudAndPublish(ros::Time ts, geometry_utils::Transformation& T);
//  bool getPoseAt(const ros::Time& t, std::string frame_id, geometry_utils::Transformation& T);
  void copilotCallback(const std_msgs::Bool& msg);

  bool FLAG_EXIT;


  /************************ member variables ************************/
private:
  ros::NodeHandle nh_, pnh_;

  std::shared_ptr<tf::Transformer> tf_;
  std::string world_frame_id_;  ///< The root frame id of the tf system
  std::string frame_id_;        ///< currently used frame id
  std::string regular_frame_id_;  ///< frame id to use during regular operation
  std::string bootstrap_frame_id_;  ///< frame id to use during bootstrapping

  // Subscribers
  ros::Subscriber tf_sub_;
  ros::Subscriber events_left_sub_, events_right_sub_, events_tri_sub_;
  ros::Subscriber stampedPose_sub_;
  ros::Subscriber copilot_sub_;
  ros::Subscriber remote_key_;

  // Publishers
  ros::Publisher pc_pub_, pc_global_pub_;
  image_transport::Publisher invDepthMap_pub_;
  image_transport::Publisher conf_map_pub_, conf_map0_pub_, conf_map1_pub_, conf_map2_pub_;
  image_transport::Publisher ev_img_pub_;
  image_transport::Publisher proj_pub_;
  image_transport::ImageTransport it_;
  tf::TransformBroadcaster broadcaster_;

  double t_last_pub_pc_;

  // online data
  std::deque<dvs_msgs::Event> events_left_, events_right_, events_tri_;
  std::map<ros::Time, geometry_utils::Transformation> poses_;

  // result
  EMVS::PointCloud::Ptr pc_, pc_global_;
  cv::Mat depth_map, confidence_map, confidence_map_0, confidence_map_1, confidence_map_2, semidense_mask;

  // Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
  double min_depth;
  double max_depth;
  double fov_deg;
  int dimX;
  int dimY;
  int dimZ;

  // Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
  int adaptive_threshold_kernel_size;
  double adaptive_threshold_c;
  int median_filter_size;

  // Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
  double radius_search;
  int min_num_negihbors;

  // Process parameters
  std::string calib_path, mocap_calib_path;
  std::string calib_type;
  int process_method;
  int num_intervals;
  int stereo_fusion;
  int temporal_fusion;

  std::mutex data_mutex_;
  std::mutex tf_mutex_;
  std::mutex state_mutex_;

  // calibration params
  Eigen::Matrix4d mat4_1_0, mat4_2_0, mat4_hand_eye;
  image_geometry::PinholeCameraModel cam0, cam1, cam2;

  // Parameters to extract semi-dense depth map from DSI
  EMVS::OptionsDepthMap opts_depth_map;

  EMVS::ShapeDSI dsi_shape;

  ros::Time current_ts_;
  int beg_ev_left_idx;
  int beg_ev_right_idx;
  int beg_ev_tri_idx;
  ros::Time beg_ev_left_ts;
  ros::Time beg_ev_right_ts;
  ros::Time beg_ev_tri_ts;
  ros::Time first_ev_ts;

  bool initialized_;
  bool initialized_tf_;
  bool auto_trigger_;
  bool on_demand_;
  bool accumulate_pc_;

  int NUM_EV_PER_MAP;
  geometry_utils::Transformation T_rv_w_;

  cv::Mat event_image0_, event_image1_, event_image_live_;

  ros::Time latest_tf_stamp_;

  enum MapperState {MAPPING, IDLE};
  MapperState state_;

  float max_duration_, min_duration_, init_wait_t_;
  bool auto_copilot_;

};


