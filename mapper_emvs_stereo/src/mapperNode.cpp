/*
* \file mapperNode.cpp
* \brief main ROS Node for mapping
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

#include <ros/ros.h>
#include <mapper_emvs_stereo/mapper.hpp>
#include <csignal>

// Input parameters

// I/O paths
DEFINE_string(bag_filename, "", "Path to the rosbag");
DEFINE_string(bag_filename_left, "input.bag", "Path to the rosbag");
DEFINE_string(bag_filename_right, "input.bag", "Path to the rosbag");
DEFINE_string(bag_filename_pose, "input.bag", "Path to the rosbag");
DEFINE_string(out_path, "./", "Output path prefix");

// Calibration
DEFINE_string(calib_type, "yaml", "Choose how to parse calib file, varies according to dataset.");
DEFINE_string(calib_path, "stereo_pinhole.yaml", "Output path prefix");
DEFINE_string(mocap_calib_path, "stereo_pinhole.yaml", "Output path prefix");

// ROS topics
DEFINE_string(event_topic0, "/davis_left/events", "Name of the event topic (default: /dvs/left/events)");
DEFINE_string(event_topic1, "/davis_right/events", "Name of the event topic (default: /dvs/right/events)");
DEFINE_string(event_topic2, "", "Name of the event topic (default: emtpy string)");
DEFINE_string(camera_info_topic0, "/davis_left/camera_info", "Name of the camera info topic (default: /dvs/left/camera_info)");
DEFINE_string(camera_info_topic1, "/davis_right/camera_info", "Name of the camera info topic (default: /dvs/right/camera_info)");
DEFINE_string(camera_info_topic2, "", "Name of the camera info topic (default: empty string)");
DEFINE_string(pose_topic, "/optitrack/davis_stereo", "Name of the pose topic (default: /optitrack/dvs)");

DEFINE_double(offset0, 0, "Event msg timestamp offset wrt poses");
DEFINE_double(offset1, 0, "Event msg timestamp offset wrt poses");
DEFINE_double(offset2, 0, "Event msg timestamp offset wrt poses");

DEFINE_double(start_time_s, 0.0, "Start time in seconds (default: 0.0)");
DEFINE_double(stop_time_s, 1000.0, "Stop time in seconds (default: 1000.0)");

// Disparity Space Image (DSI) parameters
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 5.0, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal)
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");
DEFINE_bool(save_mono, false, "If set to true, results for monocular EMVS (left and right camera) will be saved as well");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");
DEFINE_bool(late_fusion, false, "Enable optional late fusion for comparison against proposed early fusion at DSI level");

DEFINE_int32(process_method, 1, "Select processing method");
DEFINE_int32(num_intervals, 4, "Number of sub-intervals to divide into for process 2 (temporal fusion)");
DEFINE_double(ts, (FLAGS_start_time_s + FLAGS_stop_time_s)/2, "Explicitly set DSI reference at this timestamp; default: middle of trajectory");
DEFINE_double(rv_pos, 0, "Set RV position shift along the baseline from the left camera center (implemented only for process 1); default: left camera center");
DEFINE_bool(forward_looking, false, "Put RV at end of trajectory if this flag is true");
DEFINE_int32(stereo_fusion, 2, "Fusion function for stereo DSI.. default: HM");
DEFINE_int32(temporal_fusion, 4, "DSI fusion function across time.. default: AM");

// Parameters for full sequence processing
DEFINE_bool(full_seq, false, "If set true, the whole sequence is divided into chunks and are processed independently");
DEFINE_bool(save_conf_stats, false, "If set true, saves txt file containing min (non-zero) and max confidence values for each DSI");
DEFINE_double(duration, 3, "Duration of events used in a single DSI, default: 3s.");
DEFINE_double(out_skip, 10, "Time period of mapping; default: 10s.");
DEFINE_double(max_confidence, 0, "Manually set this number as the upper limit of the DSI range before normalization to [0, 255]; If set to 0, the max value of the DSI becomes its upper limit.");


//void SigHandle(int sig) {
//  onlineMapper::FLAG_EXIT = true;
//  ROS_WARN("catch sig %d", sig);
//}

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "mcemvs_mapper");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

onlineMapper online_mapper(nh, nh_private);

//  signal(SIGINT, SigHandle);
  ros::spin();
  //  ros::Rate rate(30);

  //  while (ros::ok()) {
  ////      if (online_mapper_ptr->FLAG_EXIT) {
  ////          break;
  ////        }
  //      ros::spinOnce();
  //      online_mapper_ptr->Run();
  //      rate.sleep();
  //    }

  //  LOG(INFO) << "finishing mapping";
  ////  online_mapper_ptr->Finish();

  return 0;
}
