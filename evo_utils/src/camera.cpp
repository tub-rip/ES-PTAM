#include "evo_utils/camera.hpp"

namespace evo_utils::camera {

void precomputeRectificationTable(
        std::vector<cv::Point2f>& rectified_points,
        const image_geometry::PinholeCameraModel& cam) {
    static std::vector<cv::Point2f> points;
    points.clear();
    rectified_points.clear();

    cv::Size fr = cam.fullResolution();

    for (size_t y = 0; y != fr.height; ++y) {
        for (size_t x = 0; x != fr.width; ++x) {
            points.push_back(cv::Point2f(x, y));
        }
    }

    const std::string distortion_type = cam.cameraInfo().distortion_model;
    LOG(INFO) << "Distortion type: " << distortion_type;

    if (distortion_type == "plumb_bob") {
        auto K = cv::getOptimalNewCameraMatrix(cam.fullIntrinsicMatrix(),
                                               cam.distortionCoeffs(), fr, 0.);
        cv::undistortPoints(points, rectified_points, cam.fullIntrinsicMatrix(),
                            cam.distortionCoeffs(), cv::Matx33f::eye(), K);
    } else {
        CHECK(false) << "Distortion model: " << distortion_type
                     << " is not supported.";
    }
}

image_geometry::PinholeCameraModel loadPinholeCamera(ros::NodeHandle& nh) {
    // Load camera calibration
    const std::string camera_name =
            rpg_common_ros::param<std::string>(nh, "camera_name", "");

    const std::string calib_file =
            rpg_common_ros::param<std::string>(nh, "calib_file", "");
    const std::string url = std::string("file://") + calib_file;

    camera_info_manager::CameraInfoManager cam_info(nh, camera_name, url);
    image_geometry::PinholeCameraModel cam;
    sensor_msgs::CameraInfo camera_info_msg = cam_info.getCameraInfo();
    LOG(INFO)<< camera_info_msg.distortion_model;

    // Re-compute new optimal projection matrix and replace the one passed via calib file
    if (camera_info_msg.distortion_model=="plumb_bob") {
        cv::Mat K_temp(3, 3, CV_64F, camera_info_msg.K.data());
        cv::Mat P_temp = cv::getOptimalNewCameraMatrix(K_temp, camera_info_msg.D, cv::Size(camera_info_msg.width, camera_info_msg.height), 0);

        for (int r = 0; r<3; r++){
            for (int c=0; c<4; c++){
                camera_info_msg.P[r*4+c]=P_temp.at<double>(r, c);
            }
        }
    }
    cam.fromCameraInfo(camera_info_msg);
    return cam;
}
}  // namespace evo_utils::camera
