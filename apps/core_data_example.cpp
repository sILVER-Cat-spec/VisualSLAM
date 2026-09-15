#include "vslam/core/map.h"
#include "vslam/diagnostics/logger.h"
#include "vslam/diagnostics/scoped_timer.h"
#include "vslam/sensor/rgbd_camera.h"
#include <iostream>

int main() {
  try {
    const auto logger = vslam::diagnostics::GetLogger("core_example");
    vslam::core::Map map;
    vslam::core::FrameFactory frames(map.epoch(), vslam::common::TimeBase::Simulation);
    // Synthetic prepared buffers; a future input adapter decodes actual ROS images.
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(2));
    vslam::core::Frame frame = frames.Create(
        {1000000000, vslam::common::TimeBase::Simulation}, image, depth, "sim_frame_42");

    vslam::core::Keypoint keypoint;
    keypoint.uv = Eigen::Vector2d(320,240);
    keypoint.depth_z_m = 2.0;
    cv::Mat descriptors(1,32,CV_8UC1,cv::Scalar(7));
    frame.SetFeatures(vslam::core::FeatureSet(
        vslam::core::DescriptorKind::Orb, {keypoint}, descriptors));
    frame.SetPose(vslam::geometry::SE3());  // Initial frame defines world origin.

    vslam::sensor::RGBDCamera camera(
        vslam::sensor::CameraModel(554.256258422,554.256258422,320,240,640,480),1.0);
    const auto point_c = camera.UnprojectRegisteredDepth(keypoint.uv, *keypoint.depth_z_m);
    if (!point_c) return 1;
    const Eigen::Vector3d point_w = frame.pose()->Inverse() * *point_c;
    const vslam::common::MapPointId point_id = map.AddMapPoint(point_w, descriptors);
    frame.Associate(0, point_id);

    double seconds = 0;
    vslam::common::KeyFrameId keyframe_id;
    {
      vslam::diagnostics::ScopedTimer timer(seconds);
      keyframe_id = map.InsertKeyFrame(frame);
    }
    const vslam::core::KeyFrame& keyframe = map.GetKeyFrame(keyframe_id);
    logger->Info("Promoted Frame " + std::to_string(keyframe.source_frame_id().value()) +
                 " to KeyFrame " + std::to_string(keyframe_id.value()));
    std::cout << "Map: " << map.num_keyframes() << " keyframe, " << map.num_map_points()
              << " point, " << map.num_observations() << " observation\n"
              << "World point: " << map.GetMapPoint(point_id).position_w().transpose() << '\n'
              << "Promotion seconds: " << seconds << '\n';
    return map.CheckConsistency() ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
