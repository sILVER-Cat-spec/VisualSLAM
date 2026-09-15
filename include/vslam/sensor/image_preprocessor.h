#pragma once
#include "vslam/sensor/frame_input.h"
#include "vslam/sensor/camera_model.h"
namespace vslam::sensor {
PreparedInput Prepare(const FrameInput& input, const CameraModel& camera);
// radius=0 nearest pixel; radius>0 median of valid values in a clipped window.
std::optional<double> SampleDepth(const cv::Mat& depth_z_m, const Eigen::Vector2d& uv,
                                 double min_depth, double max_depth, int radius);
}
