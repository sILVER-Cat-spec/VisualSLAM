#pragma once
#include "vslam/common/time.h"
#include <opencv2/core.hpp>
#include <string>
namespace vslam::sensor {
enum class ColorFormat { RGB, BGR, Gray };
enum class DepthRegistration { Native, RegisteredToColor };
// Caller asserts rectification/registration using calibration, not just dimensions.
struct FrameInput {
  common::Timestamp timestamp;
  cv::Mat image;
  cv::Mat depth;
  ColorFormat color_format = ColorFormat::RGB;
  DepthRegistration registration = DepthRegistration::Native;
  bool rectified = false;
  double meters_per_depth_unit = 1.0;
  std::string external_label;
};
struct PreparedInput {
  common::Timestamp timestamp;
  cv::Mat image_rgb_or_gray;
  cv::Mat depth_z_m;
  std::string external_label;
};
}  // namespace vslam::sensor
