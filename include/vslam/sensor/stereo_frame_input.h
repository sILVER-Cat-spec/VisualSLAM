#pragma once
#include <opencv2/core.hpp>
#include <string>

#include "vslam/common/time.h"
namespace vslam::sensor {
// A complete calibrated pair; timestamps retain their original capture values.
struct StereoFrameInput {
  common::Timestamp timestamp;
  common::Timestamp right_timestamp;
  cv::Mat image;
  cv::Mat right_image;
  bool rectified = false;
  bool bgr = true;
  std::string external_label;
};
struct PreparedStereoInput {
  common::Timestamp timestamp;
  common::Timestamp right_timestamp;
  cv::Mat image_rgb_or_gray;
  cv::Mat right_image;
  cv::Mat left_mask;
  cv::Mat right_mask;
  std::string external_label;
};
}  // namespace vslam::sensor
