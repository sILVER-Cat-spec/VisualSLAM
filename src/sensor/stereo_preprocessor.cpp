#include "vslam/sensor/stereo_preprocessor.h"

#include <opencv2/imgproc.hpp>
#include <stdexcept>
namespace vslam::sensor {
namespace {
// Normalize color order without sharing writable input buffers.
cv::Mat Normalize(const cv::Mat& image, bool bgr) {
  if (image.empty() || image.dims != 2 || (image.type() != CV_8UC1 && image.type() != CV_8UC3)) {
    throw std::invalid_argument("Stereo images must be nonempty uint8 gray or color");
  }
  cv::Mat result;
  if (image.type() == CV_8UC3 && bgr) {
    cv::cvtColor(image, result, cv::COLOR_BGR2RGB);
  } else {
    result = image.clone();
  }
  return result;
}
// Validate a mask or construct full support on an already rectified grid.
cv::Mat Mask(const cv::Mat& mask, cv::Size size) {
  if (mask.empty()) {
    return cv::Mat(size, CV_8UC1, cv::Scalar(255));
  }
  if (mask.type() != CV_8UC1 || mask.size() != size) {
    throw std::invalid_argument("Invalid rectification ROI mask");
  }
  return mask.clone();
}
}  // namespace
// Enforce capture and output-grid contracts before any algorithm state advances.
PreparedStereoInput PrepareStereo(const StereoFrameInput& input, const StereoCamera& camera,
                                  std::int64_t max_skew_ns,
                                  const StereoRectification& rectification) {
  if (max_skew_ns < 0 || input.timestamp.ns < 0 || input.right_timestamp.ns < 0 ||
      input.timestamp.base != input.right_timestamp.base ||
      (input.timestamp.ns > input.right_timestamp.ns
           ? input.timestamp.ns - input.right_timestamp.ns
           : input.right_timestamp.ns - input.timestamp.ns) > max_skew_ns) {
    throw std::invalid_argument("Stereo capture timestamps do not match");
  }
  PreparedStereoInput output;
  output.timestamp = input.timestamp;
  output.right_timestamp = input.right_timestamp;
  output.external_label = input.external_label;
  output.image_rgb_or_gray = Normalize(input.image, input.bgr);
  output.right_image = Normalize(input.right_image, input.bgr);
  const cv::Size size(camera.width(), camera.height());
  if (input.image.size() != size || input.right_image.size() != size) {
    throw std::invalid_argument("Raw image grid differs from calibrated source dimensions");
  }
  if (!input.rectified) {
    const cv::Mat maps[] = {rectification.left_x, rectification.left_y, rectification.right_x,
                            rectification.right_y};
    for (const auto& map : maps) {
      if (map.type() != CV_32FC1 || map.size() != size || !cv::checkRange(map)) {
        throw std::invalid_argument(
            "Raw stereo input requires finite calibrated rectification maps");
      }
    }
    cv::remap(output.image_rgb_or_gray.clone(), output.image_rgb_or_gray, maps[0], maps[1],
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(output.right_image.clone(), output.right_image, maps[2], maps[3], cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);
    if (rectification.left_mask.empty() || rectification.right_mask.empty()) {
      throw std::invalid_argument("Raw rectification requires explicit valid ROI masks");
    }
  }
  if (output.image_rgb_or_gray.size() != size || output.right_image.size() != size ||
      output.image_rgb_or_gray.type() != output.right_image.type()) {
    throw std::invalid_argument("Stereo output grid or encoding differs from calibration");
  }
  output.left_mask = Mask(rectification.left_mask, size);
  output.right_mask = Mask(rectification.right_mask, size);
  return output;
}
}  // namespace vslam::sensor
