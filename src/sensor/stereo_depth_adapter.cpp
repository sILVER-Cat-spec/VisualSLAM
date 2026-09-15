#include "vslam/sensor/stereo_depth_adapter.h"

#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
namespace vslam::sensor {
namespace {
// Bound SGBM parameters and all numerical filtering thresholds.
void Validate(const StereoDepthOptions& options) {
  if (options.num_disparities < 16 || options.num_disparities > 2048 ||
      options.num_disparities % 16 != 0 || options.min_disparity < -1024 ||
      options.min_disparity > 1024 || options.block_size < 3 || options.block_size > 21 ||
      options.block_size % 2 == 0 || !std::isfinite(options.min_depth) ||
      !std::isfinite(options.max_depth) || options.min_depth <= 0 ||
      options.max_depth <= options.min_depth || !std::isfinite(options.consistency_pixels) ||
      options.consistency_pixels <= 0) {
    throw std::invalid_argument("Invalid stereo depth settings");
  }
}
// Feed SGBM single-channel images in the already rectified grid.
cv::Mat Gray(const cv::Mat& image) {
  cv::Mat result;
  if (image.type() == CV_8UC3) {
    cv::cvtColor(image, result, cv::COLOR_RGB2GRAY);
  } else {
    result = image;
  }
  return result;
}
}  // namespace
// Keep invalid depth as NaN, including the SGBM sentinel before unit conversion.
StereoDepthResult ConvertStereoDisparity(const PreparedStereoInput& pair,
                                         const StereoCamera& camera, const cv::Mat& forward_fixed,
                                         const cv::Mat& reverse_fixed,
                                         const StereoDepthOptions& options) {
  Validate(options);
  const cv::Size size(camera.width(), camera.height());
  if (forward_fixed.type() != CV_16SC1 || reverse_fixed.type() != CV_16SC1 ||
      forward_fixed.size() != size || reverse_fixed.size() != size ||
      pair.left_mask.type() != CV_8UC1 || pair.right_mask.type() != CV_8UC1 ||
      pair.left_mask.size() != size || pair.right_mask.size() != size) {
    throw std::invalid_argument("Invalid fixed disparity or ROI grids");
  }
  StereoDepthResult result;
  result.frame.timestamp = pair.timestamp;
  result.frame.image = pair.image_rgb_or_gray.clone();
  result.frame.color_format =
      result.frame.image.channels() == 1 ? ColorFormat::Gray : ColorFormat::RGB;
  result.frame.external_label = pair.external_label;
  result.frame.registration = DepthRegistration::RegisteredToColor;
  result.frame.rectified = true;
  result.frame.meters_per_depth_unit = 1.0;
  const float invalid = std::numeric_limits<float>::quiet_NaN();
  result.frame.depth = cv::Mat(size, CV_32FC1, cv::Scalar(invalid));
  result.disparity_pixels = cv::Mat(size, CV_32FC1, cv::Scalar(invalid));
  result.valid_mask = cv::Mat::zeros(size, CV_8UC1);
  const int reverse_min = -options.min_disparity - options.num_disparities + 1;
  for (int y = 0; y < size.height; ++y) {
    for (int x = 0; x < size.width; ++x) {
      const int fixed = forward_fixed.at<short>(y, x);
      if (fixed <= (options.min_disparity - 1) * 16 || !pair.left_mask.at<unsigned char>(y, x)) {
        continue;
      }
      const double disparity = fixed / 16.0;
      const double right_u = x - disparity;
      const int right_x = cvRound(right_u);
      if (right_x < 0 || right_x >= size.width || !pair.right_mask.at<unsigned char>(y, right_x)) {
        continue;
      }
      const int reverse = reverse_fixed.at<short>(y, right_x);
      if (reverse <= (reverse_min - 1) * 16 ||
          std::abs(disparity + reverse / 16.0) > options.consistency_pixels) {
        continue;
      }
      const auto depth = camera.Depth({double(x), double(y)}, {right_u, double(y)},
                                      options.min_depth, options.max_depth);
      if (depth) {
        result.frame.depth.at<float>(y, x) = static_cast<float>(*depth);
        result.disparity_pixels.at<float>(y, x) = static_cast<float>(disparity);
        result.valid_mask.at<unsigned char>(y, x) = 255;
      }
    }
  }
  result.valid_ratio = double(cv::countNonZero(result.valid_mask)) / size.area();
  return result;
}

// Use explicit reverse SGBM and a consistency test, in addition to uniqueness/speckle filtering.
StereoDepthResult EstimateStereoDepth(const PreparedStereoInput& pair, const StereoCamera& camera,
                                      const StereoDepthOptions& options) {
  Validate(options);
  const auto left = Gray(pair.image_rgb_or_gray);
  const auto right = Gray(pair.right_image);
  const int area = options.block_size * options.block_size;
  auto forward =
      cv::StereoSGBM::create(options.min_disparity, options.num_disparities, options.block_size,
                             8 * area, 32 * area, -1, 31, 10, 100, 2, cv::StereoSGBM::MODE_SGBM);
  auto reverse = cv::StereoSGBM::create(-options.min_disparity - options.num_disparities + 1,
                                        options.num_disparities, options.block_size, 8 * area,
                                        32 * area, -1, 31, 10, 100, 2, cv::StereoSGBM::MODE_SGBM);
  cv::Mat forward_fixed;
  cv::Mat reverse_fixed;
  forward->compute(left, right, forward_fixed);
  reverse->compute(right, left, reverse_fixed);
  return ConvertStereoDisparity(pair, camera, forward_fixed, reverse_fixed, options);
}
}  // namespace vslam::sensor
