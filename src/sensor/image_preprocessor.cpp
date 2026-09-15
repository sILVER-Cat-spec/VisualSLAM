#include "vslam/sensor/image_preprocessor.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace vslam::sensor {

using std::invalid_argument;
using std::isfinite;
using std::max;
using std::min;

// Validate rectified, registered RGB-D input and create owned image buffers.
// Convert BGR to RGB when needed and scale raw camera-axis depth to meters once.
// Throw invalid_argument when input declarations, dimensions, types, or scale are invalid.
PreparedInput Prepare(const FrameInput& input, const CameraModel& camera) {
  if (!input.rectified || input.registration != DepthRegistration::RegisteredToColor) {
    throw invalid_argument("RGB-D input must be rectified and registered to RGB Z grid");
  }

  if (input.image.dims != 2 || input.image.cols != camera.width() ||
      input.image.rows != camera.height() || input.depth.dims != 2 ||
      input.depth.size() != input.image.size() ||
      (input.depth.type() != CV_32FC1 && input.depth.type() != CV_16UC1) ||
      !isfinite(input.meters_per_depth_unit) || input.meters_per_depth_unit <= 0) {
    throw invalid_argument("RGB-D input dimensions, depth type, or scale are invalid");
  }

  PreparedInput output;
  output.timestamp = input.timestamp;
  output.external_label = input.external_label;

  if (input.color_format == ColorFormat::Gray && input.image.type() == CV_8UC1) {
    output.image_rgb_or_gray = input.image.clone();
  } else if ((input.color_format == ColorFormat::RGB || input.color_format == ColorFormat::BGR) &&
             input.image.type() == CV_8UC3) {
    if (input.color_format == ColorFormat::BGR) {
      cv::cvtColor(input.image, output.image_rgb_or_gray, cv::COLOR_BGR2RGB);
    } else {
      output.image_rgb_or_gray = input.image.clone();
    }
  } else {
    throw invalid_argument("Image type does not match declared color format");
  }

  input.depth.convertTo(output.depth_z_m, CV_32F, input.meters_per_depth_unit);

  return output;
}

// Sample the median valid Z depth from a pixel neighborhood in a float32 meter image.
// Return an empty optional for an invalid pixel or a neighborhood without valid samples.
// Throw invalid_argument for an invalid depth image type, range, or sampling radius.
std::optional<double> SampleDepth(const cv::Mat& depth, const Eigen::Vector2d& uv,
                                  double min_depth, double max_depth, int radius) {
  if (depth.type() != CV_32FC1 || depth.dims != 2 || radius < 0 || radius > 16 ||
      !isfinite(min_depth) || !isfinite(max_depth) || min_depth <= 0 || max_depth <= min_depth) {
    throw invalid_argument("Invalid depth sampling contract");
  }

  if (!uv.allFinite() || uv.x() < 0 || uv.y() < 0 ||
      uv.x() >= depth.cols || uv.y() >= depth.rows) {
    return {};
  }

  // Rounding can move an otherwise valid subpixel coordinate past the image boundary.
  const int x = cvRound(uv.x());
  const int y = cvRound(uv.y());
  if (x >= depth.cols || y >= depth.rows) {
    return {};
  }

  std::vector<double> samples;
  for (int v = max(0, y - radius); v <= min(depth.rows - 1, y + radius); ++v) {
    for (int u = max(0, x - radius); u <= min(depth.cols - 1, x + radius); ++u) {
      const double z = depth.at<float>(v, u);
      if (isfinite(z) && z >= min_depth && z <= max_depth) {
        samples.push_back(z);
      }
    }
  }

  if (samples.empty()) {
    return {};
  }

  std::sort(samples.begin(), samples.end());
  const auto mid = samples.size() / 2;

  return samples.size() % 2 ? samples[mid] : (samples[mid - 1] + samples[mid]) * 0.5;
}

}  // namespace vslam::sensor
