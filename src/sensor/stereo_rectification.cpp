
#include "vslam/sensor/stereo_rectification.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
namespace vslam::sensor {
namespace {
// Copy Eigen calibration into OpenCV without writable aliases.
cv::Mat Matrix(const Eigen::Matrix3d& matrix) {
  cv::Mat result(3, 3, CV_64FC1);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.at<double>(row, column) = matrix(row, column);
    }
  }
  return result;
}
// Accept only documented finite OpenCV distortion coefficient layouts.
cv::Mat Distortion(const std::vector<double>& coefficients) {
  const auto count = coefficients.size();
  if (count != 0 && count != 4 && count != 5 && count != 8 && count != 12 && count != 14) {
    throw std::invalid_argument("Invalid distortion coefficient count");
  }
  for (double value : coefficients) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Nonfinite distortion coefficient");
    }
  }
  return coefficients.empty() ? cv::Mat() : cv::Mat(coefficients).clone();
}
}  // namespace
// Rectification owns all maps and explicitly exposes the change of left reference frame.
RectifiedStereoCalibration RectifyStereo(const CameraModel& raw_left, const CameraModel& raw_right,
                                         const std::vector<double>& left_distortion,
                                         const std::vector<double>& right_distortion,
                                         const geometry::SE3& right_from_left) {
  if (raw_left.width() != raw_right.width() || raw_left.height() != raw_right.height() ||
      right_from_left.translation().norm() <= 1e-9) {
    throw std::invalid_argument("Stereo rectification requires equal grids and nonzero baseline");
  }
  const cv::Size size(raw_left.width(), raw_left.height());
  const auto left_k = Matrix(raw_left.K());
  const auto right_k = Matrix(raw_right.K());
  const auto left_d = Distortion(left_distortion);
  const auto right_d = Distortion(right_distortion);
  const auto rotation = Matrix(right_from_left.rotation());
  cv::Mat translation(3, 1, CV_64FC1);
  for (int axis = 0; axis < 3; ++axis) {
    translation.at<double>(axis) = right_from_left.translation()[axis];
  }
  cv::Mat left_rotation;
  cv::Mat right_rotation;
  cv::Mat left_projection;
  cv::Mat right_projection;
  cv::Mat q;
  cv::Rect left_roi;
  cv::Rect right_roi;
  cv::stereoRectify(left_k, left_d, right_k, right_d, size, rotation, translation, left_rotation,
                    right_rotation, left_projection, right_projection, q, 0, 0, size, &left_roi,
                    &right_roi);
  const double baseline = -right_projection.at<double>(0, 3) / right_projection.at<double>(0, 0);
  if (baseline <= 0 || std::abs(right_projection.at<double>(1, 3)) > 1e-9 || left_roi.empty() ||
      right_roi.empty()) {
    throw std::invalid_argument("Rectified rig is not a positive horizontal stereo pair");
  }
  CameraModel left(left_projection.at<double>(0, 0), left_projection.at<double>(1, 1),
                   left_projection.at<double>(0, 2), left_projection.at<double>(1, 2), size.width,
                   size.height);
  CameraModel right(right_projection.at<double>(0, 0), right_projection.at<double>(1, 1),
                    right_projection.at<double>(0, 2), right_projection.at<double>(1, 2),
                    size.width, size.height);
  StereoRectification maps;
  cv::initUndistortRectifyMap(left_k, left_d, left_rotation, left_projection, size, CV_32FC1,
                              maps.left_x, maps.left_y);
  cv::initUndistortRectifyMap(right_k, right_d, right_rotation, right_projection, size, CV_32FC1,
                              maps.right_x, maps.right_y);
  maps.left_mask = cv::Mat::zeros(size, CV_8UC1);
  maps.right_mask = cv::Mat::zeros(size, CV_8UC1);
  maps.left_mask(left_roi).setTo(255);
  maps.right_mask(right_roi).setTo(255);
  Eigen::Matrix3d left_rectification;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      left_rectification(row, column) = left_rotation.at<double>(row, column);
    }
  }
  return {StereoCamera(left, right, baseline), std::move(maps),
          geometry::SE3(left_rectification, Eigen::Vector3d::Zero())};
}
}  // namespace vslam::sensor
