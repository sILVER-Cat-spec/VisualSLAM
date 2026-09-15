#include "vslam/sensor/stereo_camera.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace vslam::sensor {
// Validate the horizontal model rather than silently treating arbitrary extrinsics as rectified.
StereoCamera::StereoCamera(CameraModel left, CameraModel right, double baseline_m)
    : left_(std::move(left)), right_(std::move(right)), baseline_m_(baseline_m) {
  if (!std::isfinite(baseline_m) || baseline_m <= 0 || std::abs(left_.fx() - right_.fx()) > 1e-9 ||
      std::abs(left_.fy() - right_.fy()) > 1e-9 || std::abs(left_.cy() - right_.cy()) > 1e-9 ||
      left_.width() != right_.width() || left_.height() != right_.height()) {
    throw std::invalid_argument("Invalid horizontal rectified stereo calibration");
  }
}

// A camera center displacement has the opposite sign to the coordinate transformation.
geometry::SE3 StereoCamera::RightFromLeft() const {
  return geometry::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-baseline_m_, 0, 0));
}

// Reject poorly conditioned disparity before allowing it to provide metric support.
std::optional<double> StereoCamera::Depth(const Eigen::Vector2d& left_uv,
                                          const Eigen::Vector2d& right_uv, double min_depth,
                                          double max_depth, double min_disparity,
                                          double max_epipolar_error) const {
  if (!std::isfinite(min_depth) || !std::isfinite(max_depth) || min_depth <= 0 ||
      max_depth <= min_depth || !std::isfinite(min_disparity) || min_disparity <= 0 ||
      !std::isfinite(max_epipolar_error) || max_epipolar_error < 0) {
    throw std::invalid_argument("Invalid stereo triangulation limits");
  }
  if (!left_.Contains(left_uv) || !right_.Contains(right_uv) ||
      std::abs(left_uv.y() - right_uv.y()) > max_epipolar_error) {
    return std::nullopt;
  }
  const double disparity = left_uv.x() - right_uv.x() - (left_.cx() - right_.cx());
  if (!std::isfinite(disparity) || disparity < min_disparity) {
    return std::nullopt;
  }
  const double depth = left_.fx() * baseline_m_ / disparity;
  if (!std::isfinite(depth) || depth < min_depth || depth > max_depth) {
    return std::nullopt;
  }
  return depth;
}

// Reuse single-view projection after applying the calibrated fixed baseline.
std::optional<Eigen::Vector2d> StereoCamera::ProjectRight(const Eigen::Vector3d& point_left) const {
  return right_.Project(RightFromLeft() * point_left);
}
}  // namespace vslam::sensor
