#pragma once
#include "vslam/geometry/se3.h"
#include "vslam/sensor/camera_model.h"

namespace vslam::sensor {
// Fixed horizontal rectified pair. Optical axes are X right, Y down, Z forward.
// Composition preserves the single-view meaning of CameraModel.
class StereoCamera {
 public:
  // Require equal focal lengths, vertical principal points and image dimensions;
  // baseline is the positive right camera center X coordinate in meters.
  StereoCamera(CameraModel left, CameraModel right, double baseline_m);
  // Borrow the rectified left projection model.
  const CameraModel& left() const noexcept {
    return left_;
  }
  // Borrow the rectified right projection model.
  const CameraModel& right() const noexcept {
    return right_;
  }
  // Return the fixed positive baseline in meters.
  double baseline() const noexcept {
    return baseline_m_;
  }
  // Transform left optical coordinates into right coordinates; translation is -b.
  geometry::SE3 RightFromLeft() const;
  // Recover Z meters using disparity corrected for the horizontal principal-point offset.
  // Return nullopt for invalid pixels, epipolar error, small disparity or depth outside range.
  std::optional<double> Depth(const Eigen::Vector2d& left_uv, const Eigen::Vector2d& right_uv,
                              double min_depth = 0.2, double max_depth = 8.0,
                              double min_disparity = 0.5, double max_epipolar_error = 2.0) const;
  // Project a point expressed in left coordinates into the right image.
  std::optional<Eigen::Vector2d> ProjectRight(const Eigen::Vector3d& point_left) const;
  // Forward left-view geometry for the existing tracking and mapping interfaces.
  std::optional<Eigen::Vector2d> Project(const Eigen::Vector3d& p) const {
    return left_.Project(p);
  }
  // Back-project a left pixel into a ray with Z equal to one.
  std::optional<Eigen::Vector3d> BackProject(const Eigen::Vector2d& uv) const {
    return left_.BackProject(uv);
  }
  // Reconstruct a left-camera point from camera-axis Z in meters.
  std::optional<Eigen::Vector3d> Unproject(const Eigen::Vector2d& uv, double z) const {
    return left_.Unproject(uv, z);
  }
  // Check half-open bounds in the rectified left image.
  bool Contains(const Eigen::Vector2d& uv) const noexcept {
    return left_.Contains(uv);
  }
  // Return the left horizontal focal length in pixels.
  double fx() const noexcept {
    return left_.fx();
  }
  // Return the left vertical focal length in pixels.
  double fy() const noexcept {
    return left_.fy();
  }
  // Return the left horizontal principal point in pixels.
  double cx() const noexcept {
    return left_.cx();
  }
  // Return the left vertical principal point in pixels.
  double cy() const noexcept {
    return left_.cy();
  }
  // Return the common image width.
  int width() const noexcept {
    return left_.width();
  }
  // Return the common image height.
  int height() const noexcept {
    return left_.height();
  }

 private:
  CameraModel left_;
  CameraModel right_;
  double baseline_m_;
};
}  // namespace vslam::sensor
