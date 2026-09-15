#pragma once
#include "vslam/sensor/camera_model.h"

namespace vslam::sensor {

// RGB-D geometry for simulation inputs on a shared rectified RGB/depth grid.
// Registered depth is RGB-camera-axis Z, not ray distance. Native unregistered
// depth and distorted images must be prepared before using this API.
class RGBDCamera final {
 public:
  // meters_per_depth_unit: 1 for meter-valued depth, 0.001 for millimeters.
  // Throws invalid_argument unless scale is finite and positive.
  RGBDCamera(CameraModel camera, double meters_per_depth_unit);
  const CameraModel& model() const noexcept { return camera_; }
  double meters_per_depth_unit() const noexcept { return meters_per_depth_unit_; }
  std::optional<Eigen::Vector2d> Project(const Eigen::Vector3d& point_c_m) const;
  std::optional<double> DepthZMeters(double registered_depth_raw) const;
  std::optional<Eigen::Vector3d> UnprojectRegisteredDepth(
      const Eigen::Vector2d& pixel, double registered_depth_raw) const;

 private:
  CameraModel camera_;
  double meters_per_depth_unit_;
};
}  // namespace vslam::sensor
