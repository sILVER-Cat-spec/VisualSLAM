#include "vslam/sensor/rgbd_camera.h"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace vslam::sensor {
RGBDCamera::RGBDCamera(CameraModel camera, double meters_per_depth_unit)
    : camera_(std::move(camera)), meters_per_depth_unit_(meters_per_depth_unit) {
  if (!std::isfinite(meters_per_depth_unit) || meters_per_depth_unit <= 0.0)
    throw std::invalid_argument("RGBDCamera depth scale must be finite and positive");
}
std::optional<Eigen::Vector2d> RGBDCamera::Project(
    const Eigen::Vector3d& point_c_m) const {
  return camera_.Project(point_c_m);
}
std::optional<double> RGBDCamera::DepthZMeters(double raw) const {
  if (!std::isfinite(raw) || raw <= 0.0) return std::nullopt;
  const double z = raw * meters_per_depth_unit_;
  if (!std::isfinite(z) || z <= 0.0) return std::nullopt;
  return z;
}
std::optional<Eigen::Vector3d> RGBDCamera::UnprojectRegisteredDepth(
    const Eigen::Vector2d& pixel, double registered_depth_raw) const {
  const auto z = DepthZMeters(registered_depth_raw);
  if (!z) return std::nullopt;
  return camera_.Unproject(pixel, *z);
}
}  // namespace vslam::sensor
