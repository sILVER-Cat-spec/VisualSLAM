#include "vslam/sensor/camera_model.h"

#include <cmath>
#include <stdexcept>

namespace vslam::sensor {

CameraModel::CameraModel(double fx, double fy, double cx, double cy,
                         int width, int height)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), width_(width), height_(height) {
  if (!std::isfinite(fx) || !std::isfinite(fy) ||
      !std::isfinite(cx) || !std::isfinite(cy) ||
      fx <= 0.0 || fy <= 0.0 || width <= 0 || height <= 0) {
    throw std::invalid_argument("CameraModel requires finite intrinsics, positive focal lengths and dimensions");
  }
}

Eigen::Matrix3d CameraModel::K() const {
  Eigen::Matrix3d k;
  k << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
  return k;
}

std::optional<Eigen::Vector2d> CameraModel::Project(
    const Eigen::Vector3d& point_c) const {
  if (!point_c.allFinite() || point_c.z() <= 0.0) return std::nullopt;
  const Eigen::Vector2d pixel(fx_ * (point_c.x() / point_c.z()) + cx_,
                              fy_ * (point_c.y() / point_c.z()) + cy_);
  if (!pixel.allFinite()) return std::nullopt;
  return pixel;
}

std::optional<Eigen::Vector3d> CameraModel::BackProject(
    const Eigen::Vector2d& pixel) const {
  if (!pixel.allFinite()) return std::nullopt;
  const Eigen::Vector3d ray((pixel.x() - cx_) / fx_,
                            (pixel.y() - cy_) / fy_, 1.0);
  if (!ray.allFinite()) return std::nullopt;
  return ray;
}

std::optional<Eigen::Vector3d> CameraModel::Unproject(
    const Eigen::Vector2d& pixel, double depth_z) const {
  if (!std::isfinite(depth_z) || depth_z <= 0.0) return std::nullopt;
  const auto ray = BackProject(pixel);
  if (!ray) return std::nullopt;
  const Eigen::Vector3d point = *ray * depth_z;
  if (!point.allFinite()) return std::nullopt;
  return point;
}

bool CameraModel::Contains(const Eigen::Vector2d& pixel) const noexcept {
  return pixel.allFinite() && pixel.x() >= 0.0 && pixel.y() >= 0.0 &&
         pixel.x() < width_ && pixel.y() < height_;
}

}  // namespace vslam::sensor
