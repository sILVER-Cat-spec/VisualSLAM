#pragma once

#include <Eigen/Core>
#include <optional>

namespace vslam::sensor {

// Geometry on an already rectified, zero-skew pinhole pixel grid.
// Camera coordinates: +X right, +Y down, +Z forward.
// No distortion, raw-depth scaling, registration, or image ownership.
class CameraModel final {
 public:
  // Throws std::invalid_argument for nonfinite intrinsics, nonpositive focal
  // lengths, or nonpositive dimensions. Principal points may be outside the image.
  CameraModel(double fx, double fy, double cx, double cy, int width, int height);

  double fx() const noexcept { return fx_; }
  double fy() const noexcept { return fy_; }
  double cx() const noexcept { return cx_; }
  double cy() const noexcept { return cy_; }
  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }
  Eigen::Matrix3d K() const;

  // Invalid/nonfinite geometry or nonfinite results return nullopt.
  // Projection outside the image is valid; use Contains separately.
  std::optional<Eigen::Vector2d> Project(const Eigen::Vector3d& point_c) const;
  // Returns (x/z, y/z, 1), NOT a Euclidean unit vector.
  std::optional<Eigen::Vector3d> BackProject(const Eigen::Vector2d& pixel) const;
  // depth_z is positive camera-axis Z, not range along the ray.
  // Result uses the same units as depth_z (meters for prepared RGB-D depth).
  // RGB-D depth must already be registered to this camera's rectified grid.
  std::optional<Eigen::Vector3d> Unproject(
      const Eigen::Vector2d& pixel, double depth_z) const;
  // Half-open bounds: [0, width) x [0, height).
  bool Contains(const Eigen::Vector2d& pixel) const noexcept;

 private:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  int width_;
  int height_;
};

}  // namespace vslam::sensor
