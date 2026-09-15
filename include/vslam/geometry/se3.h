#pragma once
#include <Eigen/Core>

namespace vslam::geometry {
// Value type. Tcw maps world -> camera; Inverse() is Twc.
// No unconstrained writable matrix or rotation reference is exposed.
class SE3 {
 public:
  SE3() = default;
  SE3(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);
  static SE3 FromMatrix(const Eigen::Matrix4d& matrix);
  Eigen::Matrix4d Matrix() const;
  const Eigen::Matrix3d& rotation() const noexcept { return rotation_; }
  const Eigen::Vector3d& translation() const noexcept { return translation_; }
  SE3 Inverse() const;
  SE3 operator*(const SE3& other) const;
  Eigen::Vector3d operator*(const Eigen::Vector3d& point) const;
 private:
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();
};
}  // namespace vslam::geometry
