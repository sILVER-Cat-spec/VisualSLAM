#include "vslam/geometry/se3.h"
#include <Eigen/LU>
#include <cmath>
#include <stdexcept>

namespace vslam::geometry {
SE3::SE3(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
    : rotation_(rotation), translation_(translation) {
  if (!rotation.allFinite() || !translation.allFinite() ||
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-8 ||
      std::abs(rotation.determinant() - 1.0) > 1e-8)
    throw std::invalid_argument("SE3 requires a finite proper rotation and translation");
}
SE3 SE3::FromMatrix(const Eigen::Matrix4d& matrix) {
  if (!matrix.allFinite() ||
      (matrix.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-8)
    throw std::invalid_argument("SE3 matrix has invalid homogeneous row");
  return SE3(matrix.topLeftCorner<3,3>(), matrix.topRightCorner<3,1>());
}
Eigen::Matrix4d SE3::Matrix() const {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.topLeftCorner<3,3>() = rotation_;
  matrix.topRightCorner<3,1>() = translation_;
  return matrix;
}
SE3 SE3::Inverse() const { return SE3(rotation_.transpose(), -rotation_.transpose()*translation_); }
SE3 SE3::operator*(const SE3& other) const {
  return SE3(rotation_*other.rotation_, rotation_*other.translation_+translation_);
}
Eigen::Vector3d SE3::operator*(const Eigen::Vector3d& point) const {
  const Eigen::Vector3d result = rotation_*point+translation_;
  if (!point.allFinite() || !result.allFinite())
    throw std::invalid_argument("SE3 transform requires finite point and result");
  return result;
}
}  // namespace vslam::geometry
