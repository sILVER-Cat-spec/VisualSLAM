#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

namespace vslam::core {

// Supported descriptor representations for a homogeneous feature set.
enum class DescriptorKind { Orb, Sift };

// One image measurement and its optional registered camera-axis depth in meters.
struct Keypoint {
  Eigen::Vector2d uv = Eigen::Vector2d::Zero();
  int octave = 0;
  float angle = -1.0f;  // -1 means unavailable; otherwise degrees in [0,360).
  float response = 0.0f;
  float size = 1.0f;

  std::optional<double> depth_z_m;
};

// Immutable measurements. cv::Mat getters return clones to prevent writable aliases.
class FeatureSet {
 public:
  // Create an empty feature set with a validated descriptor kind.
  explicit FeatureSet(DescriptorKind kind = DescriptorKind::Orb);

  // Own keypoint values and a descriptor clone; reject invalid measurements or layout.
  FeatureSet(DescriptorKind kind, std::vector<Keypoint> keypoints, const cv::Mat& descriptors);

  // Deep-copy descriptors so the two feature sets do not share writable buffers.
  FeatureSet(const FeatureSet& other);

  // Replace measurements through a staged deep copy for exception safety.
  FeatureSet& operator=(const FeatureSet& other);

  // Transfer measurement storage from a temporary feature set.
  FeatureSet(FeatureSet&&) noexcept = default;

  // Replace measurement storage by moving it from another feature set.
  FeatureSet& operator=(FeatureSet&&) noexcept = default;

  // Identify the descriptor representation used by every feature.
  DescriptorKind kind() const noexcept {
    return kind_;
  }

  // Return the number of keypoints and corresponding descriptor rows.
  std::size_t size() const noexcept {
    return keypoints_.size();
  }

  // Borrow immutable keypoint measurements from this feature set.
  const std::vector<Keypoint>& keypoints() const noexcept {
    return keypoints_;
  }

  // Borrow one measurement; throw std::out_of_range for an invalid index.
  const Keypoint& at(std::size_t index) const {
    return keypoints_.at(index);
  }

  // Return a deep copy of all descriptors to prevent writable aliases.
  cv::Mat descriptors() const {
    return descriptors_.clone();
  }

  // Return a cloned descriptor row; throw std::out_of_range for an invalid index.
  cv::Mat Descriptor(std::size_t index) const;

 private:
  DescriptorKind kind_;
  std::vector<Keypoint> keypoints_;
  cv::Mat descriptors_;
};

// Reject an unknown descriptor kind, incompatible matrix layout, or invalid values.
void ValidateDescriptors(DescriptorKind kind, const cv::Mat& descriptors, std::size_t rows);
}  // namespace vslam::core
