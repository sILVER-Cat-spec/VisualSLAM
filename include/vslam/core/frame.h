#pragma once

#include "vslam/common/ids.h"
#include "vslam/common/time.h"
#include "vslam/core/feature_set.h"
#include "vslam/geometry/se3.h"
#include <string>

namespace vslam::core {

// Transient input and tracking state; associations are IDs, not persistent observations.
class Frame {
 public:
  // Images must already share a rectified grid; color is RGB8 or grayscale.
  // Depth is optional CV_32FC1 Z meters.
  // Own copies are acquired here; invalid depth pixels remain for the sampler to reject.
  Frame(common::FrameId id, std::uint64_t sequence, common::Timestamp timestamp,
        const cv::Mat& image, const cv::Mat& depth_z_m = {}, std::string external_label = {});

  // Deep-copy images, features, pose, and associations into an independent frame.
  Frame(const Frame& other);

  // Stage a full copy before replacing this frame to preserve it if copying fails.
  Frame& operator=(const Frame& other);

  // Transfer the frame buffers and state from another frame.
  Frame(Frame&&) noexcept = default;

  // Replace this frame by moving the source buffers and state.
  Frame& operator=(Frame&&) noexcept = default;

  // Return the frame identity, including its system epoch.
  common::FrameId id() const noexcept {
    return id_;
  }

  // Return the internal processing order used for keyframe spacing.
  std::uint64_t sequence() const noexcept {
    return sequence_;
  }

  // Return capture time in nanoseconds together with its time base.
  common::Timestamp timestamp() const noexcept {
    return timestamp_;
  }

  // Borrow the external label; it does not determine frame identity or ordering.
  const std::string& external_label() const noexcept {
    return external_label_;
  }

  // Return a cloned RGB or grayscale image that callers may safely modify.
  cv::Mat image() const {
    return image_.clone();
  }

  // Return a cloned camera-axis depth image in meters, or an empty matrix.
  cv::Mat depth_z_m() const {
    return depth_.clone();
  }

  // Borrow the immutable measurements associated with this frame.
  const FeatureSet& features() const noexcept {
    return features_;
  }

  // Borrow the optional world-to-camera pose; an empty value means no estimate.
  const std::optional<geometry::SE3>& pose() const noexcept {
    return pose_;
  }

  // Borrow feature-to-point IDs; invalid IDs denote unassociated features.
  const std::vector<common::MapPointId>& associations() const noexcept {
    return associations_;
  }

  // Validate image bounds, copy measurements, and clear all previous point associations.
  void SetFeatures(const FeatureSet& features);

  // Store a world-to-camera pose without changing measurements or associations.
  void SetPose(const geometry::SE3& tcw) {
    pose_ = tcw;
  }

  // Remove the pose estimate without changing point associations.
  void ClearPose() noexcept {
    pose_.reset();
  }

  // Assign a valid same-epoch point ID to a feature; this does not create an Observation.
  void Associate(std::size_t feature, common::MapPointId point);

  // Clear one feature association; throw std::out_of_range for an invalid index.
  void RemoveAssociation(std::size_t feature) {
    associations_.at(feature) = {};
  }

 private:
  common::FrameId id_;
  std::uint64_t sequence_;
  common::Timestamp timestamp_;
  std::string external_label_;

  cv::Mat image_;
  cv::Mat depth_;

  FeatureSet features_;

  std::optional<geometry::SE3> pose_;
  std::vector<common::MapPointId> associations_;
};

// One instance per system epoch; independent of external frame/file labels.
class FrameFactory {
 public:
  // Start independent ID, timestamp, and sequence tracking for one system epoch.
  FrameFactory(common::Epoch epoch, common::TimeBase base)
      : ids_(epoch),
        stamps_(base) {}

  // Create an owned frame; advance accepted time and sequence only after construction succeeds.
  Frame Create(common::Timestamp timestamp, const cv::Mat& image, const cv::Mat& depth_z_m = {},
               std::string external_label = {});

 private:
  common::IdGenerator<common::FrameId> ids_;
  common::TimestampSequence stamps_;
  std::uint64_t sequence_ = 0;
};

}  // namespace vslam::core
