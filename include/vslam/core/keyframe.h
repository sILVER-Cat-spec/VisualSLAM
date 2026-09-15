#pragma once

#include "vslam/core/frame.h"
#include <set>

namespace vslam::core {

class Map;

// Persistent camera view with frozen measurements; Map controls graph and pose updates.
class KeyFrame {
 public:
  // Copy a posed frame with a valid same-epoch ID; retain image clones only if requested.
  // This creates no persistent observations; use Map::InsertKeyFrame to register them.
  static KeyFrame FromFrame(common::KeyFrameId id, const Frame& frame, bool retain_images = false);

  // Return the persistent keyframe identity.
  common::KeyFrameId id() const noexcept {
    return id_;
  }

  // Return the identity of the frame from which this keyframe was promoted.
  common::FrameId source_frame_id() const noexcept {
    return source_frame_id_;
  }

  // Return the source processing order used for keyframe spacing.
  std::uint64_t source_sequence() const noexcept {
    return source_sequence_;
  }

  // Return the original frame capture time and time base.
  common::Timestamp timestamp() const noexcept {
    return timestamp_;
  }

  // Borrow the original input label for tracing this keyframe back to its source.
  const std::string& external_label() const noexcept {
    return external_label_;
  }

  // Borrow frozen keypoint and descriptor measurements.
  const FeatureSet& features() const noexcept {
    return features_;
  }

  // Borrow the current world-to-camera pose maintained by Map.
  const geometry::SE3& pose() const noexcept {
    return tcw_;
  }

  // Borrow the feature-to-point index maintained together with persistent observations.
  const std::vector<common::MapPointId>& associations() const noexcept {
    return associations_;
  }

  // Borrow the IDs of all persistent observations belonging to this keyframe.
  const std::set<common::ObservationId>& observation_ids() const noexcept {
    return observation_ids_;
  }

  // Return a clone of the retained color image, or an empty matrix if not retained.
  cv::Mat image() const {
    return image_.clone();
  }

  // Return a clone of the retained Z-depth image in meters, or an empty matrix.
  cv::Mat depth_z_m() const {
    return depth_.clone();
  }

 private:
  friend class Map;

  // Persistent camera view with frozen measurements; Map controls graph and pose updates.

  // Copy validated frame state; FromFrame checks identity and pose before this call.
  KeyFrame(common::KeyFrameId id, const Frame& frame, bool retain_images);
  common::KeyFrameId id_;
  common::FrameId source_frame_id_;
  std::uint64_t source_sequence_;
  common::Timestamp timestamp_;
  std::string external_label_;

  FeatureSet features_;

  geometry::SE3 tcw_;

  cv::Mat image_;
  cv::Mat depth_;

  std::vector<common::MapPointId> associations_;
  std::set<common::ObservationId> observation_ids_;
};

}  // namespace vslam::core
