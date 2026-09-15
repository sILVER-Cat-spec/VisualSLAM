#pragma once

#include "vslam/common/ids.h"
#include "vslam/core/feature_set.h"
#include <map>

namespace vslam::core {

class Map;

// World-space landmark owned by Map, with one observation per observing keyframe.
class MapPoint {
 public:
  // Return the persistent landmark identity.
  common::MapPointId id() const noexcept {
    return id_;
  }

  // Borrow the landmark position in world coordinates, measured in meters.
  const Eigen::Vector3d& position_w() const noexcept {
    return position_w_;
  }

  // Identify the representation of the representative descriptor.
  DescriptorKind descriptor_kind() const noexcept {
    return kind_;
  }

  // Return a descriptor clone, or an empty matrix when no descriptor was supplied.
  cv::Mat descriptor() const {
    return descriptor_.clone();
  }

  // Report the quality flag; marking a point bad does not remove its observations.
  bool is_bad() const noexcept {
    return is_bad_;
  }

  // Borrow the reverse index from observing keyframes to their observation IDs.
  const std::map<common::KeyFrameId, common::ObservationId>& observations() const noexcept {
    return observations_;
  }

 private:
  friend class Map;

  // World-space landmark owned by Map, with one observation per observing keyframe.

  // Own validated landmark geometry and a descriptor clone; only Map may construct it.
  MapPoint(common::MapPointId id, const Eigen::Vector3d& position, DescriptorKind kind,
           const cv::Mat& descriptor)
      : id_(id),
        position_w_(position),
        kind_(kind),
        descriptor_(descriptor.clone()) {}
  common::MapPointId id_;
  Eigen::Vector3d position_w_;

  DescriptorKind kind_;
  cv::Mat descriptor_;

  bool is_bad_ = false;

  std::map<common::KeyFrameId, common::ObservationId> observations_;
};

}  // namespace vslam::core
