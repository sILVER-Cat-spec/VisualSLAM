#pragma once

#include "vslam/core/observation.h"
#include "vslam/common/time.h"
#include "vslam/geometry/se3.h"
#include <vector>

namespace vslam::core {

// Detached keyframe identity, capture metadata, and world-to-camera pose.
struct KeyFrameGeometry {
  common::KeyFrameId id;
  common::FrameId source_frame_id;
  std::uint64_t source_sequence;
  common::Timestamp timestamp;
  geometry::SE3 tcw;
};

// Detached world-space landmark position in meters and its quality flag.
struct MapPointGeometry {
  common::MapPointId id;
  Eigen::Vector3d position_w;
  bool is_bad;
};

// Detached numeric diagnostic snapshot, not a restorable serialized map.
struct MapSnapshot {
  common::Epoch epoch;
  common::Revision revision;
  std::vector<KeyFrameGeometry> keyframes;
  std::vector<MapPointGeometry> points;
  std::vector<Observation> observations;
};

// Candidate world-to-camera pose submitted through Map::CommitGeometry.
struct PoseUpdate {
  common::KeyFrameId id;
  geometry::SE3 tcw;
};

// Candidate world-space landmark position in meters for a geometry batch.
struct PointUpdate {
  common::MapPointId id;
  Eigen::Vector3d position_w;
};

}  // namespace vslam::core
