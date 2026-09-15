#include "vslam/core/map.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vslam::core {

using std::invalid_argument;
using std::isfinite;
using std::move;
using std::set;

Map::Map(DescriptorKind kind, common::Epoch epoch)
    : kind_(kind),
      epoch_(epoch),
      keyframe_ids_(epoch),
      point_ids_(epoch),
      observation_ids_(epoch) {
  ValidateDescriptors(kind, {}, 0);
}

const KeyFrame& Map::GetKeyFrame(common::KeyFrameId id) const {
  return *keyframes_.at(id);
}

const MapPoint& Map::GetMapPoint(common::MapPointId id) const {
  return *points_.at(id);
}

const Observation& Map::GetObservation(common::ObservationId id) const {
  return observations_.at(id);
}

common::MapPointId Map::AddMapPoint(const Eigen::Vector3d& position, const cv::Mat& descriptor) {
  if (!position.allFinite()) {
    throw invalid_argument("MapPoint must have finite world position");
  }
  if (!descriptor.empty()) {
    ValidateDescriptors(kind_, descriptor, 1);
  }

  const auto id = point_ids_.Next();
  // Direct construction is required here because MapPoint's constructor is private to Map.
  auto point = std::unique_ptr<MapPoint>(new MapPoint(id, position, kind_, descriptor));
  points_.emplace(id, move(point));
  ++revision_;

  return id;
}

common::KeyFrameId Map::InsertKeyFrame(const Frame& frame, bool retain_images) {
  if (frame.id().epoch() != epoch_ || frame.features().kind() != kind_) {
    throw invalid_argument("Frame epoch/descriptor kind does not match Map");
  }
  for (const auto& entry : keyframes_) {
    if (entry.second->source_frame_id() == frame.id()) {
      throw invalid_argument("Frame already promoted to a KeyFrame");
    }
  }

  set<common::MapPointId> seen;
  for (auto id : frame.associations()) {
    if (!id.valid()) {
      continue;
    }
    if (GetMapPoint(id).is_bad() || !seen.insert(id).second) {
      throw invalid_argument("Copied associations reference a bad or repeated MapPoint");
    }
  }

  const auto id = keyframe_ids_.Next();
  auto keyframe = std::make_unique<KeyFrame>(KeyFrame::FromFrame(id, frame, retain_images));

  // Rebuild copied associations through AddObservation so every graph index is registered.
  std::fill(keyframe->associations_.begin(), keyframe->associations_.end(), common::MapPointId{});
  const auto previous_revision = revision_;
  keyframes_.emplace(id, move(keyframe));

  try {
    for (std::size_t i = 0; i < frame.associations().size(); ++i) {
      if (frame.associations()[i].valid()) {
        AddObservation(id, frame.associations()[i], i);
      }
    }
  } catch (...) {
    // Remove all edges created so far and restore the version, without reusing consumed IDs.
    RemoveKeyFrame(id);
    revision_ = previous_revision;
    throw;
  }

  ++revision_;

  return id;
}

common::ObservationId Map::AddObservation(common::KeyFrameId keyframe_id,
                                          common::MapPointId point_id, std::size_t feature,
                                          double pixel_sigma, double depth_sigma) {
  auto& keyframe = *keyframes_.at(keyframe_id);
  auto& point = *points_.at(point_id);
  const auto& measurement = keyframe.features_.at(feature);

  if (point.is_bad_ || keyframe.associations_.at(feature).valid() ||
      point.observations_.count(keyframe_id) || !isfinite(pixel_sigma) || pixel_sigma <= 0 ||
      !isfinite(depth_sigma) || depth_sigma <= 0) {
    throw invalid_argument("Observation conflicts with existing edge or has invalid noise");
  }

  const auto id = observation_ids_.Next();
  const Observation observation(id, keyframe_id, point_id, feature, measurement.uv,
                                measurement.depth_z_m, pixel_sigma, depth_sigma);
  observations_.emplace(id, observation);

  // Register both reverse indexes before publishing the feature-to-point association.
  try {
    keyframe.observation_ids_.insert(id);
    point.observations_.emplace(keyframe_id, id);
  } catch (...) {
    keyframe.observation_ids_.erase(id);
    point.observations_.erase(keyframe_id);
    observations_.erase(id);
    throw;
  }

  keyframe.associations_[feature] = point_id;
  ++revision_;

  return id;
}

bool Map::RemoveObservation(common::ObservationId id) {
  const auto it = observations_.find(id);
  if (it == observations_.end()) {
    return false;
  }

  const auto& observation = it->second;
  auto& keyframe = *keyframes_.at(observation.keyframe_id());
  auto& point = *points_.at(observation.map_point_id());

  // Unlink both endpoints before erasing the observation that supplies their IDs.
  keyframe.associations_[observation.feature_index()] = {};
  keyframe.observation_ids_.erase(id);
  point.observations_.erase(observation.keyframe_id());
  observations_.erase(it);
  ++revision_;

  return true;
}

bool Map::RemoveMapPoint(common::MapPointId id) {
  const auto it = points_.find(id);
  if (it == points_.end()) {
    return false;
  }

  while (!it->second->observations_.empty()) {
    RemoveObservation(it->second->observations_.begin()->second);
  }

  points_.erase(it);
  ++revision_;

  return true;
}

bool Map::RemoveKeyFrame(common::KeyFrameId id) {
  const auto it = keyframes_.find(id);
  if (it == keyframes_.end()) {
    return false;
  }
  if (protected_keyframes_.count(id)) {
    throw invalid_argument("KeyFrame is protected");
  }

  while (!it->second->observation_ids_.empty()) {
    RemoveObservation(*it->second->observation_ids_.begin());
  }

  keyframes_.erase(it);
  ++revision_;

  return true;
}

void Map::ProtectKeyFrame(common::KeyFrameId id, bool protect) {
  GetKeyFrame(id);

  if (protect) {
    protected_keyframes_.insert(id);
  } else {
    protected_keyframes_.erase(id);
  }
  ++revision_;
}

void Map::MarkMapPointBad(common::MapPointId id, bool bad) {
  points_.at(id)->is_bad_ = bad;
  ++revision_;
}

void Map::MarkObservationOutlier(common::ObservationId id, bool outlier) {
  observations_.at(id).is_outlier_ = outlier;
  ++revision_;
}

MapSnapshot Map::Snapshot() const {
  MapSnapshot snapshot{epoch_, revision_, {}, {}, {}};

  for (const auto& entry : keyframes_) {
    const auto& k = *entry.second;
    snapshot.keyframes.push_back(
        {k.id(), k.source_frame_id(), k.source_sequence(), k.timestamp(), k.pose()});
  }

  for (const auto& entry : points_) {
    snapshot.points.push_back({entry.first, entry.second->position_w(), entry.second->is_bad()});
  }

  for (const auto& entry : observations_) {
    snapshot.observations.push_back(entry.second);
  }

  return snapshot;
}

common::Status Map::CommitGeometry(common::Epoch epoch, common::Revision revision,
                                   const std::vector<PoseUpdate>& poses,
                                   const std::vector<PointUpdate>& points) {
  using common::StatusCode;

  if (epoch != epoch_ || revision != revision_) {
    return {StatusCode::StaleState, "Geometry batch was built from a stale map"};
  }

  // Validate the entire batch before modifying any persistent pose or point.
  set<common::KeyFrameId> pose_ids;
  set<common::MapPointId> point_ids;
  for (const auto& update : poses) {
    if (!keyframes_.count(update.id)) {
      return {StatusCode::NotFound, "Unknown KeyFrame"};
    }
    if (!pose_ids.insert(update.id).second) {
      return {StatusCode::Conflict, "Duplicate pose update"};
    }
  }

  for (const auto& update : points) {
    if (!points_.count(update.id)) {
      return {StatusCode::NotFound, "Unknown MapPoint"};
    }
    if (!point_ids.insert(update.id).second) {
      return {StatusCode::Conflict, "Duplicate point update"};
    }
    if (!update.position_w.allFinite()) {
      return {StatusCode::InvalidArgument, "Nonfinite point update"};
    }
  }

  for (const auto& update : poses) {
    keyframes_.at(update.id)->tcw_ = update.tcw;
  }
  for (const auto& update : points) {
    points_.at(update.id)->position_w_ = update.position_w;
  }
  if (!poses.empty() || !points.empty()) {
    ++revision_;
  }

  return {};
}

bool Map::CheckConsistency() const {
  // Check each observation against its endpoints and both reverse indexes.
  for (const auto& entry : observations_) {
    const auto& o = entry.second;
    const auto k = keyframes_.find(o.keyframe_id());
    const auto p = points_.find(o.map_point_id());
    if (k == keyframes_.end() || p == points_.end() ||
        o.feature_index() >= k->second->associations_.size() ||
        k->second->associations_[o.feature_index()] != o.map_point_id() ||
        !k->second->observation_ids_.count(o.id())) {
      return false;
    }

    const auto inverse = p->second->observations_.find(o.keyframe_id());
    if (inverse == p->second->observations_.end() || inverse->second != o.id()) {
      return false;
    }
  }

  // Check each keyframe association and ensure no observation IDs are orphaned.
  for (const auto& entry : keyframes_) {
    const auto& k = *entry.second;
    if (k.features_.size() != k.associations_.size()) {
      return false;
    }

    std::size_t count = 0;
    for (std::size_t i = 0; i < k.associations_.size(); ++i) {
      if (!k.associations_[i].valid()) {
        continue;
      }
      ++count;

      const auto p = points_.find(k.associations_[i]);
      if (p == points_.end()) {
        return false;
      }
      const auto edge = p->second->observations_.find(k.id());
      if (edge == p->second->observations_.end()) {
        return false;
      }
      const auto o = observations_.find(edge->second);
      if (o == observations_.end() || o->second.feature_index() != i) {
        return false;
      }
    }

    if (count != k.observation_ids_.size()) {
      return false;
    }
    for (auto id : k.observation_ids_) {
      const auto o = observations_.find(id);
      if (o == observations_.end() || o->second.keyframe_id() != k.id()) {
        return false;
      }
    }
  }

  // Check the landmark-to-observation direction independently of the keyframe traversal.
  for (const auto& entry : points_) {
    for (const auto& edge : entry.second->observations_) {
      const auto o = observations_.find(edge.second);
      if (o == observations_.end() || o->second.map_point_id() != entry.first ||
          o->second.keyframe_id() != edge.first) {
        return false;
      }
    }
  }

  return true;
}

void Map::Reset() {
  if (epoch_ == std::numeric_limits<common::Epoch>::max()) {
    throw std::overflow_error("Epoch exhausted");
  }

  observations_.clear();
  points_.clear();
  keyframes_.clear();
  protected_keyframes_.clear();

  ++epoch_;
  revision_ = 0;
  keyframe_ids_.AdvanceEpoch(epoch_);
  point_ids_.AdvanceEpoch(epoch_);
  observation_ids_.AdvanceEpoch(epoch_);
}

}  // namespace vslam::core
