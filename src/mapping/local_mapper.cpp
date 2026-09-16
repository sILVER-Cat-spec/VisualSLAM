#include "vslam/mapping/local_mapper.h"

#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>

namespace vslam::mapping {
using std::isfinite;

// Rank covisible keyframes by shared observations, breaking ties by newer ID.
std::vector<common::KeyFrameId> SelectNeighbors(const core::Map& map, common::KeyFrameId seed,
                                                int limit) {
  std::map<common::KeyFrameId, int> weights;
  for (auto point_id : map.GetKeyFrame(seed).associations()) {
    if (point_id.valid() && !map.GetMapPoint(point_id).is_bad()) {
      for (const auto& observation_entry : map.GetMapPoint(point_id).observations()) {
        if (observation_entry.first != seed &&
            !map.GetObservation(observation_entry.second).is_outlier()) {
          ++weights[observation_entry.first];
        }
      }
    }
  }

  // Include zero-weight keyframes as fallback neighbors.
  std::vector<std::pair<common::KeyFrameId, int>> candidates;
  for (const auto& keyframe : map.Snapshot().keyframes) {
    if (keyframe.id != seed) {
      candidates.push_back({keyframe.id, weights[keyframe.id]});
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto& left, const auto& right) {
    if (left.second != right.second) {
      return left.second > right.second;
    }
    return right.first < left.first;
  });

  std::vector<common::KeyFrameId> result;
  for (const auto& candidate : candidates) {
    if (static_cast<int>(result.size()) < limit) {
      result.push_back(candidate.first);
    }
  }
  return result;
}

// Deduplicate usable map points across the requested keyframes.
std::vector<common::MapPointId> CollectPoints(const core::Map& map,
                                              const std::vector<common::KeyFrameId>& keyframes) {
  std::set<common::MapPointId> point_ids;
  for (auto keyframe_id : keyframes) {
    for (auto point_id : map.GetKeyFrame(keyframe_id).associations()) {
      if (point_id.valid() && !map.GetMapPoint(point_id).is_bad()) {
        point_ids.insert(point_id);
      }
    }
  }
  return {point_ids.begin(), point_ids.end()};
}

// Apply sequence spacing before testing motion and tracking support.
bool ShouldInsert(const core::Frame& frame, const core::KeyFrame& last, std::size_t inliers,
                  const MappingOptions& options) {
  if (!frame.pose() || frame.sequence() < last.source_sequence()) {
    return false;
  }
  const auto sequence_gap = frame.sequence() - last.source_sequence();
  if (sequence_gap < static_cast<std::uint64_t>(options.min_gap)) {
    return false;
  }
  if (sequence_gap >= static_cast<std::uint64_t>(options.max_gap)) {
    return true;
  }

  const double translation_m =
      (frame.pose()->Inverse().translation() - last.pose().Inverse().translation()).norm();
  const double rotation_cosine = std::clamp(
      ((frame.pose()->rotation() * last.pose().rotation().transpose()).trace() - 1) * 0.5, -1.0,
      1.0);
  const double rotation_deg = std::acos(rotation_cosine) * 180 / 3.141592653589793;
  return translation_m >= options.min_translation || rotation_deg >= options.min_rotation_deg ||
         inliers < static_cast<std::size_t>(options.min_inliers);
}

// Validate mapping limits before constructing any local-map work.
LocalMapper::LocalMapper(sensor::CameraModel camera, frontend::MatcherOptions matcher,
                         MappingOptions options, optimization::SolverOptions solver)
    : camera_(std::move(camera)),
      matcher_(matcher),
      options_(options),
      solver_options_(solver),
      ba_(camera_, solver) {
  if (options.min_gap < 1 || options.max_gap < options.min_gap || options.min_inliers < 1 ||
      options.max_neighbors < 1 || options.max_keyframes < 2 || options.max_ba_points < 6 ||
      options.max_boundary_keyframes < 1 || options.max_ba_observations < 12 ||
      options.max_new_points < 1 || options.point_grace_period < 0 ||
      !isfinite(options.min_translation) || options.min_translation < 0 ||
      !isfinite(options.min_rotation_deg) || options.min_rotation_deg < 0 ||
      !isfinite(options.min_parallax_deg) || options.min_parallax_deg <= 0 ||
      !isfinite(options.max_reprojection_error) || options.max_reprojection_error <= 0) {
    throw std::invalid_argument("Invalid mapping options");
  }
}

// Gate a world-space point against the feature's pixel and optional metric depth.
bool LocalMapper::ValidProjection(const core::KeyFrame& keyframe, std::size_t feature_index,
                                  const Eigen::Vector3d& position_w) const {
  const auto position_c = keyframe.pose() * position_w;
  const auto pixel = camera_.Project(position_c);
  const auto& measurement = keyframe.features().at(feature_index);
  return pixel && (*pixel - measurement.uv).norm() <= options_.max_reprojection_error &&
         (!measurement.depth_z_m ||
          std::abs(position_c.z() - *measurement.depth_z_m) <=
              solver_options_.outlier_depth_sigma * solver_options_.depth_sigma_m);
}

// Extend observations first, then create points from depth or two-view triangulation.
std::vector<common::MapPointId> LocalMapper::Extend(
    core::Map& map, common::KeyFrameId current, const std::vector<common::KeyFrameId>& neighbors) {
  const auto& current_keyframe = map.GetKeyFrame(current);
  std::vector<common::MapPointId> created;

  // Only attach an existing point if this keyframe has no observation of it yet.
  auto add_observation = [&](common::KeyFrameId keyframe_id, std::size_t feature_index,
                             common::MapPointId point_id) {
    const auto& point = map.GetMapPoint(point_id);
    if (!point.is_bad() && !point.observations().count(keyframe_id) &&
        ValidProjection(map.GetKeyFrame(keyframe_id), feature_index, point.position_w())) {
      map.AddObservation(keyframe_id, point_id, feature_index, solver_options_.pixel_sigma,
                         solver_options_.depth_sigma_m);
    }
  };

  // Convert camera-axis depth into world coordinates using the observing pose.
  auto point_from_depth = [&](const core::KeyFrame& keyframe,
                              std::size_t feature_index) -> std::optional<Eigen::Vector3d> {
    const auto& feature = keyframe.features().at(feature_index);
    if (!feature.depth_z_m) {
      return {};
    }
    const auto position_c = camera_.Unproject(feature.uv, *feature.depth_z_m);
    if (!position_c) {
      return std::nullopt;
    }
    return Eigen::Vector3d(keyframe.pose().Inverse() * *position_c);
  };

  // Roll back the new point and its observations if insertion cannot finish.
  auto create_point = [&](const Eigen::Vector3d& position_w, std::size_t feature_index,
                          std::optional<std::pair<common::KeyFrameId, std::size_t>> other) {
    const auto point_id =
        map.AddMapPoint(position_w, current_keyframe.features().Descriptor(feature_index));
    try {
      map.AddObservation(current, point_id, feature_index, solver_options_.pixel_sigma,
                         solver_options_.depth_sigma_m);
      if (other) {
        map.AddObservation(other->first, point_id, other->second, solver_options_.pixel_sigma,
                           solver_options_.depth_sigma_m);
      }
      created.push_back(point_id);
    } catch (...) {
      map.RemoveMapPoint(point_id);
      throw;
    }
  };

  for (auto neighbor_id : neighbors) {
    const auto& neighbor = map.GetKeyFrame(neighbor_id);
    const auto matches = matcher_.MatchDescriptors(neighbor.features().descriptors(),
                                                   current_keyframe.features().descriptors());
    for (const auto& match : matches) {
      const auto neighbor_point_id = neighbor.associations()[match.first];
      const auto current_point_id = current_keyframe.associations()[match.second];
      if (neighbor_point_id.valid() && current_point_id.valid()) {
        // Fusion is a separate future policy.
        continue;
      }
      if (neighbor_point_id.valid()) {
        add_observation(current, match.second, neighbor_point_id);
        continue;
      }
      if (current_point_id.valid()) {
        add_observation(neighbor_id, match.first, current_point_id);
        continue;
      }
      if (created.size() >= static_cast<std::size_t>(options_.max_new_points)) {
        continue;
      }

      auto position_w = point_from_depth(current_keyframe, match.second);
      if (!position_w) {
        position_w = point_from_depth(neighbor, match.first);
      }
      if (!position_w) {
        // Solve the homogeneous two-view system using normalized image coordinates.
        const auto neighbor_ray = camera_.BackProject(neighbor.features().at(match.first).uv);
        const auto current_ray =
            camera_.BackProject(current_keyframe.features().at(match.second).uv);
        const Eigen::Matrix<double, 3, 4> neighbor_projection =
            neighbor.pose().Matrix().topRows<3>();
        const Eigen::Matrix<double, 3, 4> current_projection =
            current_keyframe.pose().Matrix().topRows<3>();
        Eigen::Matrix4d triangulation_system;
        triangulation_system.row(0) =
            neighbor_ray->x() * neighbor_projection.row(2) - neighbor_projection.row(0);
        triangulation_system.row(1) =
            neighbor_ray->y() * neighbor_projection.row(2) - neighbor_projection.row(1);
        triangulation_system.row(2) =
            current_ray->x() * current_projection.row(2) - current_projection.row(0);
        triangulation_system.row(3) =
            current_ray->y() * current_projection.row(2) - current_projection.row(1);
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(triangulation_system, Eigen::ComputeFullV);
        const Eigen::Vector4d homogeneous_point = svd.matrixV().col(3);
        if (!homogeneous_point.allFinite() || std::abs(homogeneous_point.w()) < 1e-12) {
          continue;
        }

        // Small parallax makes triangulated depth unreliable even with a finite solution.
        const Eigen::Vector3d triangulated_position =
            homogeneous_point.head<3>() / homogeneous_point.w();
        const Eigen::Vector3d neighbor_direction =
            triangulated_position - neighbor.pose().Inverse().translation();
        const Eigen::Vector3d current_direction =
            triangulated_position - current_keyframe.pose().Inverse().translation();
        const double direction_norm_product = neighbor_direction.norm() * current_direction.norm();
        if (direction_norm_product < 1e-12 ||
            std::acos(std::clamp(neighbor_direction.dot(current_direction) / direction_norm_product,
                                 -1.0, 1.0)) *
                    180 / 3.141592653589793 <
                options_.min_parallax_deg) {
          continue;
        }
        position_w = triangulated_position;
      }

      if (position_w && ValidProjection(current_keyframe, match.second, *position_w) &&
          ValidProjection(neighbor, match.first, *position_w)) {
        create_point(*position_w, match.second, std::make_pair(neighbor_id, match.first));
      }
    }
  }

  // Unmatched features with valid depth can seed points with a single observation.
  for (std::size_t feature_index = 0;
       feature_index < current_keyframe.features().size() &&
       created.size() < static_cast<std::size_t>(options_.max_new_points);
       ++feature_index) {
    if (!current_keyframe.associations()[feature_index].valid()) {
      const auto position_w = point_from_depth(current_keyframe, feature_index);
      if (position_w && ValidProjection(current_keyframe, feature_index, *position_w)) {
        create_point(*position_w, feature_index, {});
      }
    }
  }
  return created;
}

// Build a bounded BA snapshot with an anchored local pose and fixed boundary poses.
optimization::BAProblem LocalMapper::BuildProblem(const core::Map& map,
                                                  common::KeyFrameId current) const {
  optimization::BAProblem problem;
  problem.epoch = map.epoch();
  problem.revision = map.revision();

  auto neighbors = SelectNeighbors(map, current, options_.max_keyframes - 1);
  neighbors.push_back(current);
  std::set<common::KeyFrameId> local_keyframes(neighbors.begin(), neighbors.end());
  auto points = CollectPoints(map, neighbors);
  std::sort(points.begin(), points.end(), [&](auto left, auto right) {
    const auto left_observations = map.GetMapPoint(left).observations().size();
    const auto right_observations = map.GetMapPoint(right).observations().size();
    if (left_observations != right_observations) {
      return left_observations > right_observations;
    }
    return left < right;
  });

  std::map<common::KeyFrameId, std::size_t> pose_indices;
  const auto anchor = *local_keyframes.begin();
  for (auto point_id : points) {
    if (problem.points.size() >= static_cast<std::size_t>(options_.max_ba_points)) {
      break;
    }
    const auto& point = map.GetMapPoint(point_id);
    std::vector<common::ObservationId> observations;
    for (const auto& observation_entry : point.observations()) {
      const auto& observation = map.GetObservation(observation_entry.second);
      if (!observation.is_outlier() &&
          ValidProjection(map.GetKeyFrame(observation_entry.first), observation.feature_index(),
                          point.position_w())) {
        observations.push_back(observation_entry.second);
      }
    }
    if (observations.size() < 2) {
      continue;
    }
    if (problem.measurements.size() + observations.size() >
        static_cast<std::size_t>(options_.max_ba_observations)) {
      break;
    }

    const auto point_index = problem.points.size();
    problem.points.push_back({point_id, point.position_w()});
    for (auto observation_id : observations) {
      const auto& observation = map.GetObservation(observation_id);
      const auto keyframe_id = observation.keyframe_id();
      if (!pose_indices.count(keyframe_id)) {
        pose_indices[keyframe_id] = problem.poses.size();
        problem.poses.push_back({keyframe_id, map.GetKeyFrame(keyframe_id).pose(),
                                 !local_keyframes.count(keyframe_id) || keyframe_id == anchor});
      }
      problem.measurements.push_back({observation_id, pose_indices.at(keyframe_id), point_index,
                                      observation.uv(), observation.depth_z_m()});
    }
  }

  int boundary_count = 0;
  bool has_free_current_pose = false;
  for (const auto& pose : problem.poses) {
    boundary_count += !local_keyframes.count(pose.id);
    has_free_current_pose |= pose.id == current && !pose.fixed;
  }
  if (boundary_count > options_.max_boundary_keyframes || !has_free_current_pose) {
    problem.poses.clear();
    problem.points.clear();
    problem.measurements.clear();
  }
  return problem;
}

// Commit accepted BA geometry before pruning points and synchronizing the input frame.
MappingResult LocalMapper::Process(core::Frame& frame, core::Map& map) {
  MappingResult result;
  result.keyframe = map.InsertKeyFrame(frame);
  ++insertions_;
  const auto neighbors = SelectNeighbors(map, result.keyframe, options_.max_neighbors);
  const auto created = Extend(map, result.keyframe, neighbors);
  result.created_points = created.size();
  for (auto point_id : created) {
    births_[point_id] = insertions_;
  }

  const auto ba_start = std::chrono::steady_clock::now();
  result.ba = ba_.Solve(BuildProblem(map, result.keyframe));
  result.ba_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - ba_start).count();
  if (result.ba.usable) {
    std::vector<core::PoseUpdate> poses;
    std::vector<core::PointUpdate> points;
    for (const auto& pose : result.ba.geometry.poses) {
      if (!pose.fixed) {
        poses.push_back({pose.id, pose.tcw});
      }
    }
    for (const auto& point : result.ba.geometry.points) {
      points.push_back({point.id, point.position_w});
    }
    result.ba_committed =
        map.CommitGeometry(result.ba.geometry.epoch, result.ba.geometry.revision, poses, points)
            .ok();

    if (result.ba_committed) {
      for (auto observation_id : result.ba.outliers) {
        map.RemoveObservation(observation_id);
      }
      for (const auto& snapshot_point : map.Snapshot().points) {
        const auto& point = map.GetMapPoint(snapshot_point.id);
        const auto birth_entry = births_.find(snapshot_point.id);
        const auto birth_insertion = birth_entry == births_.end() ? 0 : birth_entry->second;
        if (point.is_bad() || point.observations().empty() ||
            (insertions_ - birth_insertion >=
                 static_cast<std::uint64_t>(options_.point_grace_period) &&
             point.observations().size() < 2)) {
          map.RemoveMapPoint(snapshot_point.id);
          births_.erase(snapshot_point.id);
          ++result.removed_points;
        }
      }
    }
  }

  const auto& keyframe = map.GetKeyFrame(result.keyframe);
  frame.SetPose(keyframe.pose());
  for (std::size_t feature_index = 0; feature_index < frame.features().size(); ++feature_index) {
    frame.RemoveAssociation(feature_index);
    if (keyframe.associations()[feature_index].valid()) {
      frame.Associate(feature_index, keyframe.associations()[feature_index]);
    }
  }
  return result;
}

// Forget point ages when starting a new mapping session.
void LocalMapper::Reset() {
  insertions_ = 0;
  births_.clear();
}
}  // namespace vslam::mapping
