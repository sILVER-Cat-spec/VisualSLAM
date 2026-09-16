#pragma once

#include "vslam/core/map.h"
#include "vslam/frontend/feature_matcher.h"
#include "vslam/optimization/local_bundle_adjuster.h"

namespace vslam::mapping {
struct MappingOptions {
  int min_gap = 2;
  int max_gap = 5;
  int min_inliers = 30;
  int max_neighbors = 5;
  int max_keyframes = 8;
  int max_ba_points = 300;
  int max_boundary_keyframes = 32;
  int max_ba_observations = 4000;
  int max_new_points = 400;
  int point_grace_period = 3;
  double min_translation = 0.05;
  double min_rotation_deg = 5;
  double min_parallax_deg = 1;
  double max_reprojection_error = 3;
};

// Return up to limit neighbors of a valid seed, ranked by shared inlier observations.
// Ties prefer larger IDs; zero-overlap keyframes may fill the remaining slots.
std::vector<common::KeyFrameId> SelectNeighbors(const core::Map& map, common::KeyFrameId seed,
                                                int limit);

// Return unique, non-bad associated points in ID order; keyframes must belong to map.
std::vector<common::MapPointId> CollectPoints(const core::Map& map,
                                              const std::vector<common::KeyFrameId>& keyframes);

// Test sequence gap, camera translation in meters, rotation in degrees, and inlier count.
// Returns false for a frame without a pose or with a sequence older than last.
bool ShouldInsert(const core::Frame& frame, const core::KeyFrame& last, std::size_t inliers,
                  const MappingOptions& options);

struct MappingResult {
  common::KeyFrameId keyframe;
  std::size_t created_points = 0;
  std::size_t removed_points = 0;
  bool ba_committed = false;
  optimization::BASolution ba;

  // Wall time of BA problem construction and solve, excluding commit and point cleanup.
  double ba_seconds = 0;
};

class LocalMapper {
 public:
  // Store camera and solver settings; invalid mapping thresholds throw std::invalid_argument.
  LocalMapper(sensor::CameraModel camera, frontend::MatcherOptions matcher, MappingOptions options,
              optimization::SolverOptions solver);

  // Insert a posed frame into its matching map, extend points, and attempt local BA.
  // Mutates map and synchronizes frame pose/associations; insertion remains if BA is rejected.
  MappingResult Process(core::Frame& frame, core::Map& map);

  // Clear insertion counts and point ages without modifying any map.
  void Reset();

 private:
  bool ValidProjection(const core::KeyFrame& keyframe, std::size_t index,
                       const Eigen::Vector3d& point) const;
  std::vector<common::MapPointId> Extend(core::Map& map, common::KeyFrameId current,
                                         const std::vector<common::KeyFrameId>& neighbors);
  optimization::BAProblem BuildProblem(const core::Map& map, common::KeyFrameId current) const;

  sensor::CameraModel camera_;
  frontend::FeatureMatcher matcher_;
  MappingOptions options_;
  optimization::SolverOptions solver_options_;
  optimization::LocalBundleAdjuster ba_;
  std::uint64_t insertions_ = 0;
  std::map<common::MapPointId, std::uint64_t> births_;
};
}  // namespace vslam::mapping
