#pragma once
#include "vslam/core/map.h"
#include "vslam/frontend/feature_matcher.h"
#include "vslam/optimization/local_bundle_adjuster.h"
namespace vslam::mapping {
struct MappingOptions {
  int min_gap=2,max_gap=5,min_inliers=30,max_neighbors=5,max_keyframes=8;
  int max_ba_points=300,max_boundary_keyframes=32,max_ba_observations=4000;
  int max_new_points=400,point_grace_period=3;
  double min_translation=0.05,min_rotation_deg=5,min_parallax_deg=1,max_reprojection_error=3;
};
std::vector<common::KeyFrameId> SelectNeighbors(const core::Map& map,common::KeyFrameId seed,int limit);
std::vector<common::MapPointId> CollectPoints(const core::Map& map,const std::vector<common::KeyFrameId>& keyframes);
bool ShouldInsert(const core::Frame& frame,const core::KeyFrame& last,std::size_t inliers,const MappingOptions& options);
struct MappingResult {
  common::KeyFrameId keyframe;
  std::size_t created_points=0,removed_points=0;
  bool ba_committed=false;
  optimization::BASolution ba;
  // Wall time of the detached BA solve, excluding geometry commit and point cleanup.
  double ba_seconds = 0;
};
class LocalMapper {
 public:
  LocalMapper(sensor::CameraModel camera,frontend::MatcherOptions matcher,MappingOptions options,
              optimization::SolverOptions solver);
  MappingResult Process(core::Frame& frame,core::Map& map);
  void Reset();
 private:
  bool ValidProjection(const core::KeyFrame& keyframe,std::size_t index,const Eigen::Vector3d& point) const;
  std::vector<common::MapPointId> Extend(core::Map& map,common::KeyFrameId current,
                                      const std::vector<common::KeyFrameId>& neighbors);
  optimization::BAProblem BuildProblem(const core::Map& map,common::KeyFrameId current) const;
  sensor::CameraModel camera_;
  frontend::FeatureMatcher matcher_;
  MappingOptions options_;
  optimization::SolverOptions solver_options_;
  optimization::LocalBundleAdjuster ba_;
  std::uint64_t insertions_=0;
  std::map<common::MapPointId,std::uint64_t> births_;
};
}
