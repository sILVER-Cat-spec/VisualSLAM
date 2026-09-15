#pragma once
#include "vslam/core/map.h"
#include "vslam/frontend/feature_matcher.h"
#include "vslam/optimization/pose_optimizer.h"
namespace vslam::tracking {
struct TrackingOptions {
  int min_correspondences=6, min_inliers=15, pnp_iterations=100;
  double pnp_error=4.0, pnp_confidence=0.99, search_radius=20.0;
  int image_margin=8;
};
struct Association { std::size_t feature; common::MapPointId point; };
struct TrackingResult {
  bool success=false;
  std::optional<geometry::SE3> tcw;
  std::vector<Association> associations;
  std::size_t matches=0, pnp_inliers=0, visible_points=0;
  optimization::PoseSolution optimization;
  std::string message;
};
class FrameTracker {
 public:
  FrameTracker(sensor::CameraModel camera, frontend::MatcherOptions matcher,
               TrackingOptions options, optimization::SolverOptions solver);
  TrackingResult Track(const core::Frame& frame,const core::KeyFrame& reference,const core::Map& map) const;
  TrackingResult RefineLocal(const core::Frame& frame,const core::Map& map,
                            const std::vector<common::MapPointId>& local_points,
                            const TrackingResult& seed) const;
 private:
  TrackingResult Optimize(const core::Frame& frame,const core::Map& map,
                          const geometry::SE3& initial,const std::vector<Association>& associations) const;
  sensor::CameraModel camera_;
  frontend::FeatureMatcher matcher_;
  TrackingOptions options_;
  optimization::PoseOptimizer optimizer_;
};
}
