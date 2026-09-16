#pragma once

#include "vslam/core/map.h"
#include "vslam/frontend/feature_matcher.h"
#include "vslam/optimization/pose_optimizer.h"

namespace vslam::tracking {
struct TrackingOptions {
  int min_correspondences = 6;
  int min_inliers = 15;
  int pnp_iterations = 100;
  double pnp_error = 4.0;  // RANSAC reprojection threshold in pixels.
  double pnp_confidence = 0.99;
  double search_radius = 20.0;  // Local-match radius in pixels.
  int image_margin = 8;         // Excluded image-border width in pixels.
};

struct Association {
  std::size_t feature;
  common::MapPointId point;
};

struct TrackingResult {
  bool success = false;
  std::optional<geometry::SE3> tcw;
  std::vector<Association> associations;
  std::size_t matches = 0;
  std::size_t pnp_inliers = 0;
  std::size_t visible_points = 0;
  optimization::PoseSolution optimization;
  std::string message;
};

class FrameTracker {
 public:
  // Store camera and solver settings; invalid tracking thresholds throw std::invalid_argument.
  FrameTracker(sensor::CameraModel camera, frontend::MatcherOptions matcher,
               TrackingOptions options, optimization::SolverOptions solver);

  // Estimate world-to-camera tcw from a reference keyframe belonging to map.
  // Inputs are unchanged; insufficient matches or an unusable pose return success=false.
  TrackingResult Track(const core::Frame& frame, const core::KeyFrame& reference,
                       const core::Map& map) const;

  // Refine a successful seed using valid local point IDs from map and pixel-distance gating.
  // An unsuccessful seed is returned unchanged; refinement failure returns success=false.
  TrackingResult RefineLocal(const core::Frame& frame, const core::Map& map,
                             const std::vector<common::MapPointId>& local_points,
                             const TrackingResult& seed) const;

 private:
  TrackingResult Optimize(const core::Frame& frame, const core::Map& map,
                          const geometry::SE3& initial,
                          const std::vector<Association>& associations) const;

  sensor::CameraModel camera_;
  frontend::FeatureMatcher matcher_;
  TrackingOptions options_;
  optimization::PoseOptimizer optimizer_;
};
}  // namespace vslam::tracking
