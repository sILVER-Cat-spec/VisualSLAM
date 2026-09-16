#include "vslam/tracking/frame_tracker.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <set>
#include <stdexcept>

namespace vslam::tracking {
using std::isfinite;

// Configure matching and pose refinement with validated tracking thresholds.
FrameTracker::FrameTracker(sensor::CameraModel camera, frontend::MatcherOptions matcher,
                           TrackingOptions options, optimization::SolverOptions solver)
    : camera_(std::move(camera)),
      matcher_(matcher),
      options_(options),
      optimizer_(camera_, solver) {
  if (options.min_correspondences < 6 || options.min_inliers < 6 || options.pnp_iterations < 1 ||
      !isfinite(options.pnp_error) || options.pnp_error <= 0 || !isfinite(options.pnp_confidence) ||
      options.pnp_confidence <= 0 || options.pnp_confidence >= 1 ||
      !isfinite(options.search_radius) || options.search_radius <= 0 || options.image_margin < 0) {
    throw std::invalid_argument("Invalid tracking options");
  }
}

// Refine a pose from map-to-image measurements and retain only accepted associations.
TrackingResult FrameTracker::Optimize(const core::Frame& frame, const core::Map& map,
                                      const geometry::SE3& initial,
                                      const std::vector<Association>& associations) const {
  TrackingResult result;
  if (associations.size() < static_cast<std::size_t>(options_.min_correspondences)) {
    result.message = "Insufficient tracking correspondences";
    return result;
  }

  std::vector<optimization::PoseMeasurement> measurements;
  for (const auto& association : associations) {
    const auto& keypoint = frame.features().at(association.feature);
    measurements.push_back(
        {map.GetMapPoint(association.point).position_w(), keypoint.uv, keypoint.depth_z_m});
  }
  result.optimization = optimizer_.Solve(initial, measurements);
  if (!result.optimization.usable) {
    result.message = result.optimization.message;
    return result;
  }

  for (std::size_t index = 0; index < associations.size(); ++index) {
    if (result.optimization.inliers[index]) {
      result.associations.push_back(associations[index]);
    }
  }
  if (result.associations.size() < static_cast<std::size_t>(options_.min_inliers)) {
    result.associations.clear();
    result.message = "Too few optimized inliers";
    return result;
  }

  result.tcw = result.optimization.tcw;
  result.success = true;
  return result;
}

// Use reference-map matches for a RANSAC pose, then refine its inlier measurements.
TrackingResult FrameTracker::Track(const core::Frame& frame, const core::KeyFrame& reference,
                                   const core::Map& map) const {
  TrackingResult failed;
  const auto matches =
      matcher_.MatchDescriptors(reference.features().descriptors(), frame.features().descriptors());
  failed.matches = matches.size();

  std::vector<Association> associations;
  std::vector<cv::Point3d> world_points;
  std::vector<cv::Point2d> image_points;
  std::set<common::MapPointId> used_points;
  for (const auto& match : matches) {
    const auto point_id = reference.associations().at(match.first);
    if (!point_id.valid() || map.GetMapPoint(point_id).is_bad() ||
        !used_points.insert(point_id).second) {
      continue;
    }
    const auto& position_w = map.GetMapPoint(point_id).position_w();
    const auto& pixel = frame.features().at(match.second).uv;
    world_points.emplace_back(position_w.x(), position_w.y(), position_w.z());
    image_points.emplace_back(pixel.x(), pixel.y());
    associations.push_back({match.second, point_id});
  }
  if (associations.size() < static_cast<std::size_t>(options_.min_correspondences)) {
    failed.message = "Too few reference-map matches";
    return failed;
  }

  cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << camera_.fx(), 0, camera_.cx(), 0, camera_.fy(),
                        camera_.cy(), 0, 0, 1);
  cv::Mat rotation_vector;
  cv::Mat translation_vector;
  cv::Mat inlier_indices;
  const bool pnp_succeeded = cv::solvePnPRansac(
      world_points, image_points, intrinsics, cv::noArray(), rotation_vector, translation_vector,
      false, options_.pnp_iterations, static_cast<float>(options_.pnp_error),
      options_.pnp_confidence, inlier_indices, cv::SOLVEPNP_EPNP);
  if (!pnp_succeeded ||
      inlier_indices.total() < static_cast<std::size_t>(options_.min_correspondences)) {
    failed.message = "PnP RANSAC failed";
    return failed;
  }

  // OpenCV's PnP output transforms world points into the current camera frame.
  cv::Mat rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix);
  Eigen::Matrix3d rotation_cw;
  Eigen::Vector3d translation_cw;
  for (int row = 0; row < 3; ++row) {
    translation_cw[row] = translation_vector.at<double>(row);
    for (int column = 0; column < 3; ++column) {
      rotation_cw(row, column) = rotation_matrix.at<double>(row, column);
    }
  }

  std::vector<Association> selected_associations;
  for (int row = 0; row < inlier_indices.rows; ++row) {
    const auto association_index = static_cast<std::size_t>(inlier_indices.at<int>(row));
    selected_associations.push_back(associations.at(association_index));
  }
  auto result =
      Optimize(frame, map, geometry::SE3(rotation_cw, translation_cw), selected_associations);
  result.matches = matches.size();
  result.pnp_inliers = inlier_indices.total();
  return result;
}

// Add unused local-map matches near predicted pixels before refining the seed pose.
TrackingResult FrameTracker::RefineLocal(const core::Frame& frame, const core::Map& map,
                                         const std::vector<common::MapPointId>& local_points,
                                         const TrackingResult& seed) const {
  if (!seed.success || !seed.tcw) {
    return seed;
  }

  std::set<common::MapPointId> used_points;
  std::set<std::size_t> used_features;
  for (const auto& association : seed.associations) {
    used_points.insert(association.point);
    used_features.insert(association.feature);
  }

  // Descriptor rows, point IDs, and projections share the same candidate index.
  cv::Mat descriptors;
  std::vector<common::MapPointId> visible_points;
  std::vector<Eigen::Vector2d> projections;
  for (auto point_id : local_points) {
    const auto& point = map.GetMapPoint(point_id);
    if (point.is_bad() || used_points.count(point_id)) {
      continue;
    }
    const auto pixel = camera_.Project(*seed.tcw * point.position_w());
    if (!pixel || pixel->x() < options_.image_margin || pixel->y() < options_.image_margin ||
        pixel->x() >= camera_.width() - options_.image_margin ||
        pixel->y() >= camera_.height() - options_.image_margin) {
      continue;
    }
    auto descriptor = point.descriptor();
    if (descriptor.empty()) {
      continue;
    }
    visible_points.push_back(point_id);
    projections.push_back(*pixel);
    descriptors.push_back(descriptor);
  }

  auto associations = seed.associations;
  for (const auto& match : matcher_.MatchDescriptors(descriptors, frame.features().descriptors())) {
    if (used_features.count(match.second) ||
        (projections[match.first] - frame.features().at(match.second).uv).norm() >
            options_.search_radius) {
      continue;
    }
    associations.push_back({match.second, visible_points[match.first]});
    used_features.insert(match.second);
  }

  auto result = Optimize(frame, map, *seed.tcw, associations);
  result.matches = seed.matches;
  result.pnp_inliers = seed.pnp_inliers;
  result.visible_points = visible_points.size();
  return result;
}
}  // namespace vslam::tracking
