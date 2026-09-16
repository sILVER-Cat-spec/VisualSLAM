#include "vslam/initialization/rgbd_initializer.h"

namespace vslam::initialization {
// Stage the first map so insufficient depth support leaves the input frame untouched.
InitializationResult InitializeRGBD(core::Frame& frame, const sensor::CameraModel& camera,
                                    std::size_t min_points, double min_depth, double max_depth) {
  InitializationResult result;
  std::vector<std::pair<std::size_t, Eigen::Vector3d>> candidates;
  for (std::size_t feature_index = 0; feature_index < frame.features().size(); ++feature_index) {
    const auto& keypoint = frame.features().at(feature_index);
    if (!keypoint.depth_z_m || *keypoint.depth_z_m < min_depth || *keypoint.depth_z_m > max_depth) {
      continue;
    }
    const auto position_c = camera.Unproject(keypoint.uv, *keypoint.depth_z_m);
    if (position_c) {
      candidates.push_back({feature_index, *position_c});
    }
  }
  if (candidates.size() < min_points) {
    return result;
  }

  // The first camera defines the world frame, so camera and world coordinates coincide.
  core::Frame staged_frame(frame);
  staged_frame.SetPose(geometry::SE3());
  auto map = std::make_unique<core::Map>(frame.features().kind(), frame.id().epoch());
  for (const auto& candidate : candidates) {
    const std::size_t feature_index = candidate.first;
    const auto point_id =
        map->AddMapPoint(candidate.second, frame.features().Descriptor(feature_index));
    staged_frame.Associate(feature_index, point_id);
  }

  result.keyframe = map->InsertKeyFrame(staged_frame);
  map->ProtectKeyFrame(result.keyframe);
  result.points = candidates.size();
  result.map = std::move(map);
  frame = std::move(staged_frame);
  return result;
}
}  // namespace vslam::initialization
