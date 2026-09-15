#include "vslam/system/slam_system.h"

#include "vslam/initialization/rgbd_initializer.h"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace vslam::system {
namespace {
// Measure diagnostic wall time without changing the pipeline state or algorithm decisions.
double SecondsSince(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
}


const char* StateName(SlamState state) {
  switch (state) {
    case SlamState::Initializing:
      return "initializing";
    case SlamState::Tracking:
      return "tracking";
    case SlamState::Lost:
      return "lost";
  }
  return "unknown";
}

SlamSystem::SlamSystem(sensor::CameraModel camera, SlamConfig config)
    : camera_(std::move(camera)),
      config_(config),
      extractor_(config.frontend),
      tracker_(camera_, config.matcher, config.tracking, config.pose_solver),
      mapper_(camera_, config.matcher, config.mapping, config.ba_solver),
      map_(std::make_unique<core::Map>(config.frontend.kind)),
      frames_(std::make_unique<core::FrameFactory>(map_->epoch(), config.time_base)) {
  if (config.min_initial_points < 6) {
    throw std::invalid_argument("Initialization requires at least six depth points");
  }
}

SlamResult SlamSystem::Process(const sensor::FrameInput& input) {
  return ProcessImpl(input, nullptr);
}

SlamResult SlamSystem::ProcessFeatures(const sensor::FrameInput& input,
                                      const core::FeatureSet& features) {
  return ProcessImpl(input, &features);
}

// Apply the same checks to extracted and supplied features before allocating
// a frame and advancing the frame factory's ordering state.
void SlamSystem::ValidateFeatures(const core::FeatureSet& features) const {
  if (features.kind() != config_.frontend.kind) {
    throw std::invalid_argument("Feature kind does not match system");
  }

  for (const auto& keypoint : features.keypoints()) {
    const bool outside_image = !camera_.Contains(keypoint.uv);
    const bool outside_depth_interval =
        keypoint.depth_z_m &&
        (*keypoint.depth_z_m < config_.frontend.min_depth ||
         *keypoint.depth_z_m > config_.frontend.max_depth);

    if (outside_image || outside_depth_interval) {
      throw std::invalid_argument("Feature outside configured grid/depth interval");
    }
  }
}

// Adopt a staged RGB-D map only on success. Insufficient depth support leaves
// the system waiting for another frame.
void SlamSystem::InitializeFrame(core::Frame& frame, SlamResult& result) {
  auto initialized = initialization::InitializeRGBD(
      frame, camera_, config_.min_initial_points,
      config_.frontend.min_depth, config_.frontend.max_depth);
  if (!initialized.map) {
    return;
  }

  map_ = std::move(initialized.map);
  last_keyframe_ = initialized.keyframe;
  result.inserted_keyframe = initialized.keyframe;
  state_ = SlamState::Tracking;

  result.tcw = frame.pose();
  result.inliers = initialized.points;
}

// Requires last_keyframe_ and validated frame features. Retry the reference
// even after loss; successful tracking may then extend the map.
void SlamSystem::TrackFrame(core::Frame& frame, SlamResult& result) {
  const auto tracking_start = std::chrono::steady_clock::now();
  auto tracked = tracker_.Track(
      frame, map_->GetKeyFrame(*last_keyframe_), *map_);

  // Refinement replaces the seed result, including on refinement failure.
  if (tracked.success) {
    auto local_keyframes = mapping::SelectNeighbors(
        *map_, *last_keyframe_, config_.mapping.max_neighbors);
    local_keyframes.push_back(*last_keyframe_);
    const auto local_points = mapping::CollectPoints(*map_, local_keyframes);
    tracked = tracker_.RefineLocal(frame, *map_, local_points, tracked);
  }

  result.tracking = tracked;
  result.tracking_seconds = SecondsSince(tracking_start);
  if (!tracked.success) {
    state_ = SlamState::Lost;
    frame.ClearPose();
    return;
  }

  frame.SetPose(*tracked.tcw);
  for (const auto& association : tracked.associations) {
    frame.Associate(association.feature, association.point);
  }
  result.inliers = tracked.associations.size();
  state_ = SlamState::Tracking;

  const bool should_insert_keyframe = mapping::ShouldInsert(
      frame, map_->GetKeyFrame(*last_keyframe_), result.inliers, config_.mapping);
  if (should_insert_keyframe) {
    const auto mapping_start = std::chrono::steady_clock::now();
    result.mapping = mapper_.Process(frame, *map_);
    result.mapping_seconds = SecondsSince(mapping_start);
    result.ba_seconds = result.mapping->ba_seconds;
    last_keyframe_ = result.mapping->keyframe;
    result.inserted_keyframe = last_keyframe_;
    result.ba_committed = result.mapping->ba_committed;
  }

  // Mapping can update the frame pose after bundle adjustment; publish that pose.
  result.tcw = frame.pose();
}

// Share preparation, error handling, and statistics between both public entry
// points. A null feature pointer selects normal feature extraction.
SlamResult SlamSystem::ProcessImpl(
    const sensor::FrameInput& input, const core::FeatureSet* supplied_features) {
  using std::chrono::steady_clock;

  const auto start = steady_clock::now();
  SlamResult result;
  result.state = state_;

  try {
    const auto prepared = sensor::Prepare(input, camera_);
    result.prepare_seconds = SecondsSince(start);
    const auto frontend_start = steady_clock::now();
    core::FeatureSet features = supplied_features
        ? *supplied_features
        : extractor_.Extract(prepared.image_rgb_or_gray, prepared.depth_z_m);
    ValidateFeatures(features);

    result.frontend_seconds = SecondsSince(frontend_start);
    result.frame = frames_->Create(
        prepared.timestamp, prepared.image_rgb_or_gray,
        prepared.depth_z_m, prepared.external_label);
    auto& frame = *result.frame;
    frame.SetFeatures(features);
    result.features = features.size();

    if (!last_keyframe_) {
      const auto initialization_start = steady_clock::now();
      InitializeFrame(frame, result);
      result.initialization_seconds = SecondsSince(initialization_start);
    } else {
      TrackFrame(frame, result);
    }
  } catch (const cv::Exception& error) {
    result.status = {common::StatusCode::NumericalFailure, error.what()};
    result.tcw.reset();
    if (result.frame) {
      result.frame->ClearPose();
    }

    // A numerical failure after initialization marks tracking as lost.
    if (last_keyframe_) {
      state_ = SlamState::Lost;
    }
  } catch (const std::invalid_argument& error) {
    // Reject this output pose without forcing a tracking state transition.
    result.status = {common::StatusCode::InvalidArgument, error.what()};
    result.tcw.reset();
    if (result.frame) {
      result.frame->ClearPose();
    }
  }

  // Report final state and map statistics on both success and handled errors.
  result.state = state_;
  result.metric_scale = last_keyframe_.has_value();
  result.map_points = map_->num_map_points();
  result.keyframes = map_->num_keyframes();
  result.elapsed_seconds =
      std::chrono::duration<double>(steady_clock::now() - start).count();
  return result;
}

void SlamSystem::Reset() {
  map_->Reset();
  frames_ = std::make_unique<core::FrameFactory>(map_->epoch(), config_.time_base);

  mapper_.Reset();
  last_keyframe_.reset();
  state_ = SlamState::Initializing;
}

}  // namespace vslam::system
