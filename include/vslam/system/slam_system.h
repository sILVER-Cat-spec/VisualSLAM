#pragma once

#include "vslam/frontend/feature_extractor.h"
#include "vslam/mapping/local_mapper.h"
#include "vslam/sensor/image_preprocessor.h"
#include "vslam/tracking/frame_tracker.h"

#include <cstddef>
#include <memory>
#include <optional>

namespace vslam::system {

struct SlamConfig {
  frontend::ExtractorOptions frontend;
  frontend::MatcherOptions matcher;
  tracking::TrackingOptions tracking;
  mapping::MappingOptions mapping;
  optimization::SolverOptions pose_solver;
  optimization::SolverOptions ba_solver;

  // Minimum valid depth-supported points needed to create the map.
  std::size_t min_initial_points = 100;
  common::TimeBase time_base = common::TimeBase::Simulation;
};

enum class SlamState {
  Initializing,  // Waiting for enough depth-supported features to create a map.
  Tracking,     // The latest tracking/initialization attempt succeeded.
  Lost          // Subsequent frames retry tracking against the last keyframe.
};

struct SlamResult {
  // An OK processing status alone does not guarantee a pose; also inspect tcw.
  common::Status status;
  SlamState state = SlamState::Initializing;

  // Owns the transient frame; does not keep the system's Map alive.
  std::optional<core::Frame> frame;
  // World-to-camera transform, translation in meters. Present on successful
  // initialization/tracking, after any local mapping pose update.
  std::optional<geometry::SE3> tcw;
  std::optional<common::KeyFrameId> inserted_keyframe;

  std::size_t features = 0;
  std::size_t inliers = 0;
  std::size_t map_points = 0;
  std::size_t keyframes = 0;

  // An RGB-D map has been initialized, even if the current frame is lost.
  bool metric_scale = false;
  bool ba_committed = false;
  double elapsed_seconds = 0;
  // Monotonic stage timings in seconds; mapping includes BA, which is also reported separately.
  double prepare_seconds = 0;
  double frontend_seconds = 0;
  double initialization_seconds = 0;
  double tracking_seconds = 0;
  double mapping_seconds = 0;
  double ba_seconds = 0;

  // Tracking diagnostics stay default during initialization; mapping is
  // populated only when the local mapper processes a new keyframe.
  tracking::TrackingResult tracking;
  std::optional<mapping::MappingResult> mapping;
};

// Synchronous RGB-D pipeline owning the map and processing modules.
// Callers must serialize processing, reset, and access to map data.
class SlamSystem {
 public:
  // Create an empty system. Invalid module options or fewer than six required
  // initialization points throw std::invalid_argument.
  SlamSystem(sensor::CameraModel camera, SlamConfig config = {});

  // Prepare registered, rectified RGB-D input, extract features, and update
  // the map. Timestamps must follow the configured time base and frame ordering;
  // raw depth is converted to meters using the input scale.
  // OpenCV and invalid-argument exceptions become status errors and clear the
  // output pose. Other exceptions propagate. Earlier map changes are not rolled
  // back if a later processing step fails.
  SlamResult Process(const sensor::FrameInput& input);

  // Use supplied features instead of extracting them (e.g. for geometry tests).
  // Input preparation and Process error handling still apply. Features must
  // match the configured descriptor kind and camera grid; optional depths are
  // camera-axis Z in meters within the configured depth interval.
  // The supplied FeatureSet is not retained by reference.
  SlamResult ProcessFeatures(const sensor::FrameInput& input,
                             const core::FeatureSet& features);

  // Borrow the current map. Initialization may replace it; Reset invalidates
  // its contents and IDs. Do not retain this reference across processing/reset.
  const core::Map& map() const noexcept { return *map_; }

  // Return the current state; an invalid-input error does not itself change it.
  SlamState state() const noexcept { return state_; }

  // Clear map/mapping history, advance the map epoch, and restart frame ordering
  // and initialization. Camera and configuration are retained.
  void Reset();

 private:
  SlamResult ProcessImpl(const sensor::FrameInput& input,
                         const core::FeatureSet* supplied_features);
  void ValidateFeatures(const core::FeatureSet& features) const;
  void InitializeFrame(core::Frame& frame, SlamResult& result);
  void TrackFrame(core::Frame& frame, SlamResult& result);

  sensor::CameraModel camera_;
  SlamConfig config_;

  frontend::FeatureExtractor extractor_;
  tracking::FrameTracker tracker_;
  mapping::LocalMapper mapper_;

  std::unique_ptr<core::Map> map_;
  std::unique_ptr<core::FrameFactory> frames_;
  std::optional<common::KeyFrameId> last_keyframe_;
  SlamState state_ = SlamState::Initializing;
};

// Return a stable lowercase label, or "unknown" for an unrecognized value.
const char* StateName(SlamState state);

}  // namespace vslam::system
