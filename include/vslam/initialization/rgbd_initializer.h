#pragma once

#include "vslam/core/map.h"
#include "vslam/sensor/camera_model.h"

namespace vslam::initialization {
struct InitializationResult {
  std::unique_ptr<core::Map> map;
  common::KeyFrameId keyframe;
  std::size_t points = 0;
};

// Build an owned candidate map from camera-axis depths in [min_depth, max_depth] meters.
// Insufficient valid points return a null map and leave frame unchanged.
// Success sets frame to the world origin, associates its points, and protects the first keyframe.
InitializationResult InitializeRGBD(core::Frame& frame, const sensor::CameraModel& camera,
                                    std::size_t min_points, double min_depth, double max_depth);
}  // namespace vslam::initialization
