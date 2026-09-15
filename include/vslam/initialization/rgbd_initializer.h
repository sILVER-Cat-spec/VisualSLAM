#pragma once
#include "vslam/core/map.h"
#include "vslam/sensor/camera_model.h"
namespace vslam::initialization {
struct InitializationResult {
  std::unique_ptr<core::Map> map;
  common::KeyFrameId keyframe;
  std::size_t points=0;
};
// Builds an unpublished candidate; waiting leaves Frame and existing state unchanged.
InitializationResult InitializeRGBD(core::Frame& frame,const sensor::CameraModel& camera,
                                    std::size_t min_points,double min_depth,double max_depth);
}
