#pragma once
#include "vslam/sensor/stereo_camera.h"
#include "vslam/sensor/stereo_frame_input.h"
namespace vslam::sensor {
// Immutable rectification maps supplied from a calibrated raw pair. Empty maps
// accept only input explicitly declared rectified. ROI masks use the output grid.
struct StereoRectification {
  cv::Mat left_x, left_y, right_x, right_y;
  cv::Mat left_mask, right_mask;
};
// Own converted RGB/gray buffers; reject incomplete pairs, invalid types/grids,
// time bases, negative capture times and excessive capture skew. Raw input needs maps.
PreparedStereoInput PrepareStereo(const StereoFrameInput& input, const StereoCamera& camera,
                                  std::int64_t max_skew_ns = 0,
                                  const StereoRectification& rectification = {});
}  // namespace vslam::sensor
