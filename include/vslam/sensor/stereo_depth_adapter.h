#pragma once
#include "vslam/sensor/frame_input.h"
#include "vslam/sensor/stereo_preprocessor.h"
namespace vslam::sensor {
struct StereoDepthOptions {
  int min_disparity = 0;
  int num_disparities = 128;
  int block_size = 5;
  double min_depth = 0.2;
  double max_depth = 8.0;
  double consistency_pixels = 1.0;
};
struct StereoDepthResult {
  FrameInput frame;
  cv::Mat disparity_pixels;
  cv::Mat valid_mask;
  double valid_ratio = 0;
};
// Interpret SGBM CV_16SC1 values in sixteenth-pixel units and produce Z meters.
// Invalid, inconsistent, masked and out-of-range pixels are NaN. Reverse disparity
// uses u_right-u_left; explicit reverse matching is required by this interface.
StereoDepthResult ConvertStereoDisparity(const PreparedStereoInput& pair,
                                         const StereoCamera& camera, const cv::Mat& forward_fixed,
                                         const cv::Mat& reverse_fixed,
                                         const StereoDepthOptions& options = {});
// Run forward and reverse SGBM, then enforce the same conversion/filtering contract.
StereoDepthResult EstimateStereoDepth(const PreparedStereoInput& pair, const StereoCamera& camera,
                                      const StereoDepthOptions& options = {});
}  // namespace vslam::sensor
