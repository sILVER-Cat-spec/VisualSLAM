#pragma once
#include "vslam/core/feature_set.h"
namespace vslam::frontend {
struct ExtractorOptions {
  core::DescriptorKind kind = core::DescriptorKind::Orb;
  int n_features = 2000;
  float scale_factor = 1.2f;
  int n_levels = 8;
  int fast_threshold = 20;
  double min_depth = 0.2, max_depth = 8.0;
  int depth_radius = 1;
};
class FeatureExtractor {
 public:
  explicit FeatureExtractor(ExtractorOptions options);
  core::FeatureSet Extract(const cv::Mat& image_rgb_or_gray, const cv::Mat& depth_z_m) const;
 private:
  ExtractorOptions options_;
};
}
