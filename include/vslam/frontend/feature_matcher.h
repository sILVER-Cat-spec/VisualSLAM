#pragma once

#include "vslam/core/feature_set.h"

namespace vslam::frontend {
// Row indices refer to the first and second descriptor matrices, respectively.
struct Match {
  std::size_t first;
  std::size_t second;
  float distance;
};

struct MatcherOptions {
  double ratio = 0.75;
  bool mutual = false;
};

class FeatureMatcher {
 public:
  // The ratio must be finite and strictly between zero and one; otherwise throws.
  explicit FeatureMatcher(MatcherOptions options = {});

  // Match descriptor rows with Hamming (CV_8UC1) or L2 (CV_32FC1) distance.
  // Empty input returns no matches; incompatible types or widths throw std::invalid_argument.
  // Results use input row indices, are sorted by distance then indices, and have unique targets.
  std::vector<Match> MatchDescriptors(const cv::Mat& first, const cv::Mat& second) const;

 private:
  MatcherOptions options_;
};
}  // namespace vslam::frontend
