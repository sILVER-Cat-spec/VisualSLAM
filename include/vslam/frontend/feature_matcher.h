#pragma once
#include "vslam/core/feature_set.h"
namespace vslam::frontend {
struct Match { std::size_t first, second; float distance; };
struct MatcherOptions { double ratio = 0.75; bool mutual = false; };
class FeatureMatcher {
 public:
  explicit FeatureMatcher(MatcherOptions options = {});
  std::vector<Match> MatchDescriptors(const cv::Mat& first, const cv::Mat& second) const;
 private:
  MatcherOptions options_;
};
}
