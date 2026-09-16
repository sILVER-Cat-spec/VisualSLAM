#include <algorithm>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <set>
#include <stdexcept>

#include "vslam/frontend/feature_extractor.h"
#include "vslam/frontend/feature_matcher.h"
#include "vslam/sensor/image_preprocessor.h"

namespace vslam::frontend {
using std::isfinite;

// Validate detector and depth settings before accepting any images.
FeatureExtractor::FeatureExtractor(ExtractorOptions options) : options_(options) {
  if (options.n_features < 1 || options.n_features > 100000 || !isfinite(options.scale_factor) ||
      options.scale_factor <= 1 || options.n_levels < 1 || options.n_levels > 32 ||
      options.fast_threshold < 0 || options.depth_radius < 0 || options.depth_radius > 16 ||
      !isfinite(options.min_depth) || !isfinite(options.max_depth) || options.min_depth <= 0 ||
      options.max_depth <= options.min_depth ||
      (options.kind != core::DescriptorKind::Orb && options.kind != core::DescriptorKind::Sift)) {
    throw std::invalid_argument("Invalid feature extractor options");
  }
}

// Attach aligned camera-axis depth to each detected image feature.
core::FeatureSet FeatureExtractor::Extract(const cv::Mat& image, const cv::Mat& depth) const {
  cv::Mat gray;
  if (image.type() == CV_8UC3) {
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  } else if (image.type() == CV_8UC1) {
    gray = image;
  } else {
    throw std::invalid_argument("Extractor requires RGB8 or gray8");
  }
  if (image.empty() || depth.type() != CV_32FC1 || depth.size() != image.size()) {
    throw std::invalid_argument("Extractor requires aligned Z depth");
  }

  cv::Ptr<cv::Feature2D> detector;
  if (options_.kind == core::DescriptorKind::Orb) {
    detector = cv::ORB::create(options_.n_features, options_.scale_factor, options_.n_levels, 31, 0,
                               2, cv::ORB::HARRIS_SCORE, 31, options_.fast_threshold);
  } else {
    detector = cv::SIFT::create(options_.n_features);
  }

  std::vector<cv::KeyPoint> detected_keypoints;
  cv::Mat descriptors;
  detector->detectAndCompute(gray, cv::noArray(), detected_keypoints, descriptors);

  std::vector<core::Keypoint> keypoints;
  keypoints.reserve(detected_keypoints.size());
  for (const auto& detected : detected_keypoints) {
    core::Keypoint keypoint;
    keypoint.uv = {detected.pt.x, detected.pt.y};
    keypoint.octave = detected.octave;
    keypoint.angle = detected.angle;
    keypoint.response = detected.response;
    keypoint.size = detected.size;
    keypoint.depth_z_m = sensor::SampleDepth(depth, keypoint.uv, options_.min_depth,
                                             options_.max_depth, options_.depth_radius);
    keypoints.push_back(keypoint);
  }

  return core::FeatureSet(options_.kind, std::move(keypoints), descriptors);
}

// Reject ratio thresholds that cannot distinguish the nearest two descriptors.
FeatureMatcher::FeatureMatcher(MatcherOptions options) : options_(options) {
  if (!isfinite(options.ratio) || options.ratio <= 0 || options.ratio >= 1) {
    throw std::invalid_argument("Ratio must lie between zero and one");
  }
}

// Apply the ratio test, optional reverse check, and deterministic target deduplication.
std::vector<Match> FeatureMatcher::MatchDescriptors(const cv::Mat& first,
                                                    const cv::Mat& second) const {
  if (first.empty() || second.empty()) {
    return {};
  }
  if (first.dims != 2 || second.dims != 2 || first.type() != second.type() ||
      first.cols != second.cols || (first.type() != CV_8UC1 && first.type() != CV_32FC1)) {
    throw std::invalid_argument("Descriptor matrices are incompatible");
  }

  const int distance_norm = first.type() == CV_8UC1 ? cv::NORM_HAMMING : cv::NORM_L2;
  cv::BFMatcher matcher(distance_norm);
  std::vector<std::vector<cv::DMatch>> forward_matches;
  std::vector<std::vector<cv::DMatch>> reverse_matches;
  matcher.knnMatch(first, second, forward_matches, 2);
  if (options_.mutual) {
    matcher.knnMatch(second, first, reverse_matches, 2);
  }

  std::vector<Match> candidates;
  for (const auto& nearest : forward_matches) {
    if (nearest.size() != 2 || nearest[0].distance >= options_.ratio * nearest[1].distance) {
      continue;
    }
    const auto& best_match = nearest[0];
    if (options_.mutual) {
      const auto& reverse_nearest =
          reverse_matches.at(static_cast<std::size_t>(best_match.trainIdx));
      if (reverse_nearest.size() != 2 ||
          reverse_nearest[0].distance >= options_.ratio * reverse_nearest[1].distance ||
          reverse_nearest[0].trainIdx != best_match.queryIdx) {
        continue;
      }
    }
    candidates.push_back({static_cast<std::size_t>(best_match.queryIdx),
                          static_cast<std::size_t>(best_match.trainIdx), best_match.distance});
  }

  // Prefer the lowest distance; feature indices break ties reproducibly.
  std::sort(candidates.begin(), candidates.end(), [](const Match& left, const Match& right) {
    if (left.distance != right.distance) {
      return left.distance < right.distance;
    }
    if (left.first != right.first) {
      return left.first < right.first;
    }
    return left.second < right.second;
  });

  std::set<std::size_t> used_target_features;
  std::vector<Match> result;
  for (const auto& match : candidates) {
    if (used_target_features.insert(match.second).second) {
      result.push_back(match);
    }
  }
  return result;
}
}  // namespace vslam::frontend
