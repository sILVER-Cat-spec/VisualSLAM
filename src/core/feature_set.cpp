#include "vslam/core/feature_set.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace vslam::core {

using std::invalid_argument;
using std::isfinite;
using std::move;

void ValidateDescriptors(DescriptorKind kind, const cv::Mat& descriptors, std::size_t rows) {
  if (kind != DescriptorKind::Orb && kind != DescriptorKind::Sift) {
    throw invalid_argument("Unknown descriptor kind");
  }

  if (rows == 0) {
    if (!descriptors.empty()) {
      throw invalid_argument("Empty features need empty descriptors");
    }
    return;
  }

  const int columns = kind == DescriptorKind::Orb ? 32 : 128;
  const int type = kind == DescriptorKind::Orb ? CV_8UC1 : CV_32FC1;
  if (rows > static_cast<std::size_t>(std::numeric_limits<int>::max()) || descriptors.dims != 2 ||
      descriptors.rows != static_cast<int>(rows) || descriptors.cols != columns ||
      descriptors.type() != type || !cv::checkRange(descriptors)) {
    throw invalid_argument("Descriptor shape/type/value does not match ORB or SIFT features");
  }
}

FeatureSet::FeatureSet(DescriptorKind kind)
    : FeatureSet(kind, {}, cv::Mat()) {}

FeatureSet::FeatureSet(DescriptorKind kind, std::vector<Keypoint> keypoints,
                       const cv::Mat& descriptors)
    : kind_(kind),
      keypoints_(move(keypoints)) {
  ValidateDescriptors(kind, descriptors, keypoints_.size());

  for (const auto& point : keypoints_) {
    if (!point.uv.allFinite() || point.octave < (kind == DescriptorKind::Sift ? -1 : 0) ||
        !isfinite(point.angle) ||
        (point.angle != -1.0f && (point.angle < 0 || point.angle >= 360)) ||
        !isfinite(point.response) || !isfinite(point.size) || point.size <= 0 ||
        (point.depth_z_m && (!isfinite(*point.depth_z_m) || *point.depth_z_m <= 0))) {
      throw invalid_argument("Invalid keypoint measurement");
    }
  }

  descriptors_ = descriptors.clone();
}

FeatureSet::FeatureSet(const FeatureSet& other)
    : kind_(other.kind_),
      keypoints_(other.keypoints_),
      descriptors_(other.descriptors_.clone()) {}

FeatureSet& FeatureSet::operator=(const FeatureSet& other) {
  if (this != &other) {
    // Complete the potentially throwing copy before replacing the current storage.
    FeatureSet copy(other);
    *this = move(copy);
  }

  return *this;
}

cv::Mat FeatureSet::Descriptor(std::size_t index) const {
  // Check the feature index before narrowing it to OpenCV's integer row index.
  keypoints_.at(index);

  return descriptors_.row(static_cast<int>(index)).clone();
}

}  // namespace vslam::core
