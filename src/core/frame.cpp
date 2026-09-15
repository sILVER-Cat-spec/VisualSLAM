#include "vslam/core/frame.h"

#include <stdexcept>
#include <utility>

namespace vslam::core {

using std::invalid_argument;
using std::move;

Frame::Frame(common::FrameId id, std::uint64_t sequence, common::Timestamp timestamp,
             const cv::Mat& image, const cv::Mat& depth, std::string external_label)
    : id_(id),
      sequence_(sequence),
      timestamp_(timestamp),
      external_label_(move(external_label)) {
  if (!id.valid() || timestamp.ns < 0 || image.empty() || image.dims != 2 ||
      (image.type() != CV_8UC1 && image.type() != CV_8UC3) ||
      (!depth.empty() &&
       (depth.dims != 2 || depth.size() != image.size() || depth.type() != CV_32FC1))) {
    throw invalid_argument(
        "Frame requires an ID, timestamp, uint8 image and aligned float Z depth");
  }

  image_ = image.clone();
  depth_ = depth.clone();
}

Frame::Frame(const Frame& other)
    : id_(other.id_),
      sequence_(other.sequence_),
      timestamp_(other.timestamp_),
      external_label_(other.external_label_),
      image_(other.image_.clone()),
      depth_(other.depth_.clone()),
      features_(other.features_),
      pose_(other.pose_),
      associations_(other.associations_) {}

Frame& Frame::operator=(const Frame& other) {
  if (this != &other) {
    // Copy every owned buffer before changing any state in the destination frame.
    Frame copy(other);
    *this = move(copy);
  }

  return *this;
}

void Frame::SetFeatures(const FeatureSet& features) {
  for (const auto& keypoint : features.keypoints()) {
    if (keypoint.uv.x() < 0 || keypoint.uv.y() < 0 || keypoint.uv.x() >= image_.cols ||
        keypoint.uv.y() >= image_.rows) {
      throw invalid_argument("Frame feature lies outside its image");
    }
  }

  // Allocate both replacements before changing measurements or their association index.
  FeatureSet copy(features);
  std::vector<common::MapPointId> associations(features.size());

  features_ = move(copy);
  associations_.swap(associations);
}

void Frame::Associate(std::size_t feature, common::MapPointId point) {
  if (!point.valid() || point.epoch() != id_.epoch()) {
    throw invalid_argument("Frame association requires a valid same-epoch MapPoint ID");
  }

  associations_.at(feature) = point;
}

Frame FrameFactory::Create(common::Timestamp timestamp, const cv::Mat& image, const cv::Mat& depth,
                           std::string label) {
  stamps_.Validate(timestamp);
  if (sequence_ == common::FrameId::invalid_value) {
    throw std::overflow_error("Frame sequence exhausted");
  }

  // A failed construction may consume an ID, but must not accept the timestamp or sequence.
  Frame frame(ids_.Next(), sequence_, timestamp, image, depth, move(label));

  stamps_.Accept(timestamp);
  ++sequence_;

  return frame;
}

}  // namespace vslam::core
