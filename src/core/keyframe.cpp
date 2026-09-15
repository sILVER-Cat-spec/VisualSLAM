#include "vslam/core/keyframe.h"

#include <stdexcept>

namespace vslam::core {

KeyFrame KeyFrame::FromFrame(common::KeyFrameId id, const Frame& frame, bool retain_images) {
  if (!id.valid() || id.epoch() != frame.id().epoch() || !frame.pose()) {
    throw std::invalid_argument("KeyFrame requires a same-epoch ID and a posed Frame");
  }

  return KeyFrame(id, frame, retain_images);
}

KeyFrame::KeyFrame(common::KeyFrameId id, const Frame& frame, bool retain_images)
    : id_(id),
      source_frame_id_(frame.id()),
      source_sequence_(frame.sequence()),
      timestamp_(frame.timestamp()),
      external_label_(frame.external_label()),
      features_(frame.features()),
      tcw_(*frame.pose()),
      image_(retain_images ? frame.image() : cv::Mat()),
      depth_(retain_images ? frame.depth_z_m() : cv::Mat()),
      associations_(frame.associations()) {}

}  // namespace vslam::core
