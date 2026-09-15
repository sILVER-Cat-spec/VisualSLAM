#pragma once

#include "vslam/common/ids.h"
#include <Eigen/Core>
#include <optional>

namespace vslam::core {

class Map;

// Persistent edge linking one keyframe feature to one world-space landmark.
class Observation {
 public:
  // Return the identity of this persistent measurement edge.
  common::ObservationId id() const noexcept {
    return id_;
  }

  // Return the observing keyframe endpoint.
  common::KeyFrameId keyframe_id() const noexcept {
    return keyframe_id_;
  }

  // Return the observed landmark endpoint.
  common::MapPointId map_point_id() const noexcept {
    return map_point_id_;
  }

  // Return the measurement index within the keyframe feature set.
  std::size_t feature_index() const noexcept {
    return feature_index_;
  }

  // Borrow the measured image coordinates in pixels.
  const Eigen::Vector2d& uv() const noexcept {
    return uv_;
  }

  // Return optional measured camera-axis depth in meters.
  std::optional<double> depth_z_m() const noexcept {
    return depth_z_m_;
  }

  // Return the stored pixel noise scale for this measurement.
  double pixel_sigma() const noexcept {
    return pixel_sigma_;
  }

  // Return the stored depth noise scale in meters.
  double depth_sigma_m() const noexcept {
    return depth_sigma_m_;
  }

  // Report the outlier flag without removing the measurement edge.
  bool is_outlier() const noexcept {
    return is_outlier_;
  }

 private:
  friend class Map;

  // Persistent edge linking one keyframe feature to one world-space landmark.

  // Freeze a validated measurement and its endpoints; Map maintains the related indexes.
  Observation(common::ObservationId id, common::KeyFrameId keyframe, common::MapPointId point,
              std::size_t feature, const Eigen::Vector2d& uv, std::optional<double> depth,
              double pixel_sigma, double depth_sigma)
      : id_(id),
        keyframe_id_(keyframe),
        map_point_id_(point),
        feature_index_(feature),
        uv_(uv),
        depth_z_m_(depth),
        pixel_sigma_(pixel_sigma),
        depth_sigma_m_(depth_sigma) {}
  common::ObservationId id_;
  common::KeyFrameId keyframe_id_;
  common::MapPointId map_point_id_;
  std::size_t feature_index_;

  Eigen::Vector2d uv_;
  std::optional<double> depth_z_m_;

  double pixel_sigma_;
  double depth_sigma_m_;

  bool is_outlier_ = false;
};

}  // namespace vslam::core
