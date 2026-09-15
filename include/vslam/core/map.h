#pragma once

#include "vslam/common/status.h"
#include "vslam/common/time.h"
#include "vslam/core/keyframe.h"
#include "vslam/core/map_point.h"
#include "vslam/core/map_snapshot.h"
#include <memory>
#include <map>
#include <set>

namespace vslam::core {

// Single-threaded owner. Returned const references expire on deletion/reset.
// IDs belong to one system/Map instance; epoch prevents reuse after its reset.
class Map {
 public:
  // Create an empty single-threaded map with the given descriptor kind and epoch.
  explicit Map(DescriptorKind kind = DescriptorKind::Orb, common::Epoch epoch = 0);

  // Prevent copying ownership of persistent map entities.
  Map(const Map&) = delete;

  // Prevent replacement through copying; use explicit map operations instead.
  Map& operator=(const Map&) = delete;

  // Return the generation that distinguishes IDs before and after Reset.
  common::Epoch epoch() const noexcept {
    return epoch_;
  }

  // Return the version used to reject stale geometry batches.
  common::Revision revision() const noexcept {
    return revision_;
  }

  // Count the currently stored keyframes.
  std::size_t num_keyframes() const noexcept {
    return keyframes_.size();
  }

  // Count stored landmarks, including any marked bad or awaiting observations.
  std::size_t num_map_points() const noexcept {
    return points_.size();
  }

  // Count persistent measurement edges, including flagged outliers.
  std::size_t num_observations() const noexcept {
    return observations_.size();
  }

  // Promotes an independent copy and registers all copied Frame associations.
  // Failure leaves map topology and revision unchanged; consumed IDs are not reused.
  common::KeyFrameId InsertKeyFrame(const Frame& frame, bool retain_images = false);

  // Create a finite world-space landmark in meters, with an optional descriptor.
  // The point initially has no observations; register them with AddObservation.
  common::MapPointId AddMapPoint(const Eigen::Vector3d& position_w, const cv::Mat& descriptor = {});

  // Register a frozen keyframe measurement and update all forward and reverse indexes.
  // Reject conflicting edges or invalid noise scales; roll back partial index allocation.
  common::ObservationId AddObservation(common::KeyFrameId keyframe, common::MapPointId point,
                                       std::size_t feature, double pixel_sigma = 1.0,
                                       double depth_sigma_m = 0.01);

  // Remove the edge and all related indexes; return false if the ID is absent.
  bool RemoveObservation(common::ObservationId id);

  // Remove a landmark and its observations; return false if the ID is absent.
  bool RemoveMapPoint(common::MapPointId id);

  // Remove a keyframe and its observations; return false if absent and throw if protected.
  // Landmarks remain stored even if removing this keyframe leaves them unobserved.
  bool RemoveKeyFrame(common::KeyFrameId id);

  // Enable or disable deletion protection for an existing keyframe; this does not fix its pose.
  void ProtectKeyFrame(common::KeyFrameId id, bool protect = true);

  // Set the quality flag on an existing landmark without deleting it or its observations.
  void MarkMapPointBad(common::MapPointId id, bool bad = true);

  // Set the outlier flag on an existing edge without removing it.
  void MarkObservationOutlier(common::ObservationId id, bool outlier = true);

  // Borrow an existing keyframe; throw std::out_of_range if the ID is absent.
  const KeyFrame& GetKeyFrame(common::KeyFrameId id) const;

  // Borrow an existing landmark; throw std::out_of_range if the ID is absent.
  const MapPoint& GetMapPoint(common::MapPointId id) const;

  // Borrow an existing observation; throw std::out_of_range if the ID is absent.
  const Observation& GetObservation(common::ObservationId id) const;

  // Copy detached geometry and measurements in ID order for diagnostics or export.
  MapSnapshot Snapshot() const;

  // Validate the complete batch before writing any geometry; reject stale versions,
  // unknown or duplicate IDs, and nonfinite points without partial geometry updates.
  // This does not evaluate solver convergence or delete outlier observations.
  common::Status CommitGeometry(common::Epoch epoch, common::Revision revision,
                                const std::vector<PoseUpdate>& poses,
                                const std::vector<PointUpdate>& points);

  // Check that observation endpoints, feature associations, and reverse indexes agree.
  bool CheckConsistency() const;

  // Clear the map and advance its epoch so old IDs cannot refer to newly created entities.
  // Throw before clearing if the epoch counter is exhausted.
  void Reset();

 private:
  DescriptorKind kind_;
  common::Epoch epoch_ = 0;
  common::Revision revision_ = 0;

  common::IdGenerator<common::KeyFrameId> keyframe_ids_;
  common::IdGenerator<common::MapPointId> point_ids_;
  common::IdGenerator<common::ObservationId> observation_ids_;

  std::map<common::KeyFrameId, std::unique_ptr<KeyFrame>> keyframes_;
  std::map<common::MapPointId, std::unique_ptr<MapPoint>> points_;
  std::map<common::ObservationId, Observation> observations_;
  std::set<common::KeyFrameId> protected_keyframes_;
};

}  // namespace vslam::core
