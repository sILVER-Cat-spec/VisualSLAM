#pragma once
#include "vslam/common/ids.h"
#include "vslam/geometry/se3.h"
#include <optional>
#include <string>
#include <vector>
namespace vslam::optimization {
struct SolverOptions {
  int max_iterations = 30;
  double pixel_sigma = 1.0, depth_sigma_m = 0.03;
  double huber_delta = 2.5;  // zero selects linear loss.
  double outlier_pixels = 5.0, outlier_depth_sigma = 5.0;
};
struct PoseMeasurement {
  Eigen::Vector3d point_w;
  Eigen::Vector2d uv;
  std::optional<double> depth_z_m;
};
struct PoseSolution {
  bool usable = false;
  std::optional<geometry::SE3> tcw;
  std::vector<bool> inliers;
  int iterations = 0;
  double initial_cost = 0, final_cost = 0;
  std::string message;
};
struct BAPose { common::KeyFrameId id; geometry::SE3 tcw; bool fixed = false; };
struct BAPoint { common::MapPointId id; Eigen::Vector3d position_w; };
struct BAMeasurement {
  common::ObservationId id;
  std::size_t pose_index, point_index;
  Eigen::Vector2d uv;
  std::optional<double> depth_z_m;
};
struct BAProblem {
  common::Epoch epoch = 0;
  common::Revision revision = 0;
  std::vector<BAPose> poses;
  std::vector<BAPoint> points;
  std::vector<BAMeasurement> measurements;
};
struct BASolution {
  bool usable = false;
  bool converged = false;
  BAProblem geometry;
  std::vector<common::ObservationId> outliers;
  int iterations = 0;
  double initial_cost = 0, final_cost = 0;
  std::string message;
};
}
