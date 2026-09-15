#pragma once
#include "vslam/optimization/optimization_problem.h"
#include "vslam/sensor/camera_model.h"
namespace vslam::optimization {
class PoseOptimizer {
 public:
  PoseOptimizer(sensor::CameraModel camera, SolverOptions options = {});
  PoseSolution Solve(const geometry::SE3& initial,const std::vector<PoseMeasurement>& observations) const;
 private:
  sensor::CameraModel camera_;
  SolverOptions options_;
};
}
