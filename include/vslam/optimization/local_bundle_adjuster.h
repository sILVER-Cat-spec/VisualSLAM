#pragma once
#include "vslam/optimization/optimization_problem.h"
#include "vslam/sensor/camera_model.h"
namespace vslam::optimization {
class LocalBundleAdjuster {
 public:
  LocalBundleAdjuster(sensor::CameraModel camera, SolverOptions options = {});
  BASolution Solve(const BAProblem& input) const;
 private:
  sensor::CameraModel camera_;
  SolverOptions options_;
};
}
