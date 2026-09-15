#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>

#include "vslam/optimization/local_bundle_adjuster.h"
#include "vslam/sensor/stereo_camera.h"
// Compare fixed geometry, topology, image measurements and initial estimates in separate
// executables.
int main() {
  try {
    const vslam::sensor::CameraModel left(500, 500, 320, 240, 640, 480);
    const vslam::sensor::StereoCamera camera(left, left, 0.12);
    const vslam::geometry::SE3 truth(Eigen::Matrix3d::Identity(), {-0.1, 0.01, 0.005});
    vslam::optimization::BAProblem problem;
    problem.poses = {
        {vslam::common::KeyFrameId(0, 0), vslam::geometry::SE3(), true},
        {vslam::common::KeyFrameId(0, 1),
         vslam::geometry::SE3(Eigen::Matrix3d::Identity(), {-0.098, 0.011, 0.006}), false}};
    std::mt19937 random(73);
    std::normal_distribution<double> noise(0, 0.25);
    for (int index = 0; index < 70; ++index) {
      const Eigen::Vector3d world((index % 10 - 4.5) * 0.14, (index / 10 - 3) * 0.13,
                                  2.3 + 0.07 * (index % 5));
      problem.points.push_back(
          {vslam::common::MapPointId(0, index), world + Eigen::Vector3d(0.001, -0.001, 0.002)});
      for (std::size_t pose = 0; pose < 2; ++pose) {
        const Eigen::Vector3d point = pose ? truth * world : world;
        auto pixel = *camera.Project(point);
        auto right = *camera.ProjectRight(point);
        pixel.x() += noise(random);
        pixel.y() += noise(random);
        right.x() += noise(random);
        right.y() = pixel.y();
#ifdef VSLAM_DEPTH_ROUTE
        const auto measurement = camera.Depth(pixel, right);
#else
        const auto measurement = vslam::sensor::StereoMeasurement{right, 1.0, 1.0, 1.0};
#endif
        problem.measurements.push_back({vslam::common::ObservationId(0, index * 2 + pose), pose,
                                        static_cast<std::size_t>(index), pixel, measurement});
      }
    }
    vslam::optimization::SolverOptions options;
    options.max_iterations = 80;
#ifdef VSLAM_DEPTH_ROUTE
    const auto result = vslam::optimization::LocalBundleAdjuster(left, options).Solve(problem);
    const char* route = "A0_fixed_measurement";
#else
    const auto result = vslam::optimization::LocalBundleAdjuster(camera, options).Solve(problem);
    const char* route = "B_fixed_measurement";
#endif
    if (!result.usable) {
      throw std::runtime_error(result.message);
    }
    const double error = (result.geometry.poses[1].tcw.translation() - truth.translation()).norm();
    if (error > 0.03 || result.final_cost > result.initial_cost) {
      throw std::runtime_error("Controlled backend solve failed accuracy/cost checks");
    }
    std::cout << std::setprecision(17) << "{\"route\":\"" << route
              << "\",\"seed\":73,\"points\":70,"
              << "\"observations\":140,\"injected_pixel_sigma\":0.25,\"translation_error_m\":"
              << error << ",\"iterations\":" << result.iterations
              << ",\"initial_cost\":" << result.initial_cost
              << ",\"final_cost\":" << result.final_cost << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
