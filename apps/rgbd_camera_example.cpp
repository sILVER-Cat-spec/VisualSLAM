#include "vslam/sensor/rgbd_camera.h"

#include <exception>
#include <iostream>
#include <optional>

int main() {
  try {
    // Example calibration only: replace with your simulator's RGB intrinsics.
    const vslam::sensor::CameraModel pinhole(500.0, 520.0, 320.0, 240.0, 640, 480);
    // One raw depth unit is one millimeter in this example.
    const vslam::sensor::RGBDCamera camera(pinhole, 0.001);
    const Eigen::Vector2d pixel(570.0, 110.0);
    const double depth_raw = 2000.0;

    if (!camera.model().Contains(pixel)) {
      std::cerr << "Pixel lies outside the image\n";
      return 1;
    }

    // An optional holds either a valid point or no value.
    const std::optional<Eigen::Vector3d> point =
        camera.UnprojectRegisteredDepth(pixel, depth_raw);
    if (!point) {
      std::cerr << "Invalid depth or pixel\n";
      return 1;
    }

    std::cout << "Pixel: " << pixel.transpose() << '\n'
              << "Raw depth: " << depth_raw << " mm\n"
              << "Camera point (meters): " << point->transpose() << '\n';

    const std::optional<Eigen::Vector2d> projected = camera.Project(*point);
    if (!projected) {
      std::cerr << "Point cannot be projected\n";
      return 1;
    }
    std::cout << "Projected pixel: " << projected->transpose() << '\n';

    const std::optional<Eigen::Vector3d> invalid =
        camera.UnprojectRegisteredDepth(pixel, 0.0);
    if (invalid) {
      std::cerr << "Zero depth should be rejected\n";
      return 1;
    }
    std::cout << "Zero depth: rejected\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Camera configuration error: " << error.what() << '\n';
    return 1;
  }
}
