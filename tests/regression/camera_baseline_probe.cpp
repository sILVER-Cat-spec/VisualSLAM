#include "vslam/sensor/rgbd_camera.h"
#include <iomanip>
#include <iostream>

int main() {
  const vslam::sensor::RGBDCamera camera(
      vslam::sensor::CameraModel(517.3, 521.8, 318.6, 239.2, 640, 480), 0.001);
  std::cout << std::setprecision(17);
  double x, y, z;
  while (std::cin >> x >> y >> z) {
    const auto uv = camera.Project({x, y, z});
    if (!uv) return 1;
    const auto p = camera.UnprojectRegisteredDepth(*uv, z / 0.001);
    if (!p) return 1;
    std::cout << uv->x() << ' ' << uv->y() << ' '
              << p->x() << ' ' << p->y() << ' ' << p->z() << '\n';
  }
  return std::cin.eof() ? 0 : 1;
}
