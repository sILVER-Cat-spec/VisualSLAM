#include "vslam/sensor/rgbd_camera.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using vslam::sensor::CameraModel;

void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  try {
    const CameraModel c(500, 520, 320, 240, 640, 480);
    Check(c.K()(0, 0) == 500 && c.K()(1, 1) == 520 &&
          c.K()(0, 2) == 320 && c.K()(1, 2) == 240 &&
          c.K()(2, 2) == 1 && c.K()(0, 1) == 0, "intrinsic matrix");
    const auto uv = c.Project({1, -0.5, 2});
    Check(uv && (*uv - Eigen::Vector2d(570, 110)).norm() < 1e-12, "projection");
    const auto ray = c.BackProject({570, 110});
    Check(ray && (*ray - Eigen::Vector3d(0.5, -0.25, 1)).norm() < 1e-12, "unit-Z ray");
    const auto point = c.Unproject({570, 110}, 2);
    Check(point && (*point - Eigen::Vector3d(1, -0.5, 2)).norm() < 1e-12, "Z depth");
    for (int i = -20; i <= 20; ++i) {
      const Eigen::Vector3d p(i * 0.3, i * -0.15, 0.1 + (i + 20) * 0.2);
      const auto pixel = c.Project(p);
      Check(pixel.has_value(), "round-trip projection");
      const auto recovered = c.Unproject(*pixel, p.z());
      Check(recovered && (*recovered - p).norm() < 1e-12, "round trip");
    }
    Check(c.Contains({0, 0}) && c.Contains({639.99, 479.99}), "inside bounds");
    Check(!c.Contains({640, 0}) && !c.Contains({0, 480}) &&
          !c.Contains({-0.01, 20}), "outside bounds");
    Check(c.Project({100, 0, 1}).has_value(), "off-image projection is valid");
    Check(c.Unproject({-20, 500}, 1).has_value(), "off-image unprojection is valid");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double z : {0.0, -1.0, nan, inf}) {
      Check(!c.Project({0, 0, z}), "invalid project depth");
      Check(!c.Unproject({320, 240}, z), "invalid unproject depth");
    }
    for (double bad : {nan, inf, -inf}) {
      Check(!c.Project({bad, 0, 1}) && !c.Project({0, bad, 1}), "nonfinite point");
      Check(!c.BackProject({bad, 0}) && !c.Unproject({0, bad}, 1) &&
            !c.Contains({bad, 0}), "nonfinite pixel");
    }
    const double huge = std::numeric_limits<double>::max();
    Check(!c.Project({huge, 0, 1}), "projection overflow");
    Check(!c.Unproject({huge, 0}, huge), "unprojection overflow");
    auto invalid = [](double fx, double fy, double cx, double cy, int w, int h) {
      try { const CameraModel unused(fx, fy, cx, cy, w, h); }
      catch (const std::invalid_argument&) { return true; }
      return false;
    };
    Check(invalid(0, 1, 0, 0, 1, 1) && invalid(1, -1, 0, 0, 1, 1) &&
          invalid(nan, 1, 0, 0, 1, 1) && invalid(1, inf, 0, 0, 1, 1) &&
          invalid(1, 1, nan, 0, 1, 1) && invalid(1, 1, 0, inf, 1, 1) &&
          invalid(1, 1, 0, 0, 0, 1) && invalid(1, 1, 0, 0, 1, -1),
          "invalid calibration");
    const CameraModel cropped(1, 1, -10, 20, 1, 1);
    Check(cropped.BackProject({0, 0}).has_value(), "outside principal point allowed");
    const vslam::sensor::RGBDCamera rgbd_mm(c, 0.001);
    const vslam::sensor::RGBDCamera rgbd_m(c, 1.0);
    const auto mm = rgbd_mm.UnprojectRegisteredDepth({570, 110}, 2000);
    const auto meters = rgbd_m.UnprojectRegisteredDepth({570, 110}, 2);
    Check(mm && meters && (*mm - *meters).norm() < 1e-12 &&
          (*mm - Eigen::Vector3d(1, -0.5, 2)).norm() < 1e-12, "RGB-D units");
    Check(rgbd_mm.Project(*mm)->isApprox(Eigen::Vector2d(570, 110)), "RGB-D projection");
    for (double bad : {0.0, -1.0, nan, inf}) {
      Check(!rgbd_mm.DepthZMeters(bad) &&
            !rgbd_mm.UnprojectRegisteredDepth({320, 240}, bad), "invalid raw depth");
      bool threw = false;
      try { const vslam::sensor::RGBDCamera unused(c, bad); }
      catch (const std::invalid_argument&) { threw = true; }
      Check(threw, "invalid depth scale");
    }
    const vslam::sensor::RGBDCamera large_scale(c, 2.0);
    Check(!large_scale.DepthZMeters(huge), "depth conversion overflow");
    std::cout << "Camera geometry checks passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
