
#pragma once
#include <vector>

#include "vslam/sensor/stereo_preprocessor.h"
namespace vslam::sensor {
struct RectifiedStereoCalibration {
  StereoCamera camera;
  StereoRectification maps;
  geometry::SE3 rectified_left_from_raw_left;
};
// Compute fixed maps from two calibrated pinhole cameras, OpenCV distortion vectors
// and T_right_left (meters). Output uses the original image size and horizontal
// rectified geometry. Reject vertical/reversed rigs and invalid calibration.
// The left rectification rotation must also be used by evaluation/publication.
RectifiedStereoCalibration RectifyStereo(const CameraModel& raw_left, const CameraModel& raw_right,
                                         const std::vector<double>& left_distortion,
                                         const std::vector<double>& right_distortion,
                                         const geometry::SE3& right_from_left);
}  // namespace vslam::sensor
