
#include <algorithm>

#include "vslam/io/stereo_dataset_io.h"
#include "vslam/sensor/stereo_rectification.h"
#ifdef VSLAM_DEPTH_ROUTE
#include "vslam/sensor/stereo_depth_adapter.h"
#else
#include "vslam/frontend/stereo_matcher.h"
#endif
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
namespace {
// Fail tests independently of NDEBUG so Release builds exercise every assertion.
void Check(bool value, const char* message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}
// Verify a contract violation is rejected rather than silently coerced.
template <class Function>
void Reject(Function function, const char* message) {
  bool rejected = false;
  try {
    function();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, message);
}
// Construct a pair with distinct principal points to expose naive fx*b/d errors.
vslam::sensor::StereoCamera Camera() {
  return {vslam::sensor::CameraModel(500, 500, 320, 240, 640, 480),
          vslam::sensor::CameraModel(500, 500, 315, 240, 640, 480), 0.12};
}
// Build deterministic image buffers with exact capture synchronization.
vslam::sensor::StereoFrameInput Input() {
  vslam::sensor::StereoFrameInput input;
  input.timestamp = {10, vslam::common::TimeBase::Simulation};
  input.right_timestamp = input.timestamp;
  input.rectified = true;
  input.image = cv::Mat(480, 640, CV_8UC1, cv::Scalar(42));
  input.right_image = input.image.clone();
  return input;
}
// Exercise sign, units, principal-point correction and poorly conditioned disparity rejection.
void TestGeometry() {
  const auto camera = Camera();
  const Eigen::Vector3d point(0, 0, 2);
  const auto left = *camera.Project(point);
  const auto right = *camera.ProjectRight(point);
  Check(std::abs(left.x() - right.x() - 35) < 1e-12, "raw disparity includes principal offset");
  Check(std::abs(*camera.Depth(left, right) - 2) < 1e-12, "metric triangulation");
  Check(camera.RightFromLeft().translation().x() == -0.12, "fixed transform sign");
  Check(!camera.Depth(left, left), "negative corrected disparity rejected");
  Check(!camera.Depth(left, {left.x() - 5.01, left.y()}), "tiny disparity rejected");
  Check(!camera.Depth(left, {right.x(), right.y() + 3}), "epipolar mismatch rejected");
  Check(!camera.Depth(left, {-1, right.y()}), "right ROI bounds");
  Reject([&] { vslam::sensor::StereoCamera bad(camera.left(), camera.right(), 0); },
         "zero baseline");
}
// Bad pairs must fail before a frame ID/timestamp can be committed by the pipeline.
void TestInput() {
  const auto camera = Camera();
  auto input = Input();
  auto prepared = vslam::sensor::PrepareStereo(input, camera);
  input.right_image.setTo(0);
  Check(prepared.right_image.at<unsigned char>(0, 0) == 42, "right image owns its buffer");
  input.right_image.release();
  Reject([&] { vslam::sensor::PrepareStereo(input, camera); }, "missing right frame");
  input = Input();
  ++input.right_timestamp.ns;
  Reject([&] { vslam::sensor::PrepareStereo(input, camera); }, "capture mismatch");
  input = Input();
  const auto computed =
      vslam::sensor::RectifyStereo(camera.left(), camera.right(), {}, {}, camera.RightFromLeft());
  Check(std::abs(computed.camera.baseline() - camera.baseline()) < 1e-10,
        "computed rectification baseline");
  Check(computed.maps.left_x.size() == input.image.size(), "computed rectification grid");
  input.rectified = false;
  Reject([&] { vslam::sensor::PrepareStereo(input, camera); }, "raw input without calibration");
  vslam::sensor::StereoRectification rectification;
  rectification.left_x.create(480, 640, CV_32FC1);
  rectification.left_y.create(480, 640, CV_32FC1);
  for (int y = 0; y < 480; ++y) {
    for (int x = 0; x < 640; ++x) {
      rectification.left_x.at<float>(y, x) = x;
      rectification.left_y.at<float>(y, x) = y;
    }
  }
  rectification.right_x = rectification.left_x.clone();
  rectification.right_y = rectification.left_y.clone();
  rectification.left_mask = cv::Mat(480, 640, CV_8UC1, cv::Scalar(255));
  rectification.right_mask = rectification.left_mask.clone();
  prepared = vslam::sensor::PrepareStereo(input, camera, 0, rectification);
  Check(cv::norm(prepared.right_image, input.right_image) == 0, "identity remap preserves grid");
}
#ifdef VSLAM_DEPTH_ROUTE
// Fixed-point conversion and explicit reverse disparity protect against units and occlusion errors.
void TestMeasurements() {
  const auto camera = Camera();
  auto pair = vslam::sensor::PrepareStereo(Input(), camera);
  cv::Mat forward(480, 640, CV_16SC1, cv::Scalar(35 * 16));
  cv::Mat reverse(480, 640, CV_16SC1, cv::Scalar(-35 * 16));
  auto result = vslam::sensor::ConvertStereoDisparity(pair, camera, forward, reverse);
  Check(result.frame.depth.at<float>(100, 100) == 2, "sixteenth-pixel disparity converted once");
  Check(std::isnan(result.frame.depth.at<float>(0, 0)), "occluded border invalid");
  reverse.at<short>(100, 65) = -20 * 16;
  result = vslam::sensor::ConvertStereoDisparity(pair, camera, forward, reverse);
  Check(std::isnan(result.frame.depth.at<float>(100, 100)), "reverse inconsistency invalid");
  forward.at<short>(100, 100) = -16;
  result = vslam::sensor::ConvertStereoDisparity(pair, camera, forward, reverse);
  Check(std::isnan(result.frame.depth.at<float>(100, 100)), "SGBM sentinel invalid");
  vslam::system::SlamConfig configuration;
  configuration.min_initial_points = 30;
  configuration.mapping.max_gap = 2;
  vslam::system::SlamSystem system(camera.left(), configuration);
  auto synthetic = Input();
  cv::RNG random(73);
  random.fill(synthetic.image, cv::RNG::UNIFORM, 0, 256);
  synthetic.image(cv::Rect(35, 0, 605, 480))
      .copyTo(synthetic.right_image(cv::Rect(0, 0, 605, 480)));
  const auto textured_pair = vslam::sensor::PrepareStereo(synthetic, camera);
  auto adapted = vslam::sensor::EstimateStereoDepth(textured_pair, camera);
  int commits = 0;
  for (int frame = 0; frame < 5; ++frame) {
    adapted.frame.timestamp.ns = frame + 1;
    const auto tracked = system.Process(adapted.frame);
    Check(tracked.tcw.has_value(), "SGBM adapter reaches real-image tracking");
    commits += tracked.ba_committed;
  }
  Check(commits > 0, "SGBM adapter reaches local BA");
  pair.left_mask.setTo(0);
  result = vslam::sensor::ConvertStereoDisparity(pair, camera, forward, reverse);
  Check(result.valid_ratio == 0, "masked depth cannot enter SLAM");
}
#else
// Freeze right pixels/noise through all owning layers, and verify actual image matching has metric
// scale.
void TestMeasurements() {
  const auto camera = Camera();
  auto input = Input();
  cv::RNG random(73);
  random.fill(input.image, cv::RNG::UNIFORM, 0, 256);
  input.image(cv::Rect(35, 0, 605, 480)).copyTo(input.right_image(cv::Rect(0, 0, 605, 480)));
  const auto pair = vslam::sensor::PrepareStereo(input, camera);
  vslam::frontend::ExtractorOptions options;
  const auto left = vslam::frontend::FeatureExtractor(options).Extract(pair.image_rgb_or_gray);
  const auto matched = vslam::frontend::MatchStereo(left, pair, camera, options);
  std::size_t stereo = 0;
  for (const auto& feature : matched.keypoints()) {
    if (feature.stereo) {
      ++stereo;
      Check(std::abs(*feature.depth_z_m - 2) < 0.15, "image-derived metric depth");
    }
  }
  Check(stereo > 100, "real stereo frontend produces enough matches");
  auto points = matched.keypoints();
  auto selected = std::find_if(points.begin(), points.end(),
                               [](const auto& point) { return bool(point.stereo); });
  const auto index = static_cast<std::size_t>(selected - points.begin());
  selected->stereo->left_sigma_px = 1.5;
  selected->stereo->right_sigma_px = 0.7;
  vslam::core::FeatureSet features(matched.kind(), points, matched.descriptors());
  vslam::core::Frame frame(vslam::common::FrameId(0, 0), 0, input.timestamp, input.image,
                           input.right_image);
  frame.SetFeatures(features);
  frame.SetPose(vslam::geometry::SE3());
  vslam::core::Map map;
  const auto id = map.AddMapPoint({0, 0, 2}, features.Descriptor(index));
  frame.Associate(index, id);
  map.InsertKeyFrame(frame);
  const auto snapshot = map.Snapshot();
  const auto observation = snapshot.observations.at(0).stereo();
  Check(observation && observation->right_uv == selected->stereo->right_uv,
        "persistent right pixel");
  Check(observation->right_sigma_px == 0.7 && observation->left_sigma_px == 1.5,
        "persistent pixel noise");
  Check(map.CheckConsistency(), "compound edge keeps bidirectional indices");
}
#endif
}  // namespace
// Run analytic and actual-image checks in both independent project builds.
int main() {
  try {
    TestGeometry();
    TestInput();
    TestMeasurements();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
