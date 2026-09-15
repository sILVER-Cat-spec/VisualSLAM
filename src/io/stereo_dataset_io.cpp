#include "vslam/io/stereo_dataset_io.h"

#include <cmath>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
namespace vslam::io {
namespace {
// Read a finite scalar without accepting string coercion or missing fields.
double Number(const cv::FileStorage& file, const char* name) {
  const auto node = file[name];
  if ((!node.isInt() && !node.isReal()) || !std::isfinite(static_cast<double>(node))) {
    throw std::invalid_argument(std::string("Missing or invalid numeric field: ") + name);
  }
  return static_cast<double>(node);
}
// Require an integral scalar in a bounded range before narrowing it.
int Integer(const cv::FileStorage& file, const char* name, int low, int high) {
  const double value = Number(file, name);
  if (value != std::floor(value) || value < low || value > high) {
    throw std::invalid_argument(std::string("Integer out of range: ") + name);
  }
  return static_cast<int>(value);
}
}  // namespace
// A separate schema prevents accidental interpretation of a right image as RGB-D depth.
StereoConfiguration ReadStereoConfiguration(const std::filesystem::path& path) {
  cv::FileStorage file(path.string(), cv::FileStorage::READ);
  if (!file.isOpened()) {
    throw std::invalid_argument("Cannot open stereo configuration");
  }
  const std::set<std::string> allowed = {"schema_version",
                                         "rectified",
                                         "fx",
                                         "fy",
                                         "cx",
                                         "cy",
                                         "right_cx",
                                         "width",
                                         "height",
                                         "baseline_m",
                                         "n_features",
                                         "min_initial_points",
                                         "min_depth",
                                         "max_depth",
                                         "keyframe_min_gap",
                                         "keyframe_max_gap",
                                         "max_ba_points",
                                         "max_keyframes",
                                         "max_new_points",
                                         "pose_iterations",
                                         "ba_iterations"};
  std::set<std::string> seen;
  for (const auto& node : file.root()) {
    if (!allowed.count(node.name()) || !seen.insert(node.name()).second) {
      throw std::invalid_argument("Unknown stereo configuration field: " + node.name());
    }
  }
  if (Integer(file, "schema_version", 2, 2) != 2 || Integer(file, "rectified", 1, 1) != 1) {
    throw std::invalid_argument("Runner requires schema 2 rectified input");
  }
  const double fx = Number(file, "fx");
  const double fy = Number(file, "fy");
  const double cx = Number(file, "cx");
  const double cy = Number(file, "cy");
  const int width = Integer(file, "width", 16, 16384);
  const int height = Integer(file, "height", 16, 16384);
  sensor::CameraModel left(fx, fy, cx, cy, width, height);
  sensor::CameraModel right(fx, fy, Number(file, "right_cx"), cy, width, height);
  StereoConfiguration config{sensor::StereoCamera(left, right, Number(file, "baseline_m")), {}};
  auto& slam = config.slam;
  slam.frontend.n_features = Integer(file, "n_features", 6, 100000);
  slam.min_initial_points = Integer(file, "min_initial_points", 6, 100000);
  slam.frontend.min_depth = Number(file, "min_depth");
  slam.frontend.max_depth = Number(file, "max_depth");
  slam.frontend.depth_radius = 0;
  slam.mapping.min_gap = Integer(file, "keyframe_min_gap", 1, 100000);
  slam.mapping.max_gap = Integer(file, "keyframe_max_gap", slam.mapping.min_gap, 100000);
  slam.mapping.max_ba_points = Integer(file, "max_ba_points", 6, 100000);
  slam.mapping.max_keyframes = Integer(file, "max_keyframes", 2, 1000);
  slam.mapping.max_new_points = Integer(file, "max_new_points", 1, 100000);
  slam.pose_solver.max_iterations = Integer(file, "pose_iterations", 1, 1000);
  slam.ba_solver.max_iterations = Integer(file, "ba_iterations", 1, 1000);
  if (slam.frontend.min_depth <= 0 || slam.frontend.max_depth <= slam.frontend.min_depth) {
    throw std::invalid_argument("Invalid metric depth interval");
  }
  return config;
}

// Persist configurable values; fixed algorithm settings are versioned in the runner report.
void WriteStereoConfiguration(const std::filesystem::path& path,
                              const StereoConfiguration& config) {
  cv::FileStorage file(path.string(), cv::FileStorage::WRITE);
  if (!file.isOpened()) {
    throw std::runtime_error("Cannot write effective stereo configuration");
  }
  const auto& camera = config.camera;
  const auto& slam = config.slam;
  file << "schema_version" << 2 << "rectified" << 1;
  file << "fx" << camera.fx() << "fy" << camera.fy() << "cx" << camera.cx() << "cy" << camera.cy();
  file << "right_cx" << camera.right().cx() << "baseline_m" << camera.baseline();
  file << "width" << camera.width() << "height" << camera.height();
  file << "n_features" << slam.frontend.n_features << "min_initial_points"
       << int(slam.min_initial_points);
  file << "min_depth" << slam.frontend.min_depth << "max_depth" << slam.frontend.max_depth;
  file << "keyframe_min_gap" << slam.mapping.min_gap << "keyframe_max_gap" << slam.mapping.max_gap;
  file << "max_ba_points" << slam.mapping.max_ba_points << "max_keyframes"
       << slam.mapping.max_keyframes;
  file << "max_new_points" << slam.mapping.max_new_points;
  file << "pose_iterations" << slam.pose_solver.max_iterations << "ba_iterations"
       << slam.ba_solver.max_iterations;
}

// Pair by recorded capture stamps, never row numbers from separate image lists.
std::vector<StereoEntry> ReadStereoManifest(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::string line;
  if (!std::getline(input, line)) {
    throw std::invalid_argument("Missing stereo manifest header");
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != "# stereo_manifest_version=2") {
    throw std::invalid_argument("Missing stereo manifest version 2 header");
  }
  std::vector<StereoEntry> entries;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream row(line);
    StereoEntry entry;
    std::string left;
    std::string right;
    std::string extra;
    if (!(row >> entry.left_ns >> entry.right_ns >> left >> right) || (row >> extra) ||
        entry.left_ns < 0 || entry.right_ns < 0 ||
        (!entries.empty() &&
         (entry.left_ns <= entries.back().left_ns || entry.right_ns <= entries.back().right_ns))) {
      throw std::invalid_argument("Invalid or unordered stereo manifest row");
    }
    entry.left_path = path.parent_path() / left;
    entry.right_path = path.parent_path() / right;
    entries.push_back(entry);
  }
  if (entries.empty()) {
    throw std::invalid_argument("Empty stereo manifest");
  }
  return entries;
}

// OpenCV decoding normalizes storage formats; the preprocessor validates calibrated dimensions.
sensor::StereoFrameInput ReadStereoFrame(const StereoEntry& entry) {
  sensor::StereoFrameInput input;
  // This reader is dedicated to schema 2 datasets explicitly declared rectified.
  input.rectified = true;
  input.timestamp = {entry.left_ns, common::TimeBase::Simulation};
  input.right_timestamp = {entry.right_ns, common::TimeBase::Simulation};
  input.image = cv::imread(entry.left_path.string(), cv::IMREAD_COLOR);
  input.right_image = cv::imread(entry.right_path.string(), cv::IMREAD_COLOR);
  input.external_label = entry.left_path.filename().string();
  if (input.image.empty() || input.right_image.empty()) {
    throw std::invalid_argument("Cannot decode a complete stereo pair");
  }
  return input;
}
}  // namespace vslam::io
