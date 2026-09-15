#pragma once
#include <filesystem>

#include "vslam/sensor/stereo_preprocessor.h"
#include "vslam/system/slam_system.h"
namespace vslam::io {
struct StereoConfiguration {
  sensor::StereoCamera camera;
  system::SlamConfig slam;
};
struct StereoEntry {
  std::int64_t left_ns;
  std::int64_t right_ns;
  std::filesystem::path left_path;
  std::filesystem::path right_path;
};
// Parse schema 2 for a horizontal rectified pair; unknown fields and malformed values throw.
StereoConfiguration ReadStereoConfiguration(const std::filesystem::path& path);
// Write every accepted configurable field, including effective defaults.
void WriteStereoConfiguration(const std::filesystem::path& path, const StereoConfiguration& config);
// Read an explicit schema header then left_ns right_ns left_path right_path rows.
// Require strict ordering in both streams; relative paths resolve against the manifest.
std::vector<StereoEntry> ReadStereoManifest(const std::filesystem::path& path);
// Decode both images as BGR8 and preserve original timestamps; missing images throw.
sensor::StereoFrameInput ReadStereoFrame(const StereoEntry& entry);
}  // namespace vslam::io
