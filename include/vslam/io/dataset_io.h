#pragma once
#include "vslam/system/slam_system.h"
#include <filesystem>
namespace vslam::io {
struct DatasetConfiguration {
  sensor::CameraModel camera;
  system::SlamConfig slam;
  double depth_scale=1.0;
};
DatasetConfiguration ReadConfiguration(const std::filesystem::path& path);
void WriteConfiguration(const std::filesystem::path& path,const DatasetConfiguration& config);
struct DatasetEntry {
  std::int64_t timestamp_ns;
  std::filesystem::path rgb_path,depth_path;
};
std::vector<DatasetEntry> ReadManifest(const std::filesystem::path& path);
sensor::FrameInput ReadFrame(const DatasetEntry& entry,double depth_scale);
}
