#pragma once
#include "vslam/io/dataset_io.h"
namespace vslam::io {
struct CloudFrame { DatasetEntry input; geometry::SE3 twc; };
// Colored RGB-D reconstruction in the estimated world frame, sampled every stride pixels.
std::size_t WriteRGBDCloud(const std::filesystem::path& path,const std::vector<CloudFrame>& frames,
                          const DatasetConfiguration& config,int stride=8);
void WriteSparseCloud(const std::filesystem::path& path,const core::MapSnapshot& snapshot);
}
