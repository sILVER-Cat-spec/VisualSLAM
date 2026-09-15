#include "vslam/io/stereo_dataset_io.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
namespace {
// Keep checks active in Release builds.
void Check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}
// Reject malformed input through the public contract.
template<class Function> void Reject(Function function, const char* message) {
  bool rejected = false;
  try {
    function();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, message);
}
// Replace a temporary fixture while preserving real newlines and text endings.
void Write(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path);
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output << text;
}
}
// Validate independent schema 2 roundtrips and reject ambiguous or unordered manifests.
int main() {
  try {
    const auto root = std::filesystem::temp_directory_path() /
        (std::string("stereo_io_") + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directory(root);
    const auto camera = vslam::sensor::CameraModel(500, 500, 320, 240, 640, 480);
    const vslam::io::StereoConfiguration configuration{vslam::sensor::StereoCamera(camera, camera, 0.12), {}};
    const auto config_path = root / "camera.yaml";
    vslam::io::WriteStereoConfiguration(config_path, configuration);
    const auto decoded = vslam::io::ReadStereoConfiguration(config_path);
    Check(decoded.camera.baseline() == 0.12, "configuration preserves metric baseline");
    {
      std::ofstream append(config_path, std::ios::app);
      append << "typo_field: 1\n";
    }
    Reject([&] { vslam::io::ReadStereoConfiguration(config_path); }, "unknown config field");
    const auto manifest = root / "manifest.txt";
    Write(manifest, "# stereo_manifest_version=2\r\n1 1 left.png right.png\r\n2 2 next_left.png next_right.png\r\n");
    const auto entries = vslam::io::ReadStereoManifest(manifest);
    Check(entries.size() == 2 && entries[0].left_path == root / "left.png", "explicit stereo manifest paths");
    Write(manifest, "1 left.png depth.tiff\n");
    Reject([&] { vslam::io::ReadStereoManifest(manifest); }, "RGB-D manifest cannot be reinterpreted");
    Write(manifest, "# stereo_manifest_version=2\n1 1 a b\n2 1 c d\n");
    Reject([&] { vslam::io::ReadStereoManifest(manifest); }, "duplicate right timestamp");
    Write(manifest, "# stereo_manifest_version=2\n1 1 a b trailing\n");
    Reject([&] { vslam::io::ReadStereoManifest(manifest); }, "unexpected manifest field");
    std::filesystem::remove(manifest);
    std::filesystem::remove(config_path);
    std::filesystem::remove(root);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
