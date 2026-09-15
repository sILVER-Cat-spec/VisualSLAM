#include "vslam/frontend/feature_extractor.h"
#include "vslam/initialization/rgbd_initializer.h"
#include "vslam/io/dataset_io.h"
#include "vslam/io/point_cloud_writer.h"
#include "vslam/sensor/image_preprocessor.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

// Try frames in order and stop at the first successful initial map; return nonzero on failure.
// Usage: initial_mapping_example CONFIG.yaml MANIFEST.txt NEW_OUTPUT_DIRECTORY.
// The output parent must exist. No tracking, local mapping, or BA is invoked.
int main(int argc, char** argv) {
  try {
    if (argc != 4) {
      std::cerr << "Usage: " << argv[0] << " CONFIG.yaml MANIFEST.txt NEW_OUTPUT_DIRECTORY\n";
      return 2;
    }
    const auto config = vslam::io::ReadConfiguration(argv[1]);
    const auto entries = vslam::io::ReadManifest(argv[2]);
    const std::filesystem::path output(argv[3]);
    if (entries.empty()) {
      throw std::invalid_argument("Manifest has no frames");
    }
    if (std::filesystem::exists(output)) {
      throw std::invalid_argument("Output directory already exists; choose a new directory");
    }
    cv::setNumThreads(1);
    cv::setRNGSeed(73);

    const vslam::frontend::FeatureExtractor extractor(config.slam.frontend);
    vslam::core::FrameFactory frames(0, config.slam.time_base);

    for (std::size_t frame_index = 0; frame_index < entries.size(); ++frame_index) {
      std::cout << "Initialization attempt at dataset index " << frame_index << '\n';
      std::cout << "[1] Read and prepare one RGB-D frame.\n";
      const auto input = vslam::io::ReadFrame(entries[frame_index], config.depth_scale);
      const auto prepared = vslam::sensor::Prepare(input, config.camera);

      // Attach sampled camera-axis depth in meters to each detected feature.
      const auto features = extractor.Extract(prepared.image_rgb_or_gray, prepared.depth_z_m);
      auto frame = frames.Create(prepared.timestamp, prepared.image_rgb_or_gray,
                                 prepared.depth_z_m, prepared.external_label);
      frame.SetFeatures(features);
      std::cout << "[2] Extract features and sample depth: " << features.size() << " features.\n";

      std::cout << "[3] Back-project valid measurements; set Tcw = identity;\n"
                   "    add map points, associate features, insert and protect the first keyframe.\n";
      auto initialized = vslam::initialization::InitializeRGBD(
          frame, config.camera, config.slam.min_initial_points,
          config.slam.frontend.min_depth, config.slam.frontend.max_depth);
      if (!initialized.map) {
        std::cerr << "Frame could not initialize: insufficient valid support"
                     " or degenerate geometry. Required points: "
                  << config.slam.min_initial_points << '\n';
        continue;
      }

      const auto& map = *initialized.map;
      if (!map.CheckConsistency() || map.num_keyframes() != 1 ||
          map.num_map_points() != initialized.points ||
          map.num_observations() != initialized.points) {
        throw std::runtime_error("Initial map graph is inconsistent");
      }
      const auto snapshot = map.Snapshot();

      // Save only after successful initialization; never overwrite an existing run.
      if (!std::filesystem::create_directory(output)) {
        throw std::runtime_error("Could not create a new output directory");
      }
      vslam::io::WriteSparseCloud(output / "initial_map.ply", snapshot);
      std::ofstream measurements;
      measurements.exceptions(std::ios::failbit | std::ios::badbit);
      measurements.open(output / "initial_observations.csv");
      measurements << std::setprecision(17);
      measurements << "feature_index,u,v,depth_z_m,keyframe_id,map_point_id,x_m,y_m,z_m\n";
      for (const auto& observation : snapshot.observations) {
        const auto index = observation.feature_index();
        const auto& feature = frame.features().at(index);
        const auto& position = map.GetMapPoint(observation.map_point_id()).position_w();
        measurements << index << ',' << feature.uv.x() << ',' << feature.uv.y() << ','
                     << *feature.depth_z_m << ',';

        measurements << initialized.keyframe.value() << ',' << observation.map_point_id().value()
                     << ',' << position.x() << ',' << position.y() << ',' << position.z() << '\n';
      }
      measurements.close();

      std::cout << "[4] Initial map: " << map.num_keyframes() << " keyframe, "
                << map.num_map_points() << " points, " << map.num_observations()
                << " observations.\n"
                << "World frame = initialized camera optical frame; coordinates are in meters.\n"
                << "Wrote " << output << "/initial_map.ply and initial_observations.csv\n";
      return 0;
    }
    std::cerr << "No frame could initialize the map.\n";
    return 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
