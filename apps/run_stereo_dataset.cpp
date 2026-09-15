#include "vslam/io/point_cloud_writer.h"
#include "vslam/io/stereo_dataset_io.h"
#ifdef VSLAM_DEPTH_ROUTE
#include "vslam/sensor/stereo_depth_adapter.h"
#endif
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#ifdef __linux__
#include <sys/resource.h>
#endif
namespace {
using Clock = std::chrono::steady_clock;
// Open output with exceptions so disk failures cannot be reported as successful runs.
std::ofstream Output(const std::filesystem::path& path) {
  std::ofstream file(path);
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file << std::setprecision(17);
  return file;
}
// Measure wall time in seconds with a monotonic clock.
double Elapsed(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
// Report a reproducible nearest-rank percentile for a nonempty sample vector.
double Percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  return values.at(static_cast<std::size_t>(std::ceil(fraction * values.size())) - 1);
}
}  // namespace

// Run one complete manifest without subsampling; output poses are left-camera-to-world TUM.
int main(int argc, char** argv) {
  try {
    if (argc != 4) {
      std::cerr << "Usage: " << argv[0] << " CONFIG.yaml MANIFEST.txt NEW_OUTPUT_DIRECTORY\n";
      return 2;
    }
    cv::setNumThreads(1);
    cv::setRNGSeed(73);
    const auto config = vslam::io::ReadStereoConfiguration(argv[1]);
    const auto entries = vslam::io::ReadStereoManifest(argv[2]);
    const std::filesystem::path output(argv[3]);
    if (std::filesystem::exists(output)) {
      throw std::invalid_argument("Output directory already exists");
    }
    std::filesystem::create_directories(output);
#ifdef VSLAM_DEPTH_ROUTE
    const char* route = "A0_sgbm_rgbd";
    vslam::system::SlamSystem slam(config.camera.left(), config.slam);
#else
    const char* route = "B_native_stereo";
    vslam::system::SlamSystem slam(config.camera, config.slam);
#endif
    vslam::io::WriteStereoConfiguration(output / "effective_config.yaml", config);
    auto defaults = Output(output / "fixed_settings.yaml");
    defaults << "schema_version: 2\n";
    defaults << "temporal_match_ratio: " << config.slam.matcher.ratio << '\n';
    defaults << "temporal_mutual: " << config.slam.matcher.mutual << '\n';
    defaults << "feature_scale_factor: " << config.slam.frontend.scale_factor << '\n';
    defaults << "feature_levels: " << config.slam.frontend.n_levels << '\n';
    defaults << "fast_threshold: " << config.slam.frontend.fast_threshold << '\n';
    defaults << "pnp_iterations: " << config.slam.tracking.pnp_iterations << '\n';
    defaults << "pnp_error_px: " << config.slam.tracking.pnp_error << '\n';
    defaults << "pnp_confidence: " << config.slam.tracking.pnp_confidence << '\n';
    defaults << "tracking_min_inliers: " << config.slam.tracking.min_inliers << '\n';
    defaults << "mapping_neighbors: " << config.slam.mapping.max_neighbors << '\n';
    defaults << "mapping_min_translation_m: " << config.slam.mapping.min_translation << '\n';
    defaults << "mapping_min_rotation_deg: " << config.slam.mapping.min_rotation_deg << '\n';
    defaults << "mapping_min_parallax_deg: " << config.slam.mapping.min_parallax_deg << '\n';
    defaults << "mapping_max_reprojection_px: " << config.slam.mapping.max_reprojection_error << '\n';
    defaults << "ba_observation_budget: " << config.slam.mapping.max_ba_observations << '\n';
    defaults << "pose_pixel_sigma: " << config.slam.pose_solver.pixel_sigma << '\n';
    defaults << "pose_huber_delta: " << config.slam.pose_solver.huber_delta << '\n';
    defaults << "pose_outlier_pixels: " << config.slam.pose_solver.outlier_pixels << '\n';
    defaults << "ba_pixel_sigma: " << config.slam.ba_solver.pixel_sigma << '\n';
    defaults << "ba_huber_delta: " << config.slam.ba_solver.huber_delta << '\n';
    defaults << "ba_outlier_pixels: " << config.slam.ba_solver.outlier_pixels << '\n';
    defaults << "minimum_corrected_disparity_px: 0.5\nmaximum_epipolar_error_px: 2\n";
#ifdef VSLAM_DEPTH_ROUTE
    defaults << "depth_sigma_m: " << config.slam.pose_solver.depth_sigma_m << '\n';
    defaults << "depth_radius: 0\nsgbm_min_disparity: 0\nsgbm_num_disparities: 128\nsgbm_block_size: 5\n";
    defaults << "sgbm_uniqueness: 10\nsgbm_speckle_window: 100\nsgbm_speckle_range: 2\n";
    defaults << "reverse_consistency_px: 1\n";
#else
    defaults << "stereo_descriptor_ratio: 0.75\npatch_width: 9\npatch_search_radius: 3\n";
    defaults << "patch_max_mean_squared_error: 600\npatch_min_curvature: 1\n";
    defaults << "stereo_left_sigma_px: 1\nstereo_right_sigma_px: 1\n";
#endif
    auto trajectory = Output(output / "trajectory_tum.txt");
    auto frames = Output(output / "frames.csv");
    auto ba_log = Output(output / "ba.log");
    frames << "schema_version,left_ns,right_ns,state,pose_valid,features,inliers,map_points,"
              "keyframes,ba_committed,input_support_ratio,read_seconds,adapter_seconds,slam_"
              "seconds,total_seconds,tracking_message\n";
    ba_log << "left_ns,usable,committed,iterations,initial_cost,final_cost,message\n";
    std::size_t successes = 0;
    std::size_t rejected = 0;
    std::size_t ba_attempts = 0;
    std::size_t ba_commits = 0;
    std::size_t lost_events = 0;
    std::size_t recoveries = 0;
    std::int64_t initialized_ns = -1;
    bool was_lost = false;
    std::map<std::string, std::vector<double>> stages;
    std::vector<double> totals;
    std::vector<double> adapters;
    std::vector<double> processing;
    for (const auto& entry : entries) {
      const auto start = Clock::now();
      const auto input = vslam::io::ReadStereoFrame(entry);
      const double read_seconds = Elapsed(start);
      const auto algorithm_start = Clock::now();
      double support_ratio = 0;
      double adapter_seconds = 0;
#ifdef VSLAM_DEPTH_ROUTE
      const auto pair = vslam::sensor::PrepareStereo(input, config.camera);
      vslam::sensor::StereoDepthOptions options;
      options.min_depth = config.slam.frontend.min_depth;
      options.max_depth = config.slam.frontend.max_depth;
      const auto adapted = vslam::sensor::EstimateStereoDepth(pair, config.camera, options);
      adapter_seconds = Elapsed(algorithm_start);
      support_ratio = adapted.valid_ratio;
      const auto result = slam.Process(adapted.frame);
#else
      const auto result = slam.Process(input);
      if (result.frame && result.features > 0) {
        std::size_t stereo_count = 0;
        for (const auto& feature : result.frame->features().keypoints()) {
          stereo_count += feature.stereo.has_value();
        }
        support_ratio = double(stereo_count) / result.features;
      }
#endif
      const double total_seconds = Elapsed(algorithm_start);
      stages["prepare"].push_back(result.prepare_seconds);
      stages["frontend"].push_back(result.frontend_seconds);
      stages["initialization"].push_back(result.initialization_seconds);
      stages["tracking"].push_back(result.tracking_seconds);
      stages["mapping_including_ba"].push_back(result.mapping_seconds);
      stages["ba"].push_back(result.ba_seconds);
      stages["external_adapter"].push_back(adapter_seconds);
      stages["disk_read"].push_back(read_seconds);
      stages["algorithm_total"].push_back(total_seconds);
      totals.push_back(total_seconds);
      adapters.push_back(adapter_seconds);
      processing.push_back(result.elapsed_seconds);
      rejected += !result.status.ok();
      if (!result.status.ok()) {
        std::cerr << "Rejected frame " << entry.left_ns << '\n';
      }
      const bool lost = result.state == vslam::system::SlamState::Lost;
      lost_events += lost && !was_lost;
      recoveries += was_lost && result.tcw.has_value();
      was_lost = lost;
      if (result.tcw) {
        ++successes;
        if (initialized_ns < 0) {
          initialized_ns = entry.left_ns;
        }
        const auto twc = result.tcw->Inverse();
        const Eigen::Quaterniond quaternion(twc.rotation());
        trajectory << entry.left_ns / 1e9 << ' ' << twc.translation().transpose() << ' '
                   << quaternion.x() << ' ' << quaternion.y() << ' ' << quaternion.z() << ' '
                   << quaternion.w() << '\n';
      }
      if (result.mapping) {
        ++ba_attempts;
        ba_commits += result.ba_committed;
        const auto& ba = result.mapping->ba;
        ba_log << entry.left_ns << ',' << ba.usable << ',' << result.ba_committed << ','
               << ba.iterations << ',' << ba.initial_cost << ',' << ba.final_cost << ",\""
               << ba.message << "\"\n";
      }
      frames << 2 << ',' << entry.left_ns << ',' << entry.right_ns << ','
             << vslam::system::StateName(result.state) << ',' << result.tcw.has_value() << ','
             << result.features << ',' << result.inliers << ',' << result.map_points << ','
             << result.keyframes << ',' << result.ba_committed << ',' << support_ratio << ','
             << read_seconds << ',' << adapter_seconds << ',' << result.elapsed_seconds << ','
             << total_seconds << ",\"" << result.tracking.message << "\"\n";
      if (!slam.map().CheckConsistency()) {
        throw std::runtime_error("Map consistency failed");
      }
    }
    auto timings = Output(output / "stage_timings.csv");
    timings << "stage,all_frame_samples,median_seconds,p95_seconds,active_samples,active_median_seconds,active_p95_seconds\n";
    for (const auto& stage : stages) {
      std::vector<double> active;
      for (double value : stage.second) {
        if (value > 0) {
          active.push_back(value);
        }
      }
      timings << stage.first << ',' << stage.second.size() << ',' << Percentile(stage.second, 0.5)
              << ',' << Percentile(stage.second, 0.95) << ',' << active.size() << ','
              << (active.empty() ? 0 : Percentile(active, 0.5)) << ','
              << (active.empty() ? 0 : Percentile(active, 0.95)) << '\n';
    }
    const auto snapshot = slam.map().Snapshot();
    vslam::io::WriteSparseCloud(output / "sparse_map.ply", snapshot);
    auto points = Output(output / "map_points.csv");
    points << "id,x_m,y_m,z_m,is_bad\n";
    for (const auto& point : snapshot.points) {
      points << point.id.value() << ',' << point.position_w.x() << ',' << point.position_w.y()
             << ',' << point.position_w.z() << ',' << point.is_bad << '\n';
    }
    auto keyframes = Output(output / "keyframes.csv");
    keyframes << "id,left_ns,x_m,y_m,z_m,qx,qy,qz,qw\n";
    for (const auto& key : snapshot.keyframes) {
      const auto twc = key.tcw.Inverse();
      const Eigen::Quaterniond q(twc.rotation());
      keyframes << key.id.value() << ',' << key.timestamp.ns << ',' << twc.translation().x() << ','
                << twc.translation().y() << ',' << twc.translation().z() << ',' << q.x() << ','
                << q.y() << ',' << q.z() << ',' << q.w() << '\n';
    }
    long peak_rss_kib = 0;
#ifdef __linux__
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    peak_rss_kib = usage.ru_maxrss;
#endif
    auto summary = Output(output / "summary.json");
    summary
        << "{\n  \"schema_version\": 2,\n  \"route\": \"" << route << "\",\n"
        << "  \"frames\": " << entries.size() << ",\n  \"successful_poses\": " << successes
        << ",\n  \"coverage\": " << double(successes) / entries.size()
        << ",\n  \"rejected_frames\": " << rejected
        << ",\n  \"initialization_ns\": " << initialized_ns
        << ",\n  \"lost_events\": " << lost_events << ",\n  \"recoveries\": " << recoveries
        << ",\n  \"map_points\": " << snapshot.points.size()
        << ",\n  \"keyframes\": " << snapshot.keyframes.size()
        << ",\n  \"ba_attempts\": " << ba_attempts << ",\n  \"ba_commits\": " << ba_commits
        << ",\n  \"total_median_seconds\": " << Percentile(totals, 0.5)
        << ",\n  \"total_p95_seconds\": " << Percentile(totals, 0.95)
        << ",\n  \"adapter_median_seconds\": " << Percentile(adapters, 0.5)
        << ",\n  \"slam_median_seconds\": " << Percentile(processing, 0.5)
        << ",\n  \"peak_rss_kib\": " << peak_rss_kib
        << ",\n  \"timing_excludes_disk_read\": true,\n  \"seed\": 73,\n  \"opencv_threads\": 1,\n"
        << "  \"opencv_version\": \"" << CV_VERSION << "\"\n}\n";
    std::cout << route << ": " << successes << '/' << entries.size() << " poses, " << ba_commits
              << " BA commits\n";
    return successes > 0 && rejected == 0 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
