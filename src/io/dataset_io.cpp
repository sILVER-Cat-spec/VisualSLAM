#include "vslam/io/dataset_io.h"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <sstream>
#include <set>
#include <stdexcept>
namespace vslam::io {
namespace {
template<class Function> void Fields(system::SlamConfig& c,double& scale,Function f) {
  f("depth_scale",scale);
  f("n_features",c.frontend.n_features); f("scale_factor",c.frontend.scale_factor);
  f("n_levels",c.frontend.n_levels); f("fast_threshold",c.frontend.fast_threshold);
  f("min_depth",c.frontend.min_depth); f("max_depth",c.frontend.max_depth); f("depth_radius",c.frontend.depth_radius);
  f("match_ratio",c.matcher.ratio);
  f("min_correspondences",c.tracking.min_correspondences); f("tracking_min_inliers",c.tracking.min_inliers);
  f("pnp_iterations",c.tracking.pnp_iterations); f("pnp_error",c.tracking.pnp_error);
  f("pnp_confidence",c.tracking.pnp_confidence); f("search_radius",c.tracking.search_radius); f("image_margin",c.tracking.image_margin);
  f("keyframe_min_gap",c.mapping.min_gap); f("keyframe_max_gap",c.mapping.max_gap); f("keyframe_min_inliers",c.mapping.min_inliers);
  f("min_translation",c.mapping.min_translation); f("min_rotation_deg",c.mapping.min_rotation_deg);
  f("max_neighbors",c.mapping.max_neighbors); f("max_keyframes",c.mapping.max_keyframes);
  f("max_ba_points",c.mapping.max_ba_points); f("max_boundary_keyframes",c.mapping.max_boundary_keyframes);
  f("max_ba_observations",c.mapping.max_ba_observations); f("max_new_points",c.mapping.max_new_points);
  f("point_grace_period",c.mapping.point_grace_period); f("min_parallax_deg",c.mapping.min_parallax_deg);
  f("max_reprojection_error",c.mapping.max_reprojection_error);
  f("pose_iterations",c.pose_solver.max_iterations); f("pose_pixel_sigma",c.pose_solver.pixel_sigma);
  f("pose_depth_sigma",c.pose_solver.depth_sigma_m); f("pose_huber_delta",c.pose_solver.huber_delta);
  f("pose_outlier_pixels",c.pose_solver.outlier_pixels); f("pose_outlier_depth_sigma",c.pose_solver.outlier_depth_sigma);
  f("ba_iterations",c.ba_solver.max_iterations); f("ba_pixel_sigma",c.ba_solver.pixel_sigma);
  f("ba_depth_sigma",c.ba_solver.depth_sigma_m); f("ba_huber_delta",c.ba_solver.huber_delta);
  f("ba_outlier_pixels",c.ba_solver.outlier_pixels); f("ba_outlier_depth_sigma",c.ba_solver.outlier_depth_sigma);
}
}
DatasetConfiguration ReadConfiguration(const std::filesystem::path& path) {
  cv::FileStorage fs(path.string(),cv::FileStorage::READ);
  if(!fs.isOpened()) throw std::runtime_error("Cannot open configuration");
  for(const char* name:{"schema_version","fx","fy","cx","cy","width","height"})
    if(fs[name].empty()) throw std::invalid_argument(std::string("Missing configuration field: ")+name);
  if(static_cast<int>(fs["schema_version"])!=1) throw std::invalid_argument("Unsupported configuration schema");
  DatasetConfiguration config{sensor::CameraModel(static_cast<double>(fs["fx"]),static_cast<double>(fs["fy"]),
      static_cast<double>(fs["cx"]),static_cast<double>(fs["cy"]),static_cast<int>(fs["width"]),static_cast<int>(fs["height"])),{},1.0};
  std::set<std::string> names={"schema_version","fx","fy","cx","cy","width","height","descriptor","mutual","min_initial_points"};
  Fields(config.slam,config.depth_scale,[&](const char* name,auto& value) {
    names.insert(name); if(!fs[name].empty()) fs[name]>>value;
  });
  std::string descriptor="ORB"; if(!fs["descriptor"].empty()) fs["descriptor"]>>descriptor;
  if(descriptor=="ORB") config.slam.frontend.kind=core::DescriptorKind::Orb;
  else if(descriptor=="SIFT") config.slam.frontend.kind=core::DescriptorKind::Sift;
  else throw std::invalid_argument("Descriptor must be ORB or SIFT");
  if(!fs["mutual"].empty()) {
    const int value=static_cast<int>(fs["mutual"]);
    if(value!=0 && value!=1) throw std::invalid_argument("mutual must be 0 or 1");
    config.slam.matcher.mutual=value!=0;
  }
  if(!fs["min_initial_points"].empty()) {
    int value=static_cast<int>(fs["min_initial_points"]);
    if(value<6) throw std::invalid_argument("min_initial_points must be >=6");
    config.slam.min_initial_points=static_cast<std::size_t>(value);
  }
  for(const auto& node:fs.root()) if(!names.count(node.name())) throw std::invalid_argument("Unknown configuration field: "+node.name());
  if(!std::isfinite(config.depth_scale) || config.depth_scale<=0) throw std::invalid_argument("Invalid depth scale");
  // Construction validates every pipeline option without processing or changing data.
  system::SlamSystem validation(config.camera,config.slam);
  return config;
}
void WriteConfiguration(const std::filesystem::path& path,const DatasetConfiguration& config) {
  cv::FileStorage fs(path.string(),cv::FileStorage::WRITE);
  if(!fs.isOpened()) throw std::runtime_error("Cannot write effective configuration");
  fs<<"schema_version"<<1<<"fx"<<config.camera.fx()<<"fy"<<config.camera.fy()
    <<"cx"<<config.camera.cx()<<"cy"<<config.camera.cy()<<"width"<<config.camera.width()<<"height"<<config.camera.height();
  fs<<"descriptor"<<(config.slam.frontend.kind==core::DescriptorKind::Orb?"ORB":"SIFT")
    <<"mutual"<<static_cast<int>(config.slam.matcher.mutual)
    <<"min_initial_points"<<static_cast<int>(config.slam.min_initial_points);
  auto settings=config.slam; double scale=config.depth_scale;
  Fields(settings,scale,[&](const char* name,auto& value) { fs<<name<<value; });
}
std::vector<DatasetEntry> ReadManifest(const std::filesystem::path& path) {
  std::ifstream input(path);
  if(!input) throw std::runtime_error("Cannot read manifest");
  std::vector<DatasetEntry> result; std::string line;
  while(std::getline(input,line)) {
    if(line.empty() || line[0]=='#') continue;
    std::istringstream row(line); DatasetEntry entry; std::string rgb,depth,extra;
    if(!(row>>entry.timestamp_ns>>rgb>>depth) || (row>>extra) || entry.timestamp_ns<0 ||
       (!result.empty() && entry.timestamp_ns<=result.back().timestamp_ns))
      throw std::invalid_argument("Manifest requires increasing timestamp_ns rgb_path depth_path rows");
    entry.rgb_path=path.parent_path()/rgb; entry.depth_path=path.parent_path()/depth;
    result.push_back(entry);
  }
  if(result.empty()) throw std::invalid_argument("Empty manifest");
  return result;
}
sensor::FrameInput ReadFrame(const DatasetEntry& entry,double depth_scale) {
  sensor::FrameInput frame; frame.timestamp={entry.timestamp_ns,common::TimeBase::Simulation};
  frame.image=cv::imread(entry.rgb_path.string(),cv::IMREAD_COLOR);
  frame.depth=cv::imread(entry.depth_path.string(),cv::IMREAD_UNCHANGED);
  if(frame.image.empty() || frame.depth.empty()) throw std::runtime_error("Cannot decode dataset images");
  frame.color_format=sensor::ColorFormat::BGR; frame.rectified=true;
  frame.registration=sensor::DepthRegistration::RegisteredToColor;
  frame.meters_per_depth_unit=depth_scale; frame.external_label=entry.rgb_path.filename().string();
  return frame;
}
}
