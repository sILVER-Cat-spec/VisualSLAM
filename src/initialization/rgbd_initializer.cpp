#include "vslam/initialization/rgbd_initializer.h"
namespace vslam::initialization {
InitializationResult InitializeRGBD(core::Frame& frame,const sensor::CameraModel& camera,
    std::size_t min_points,double min_depth,double max_depth) {
  InitializationResult result;
  std::vector<std::pair<std::size_t,Eigen::Vector3d>> candidates;
  for(std::size_t i=0;i<frame.features().size();++i) {
    const auto& k=frame.features().at(i);
    if(!k.depth_z_m || *k.depth_z_m<min_depth || *k.depth_z_m>max_depth) continue;
    const auto p=camera.Unproject(k.uv,*k.depth_z_m);
    if(p) candidates.push_back({i,*p});
  }
  if(candidates.size()<min_points) return result;
  core::Frame staged(frame);
  staged.SetPose(geometry::SE3());
  auto map=std::make_unique<core::Map>(frame.features().kind(),frame.id().epoch());
  for(const auto& item:candidates) {
    const auto id=map->AddMapPoint(item.second,frame.features().Descriptor(item.first));
    staged.Associate(item.first,id);
  }
  result.keyframe=map->InsertKeyFrame(staged);
  map->ProtectKeyFrame(result.keyframe);
  result.points=candidates.size(); result.map=std::move(map);
  frame=std::move(staged);
  return result;
}
}
