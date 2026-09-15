#include "vslam/tracking/frame_tracker.h"
#include <opencv2/calib3d.hpp>
#include <cmath>
#include <set>
#include <stdexcept>
namespace vslam::tracking {
FrameTracker::FrameTracker(sensor::CameraModel c,frontend::MatcherOptions m,
    TrackingOptions o,optimization::SolverOptions s):camera_(std::move(c)),matcher_(m),options_(o),optimizer_(camera_,s) {
  if(o.min_correspondences<6 || o.min_inliers<6 || o.pnp_iterations<1 ||
     !std::isfinite(o.pnp_error) || o.pnp_error<=0 || !std::isfinite(o.pnp_confidence) ||
     o.pnp_confidence<=0 || o.pnp_confidence>=1 || !std::isfinite(o.search_radius) ||
     o.search_radius<=0 || o.image_margin<0) throw std::invalid_argument("Invalid tracking options");
}
TrackingResult FrameTracker::Optimize(const core::Frame& f,const core::Map& map,
    const geometry::SE3& initial,const std::vector<Association>& associations) const {
  TrackingResult result;
  if(associations.size()<static_cast<std::size_t>(options_.min_correspondences)) {
    result.message="Insufficient tracking correspondences"; return result;
  }
  std::vector<optimization::PoseMeasurement> measurements;
  for(const auto& a:associations) {
    const auto& k=f.features().at(a.feature);
    measurements.push_back({map.GetMapPoint(a.point).position_w(),k.uv,k.depth_z_m});
  }
  result.optimization=optimizer_.Solve(initial,measurements);
  if(!result.optimization.usable) { result.message=result.optimization.message; return result; }
  for(std::size_t i=0;i<associations.size();++i)
    if(result.optimization.inliers[i]) result.associations.push_back(associations[i]);
  if(result.associations.size()<static_cast<std::size_t>(options_.min_inliers)) {
    result.associations.clear(); result.message="Too few optimized inliers"; return result;
  }
  result.tcw=result.optimization.tcw; result.success=true;
  return result;
}
TrackingResult FrameTracker::Track(const core::Frame& frame,const core::KeyFrame& ref,const core::Map& map) const {
  TrackingResult failed;
  const auto matches=matcher_.MatchDescriptors(ref.features().descriptors(),frame.features().descriptors());
  failed.matches=matches.size();
  std::vector<Association> associations; std::vector<cv::Point3d> xyz; std::vector<cv::Point2d> uv;
  std::set<common::MapPointId> used;
  for(const auto& m:matches) {
    const auto id=ref.associations().at(m.first);
    if(!id.valid() || map.GetMapPoint(id).is_bad() || !used.insert(id).second) continue;
    const auto& p=map.GetMapPoint(id).position_w(); const auto& q=frame.features().at(m.second).uv;
    xyz.emplace_back(p.x(),p.y(),p.z()); uv.emplace_back(q.x(),q.y());
    associations.push_back({m.second,id});
  }
  if(associations.size()<static_cast<std::size_t>(options_.min_correspondences)) {
    failed.message="Too few reference-map matches"; return failed;
  }
  cv::Mat k=(cv::Mat_<double>(3,3)<<camera_.fx(),0,camera_.cx(),0,camera_.fy(),camera_.cy(),0,0,1);
  cv::Mat rvec,tvec,inliers;
  const bool ok=cv::solvePnPRansac(xyz,uv,k,cv::noArray(),rvec,tvec,false,options_.pnp_iterations,
      static_cast<float>(options_.pnp_error),options_.pnp_confidence,inliers,cv::SOLVEPNP_EPNP);
  if(!ok || inliers.total()<static_cast<std::size_t>(options_.min_correspondences)) {
    failed.message="PnP RANSAC failed"; return failed;
  }
  cv::Mat rotation; cv::Rodrigues(rvec,rotation);
  Eigen::Matrix3d r; Eigen::Vector3d t;
  for(int i=0;i<3;++i) { t[i]=tvec.at<double>(i); for(int j=0;j<3;++j) r(i,j)=rotation.at<double>(i,j); }
  std::vector<Association> selected;
  for(int i=0;i<inliers.rows;++i) selected.push_back(associations.at(static_cast<std::size_t>(inliers.at<int>(i))));
  auto result=Optimize(frame,map,geometry::SE3(r,t),selected);
  result.matches=matches.size(); result.pnp_inliers=inliers.total();
  return result;
}
TrackingResult FrameTracker::RefineLocal(const core::Frame& frame,const core::Map& map,
    const std::vector<common::MapPointId>& local_points,const TrackingResult& seed) const {
  if(!seed.success || !seed.tcw) return seed;
  std::set<common::MapPointId> used_points; std::set<std::size_t> used_features;
  for(const auto& a:seed.associations) { used_points.insert(a.point); used_features.insert(a.feature); }
  cv::Mat descriptors;
  std::vector<common::MapPointId> visible; std::vector<Eigen::Vector2d> projections;
  for(auto id:local_points) {
    const auto& p=map.GetMapPoint(id);
    if(p.is_bad() || used_points.count(id)) continue;
    const auto uv=camera_.Project(*seed.tcw*p.position_w());
    if(!uv || uv->x()<options_.image_margin || uv->y()<options_.image_margin ||
       uv->x()>=camera_.width()-options_.image_margin || uv->y()>=camera_.height()-options_.image_margin) continue;
    auto d=p.descriptor(); if(d.empty()) continue;
    visible.push_back(id); projections.push_back(*uv); descriptors.push_back(d);
  }
  auto associations=seed.associations;
  for(const auto& m:matcher_.MatchDescriptors(descriptors,frame.features().descriptors())) {
    if(used_features.count(m.second) ||
       (projections[m.first]-frame.features().at(m.second).uv).norm()>options_.search_radius) continue;
    associations.push_back({m.second,visible[m.first]}); used_features.insert(m.second);
  }
  auto result=Optimize(frame,map,*seed.tcw,associations);
  result.matches=seed.matches; result.pnp_inliers=seed.pnp_inliers; result.visible_points=visible.size();
  return result;
}
}
