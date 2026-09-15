#include <chrono>
#include "vslam/mapping/local_mapper.h"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
namespace vslam::mapping {
std::vector<common::KeyFrameId> SelectNeighbors(const core::Map& map,common::KeyFrameId seed,int limit) {
  std::map<common::KeyFrameId,int> weights;
  for(auto id:map.GetKeyFrame(seed).associations()) if(id.valid() && !map.GetMapPoint(id).is_bad())
    for(const auto& edge:map.GetMapPoint(id).observations())
      if(edge.first!=seed && !map.GetObservation(edge.second).is_outlier()) ++weights[edge.first];
  std::vector<std::pair<common::KeyFrameId,int>> candidates;
  for(const auto& k:map.Snapshot().keyframes) if(k.id!=seed) candidates.push_back({k.id,weights[k.id]});
  std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b) {
    return a.second!=b.second ? a.second>b.second : b.first<a.first;
  });
  std::vector<common::KeyFrameId> result;
  for(const auto& c:candidates) if(static_cast<int>(result.size())<limit) result.push_back(c.first);
  return result;
}
std::vector<common::MapPointId> CollectPoints(const core::Map& map,const std::vector<common::KeyFrameId>& keys) {
  std::set<common::MapPointId> ids;
  for(auto key:keys) for(auto id:map.GetKeyFrame(key).associations())
    if(id.valid() && !map.GetMapPoint(id).is_bad()) ids.insert(id);
  return {ids.begin(),ids.end()};
}
bool ShouldInsert(const core::Frame& frame,const core::KeyFrame& last,std::size_t inliers,const MappingOptions& o) {
  if(!frame.pose() || frame.sequence()<last.source_sequence()) return false;
  auto gap=frame.sequence()-last.source_sequence();
  if(gap<static_cast<std::uint64_t>(o.min_gap)) return false;
  if(gap>=static_cast<std::uint64_t>(o.max_gap)) return true;
  const double translation=(frame.pose()->Inverse().translation()-last.pose().Inverse().translation()).norm();
  const double cosine=std::clamp(((frame.pose()->rotation()*last.pose().rotation().transpose()).trace()-1)*0.5,-1.0,1.0);
  return translation>=o.min_translation || std::acos(cosine)*180/3.141592653589793>=o.min_rotation_deg ||
         inliers<static_cast<std::size_t>(o.min_inliers);
}
LocalMapper::LocalMapper(sensor::CameraModel c,frontend::MatcherOptions m,MappingOptions o,
    optimization::SolverOptions s):camera_(std::move(c)),matcher_(m),options_(o),solver_options_(s),ba_(camera_,s) {
  if(o.min_gap<1 || o.max_gap<o.min_gap || o.min_inliers<1 || o.max_neighbors<1 ||
     o.max_keyframes<2 || o.max_ba_points<6 || o.max_boundary_keyframes<1 ||
     o.max_ba_observations<12 || o.max_new_points<1 || o.point_grace_period<0 ||
     !std::isfinite(o.min_translation) || o.min_translation<0 ||
     !std::isfinite(o.min_rotation_deg) || o.min_rotation_deg<0 ||
     !std::isfinite(o.min_parallax_deg) || o.min_parallax_deg<=0 ||
     !std::isfinite(o.max_reprojection_error) || o.max_reprojection_error<=0)
    throw std::invalid_argument("Invalid mapping options");
}
bool LocalMapper::ValidProjection(const core::KeyFrame& k,std::size_t i,const Eigen::Vector3d& p) const {
  const auto pc=k.pose()*p;
  const auto uv=camera_.Project(pc);
  const auto& measurement=k.features().at(i);
  return uv && (*uv-measurement.uv).norm()<=options_.max_reprojection_error &&
         (!measurement.depth_z_m || std::abs(pc.z()-*measurement.depth_z_m)<=solver_options_.outlier_depth_sigma*solver_options_.depth_sigma_m);
}
std::vector<common::MapPointId> LocalMapper::Extend(core::Map& map,common::KeyFrameId current,
    const std::vector<common::KeyFrameId>& neighbors) {
  const auto& key=map.GetKeyFrame(current);
  std::vector<common::MapPointId> created;
  auto add_edge=[&](common::KeyFrameId k,std::size_t index,common::MapPointId p) {
    const auto& point=map.GetMapPoint(p);
    if(!point.is_bad() && !point.observations().count(k) && ValidProjection(map.GetKeyFrame(k),index,point.position_w()))
      map.AddObservation(k,p,index,solver_options_.pixel_sigma,solver_options_.depth_sigma_m);
  };
  auto from_depth=[&](const core::KeyFrame& k,std::size_t i)->std::optional<Eigen::Vector3d> {
    const auto& feature=k.features().at(i);
    if(!feature.depth_z_m) return {};
    const auto p=camera_.Unproject(feature.uv,*feature.depth_z_m);
    return p ? std::optional<Eigen::Vector3d>(k.pose().Inverse()* *p) : std::nullopt;
  };
  auto create=[&](const Eigen::Vector3d& p,std::size_t i,std::optional<std::pair<common::KeyFrameId,std::size_t>> other) {
    const auto id=map.AddMapPoint(p,key.features().Descriptor(i));
    try {
      map.AddObservation(current,id,i,solver_options_.pixel_sigma,solver_options_.depth_sigma_m);
      if(other) map.AddObservation(other->first,id,other->second,solver_options_.pixel_sigma,solver_options_.depth_sigma_m);
      created.push_back(id);
    } catch(...) { map.RemoveMapPoint(id); throw; }
  };
  for(auto neighbor:neighbors) {
    const auto& old=map.GetKeyFrame(neighbor);
    for(const auto& match:matcher_.MatchDescriptors(old.features().descriptors(),key.features().descriptors())) {
      const auto old_id=old.associations()[match.first],new_id=key.associations()[match.second];
      if(old_id.valid() && new_id.valid()) continue; // Fusion is a separate future policy.
      if(old_id.valid()) { add_edge(current,match.second,old_id); continue; }
      if(new_id.valid()) { add_edge(neighbor,match.first,new_id); continue; }
      if(created.size()>=static_cast<std::size_t>(options_.max_new_points)) continue;
      auto point=from_depth(key,match.second);
      if(!point) point=from_depth(old,match.first);
      if(!point) {
        const auto a=camera_.BackProject(old.features().at(match.first).uv);
        const auto b=camera_.BackProject(key.features().at(match.second).uv);
        const Eigen::Matrix<double,3,4> pa=old.pose().Matrix().topRows<3>(),pb=key.pose().Matrix().topRows<3>();
        Eigen::Matrix4d matrix;
        matrix.row(0)=a->x()*pa.row(2)-pa.row(0); matrix.row(1)=a->y()*pa.row(2)-pa.row(1);
        matrix.row(2)=b->x()*pb.row(2)-pb.row(0); matrix.row(3)=b->y()*pb.row(2)-pb.row(1);
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(matrix,Eigen::ComputeFullV);
        const Eigen::Vector4d homogeneous=svd.matrixV().col(3);
        if(!homogeneous.allFinite() || std::abs(homogeneous.w())<1e-12) continue;
        const Eigen::Vector3d p=homogeneous.head<3>()/homogeneous.w();
        const Eigen::Vector3d ra=p-old.pose().Inverse().translation(),rb=p-key.pose().Inverse().translation();
        const double denom=ra.norm()*rb.norm();
        if(denom<1e-12 || std::acos(std::clamp(ra.dot(rb)/denom,-1.0,1.0))*180/3.141592653589793<options_.min_parallax_deg) continue;
        point=p;
      }
      if(point && ValidProjection(key,match.second,*point) && ValidProjection(old,match.first,*point))
        create(*point,match.second,std::make_pair(neighbor,match.first));
    }
  }
  for(std::size_t i=0;i<key.features().size() && created.size()<static_cast<std::size_t>(options_.max_new_points);++i)
    if(!key.associations()[i].valid()) {
      const auto p=from_depth(key,i); if(p && ValidProjection(key,i,*p)) create(*p,i,{});
    }
  return created;
}
optimization::BAProblem LocalMapper::BuildProblem(const core::Map& map,common::KeyFrameId current) const {
  optimization::BAProblem problem; problem.epoch=map.epoch(); problem.revision=map.revision();
  auto neighbors=SelectNeighbors(map,current,options_.max_keyframes-1); neighbors.push_back(current);
  std::set<common::KeyFrameId> local(neighbors.begin(),neighbors.end());
  auto points=CollectPoints(map,neighbors);
  std::sort(points.begin(),points.end(),[&](auto a,auto b) {
    const auto na=map.GetMapPoint(a).observations().size(),nb=map.GetMapPoint(b).observations().size();
    return na!=nb ? na>nb : a<b;
  });
  std::map<common::KeyFrameId,std::size_t> pose_indices;
  const auto anchor=*local.begin();
  for(auto id:points) {
    if(problem.points.size()>=static_cast<std::size_t>(options_.max_ba_points)) break;
    const auto& point=map.GetMapPoint(id);
    std::vector<common::ObservationId> observations;
    for(const auto& edge:point.observations()) {
      const auto& o=map.GetObservation(edge.second);
      if(!o.is_outlier() && ValidProjection(map.GetKeyFrame(edge.first),o.feature_index(),point.position_w()))
        observations.push_back(edge.second);
    }
    if(observations.size()<2) continue;
    if(problem.measurements.size()+observations.size()>static_cast<std::size_t>(options_.max_ba_observations)) break;
    const auto index=problem.points.size(); problem.points.push_back({id,point.position_w()});
    for(auto obs_id:observations) {
      const auto& o=map.GetObservation(obs_id); const auto key=o.keyframe_id();
      if(!pose_indices.count(key)) {
        pose_indices[key]=problem.poses.size();
        problem.poses.push_back({key,map.GetKeyFrame(key).pose(),!local.count(key) || key==anchor});
      }
      problem.measurements.push_back({obs_id,pose_indices.at(key),index,o.uv(),o.depth_z_m()});
    }
  }
  int boundary=0; bool has_current=false;
  for(const auto& p:problem.poses) { boundary+=!local.count(p.id); has_current|=p.id==current && !p.fixed; }
  if(boundary>options_.max_boundary_keyframes || !has_current) {
    problem.poses.clear(); problem.points.clear(); problem.measurements.clear();
  }
  return problem;
}
// Insert and extend a keyframe, time its BA solve, then commit accepted geometry and synchronize Frame.
MappingResult LocalMapper::Process(core::Frame& frame, core::Map& map) {
  MappingResult result;
  result.keyframe = map.InsertKeyFrame(frame);
  ++insertions_;
  const auto neighbors = SelectNeighbors(map, result.keyframe, options_.max_neighbors);
  const auto created = Extend(map, result.keyframe, neighbors);
  result.created_points = created.size();
  for (auto id : created)
    births_[id] = insertions_;
  const auto ba_start = std::chrono::steady_clock::now();
  result.ba = ba_.Solve(BuildProblem(map, result.keyframe));
  result.ba_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - ba_start).count();
  if (result.ba.usable) {
    std::vector<core::PoseUpdate> poses;
    std::vector<core::PointUpdate> points;
    for (const auto& p : result.ba.geometry.poses)
      if (!p.fixed)
        poses.push_back({p.id, p.tcw});
    for (const auto& p : result.ba.geometry.points)
      points.push_back({p.id, p.position_w});
    result.ba_committed =
        map.CommitGeometry(result.ba.geometry.epoch, result.ba.geometry.revision, poses, points)
            .ok();
    if (result.ba_committed) {
      for (auto id : result.ba.outliers)
        map.RemoveObservation(id);
      for (const auto& p : map.Snapshot().points) {
        const auto& point = map.GetMapPoint(p.id);
        const auto it = births_.find(p.id);
        const auto birth = it == births_.end() ? 0 : it->second;
        if (point.is_bad() || point.observations().empty() ||
            (insertions_ - birth >= static_cast<std::uint64_t>(options_.point_grace_period) &&
             point.observations().size() < 2)) {
          map.RemoveMapPoint(p.id);
          births_.erase(p.id);
          ++result.removed_points;
        }
      }
    }
  }
  const auto& key = map.GetKeyFrame(result.keyframe);
  frame.SetPose(key.pose());
  for (std::size_t i = 0; i < frame.features().size(); ++i) {
    frame.RemoveAssociation(i);
    if (key.associations()[i].valid())
      frame.Associate(i, key.associations()[i]);
  }
  return result;
}
void LocalMapper::Reset() { insertions_=0; births_.clear(); }
}
