#include "vslam/optimization/pose_optimizer.h"
#include "vslam/optimization/local_bundle_adjuster.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace vslam::optimization {
namespace {
using PoseArray=std::array<double,6>;  // Internal [angle-axis, translation], NOT public SE3 increments.
using PointArray=std::array<double,3>;
void ValidateOptions(const SolverOptions& o) {
  if(o.max_iterations<1 || !std::isfinite(o.pixel_sigma) || o.pixel_sigma<=0 ||
     !std::isfinite(o.depth_sigma_m) || o.depth_sigma_m<=0 ||
     !std::isfinite(o.huber_delta) || o.huber_delta<0 ||
     !std::isfinite(o.outlier_pixels) || o.outlier_pixels<=0 ||
     !std::isfinite(o.outlier_depth_sigma) || o.outlier_depth_sigma<=0)
    throw std::invalid_argument("Invalid solver options");
}
PoseArray Pack(const geometry::SE3& pose) {
  const Eigen::AngleAxisd aa(pose.rotation());
  const Eigen::Vector3d rotation=aa.axis()*aa.angle();
  return {rotation.x(),rotation.y(),rotation.z(),pose.translation().x(),pose.translation().y(),pose.translation().z()};
}
geometry::SE3 Unpack(const PoseArray& a) {
  Eigen::Vector3d w(a[0],a[1],a[2]);
  Eigen::Matrix3d r=Eigen::Matrix3d::Identity();
  const double angle=w.norm();
  if(angle>1e-15) r=Eigen::AngleAxisd(angle,w/angle).toRotationMatrix();
  return geometry::SE3(r,{a[3],a[4],a[5]});
}
bool ValidMeasurement(const Eigen::Vector2d& uv,std::optional<double> z) {
  return uv.allFinite() && (!z || (std::isfinite(*z) && *z>0));
}
bool Inlier(const sensor::CameraModel& c,const geometry::SE3& pose,const Eigen::Vector3d& point,
            const Eigen::Vector2d& uv,std::optional<double> z,const SolverOptions& o) {
  if(!point.allFinite() || !ValidMeasurement(uv,z)) return false;
  const Eigen::Vector3d pc=pose*point;
  const auto pixel=c.Project(pc);
  return pixel && (*pixel-uv).norm()<=o.outlier_pixels &&
         (!z || std::abs(pc.z()-*z)<=o.outlier_depth_sigma*o.depth_sigma_m);
}
bool NonCollinear(const std::vector<Eigen::Vector3d>& points) {
  if(points.size()<3) return false;
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();
  for(const auto& p:points) mean+=p;
  mean/=static_cast<double>(points.size());
  Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
  for(const auto& p:points) covariance+=(p-mean)*(p-mean).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
  return eig.info()==Eigen::Success && eig.eigenvalues()[2]>1e-12 &&
         eig.eigenvalues()[1]>eig.eigenvalues()[2]*1e-8;
}
template<int N> struct Reprojection {
  double fx,fy,cx,cy,u,v,z,pixel_sigma,depth_sigma;
  template<typename T> bool operator()(const T* pose,const T* point,T* r) const {
    T pc[3];
    ceres::AngleAxisRotatePoint(pose,point,pc);
    for(int i=0;i<3;++i) pc[i]+=pose[i+3];
    if(pc[2]<=T(1e-8)) return false; // Reject invalid step; residual dimension remains fixed.
    r[0]=(T(fx)*pc[0]/pc[2]+T(cx)-T(u))/T(pixel_sigma);
    r[1]=(T(fy)*pc[1]/pc[2]+T(cy)-T(v))/T(pixel_sigma);
    if constexpr(N==3) r[2]=(pc[2]-T(z))/T(depth_sigma);
    return true;
  }
};
ceres::CostFunction* Cost(const sensor::CameraModel& c,const Eigen::Vector2d& uv,
                          std::optional<double> z,const SolverOptions& o) {
  if(z) return new ceres::AutoDiffCostFunction<Reprojection<3>,3,6,3>(
      new Reprojection<3>{c.fx(),c.fy(),c.cx(),c.cy(),uv.x(),uv.y(),*z,o.pixel_sigma,o.depth_sigma_m});
  return new ceres::AutoDiffCostFunction<Reprojection<2>,2,6,3>(
      new Reprojection<2>{c.fx(),c.fy(),c.cx(),c.cy(),uv.x(),uv.y(),0,o.pixel_sigma,o.depth_sigma_m});
}
ceres::LossFunction* Loss(const SolverOptions& o) {
  return o.huber_delta>0 ? new ceres::HuberLoss(o.huber_delta) : nullptr;
}
ceres::Solver::Options Options(const SolverOptions& o) {
  ceres::Solver::Options options;
  options.max_num_iterations=o.max_iterations;
  options.num_threads=1;
  options.logging_type=ceres::SILENT;
  options.function_tolerance=1e-6;
  options.gradient_tolerance=1e-8;
  options.parameter_tolerance=1e-8;
  return options;
}
bool Accepted(const ceres::Solver::Summary& s) {
  return s.termination_type==ceres::CONVERGENCE && s.IsSolutionUsable() &&
         std::isfinite(s.initial_cost) && std::isfinite(s.final_cost) &&
         s.final_cost<=s.initial_cost+1e-8*std::max(1.0,s.initial_cost);
}
// Conservative RGB-D observability check on the actual active measurement set.
// Each component must reach a fixed pose with >=6 noncollinear depth-supported points.
bool Anchored(const BAProblem& p,const std::vector<bool>& active) {
  if(p.poses.empty() || p.points.empty()) return false;
  std::vector<int> point_count(p.points.size(),0),pose_count(p.poses.size(),0);
  std::vector<bool> reached_pose(p.poses.size(),false),reached_point(p.points.size(),false);
  bool variable=false,anchor=false;
  for(std::size_t i=0;i<p.poses.size();++i) {
    variable|=!p.poses[i].fixed;
    std::vector<Eigen::Vector3d> depths;
    for(std::size_t j=0;j<p.measurements.size();++j) {
      const auto& m=p.measurements[j];
      if(active[j] && m.pose_index==i && m.depth_z_m) depths.push_back(p.points[m.point_index].position_w);
    }
    if(p.poses[i].fixed && depths.size()>=6 && NonCollinear(depths)) {
      reached_pose[i]=true; anchor=true;
    }
  }
  for(std::size_t j=0;j<p.measurements.size();++j) if(active[j]) {
    ++point_count[p.measurements[j].point_index]; ++pose_count[p.measurements[j].pose_index];
  }
  if(!variable || !anchor) return false;
  for(int n:point_count) if(n<2) return false;
  for(std::size_t i=0;i<pose_count.size();++i) if(!p.poses[i].fixed && pose_count[i]<6) return false;
  bool changed=true;
  while(changed) {
    changed=false;
    for(std::size_t j=0;j<p.measurements.size();++j) if(active[j]) {
      const auto& m=p.measurements[j];
      if(reached_pose[m.pose_index] && !reached_point[m.point_index]) {
        reached_point[m.point_index]=true; changed=true;
      }
      if(reached_point[m.point_index] && !reached_pose[m.pose_index]) {
        reached_pose[m.pose_index]=true; changed=true;
      }
    }
  }
  return std::all_of(reached_pose.begin(),reached_pose.end(),[](bool x){return x;}) &&
         std::all_of(reached_point.begin(),reached_point.end(),[](bool x){return x;});
}
}
PoseOptimizer::PoseOptimizer(sensor::CameraModel c,SolverOptions o):camera_(std::move(c)),options_(o) { ValidateOptions(o); }
PoseSolution PoseOptimizer::Solve(const geometry::SE3& initial,const std::vector<PoseMeasurement>& observations) const {
  PoseSolution result;
  result.inliers.resize(observations.size(),false);
  std::vector<Eigen::Vector3d> positions;
  for(const auto& m:observations) {
    if(!m.point_w.allFinite() || !ValidMeasurement(m.uv,m.depth_z_m) || !camera_.Project(initial*m.point_w)) {
      result.message="Invalid pose measurement"; return result;
    }
    positions.push_back(m.point_w);
  }
  if(observations.size()<6 || !NonCollinear(positions)) { result.message="Insufficient noncollinear pose constraints"; return result; }
  PoseArray pose=Pack(initial);
  std::vector<PointArray> points;
  for(const auto& m:observations) points.push_back({m.point_w.x(),m.point_w.y(),m.point_w.z()});
  ceres::Problem problem;
  for(std::size_t i=0;i<observations.size();++i) {
    problem.AddResidualBlock(Cost(camera_,observations[i].uv,observations[i].depth_z_m,options_),
                             Loss(options_),pose.data(),points[i].data());
    problem.SetParameterBlockConstant(points[i].data());
  }
  auto options=Options(options_); options.linear_solver_type=ceres::DENSE_QR;
  ceres::Solver::Summary summary; ceres::Solve(options,&problem,&summary);
  result.iterations=static_cast<int>(summary.iterations.size());
  result.initial_cost=summary.initial_cost; result.final_cost=summary.final_cost;
  result.message=summary.BriefReport();
  if(!Accepted(summary)) return result;
  const auto candidate=Unpack(pose);
  for(std::size_t i=0;i<observations.size();++i) {
    const auto& m=observations[i];
    result.inliers[i]=Inlier(camera_,candidate,m.point_w,m.uv,m.depth_z_m,options_);
  }
  if(std::count(result.inliers.begin(),result.inliers.end(),true)<6) return result;
  result.usable=true; result.tcw=candidate;
  return result;
}
LocalBundleAdjuster::LocalBundleAdjuster(sensor::CameraModel c,SolverOptions o):camera_(std::move(c)),options_(o) { ValidateOptions(o); }
BASolution LocalBundleAdjuster::Solve(const BAProblem& input) const {
  BASolution result; result.geometry=input;
  auto& p=result.geometry;
  std::set<common::KeyFrameId> pose_ids; std::set<common::MapPointId> point_ids;
  std::set<common::ObservationId> observation_ids; std::set<std::pair<std::size_t,std::size_t>> edges;
  for(const auto& pose:p.poses)
    if(!pose.id.valid() || pose.id.epoch()!=p.epoch || !pose_ids.insert(pose.id).second) {
      result.message="Invalid BA pose IDs"; return result;
    }
  for(const auto& point:p.points)
    if(!point.id.valid() || point.id.epoch()!=p.epoch || !point.position_w.allFinite() || !point_ids.insert(point.id).second) {
      result.message="Invalid BA points"; return result;
    }
  std::vector<bool> active(p.measurements.size(),false);
  for(std::size_t j=0;j<p.measurements.size();++j) {
    const auto& m=p.measurements[j];
    if(m.pose_index>=p.poses.size() || m.point_index>=p.points.size() || !m.id.valid() ||
       m.id.epoch()!=p.epoch || !observation_ids.insert(m.id).second ||
       !edges.insert({m.pose_index,m.point_index}).second || !ValidMeasurement(m.uv,m.depth_z_m)) {
      result.message="Invalid BA observation"; return result;
    }
    active[j]=Inlier(camera_,p.poses[m.pose_index].tcw,p.points[m.point_index].position_w,m.uv,m.depth_z_m,options_);
  }
  if(!Anchored(p,active)) { result.message="BA active graph lacks RGB-D anchor, connectivity, or constraints"; return result; }
  std::vector<PoseArray> poses; std::vector<PointArray> points;
  for(const auto& pose:p.poses) poses.push_back(Pack(pose.tcw));
  for(const auto& point:p.points) points.push_back({point.position_w.x(),point.position_w.y(),point.position_w.z()});
  ceres::Problem problem;
  auto ordering=std::make_shared<ceres::ParameterBlockOrdering>();
  for(std::size_t i=0;i<poses.size();++i) {
    problem.AddParameterBlock(poses[i].data(),6);
    if(p.poses[i].fixed) problem.SetParameterBlockConstant(poses[i].data());
    ordering->AddElementToGroup(poses[i].data(),1);
  }
  for(auto& point:points) { problem.AddParameterBlock(point.data(),3); ordering->AddElementToGroup(point.data(),0); }
  for(std::size_t j=0;j<p.measurements.size();++j) if(active[j]) {
    const auto& m=p.measurements[j];
    problem.AddResidualBlock(Cost(camera_,m.uv,m.depth_z_m,options_),Loss(options_),
                             poses[m.pose_index].data(),points[m.point_index].data());
  }
  auto options=Options(options_); options.linear_solver_type=ceres::DENSE_SCHUR;
  options.linear_solver_ordering=ordering;
  ceres::Solver::Summary summary; ceres::Solve(options,&problem,&summary);
  result.initial_cost=summary.initial_cost; result.final_cost=summary.final_cost;
  result.iterations=static_cast<int>(summary.iterations.size());
  result.converged=summary.termination_type==ceres::CONVERGENCE;
  result.message=summary.BriefReport();
  if(!Accepted(summary)) return result;
  for(std::size_t i=0;i<poses.size();++i) if(!p.poses[i].fixed) p.poses[i].tcw=Unpack(poses[i]);
  for(std::size_t i=0;i<points.size();++i) {
    p.points[i].position_w={points[i][0],points[i][1],points[i][2]};
    if(!p.points[i].position_w.allFinite()) return result;
  }
  for(std::size_t j=0;j<p.measurements.size();++j) {
    const auto& m=p.measurements[j];
    active[j]=active[j] && Inlier(camera_,p.poses[m.pose_index].tcw,p.points[m.point_index].position_w,m.uv,m.depth_z_m,options_);
    if(!active[j]) result.outliers.push_back(m.id);
  }
  if(!Anchored(p,active)) { result.message="BA final inliers lost anchor or constraints"; return result; }
  result.usable=true;
  return result;
}
}  // namespace vslam::optimization
