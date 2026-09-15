#include "vslam/system/slam_system.h"
#include "vslam/optimization/local_bundle_adjuster.h"
#include <opencv2/imgproc.hpp>
#include <random>
#include <iostream>
#include <stdexcept>
using namespace vslam;
void Check(bool condition,const char* why) { if(!condition) throw std::runtime_error(why); }
sensor::FrameInput Input(std::int64_t stamp) {
  sensor::FrameInput input;
  input.timestamp={stamp,common::TimeBase::Simulation};
  input.image=cv::Mat(480,640,CV_8UC3,cv::Scalar(80,90,100));
  input.depth=cv::Mat(480,640,CV_32FC1,cv::Scalar(2));
  input.rectified=true; input.registration=sensor::DepthRegistration::RegisteredToColor;
  return input;
}
std::vector<Eigen::Vector3d> World() {
  std::vector<Eigen::Vector3d> points;
  for(int y=0;y<7;++y) for(int x=0;x<10;++x) points.emplace_back((x-4.5)*0.14,(y-3)*0.13,2.3+0.07*((x*3+y)%5));
  return points;
}
core::FeatureSet Features(const sensor::CameraModel& c,const geometry::SE3& pose,bool depth=true,int extra=0) {
  auto world=World(); std::vector<core::Keypoint> points;
  cv::Mat descriptors(static_cast<int>(world.size())+extra,32,CV_8UC1);
  std::mt19937 rng(73);
  for(int i=0;i<descriptors.rows;++i) for(int j=0;j<32;++j) descriptors.at<unsigned char>(i,j)=static_cast<unsigned char>(rng()%256);
  for(const auto& p:world) {
    const auto pc=pose*p; core::Keypoint k;
    k.uv=*c.Project(pc); if(depth) k.depth_z_m=pc.z(); points.push_back(k);
  }
  for(int i=0;i<extra;++i) { core::Keypoint k; const auto pc=pose * *c.Unproject({80.0+i*8,80},3); k.uv=*c.Project(pc); k.depth_z_m=pc.z(); points.push_back(k); }
  return {core::DescriptorKind::Orb,std::move(points),descriptors};
}
void TestFrontendAndInput() {
  const sensor::CameraModel camera(500,500,320,240,640,480);
  auto input=Input(1);
  input.depth=cv::Mat(480,640,CV_16UC1,cv::Scalar(2000)); input.meters_per_depth_unit=0.001;
  auto prepared=sensor::Prepare(input,camera);
  Check(prepared.depth_z_m.at<float>(0,0)==2,"depth scale once");
  input.depth.setTo(0);
  Check(prepared.depth_z_m.at<float>(0,0)==2,"prepared owns depth");
  input.registration=sensor::DepthRegistration::Native;
  bool rejected=false;
  try { sensor::Prepare(input,camera); } catch(const std::invalid_argument&) { rejected=true; }
  Check(rejected,"reject unregistered input");
  cv::RNG rng(12); rng.fill(prepared.image_rgb_or_gray,cv::RNG::UNIFORM,0,256);
  frontend::FeatureExtractor extractor({});
  auto features=extractor.Extract(prepared.image_rgb_or_gray,prepared.depth_z_m);
  Check(features.size()>100 && features.at(0).depth_z_m==2,"real ORB output and metric depths");
  frontend::FeatureMatcher matcher({0.75,true});
  Check(matcher.MatchDescriptors(features.descriptors(),features.descriptors()).size()==features.size(),"mutual ORB self matching");
}
void TestSequence() {
  const sensor::CameraModel camera(500,500,320,240,640,480);
  system::SlamConfig config; config.min_initial_points=30; config.mapping.max_gap=2;
  config.mapping.min_gap=2; config.mapping.max_ba_points=100; config.ba_solver.max_iterations=40;
  system::SlamSystem slam(camera,config);
  auto result=slam.ProcessFeatures(Input(1),core::FeatureSet());
  Check(result.state==system::SlamState::Initializing && !result.tcw && slam.map().num_map_points()==0,"waiting initialization");
  int inserted=0,committed=0;
  for(int i=0;i<9;++i) {
    geometry::SE3 expected(Eigen::Matrix3d::Identity(),{-0.018*i,0.002*i,0});
    result=slam.ProcessFeatures(Input(i+2),Features(camera,expected,true,i>=4?10:0));
    Check(result.status.ok() && result.state==system::SlamState::Tracking && result.tcw.has_value(),"consistent tracking");
    Check((result.tcw->Matrix()-expected.Matrix()).norm()<0.003,"metric pose accuracy");
    Check(slam.map().CheckConsistency(),"map graph consistency");
    if(result.inserted_keyframe) {
      ++inserted;
      const auto& key=slam.map().GetKeyFrame(*result.inserted_keyframe);
      Check(result.frame->pose()->Matrix().isApprox(key.pose().Matrix(),1e-12),"post-BA frame pose sync");
      Check(result.frame->associations()==key.associations(),"post-BA frame graph sync");
    }
    if(result.ba_committed) ++committed;
  }
  Check(inserted>=4 && committed>=1,"keyframes and BA actually executed");
  const auto graph=slam.map().Snapshot();
  Check(graph.points.size()>=World().size(),"mapping point creation");
  auto blank=slam.ProcessFeatures(Input(20),core::FeatureSet());
  Check(blank.state==system::SlamState::Lost && !blank.tcw && !blank.frame->pose(),"lost publishes no pose");
  Check(slam.map().revision()==graph.revision,"lost frame cannot mutate map");
  geometry::SE3 return_pose(Eigen::Matrix3d::Identity(),{-0.144,0.016,0});
  auto recovered=slam.ProcessFeatures(Input(21),Features(camera,return_pose));
  Check(recovered.tcw.has_value() && recovered.state==system::SlamState::Tracking,"retry last keyframe after loss");
  auto invalid=slam.ProcessFeatures(Input(21),Features(camera,return_pose));
  Check(!invalid.status.ok() && !invalid.tcw,"repeated timestamp rejected");
  const auto epoch=slam.map().epoch(); slam.Reset();
  auto reset=slam.ProcessFeatures(Input(1),Features(camera,geometry::SE3()));
  Check(reset.tcw && slam.map().epoch()==epoch+1 && reset.frame->id().epoch()==epoch+1,"reset reinitializes new epoch");
  std::cout<<"Sequence: "<<inserted<<" keyframes, "<<committed<<" accepted BA batches\n";
}
void TestBAAcceptanceAndRejection() {
  const sensor::CameraModel camera(500,500,320,240,640,480);
  optimization::BAProblem problem; problem.epoch=0; problem.revision=8;
  const geometry::SE3 true_pose(Eigen::Matrix3d::Identity(),{-0.1,0,0});
  problem.poses={{common::KeyFrameId(0,0),geometry::SE3(),true},
                 {common::KeyFrameId(0,1),geometry::SE3(Eigen::Matrix3d::Identity(),{-0.098,0.001,0}),false}};
  const auto world=World();
  for(std::size_t i=0;i<world.size();++i) {
    problem.points.push_back({common::MapPointId(0,i),world[i]+Eigen::Vector3d(0.0005,-0.0005,0.001)});
    for(std::size_t j=0;j<2;++j) {
      const auto pc=j?true_pose*world[i]:world[i];
      problem.measurements.push_back({common::ObservationId(0,2*i+j),j,i,*camera.Project(pc),pc.z()});
    }
  }
  optimization::SolverOptions options; options.huber_delta=0; options.max_iterations=50;
  optimization::LocalBundleAdjuster solver(camera,options);
  auto result=solver.Solve(problem);
  Check(result.usable && result.converged && result.final_cost<result.initial_cost,"BA improves noisy geometry");
  Check(result.geometry.poses[0].tcw.Matrix()==problem.poses[0].tcw.Matrix(),"fixed boundary unchanged");
  Check((result.geometry.poses[1].tcw.Matrix()-true_pose.Matrix()).norm()<1e-5,"BA known solution");
  auto boundary=problem;
  boundary.poses.push_back({common::KeyFrameId(0,2),geometry::SE3(),true});
  boundary.measurements.push_back({common::ObservationId(0,1000),2,0,*camera.Project(world[0]),world[0].z()});
  Check(solver.Solve(boundary).usable,"fixed boundary with one observation is valid");
  auto no_anchor=problem; for(auto& m:no_anchor.measurements) m.depth_z_m.reset();
  Check(!solver.Solve(no_anchor).usable,"no metric anchor rejected");
  auto gated=problem; for(auto& m:gated.measurements) if(m.pose_index==0) m.depth_z_m=20;
  Check(!solver.Solve(gated).usable,"gated-away anchor rejected");
  auto invalid=problem; invalid.measurements[0].point_index=99999;
  Check(!solver.Solve(invalid).usable,"invalid BA edge rejected");
  options.max_iterations=1;
  auto limited=optimization::LocalBundleAdjuster(camera,options).Solve(problem);
  Check(!limited.usable,"unconverged BA not accepted");
  Check(problem.poses[1].tcw.translation().x()==-0.098,"solver never mutates input");
}
int main() {
  try { TestFrontendAndInput(); TestBAAcceptanceAndRejection(); TestSequence(); return 0; }
  catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
