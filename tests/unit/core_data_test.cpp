#include "vslam/core/map.h"
#include "vslam/sensor/rgbd_camera.h"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>

using namespace vslam;
void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template <class Function> void Reject(Function action, const char* message) {
  bool rejected = false;
  try { action(); } catch (const std::exception&) { rejected = true; }
  Check(rejected, message);
}
core::FeatureSet Features() {
  core::Keypoint a, b;
  a.uv = {1,1}; a.depth_z_m = 2;
  b.uv = {2,2};
  return core::FeatureSet(core::DescriptorKind::Orb, {a,b}, cv::Mat(2,32,CV_8UC1,cv::Scalar(9)));
}
core::Frame MakeFrame(core::FrameFactory& factory, std::int64_t stamp) {
  auto frame = factory.Create({stamp,common::TimeBase::Simulation}, cv::Mat(4,4,CV_8UC3,cv::Scalar(1,2,3)));
  frame.SetFeatures(Features());
  frame.SetPose(geometry::SE3());
  return frame;
}
void TestCommonAndPose() {
  static_assert(!std::is_convertible_v<common::FrameId, common::MapPointId>);
  static_assert(!std::is_copy_constructible_v<core::FrameFactory>);
  static_assert(!std::is_move_constructible_v<common::IdGenerator<common::FrameId>>);
  Check(!common::MapPointId().valid(), "default ID invalid");
  common::IdGenerator<common::FrameId> ids(3);
  Check(ids.Next() == common::FrameId(3,0) && ids.Next() == common::FrameId(3,1), "scoped ID allocation");
  ids.AdvanceEpoch(4);
  Check(ids.Next()==common::FrameId(4,0), "advance allocator epoch");
  Reject([&] { ids.AdvanceEpoch(4); }, "same epoch cannot reset allocator");
  common::StatusOr<int> good(4);
  common::StatusOr<int> bad(common::Status(common::StatusCode::NotFound,"missing"));
  Check(good.ok() && good.value()==4 && !bad.ok(), "StatusOr value/error");
  Reject([&] { bad.value(); }, "error has no value");
  Reject([] { common::StatusOr<int> invalid(common::Status{}); }, "OK without value forbidden");
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(0.7,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const geometry::SE3 tcw(rotation,{1,2,3});
  const Eigen::Vector3d pw(4,-2,1);
  Check((tcw.Inverse()*(tcw*pw)-pw).norm()<1e-12, "Tcw inverse direction");
  Check((tcw*tcw.Inverse()).Matrix().isApprox(Eigen::Matrix4d::Identity(),1e-12), "SE3 composition");
  Check(geometry::SE3::FromMatrix(tcw.Matrix()).Matrix().isApprox(tcw.Matrix()), "SE3 matrix");
  Reject([] { geometry::SE3(Eigen::Matrix3d::Zero(),Eigen::Vector3d::Zero()); }, "invalid rotation");
  Eigen::Matrix4d matrix=Eigen::Matrix4d::Identity(); matrix(3,0)=1;
  Reject([&] { geometry::SE3::FromMatrix(matrix); }, "invalid homogeneous row");
}
void TestFrameOwnershipAndValidation() {
  core::FrameFactory factory(0,common::TimeBase::Simulation);
  cv::Mat image(4,4,CV_8UC3,cv::Scalar(1,2,3));
  cv::Mat depth(4,4,CV_32FC1,cv::Scalar(2));
  auto frame=factory.Create({0,common::TimeBase::Simulation},image,depth,"external_999");
  image.setTo(0); depth.setTo(0);
  Check(frame.image().at<cv::Vec3b>(0,0)[0]==1 && frame.depth_z_m().at<float>(0,0)==2,"owned image buffers");
  auto returned=frame.image(); returned.setTo(0);
  Check(frame.image().at<cv::Vec3b>(0,0)[0]==1,"getter cannot mutate frame image");
  frame.SetFeatures(Features());
  auto descriptors=frame.features().descriptors(); descriptors.setTo(0);
  Check(frame.features().Descriptor(0).at<unsigned char>(0,0)==9,"descriptor getter isolation");
  frame.Associate(0,common::MapPointId(0,3));
  core::Frame copy=frame;
  frame.SetFeatures(core::FeatureSet());
  Check(copy.features().size()==2 && copy.associations()[0].valid(),"frame copy isolation");
  Check(frame.associations().empty(),"features reset associations");
  Reject([&] { factory.Create({0,common::TimeBase::Simulation},image); },"duplicate timestamp");
  Reject([&] { factory.Create({-1,common::TimeBase::Simulation},image); },"negative timestamp");
  Reject([&] { factory.Create({1,common::TimeBase::Unix},image); },"wrong time base");
  Reject([&] { factory.Create({1,common::TimeBase::Simulation},cv::Mat()); },"empty image");
  auto next=factory.Create({1,common::TimeBase::Simulation},image);
  Check(next.sequence()==1 && next.id()!=copy.id(),"failed construction does not consume sequence");
  Reject([&] { next.Associate(0,common::MapPointId(1,3)); },"wrong epoch association");
  Reject([&] { core::FeatureSet(core::DescriptorKind::Orb,Features().keypoints(),cv::Mat(1,32,CV_8U)); },"descriptor rows");
  Reject([&] { core::FeatureSet(core::DescriptorKind::Sift,Features().keypoints(),cv::Mat(2,32,CV_8U)); },"descriptor type");
  auto keypoints=Features().keypoints(); keypoints[0].depth_z_m=0;
  Reject([&] { core::FeatureSet(core::DescriptorKind::Orb,keypoints,cv::Mat(2,32,CV_8U)); },"invalid feature depth");
  auto sift_points=Features().keypoints(); sift_points[0].octave=-1;
  core::FeatureSet sift(core::DescriptorKind::Sift,sift_points,cv::Mat(2,128,CV_32F,cv::Scalar(0.5)));
  Check(sift.size()==2, "SIFT initial negative octave supported");
  Reject([&] { core::KeyFrame::FromFrame(common::KeyFrameId(0,1),copy); },"promotion needs pose");
}
void TestMapTopologyAndCommit() {
  core::Map map;
  core::FrameFactory factory(map.epoch(),common::TimeBase::Simulation);
  auto frame=MakeFrame(factory,1);
  const auto p0=map.AddMapPoint({0,0,2},frame.features().Descriptor(0));
  const auto p1=map.AddMapPoint({1,0,2});
  frame.Associate(0,p0);
  const auto key=map.InsertKeyFrame(frame,true);
  Check(map.GetKeyFrame(key).source_frame_id()==frame.id() &&
        map.GetKeyFrame(key).source_sequence()==frame.sequence(),"promotion origin");
  Check(map.GetKeyFrame(key).image().rows==4,"retained image");
  Check(map.num_observations()==1 && map.CheckConsistency(),"promotion registers copied edge");
  const auto obs=*map.GetKeyFrame(key).observation_ids().begin();
  Check(map.GetObservation(obs).depth_z_m()==2 &&
        map.GetObservation(obs).uv()==Eigen::Vector2d(1,1),"immutable measurement from feature");
  frame.RemoveAssociation(0); frame.SetFeatures(core::FeatureSet());
  Check(map.GetKeyFrame(key).features().size()==2 && map.GetKeyFrame(key).associations()[0]==p0,"keyframe independent");
  const auto rev=map.revision();
  Reject([&] { map.AddObservation(key,p1,0); },"feature conflict");
  Reject([&] { map.AddObservation(key,p0,1); },"one point per keyframe");
  Reject([&] { map.AddObservation(key,p1,5); },"invalid feature index");
  Reject([&] { map.AddObservation(key,p1,1,0); },"invalid sigma");
  Check(map.revision()==rev && map.CheckConsistency(),"rejected observations unchanged");
  auto obs1=map.AddObservation(key,p1,1);
  Check(!map.GetObservation(obs1).depth_z_m(),"optional mono measurement");
  map.MarkObservationOutlier(obs1);
  Check(map.GetObservation(obs1).is_outlier(),"outlier flag through Map");
  const auto snapshot=map.Snapshot();
  const auto before=map.GetMapPoint(p0).position_w();
  auto status=map.CommitGeometry(map.epoch(),map.revision(),{},
      {{p0,{9,9,9}},{common::MapPointId(0,999),{1,2,3}}});
  Check(!status.ok() && map.GetMapPoint(p0).position_w()==before,"batch validates before any write");
  status=map.CommitGeometry(map.epoch(),map.revision(),{},{{p0,{9,9,9}},{p0,{1,2,3}}});
  Check(!status.ok(),"duplicate geometry update");
  status=map.CommitGeometry(map.epoch(),map.revision(),{},{{p0,{std::numeric_limits<double>::quiet_NaN(),0,1}}});
  Check(!status.ok(),"nonfinite geometry update");
  status=map.CommitGeometry(map.epoch(),map.revision(),{{key,geometry::SE3(Eigen::Matrix3d::Identity(),{1,0,0})}},{{p0,{0,0,3}}});
  Check(status.ok() && map.GetMapPoint(p0).position_w().z()==3,"accepted geometry batch");
  Check(snapshot.points[0].position_w.z()==2,"snapshot detached from later writes");
  Check(!map.CommitGeometry(snapshot.epoch,snapshot.revision,{},{}).ok(),"stale revision rejected");
  auto f2=MakeFrame(factory,2); f2.Associate(0,p0);
  auto k2=map.InsertKeyFrame(f2);
  Check(map.GetKeyFrame(k2).image().empty(),"keyframes omit images by default");
  Check(map.RemoveMapPoint(p0) && !map.GetKeyFrame(key).associations()[0].valid() &&
        !map.GetKeyFrame(k2).associations()[0].valid() && map.CheckConsistency(),"cascading point removal");
  map.ProtectKeyFrame(key);
  Reject([&] { map.RemoveKeyFrame(key); },"protected keyframe removal");
  map.ProtectKeyFrame(key,false);
  Check(map.RemoveKeyFrame(key) && map.GetMapPoint(p1).observations().empty(),"keyframe edge cleanup");
  Check(!map.RemoveObservation(obs) && !map.RemoveMapPoint(p0),"deletion idempotence");
  Check(map.CheckConsistency(),"consistent after deletion");
  const auto old_epoch=map.epoch();
  map.Reset();
  Check(map.epoch()==old_epoch+1 && map.num_keyframes()==0 && map.num_map_points()==0,"reset advances epoch");
  const auto new_point=map.AddMapPoint({0,0,1});
  Check(new_point.value()==0 && new_point!=p0,"epoch distinguishes reused numeric ID");
  Reject([&] { map.InsertKeyFrame(f2); },"stale frame after reset");
  Check(!map.RemoveMapPoint(p0) && map.num_map_points()==1,"old ID cannot remove new point");
}
void TestRejectedPromotion() {
  core::Map map;
  core::FrameFactory factory(0,common::TimeBase::Simulation);
  auto frame=MakeFrame(factory,1);
  const auto p=map.AddMapPoint({0,0,2});
  frame.Associate(0,p); frame.Associate(1,p);
  auto rev=map.revision();
  Reject([&] { map.InsertKeyFrame(frame); },"repeated point promotion");
  Check(map.revision()==rev && map.num_keyframes()==0 && map.CheckConsistency(),"failed promotion unchanged");
  frame.RemoveAssociation(1);
  map.MarkMapPointBad(p);
  Reject([&] { map.InsertKeyFrame(frame); },"bad point promotion");
  map.MarkMapPointBad(p,false);
  map.InsertKeyFrame(frame);
  Reject([&] { map.InsertKeyFrame(frame); },"duplicate frame promotion");
  auto stale=MakeFrame(factory,2); stale.Associate(0,common::MapPointId(0,999));
  Reject([&] { map.InsertKeyFrame(stale); },"unknown point promotion");
  Check(map.CheckConsistency(),"consistent after rejected promotions");
}
int main() {
  try {
    TestCommonAndPose(); TestFrameOwnershipAndValidation(); TestMapTopologyAndCommit(); TestRejectedPromotion();
    std::cout << "Core ownership, topology, time, epoch, and geometry checks passed\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
