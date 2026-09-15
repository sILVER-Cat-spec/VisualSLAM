#include "vslam/core/map.h"
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>

// Inject failures into STL allocations to exercise rollback between index writes.
// Disabled during setup, checks, exception reporting, and OpenCV buffer allocation.
namespace { long allocations_left = -1; }
void* operator new(std::size_t size) {
  if (allocations_left == 0) throw std::bad_alloc();
  if (allocations_left > 0) --allocations_left;
  if (void* p=std::malloc(size ? size : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p,std::size_t) noexcept { std::free(p); }
void Check(bool condition) { if (!condition) throw std::runtime_error("Allocation rollback failed"); }
int main() {
  try {
    bool observation_success=false, promotion_success=false;
    int observation_failures=0, promotion_failures=0;
    for (long budget=0;budget<100 && (!observation_success || !promotion_success);++budget) {
      vslam::core::Map map;
      vslam::core::FrameFactory factory(0,vslam::common::TimeBase::Simulation);
      auto frame=factory.Create({1,vslam::common::TimeBase::Simulation},cv::Mat(4,4,CV_8U,cv::Scalar(0)));
      vslam::core::Keypoint a,b; a.uv={1,1}; b.uv={2,2};
      frame.SetFeatures(vslam::core::FeatureSet(vslam::core::DescriptorKind::Orb,{a,b},cv::Mat(2,32,CV_8U,cv::Scalar(0))));
      frame.SetPose(vslam::geometry::SE3());
      auto p0=map.AddMapPoint({0,0,1}),p1=map.AddMapPoint({1,0,1});
      auto k=map.InsertKeyFrame(frame);
      if (!observation_success) {
        const auto rev=map.revision();
        allocations_left=budget;
        try { map.AddObservation(k,p0,0); allocations_left=-1; observation_success=true; }
        catch(const std::bad_alloc&) {
          allocations_left=-1; ++observation_failures;
          Check(map.revision()==rev && map.num_observations()==0 && map.CheckConsistency());
        }
      }
      if (!promotion_success) {
        auto next=factory.Create({2,vslam::common::TimeBase::Simulation},frame.image());
        next.SetFeatures(frame.features()); next.SetPose(vslam::geometry::SE3());
        next.Associate(0,p0); next.Associate(1,p1);
        const auto rev=map.revision();
        const auto edges=map.num_observations();
        allocations_left=budget;
        try { map.InsertKeyFrame(next); allocations_left=-1; promotion_success=true; }
        catch(const std::bad_alloc&) {
          allocations_left=-1; ++promotion_failures;
          Check(map.revision()==rev && map.num_keyframes()==1 &&
                map.num_observations()==edges && map.CheckConsistency());
        }
      }
    }
    Check(observation_success && promotion_success && observation_failures>=3 && promotion_failures>=3);
    std::cout << "Rollback verified at " << observation_failures << " observation and "
              << promotion_failures << " promotion allocation points\n";
    return 0;
  } catch(const std::exception& e) { allocations_left=-1; std::cerr << e.what() << '\n'; return 1; }
}
