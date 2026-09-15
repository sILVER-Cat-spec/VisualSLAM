#include "vslam/frontend/feature_extractor.h"
#include "vslam/frontend/feature_matcher.h"
#include "vslam/sensor/image_preprocessor.h"
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
namespace vslam::frontend {
FeatureExtractor::FeatureExtractor(ExtractorOptions options):options_(options) {
  if(options.n_features<1 || options.n_features>100000 || !std::isfinite(options.scale_factor) ||
     options.scale_factor<=1 || options.n_levels<1 || options.n_levels>32 ||
     options.fast_threshold<0 || options.depth_radius<0 || options.depth_radius>16 ||
     !std::isfinite(options.min_depth) || !std::isfinite(options.max_depth) ||
     options.min_depth<=0 || options.max_depth<=options.min_depth ||
     (options.kind!=core::DescriptorKind::Orb && options.kind!=core::DescriptorKind::Sift))
    throw std::invalid_argument("Invalid feature extractor options");
}
core::FeatureSet FeatureExtractor::Extract(const cv::Mat& image,const cv::Mat& depth) const {
  cv::Mat gray;
  if(image.type()==CV_8UC3) cv::cvtColor(image,gray,cv::COLOR_RGB2GRAY);
  else if(image.type()==CV_8UC1) gray=image;
  else throw std::invalid_argument("Extractor requires RGB8 or gray8");
  if(image.empty() || depth.type()!=CV_32FC1 || depth.size()!=image.size())
    throw std::invalid_argument("Extractor requires aligned Z depth");
  cv::Ptr<cv::Feature2D> detector;
  if(options_.kind==core::DescriptorKind::Orb)
    detector=cv::ORB::create(options_.n_features,options_.scale_factor,options_.n_levels,31,0,2,
                             cv::ORB::HARRIS_SCORE,31,options_.fast_threshold);
  else detector=cv::SIFT::create(options_.n_features);
  std::vector<cv::KeyPoint> points; cv::Mat descriptors;
  detector->detectAndCompute(gray,cv::noArray(),points,descriptors);
  std::vector<core::Keypoint> result;
  result.reserve(points.size());
  for(const auto& p:points) {
    core::Keypoint k;
    k.uv={p.pt.x,p.pt.y}; k.octave=p.octave; k.angle=p.angle;
    k.response=p.response; k.size=p.size;
    k.depth_z_m=sensor::SampleDepth(depth,k.uv,options_.min_depth,options_.max_depth,options_.depth_radius);
    result.push_back(k);
  }
  return core::FeatureSet(options_.kind,std::move(result),descriptors);
}
FeatureMatcher::FeatureMatcher(MatcherOptions options):options_(options) {
  if(!std::isfinite(options.ratio) || options.ratio<=0 || options.ratio>=1)
    throw std::invalid_argument("Ratio must lie between zero and one");
}
std::vector<Match> FeatureMatcher::MatchDescriptors(const cv::Mat& a,const cv::Mat& b) const {
  if(a.empty() || b.empty()) return {};
  if(a.dims!=2 || b.dims!=2 || a.type()!=b.type() || a.cols!=b.cols ||
     (a.type()!=CV_8UC1 && a.type()!=CV_32FC1))
    throw std::invalid_argument("Descriptor matrices are incompatible");
  cv::BFMatcher matcher(a.type()==CV_8UC1?cv::NORM_HAMMING:cv::NORM_L2);
  std::vector<std::vector<cv::DMatch>> forward,reverse;
  matcher.knnMatch(a,b,forward,2);
  if(options_.mutual) matcher.knnMatch(b,a,reverse,2);
  std::vector<Match> candidates;
  for(const auto& pair:forward) {
    if(pair.size()!=2 || pair[0].distance>=options_.ratio*pair[1].distance) continue;
    const auto& m=pair[0];
    if(options_.mutual) {
      const auto& back=reverse.at(static_cast<std::size_t>(m.trainIdx));
      if(back.size()!=2 || back[0].distance>=options_.ratio*back[1].distance || back[0].trainIdx!=m.queryIdx) continue;
    }
    candidates.push_back({static_cast<std::size_t>(m.queryIdx),static_cast<std::size_t>(m.trainIdx),m.distance});
  }
  std::sort(candidates.begin(),candidates.end(),[](const Match& x,const Match& y) {
    if(x.distance!=y.distance) return x.distance<y.distance;
    if(x.first!=y.first) return x.first<y.first;
    return x.second<y.second;
  });
  std::set<std::size_t> used;
  std::vector<Match> result;
  for(const auto& m:candidates) if(used.insert(m.second).second) result.push_back(m);
  return result;
}
}
