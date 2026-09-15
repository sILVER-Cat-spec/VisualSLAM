#include "vslam/io/point_cloud_writer.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
namespace vslam::io {
namespace {
struct Vertex { float x,y,z; unsigned char r,g,b; };
void Write(const std::filesystem::path& path,const std::vector<Vertex>& vertices) {
  static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
  std::ofstream file(path,std::ios::binary);
  file.exceptions(std::ios::failbit|std::ios::badbit);
  file<<"ply\nformat binary_little_endian 1.0\ncomment XYZ in estimated world meters\nelement vertex "<<vertices.size()
      <<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
  for(const auto& v:vertices) {
    std::array<unsigned char,15> bytes{};
    const float xyz[3]={v.x,v.y,v.z};
    for(int i=0;i<3;++i) {
      std::uint32_t bits=0; std::memcpy(&bits,&xyz[i],4);
      for(int b=0;b<4;++b) bytes[4*i+b]=static_cast<unsigned char>((bits>>(8*b))&255);
    }
    bytes[12]=v.r; bytes[13]=v.g; bytes[14]=v.b;
    file.write(reinterpret_cast<const char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
  }
}
}
std::size_t WriteRGBDCloud(const std::filesystem::path& path,const std::vector<CloudFrame>& frames,
                          const DatasetConfiguration& config,int stride) {
  if(stride<1) throw std::invalid_argument("Point cloud stride must be positive");
  std::vector<Vertex> vertices;
  for(const auto& frame:frames) {
    const auto prepared=sensor::Prepare(ReadFrame(frame.input,config.depth_scale),config.camera);
    for(int v=0;v<config.camera.height();v+=stride)
      for(int u=0;u<config.camera.width();u+=stride) {
        const double z=prepared.depth_z_m.at<float>(v,u);
        if(!std::isfinite(z) || z<config.slam.frontend.min_depth || z>config.slam.frontend.max_depth) continue;
        const auto pc=config.camera.Unproject({static_cast<double>(u),static_cast<double>(v)},z);
        if(!pc) continue;
        const auto pw=frame.twc * *pc;
        const auto color=prepared.image_rgb_or_gray.at<cv::Vec3b>(v,u);
        const Vertex point{static_cast<float>(pw.x()),static_cast<float>(pw.y()),static_cast<float>(pw.z()),color[0],color[1],color[2]};
        if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
          throw std::runtime_error("Point cloud coordinate overflow");
        vertices.push_back(point);
      }
  }
  Write(path,vertices);
  return vertices.size();
}
void WriteSparseCloud(const std::filesystem::path& path,const core::MapSnapshot& snapshot) {
  std::vector<Vertex> vertices;
  for(const auto& p:snapshot.points) if(!p.is_bad)
    vertices.push_back({static_cast<float>(p.position_w.x()),static_cast<float>(p.position_w.y()),
                       static_cast<float>(p.position_w.z()),30,200,100});
  Write(path,vertices);
}
}
