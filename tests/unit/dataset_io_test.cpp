#include "vslam/io/dataset_io.h"
#include "vslam/io/point_cloud_writer.h"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
void Check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main() {
  try {
    const std::filesystem::path root="dataset_io_test_output";
    std::filesystem::create_directories(root);
    const auto path=root/"config.yaml";
    const vslam::io::DatasetConfiguration original{vslam::sensor::CameraModel(500,510,320,240,640,480),{},1.0};
    vslam::io::WriteConfiguration(path,original);
    const auto restored=vslam::io::ReadConfiguration(path);
    Check(restored.camera.fy()==510 && restored.slam.frontend.n_features==2000 &&
          restored.slam.mapping.max_ba_points==300,"configuration roundtrip");
    {std::ofstream file(path,std::ios::app); file<<"unknown_option: 1\n";}
    bool rejected=false;
    try {vslam::io::ReadConfiguration(path);} catch(const std::invalid_argument&) {rejected=true;}
    Check(rejected,"unknown config key rejected");
    const auto manifest=root/"manifest.txt";
    {std::ofstream file(manifest); file<<"# nanoseconds RGB depth\n1 rgb.png depth.tiff\n2 rgb2.png depth2.tiff\n";}
    Check(vslam::io::ReadManifest(manifest).size()==2,"manifest parser");
    {std::ofstream file(manifest,std::ios::app); file<<"2 rgb3.png depth3.tiff\n";}
    rejected=false;
    try {vslam::io::ReadManifest(manifest);} catch(const std::invalid_argument&) {rejected=true;}
    Check(rejected,"duplicate timestamps rejected");
    vslam::io::DatasetConfiguration cloud_config{vslam::sensor::CameraModel(2,2,0.5,0.5,2,2),{},1.0};
    cv::imwrite((root/"rgb.png").string(),cv::Mat(2,2,CV_8UC3,cv::Scalar(30,20,10)));
    cv::imwrite((root/"depth.tiff").string(),cv::Mat(2,2,CV_32FC1,cv::Scalar(2)));
    vslam::io::CloudFrame cloud_frame{{1,root/"rgb.png",root/"depth.tiff"},
        vslam::geometry::SE3(Eigen::Matrix3d::Identity(),{1,0,0})};
    const auto ply=root/"cloud.ply";
    Check(vslam::io::WriteRGBDCloud(ply,{cloud_frame},cloud_config,1)==4,"cloud point count");
    std::ifstream cloud(ply,std::ios::binary); std::string line;
    while(std::getline(cloud,line) && line!="end_header") {}
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(cloud)),{});
    Check(bytes.size()==60 && bytes[12]==10 && bytes[13]==20 && bytes[14]==30,"PLY binary layout and RGB");
    Check(bytes[0]==0 && bytes[1]==0 && bytes[2]==0 && bytes[3]==63,"world X is 0.5 float32 little endian");
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
