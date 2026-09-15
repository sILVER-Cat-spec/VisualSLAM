#include "vslam/io/dataset_io.h"
#include "vslam/io/point_cloud_writer.h"
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
struct TrajectoryEntry {
  std::size_t dataset_index;
  std::int64_t timestamp_ns;
  vslam::common::KeyFrameId reference;
  vslam::geometry::SE3 frame_from_reference;
};
// Run an offline RGB-D sequence and export tracking statistics, trajectory, and point clouds.
int main(int argc,char** argv) {
  try {
    if(argc<4 || argc>5) {
      std::cerr<<"Usage: run_dataset manifest.txt camera.yaml output_directory [max_frames]\n"; return 2;
    }

    // read config and manifest
    const auto config=vslam::io::ReadConfiguration(argv[2]);
    const auto entries=vslam::io::ReadManifest(argv[1]);

    const std::filesystem::path output=argv[3];
    if(std::filesystem::exists(output/"frames.csv")) {
      throw std::runtime_error("Output already contains a run; choose a new directory");
    }
    std::filesystem::create_directories(output);

    vslam::io::WriteConfiguration(output/"effective_config.yaml",config);

    std::size_t limit=entries.size();
    if(argc==5) {
      const int n=std::stoi(argv[4]);
      if(n<1) {
        throw std::invalid_argument("max_frames must be positive");
      }
      limit=std::min(limit,static_cast<std::size_t>(n));
    }

    cv::setNumThreads(1);
    cv::setRNGSeed(7);

    vslam::system::SlamSystem slam(config.camera,config.slam);

    std::ofstream stats(output/"frames.csv");
    stats.exceptions(std::ios::failbit|std::ios::badbit);
    stats << "timestamp_ns,state,success,features,inliers,keyframes,points,"
             "keyframe_inserted,ba_committed,seconds,ba_initial_cost,ba_final_cost\n";

    std::ofstream ba_log(output/"ba.log");
    ba_log.exceptions(std::ios::failbit|std::ios::badbit);

    std::vector<TrajectoryEntry> trajectory;
    std::optional<vslam::common::KeyFrameId> reference;
    std::size_t successes=0,commits=0,attempts=0;

    for(std::size_t i=0;i<limit;++i) {
      auto result=slam.Process(vslam::io::ReadFrame(entries[i],config.depth_scale));
      if(!result.status.ok()) throw std::runtime_error(result.status.message());
      if(result.inserted_keyframe) reference=result.inserted_keyframe;
      if(result.tcw) {
        ++successes;
        trajectory.push_back({i,entries[i].timestamp_ns,*reference,
            *result.tcw*slam.map().GetKeyFrame(*reference).pose().Inverse()});
      }
      commits+=result.ba_committed;
      if(result.mapping) {
        ++attempts;
        ba_log<<i<<": "<<result.mapping->ba.message<<'\n';
      }
      stats<<entries[i].timestamp_ns<<','<<vslam::system::StateName(result.state)
           <<','<<result.tcw.has_value()<<','<<result.features<<','<<result.inliers
           <<','<<result.keyframes<<','<<result.map_points
           <<','<<result.inserted_keyframe.has_value()<<','<<result.ba_committed
           <<','<<result.elapsed_seconds<<','<<(result.mapping?result.mapping->ba.initial_cost:0)
           <<','<<(result.mapping?result.mapping->ba.final_cost:0)<<'\n';
      if(!slam.map().CheckConsistency()) throw std::runtime_error("Map graph consistency failed");
      if(i%20==0) std::cout<<"frame "<<i<<": "<<vslam::system::StateName(result.state)<<", "<<result.inliers<<" inliers\n";
    }

    std::ofstream poses(output/"trajectory_tum.txt");
    poses.exceptions(std::ios::failbit|std::ios::badbit);
    poses<<"# simulation seconds; Twc; meters; quaternion qx qy qz qw; successful frames only\n"<<std::setprecision(17);

    for(const auto& entry:trajectory) {
      const auto twc=(entry.frame_from_reference*slam.map().GetKeyFrame(entry.reference).pose()).Inverse();
      const Eigen::Quaterniond q(twc.rotation()); const auto& t=twc.translation();
      poses<<static_cast<double>(entry.timestamp_ns)*1e-9<<' '<<t.x()<<' '<<t.y()<<' '<<t.z()<<' '
           <<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
    }

    std::ofstream points(output/"map_points.csv");
    points.exceptions(std::ios::failbit|std::ios::badbit);
    points<<"epoch,id,x_m,y_m,z_m,observations\n"<<std::setprecision(17);
    for(const auto& p:slam.map().Snapshot().points){
      points<<p.id.epoch()<<','<<p.id.value()<<','<<p.position_w.x()<<','<<p.position_w.y()<<','<<p.position_w.z()
            <<','<<slam.map().GetMapPoint(p.id).observations().size()<<'\n';
    }

    std::ofstream keyframes(output/"keyframes.csv");
    keyframes.exceptions(std::ios::failbit|std::ios::badbit);
    keyframes<<"epoch,id,source_frame_id,source_sequence,timestamp_ns,tx_m,ty_m,tz_m,qx,qy,qz,qw\n"<<std::setprecision(17);
    const auto snapshot=slam.map().Snapshot();
    for(const auto& k:snapshot.keyframes) {
      const auto twc=k.tcw.Inverse(); const Eigen::Quaterniond q(twc.rotation()); const auto& t=twc.translation();
      keyframes<<k.id.epoch()<<','<<k.id.value()<<','<<k.source_frame_id.value()<<','<<k.source_sequence
               <<','<<k.timestamp.ns<<','<<t.x()<<','<<t.y()<<','<<t.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';
    }

    std::ofstream observations(output/"observations.csv");
    observations.exceptions(std::ios::failbit|std::ios::badbit);
    observations<<"epoch,id,keyframe_id,map_point_id,feature_index,u,v,depth_z_m\n"<<std::setprecision(17);
    for(const auto& o:snapshot.observations) {
      observations<<o.id().epoch()<<','<<o.id().value()<<','<<o.keyframe_id().value()<<','<<o.map_point_id().value()
                  <<','<<o.feature_index()<<','<<o.uv().x()<<','<<o.uv().y()<<',';
      if(o.depth_z_m()) observations<<*o.depth_z_m();
      observations<<'\n';
    }

    std::vector<vslam::io::CloudFrame> cloud_frames;
    for(const auto& entry:trajectory)
      cloud_frames.push_back({entries[entry.dataset_index], (entry.frame_from_reference*slam.map().GetKeyFrame(entry.reference).pose()).Inverse()});
    const auto dense_points=vslam::io::WriteRGBDCloud(output/"rgbd_cloud.ply",cloud_frames,config,8);
    vslam::io::WriteSparseCloud(output/"map_points.ply",snapshot);
    std::cout<<"Exported "<<dense_points<<" colored RGB-D points"<<std::endl;

    std::ofstream summary(output/"summary.json");
    summary.exceptions(std::ios::failbit|std::ios::badbit);
    summary<<"{\n  \"frames\": "<<limit<<",\n  \"successful_frames\": "<<successes
           <<",\n  \"keyframes\": "<<slam.map().num_keyframes()<<",\n  \"map_points\": "<<slam.map().num_map_points()
           <<",\n  \"ba_attempts\": "<<attempts<<",\n  \"ba_committed\": "<<commits
           <<",\n  \"opencv_version\": \""<<CV_VERSION<<"\",\n  \"random_seed\": 7,\n  \"metric_scale\": "<<(slam.map().num_keyframes()?"true":"false")<<"\n}\n";
    std::cout<<"Finished: "<<successes<<'/'<<limit<<" successful frames, "<<commits<<'/'<<attempts<<" BA commits\n";
    return successes?0:1;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
