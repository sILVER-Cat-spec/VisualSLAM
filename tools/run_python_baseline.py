"""Run the unmodified sibling Python RGB-D baseline on the C++ manifest."""
import argparse
import csv
import json
import pathlib
import sys
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

parser=argparse.ArgumentParser()
parser.add_argument("manifest",type=pathlib.Path)
parser.add_argument("camera",type=pathlib.Path)
parser.add_argument("output",type=pathlib.Path)
parser.add_argument("--baseline",type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2]/"VisualSLAM")
args=parser.parse_args()
if args.output.exists():
    parser.error("Choose a new output directory")
sys.path.insert(0,str(args.baseline.resolve()))
from camera_model.rgbd_camera import RGBDCamera
from slam.system import SLAMSystem

cv2.setNumThreads(1)
cv2.setRNGSeed(7)
np.random.seed(7)
fs=cv2.FileStorage(str(args.camera),cv2.FILE_STORAGE_READ)
def scalar(name): return fs.getNode(name).real()
K=np.array([[scalar("fx"),0,scalar("cx")],[0,scalar("fy"),scalar("cy")],[0,0,1]])
camera=RGBDCamera("simulator",K,np.zeros(5),(int(scalar("width")),int(scalar("height"))),
                  depth_scale=scalar("depth_scale") or 1.0)
fs.release()
config={
 "frontend":{"detector":{"type":"ORB","n_features":2000,"scale_factor":1.2,"n_levels":8,"fast_threshold":20},
             "matcher":{"ratio_test":0.75,"cross_check":False}},
 "initialization":{"min_map_points":100,"min_depth":0.2,"max_depth":8.0,"depth_window_radius":1},
 "tracking":{"frame_tracker":{"min_correspondences":6,"min_inliers":15,"pnp_iterations":100,
                                "pnp_reprojection_error":4.0,"pnp_confidence":0.99},
             "local_map_tracker":{"min_correspondences":6,"min_inliers":15,"search_radius":20.0,"image_margin":8},
             "max_neighbors":5},
 "mapping":{"keyframe_selector":{"min_gap":2,"max_gap":5,"min_inliers":30,"min_translation":0.05,"min_rotation_deg":5.0},
            "point_builder":{"min_depth":0.2,"max_depth":8.0,"min_parallax_deg":1.0,"max_reprojection_error":3.0},
            "local_mapper":{"max_neighbors":5,"max_keyframes":8,"point_grace_period":3}},
 "optimization":{"pose_optimizer":{"pixel_sigma":1.0,"depth_sigma":0.03,"robust_kernel":"huber",
                                     "robust_delta":2.5,"outlier_threshold":5.0,"max_nfev":30},
                 "local_ba":{"pixel_sigma":1.0,"depth_sigma":0.03,"loss":"huber","huber_delta":2.5,
                             "outlier_threshold":5.0,"max_nfev":30,"use_depth":True}}}
slam=SLAMSystem(camera,config,rectify_input=False)
args.output.mkdir(parents=True)
(args.output/"effective_python_config.json").write_text(json.dumps(config,indent=2)+"\n")
entries=[line.split() for line in args.manifest.read_text().splitlines() if line and not line.startswith("#")]
trajectory=[]
successes=commits=attempts=0
start_all=time.perf_counter()
with (args.output/"frames.csv").open("w",newline="") as stream:
    writer=csv.writer(stream)
    writer.writerow(["timestamp_ns","state","success","features","inliers","keyframes","points","keyframe_inserted","ba_committed","seconds"])
    for i,(stamp,rgb,depth) in enumerate(entries):
        image=cv2.imread(str(args.manifest.parent/rgb))
        depth_image=cv2.imread(str(args.manifest.parent/depth),cv2.IMREAD_UNCHANGED)
        begin=time.perf_counter()
        result=slam.process(image,int(stamp)*1e-9,depth_image,frame_id=i)
        elapsed=time.perf_counter()-begin
        if result.success:
            successes+=1
            reference=slam.last_keyframe
            relative=result.frame.Tcw @ np.linalg.inv(reference.Tcw)
            trajectory.append((int(stamp),reference.id,relative))
        if result.ba is not None:
            attempts+=1
            commits+=bool(result.ba.accepted)
        inliers=result.local_tracking.num_inliers if result.local_tracking is not None else 0
        writer.writerow([stamp,result.state.value,int(result.success),result.frame.num_features,inliers,
                         len(slam.map.keyframes) if slam.map else 0,len(slam.map.map_points) if slam.map else 0,
                         int(result.keyframe is not None),int(result.ba.accepted) if result.ba is not None else 0,elapsed])
        stream.flush()
        if i%20==0: print(f"Python frame {i}: {result.state.value}, {inliers} inliers",flush=True)
with (args.output/"trajectory_tum.txt").open("w") as stream:
    stream.write("# simulation seconds; Twc; meters; qx qy qz qw; successful frames only\n")
    for stamp,reference,relative in trajectory:
        twc=np.linalg.inv(relative @ slam.map.keyframes[reference].Tcw)
        values=[stamp*1e-9,*twc[:3,3],*Rotation.from_matrix(twc[:3,:3]).as_quat()]
        stream.write(" ".join(format(float(v),".17g") for v in values)+"\n")
with (args.output/"map_points.csv").open("w",newline="") as stream:
    writer=csv.writer(stream)
    writer.writerow(["epoch","id","x_m","y_m","z_m","observations"])
    if slam.map:
        for p in slam.map.map_points.values():
            writer.writerow([0,p.id,*p.position_w,p.num_observations])
summary={"frames":len(entries),"successful_frames":successes,"keyframes":len(slam.map.keyframes) if slam.map else 0,
         "map_points":len(slam.map.map_points) if slam.map else 0,"ba_attempts":attempts,"ba_committed":commits,
         "elapsed_seconds":time.perf_counter()-start_all,"opencv_version":cv2.__version__,
         "baseline_path":str(args.baseline.resolve()),"random_seed":7}
(args.output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
print(json.dumps(summary,indent=2),flush=True)
