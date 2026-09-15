"""Read-only ROS2 SQLite export of exact RGB/depth pairs for run_dataset.

Source /opt/ros/humble/setup.bash first. Output depth TIFF preserves float32.
The exporter uses K and reports the known P mismatch; it does not repair the bag.
"""
import argparse
import json
import pathlib
import sqlite3

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image

parser = argparse.ArgumentParser()
parser.add_argument("bag", type=pathlib.Path)
parser.add_argument("output", type=pathlib.Path)
parser.add_argument("--max-frames", type=int, default=120)
parser.add_argument("--stride", type=int, default=1)
args = parser.parse_args()
if args.max_frames < 1 or args.stride < 1:
    parser.error("max-frames and stride must be positive")
if args.output.exists():
    parser.error("choose a new output directory")
databases = list(args.bag.glob("*.db3"))
if len(databases) != 1:
    parser.error("this exporter requires a single SQLite database per bag")
connection = sqlite3.connect(databases[0].resolve().as_uri() + "?mode=ro", uri=True)
topics = dict(connection.execute("SELECT name,id FROM topics"))
rgb_id = topics["/camera/color/image_raw"]
depth_id = topics["/camera/aligned_depth_to_color/image_raw"]
info_id = topics["/camera/color/camera_info"]
info = deserialize_message(connection.execute(
    "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1", (info_id,)
).fetchone()[0], CameraInfo)
K = np.array(info.k).reshape(3, 3)
if not np.isfinite(K).all() or not np.allclose(info.d, 0) or not np.allclose(np.array(info.r).reshape(3,3), np.eye(3)):
    raise ValueError("export requires a finite undistorted rectified camera")
rows = connection.execute(
    "SELECT r.id,d.id FROM messages r JOIN messages d ON r.timestamp=d.timestamp "
    "WHERE r.topic_id=? AND d.topic_id=? ORDER BY r.timestamp,r.id LIMIT ?",
    (rgb_id, depth_id, args.max_frames * args.stride),
).fetchall()[::args.stride]
if not rows:
    raise ValueError("No exact bag-timestamp pairs; use bags with header-based timeline")
args.output.mkdir(parents=True)
manifest = []
last_stamp = -1
for index, (rgb_row, depth_row) in enumerate(rows):
    def read(row_id):
        return deserialize_message(connection.execute("SELECT data FROM messages WHERE id=?", (row_id,)).fetchone()[0], Image)
    rgb, depth = read(rgb_row), read(depth_row)
    stamp = rgb.header.stamp.sec * 10**9 + rgb.header.stamp.nanosec
    depth_stamp = depth.header.stamp.sec * 10**9 + depth.header.stamp.nanosec
    if stamp != depth_stamp or stamp <= last_stamp:
        raise ValueError("Header timestamps do not form increasing exact pairs")
    last_stamp = stamp
    if (rgb.width, rgb.height) != (info.width, info.height) or (depth.width, depth.height) != (info.width, info.height):
        raise ValueError("Images do not match CameraInfo")
    if rgb.header.frame_id != info.header.frame_id or depth.header.frame_id != info.header.frame_id:
        raise ValueError("Image and calibration frame IDs differ")
    if rgb.encoding != "rgb8" or depth.encoding != "32FC1":
        raise ValueError("Exporter expects rgb8 and meter-valued 32FC1")
    image = np.ndarray((rgb.height, rgb.width, 3), dtype=np.uint8, buffer=rgb.data, strides=(rgb.step, 3, 1))
    depth_image = np.ndarray((depth.height, depth.width), dtype=">f4" if depth.is_bigendian else "<f4",
                             buffer=depth.data, strides=(depth.step,4)).astype(np.float32)
    color_name, depth_name = f"rgb_{index:06d}.png", f"depth_{index:06d}.tiff"
    if not cv2.imwrite(str(args.output/color_name), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
        raise RuntimeError("Failed writing RGB image")
    if not cv2.imwrite(str(args.output/depth_name), depth_image):
        raise RuntimeError("Failed writing depth image")
    manifest.append(f"{stamp} {color_name} {depth_name}\n")
(args.output/"manifest.txt").write_text("".join(manifest))
(args.output/"camera.yaml").write_text(
    f"%YAML:1.0\n---\nschema_version: 1\nfx: {K[0,0]}\nfy: {K[1,1]}\ncx: {K[0,2]}\ncy: {K[1,2]}\n"
    f"width: {info.width}\nheight: {info.height}\ndepth_scale: 1.0\n")
report = {"source": str(args.bag.resolve()), "frames": len(rows), "stride": args.stride,
          "K": info.k.tolist(), "P": info.p.tolist(), "P_matches_K": bool(np.allclose(np.array(info.p).reshape(3,4)[:,:3], K)),
          "depth_convention": "registered camera Z meters; inherited simulation contract"}
(args.output/"export_report.json").write_text(json.dumps(report, indent=2)+"\n")
print(json.dumps(report, indent=2))
connection.close()
