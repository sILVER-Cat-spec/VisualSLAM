#!/usr/bin/env python3
"""Render this project's binary XYZ/RGB PLY export and estimated camera path."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("cloud", type=Path)
parser.add_argument("trajectory", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
with args.cloud.open("rb") as stream:
    header = []
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("Missing PLY header")
        header.append(line.decode("ascii").strip())
        if line == b"end_header\n":
            break
    if "format binary_little_endian 1.0" not in header:
        raise ValueError("Expected project binary PLY format")
    count = int(next(line for line in header if line.startswith("element vertex ")).split()[-1])
    dtype = np.dtype([("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])
    points = np.fromfile(stream, dtype=dtype, count=count)
    if len(points) != count or stream.read(1):
        raise ValueError("Unexpected PLY payload size")
sample = points[::max(1, len(points) // 80000)]
trajectory = np.atleast_2d(np.loadtxt(args.trajectory))
fig = plt.figure(figsize=(12, 9), constrained_layout=True)
ax = fig.add_subplot(111, projection="3d")
xyz = sample["xyz"]
ax.scatter(*xyz.T, c=sample["rgb"] / 255.0, s=.6, depthshade=False, rasterized=True)
# Do not connect failed intervals in the estimated path.
step = np.median(np.diff(trajectory[:, 0]))
for segment in np.split(trajectory, np.flatnonzero(np.diff(trajectory[:, 0]) > step * 1.5) + 1):
    ax.plot(*segment[:, 1:4].T, color="#ff2945", lw=2, label=None)
ax.scatter([0], [0], [0], c="#ff2945", marker="*", s=100)
ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
       title=f"C++ RGB-D reconstruction | {count:,} colored points\n"
             "Preview subsampled; red = camera trajectory; star = first camera")
ax.set_box_aspect(np.maximum(np.ptp(xyz, axis=0), .01))
ax.view_init(elev=-65, azim=-90)
fig.savefig(args.output, dpi=150)
print(f"Validated {count} PLY vertices; preview contains {len(sample)} points")