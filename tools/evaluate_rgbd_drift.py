#!/usr/bin/env python3
"""Evaluate first-pose-aligned metric trajectories against recorded ROS TF.
TF is evaluation-only: this program never feeds reference poses into SLAM.
Run after sourcing /opt/ros/humble/setup.bash.
"""
import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def matrix(transform):
    t = transform.translation
    q = transform.rotation
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [t.x, t.y, t.z]
    return result


def reference_from_bag(path, stamps):
    database = next(path.glob("*.db3")) if path.is_dir() else path
    with sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True) as connection:
        topics = dict(connection.execute("select name,id from topics"))
        static = {}
        for data, in connection.execute(
                "select data from messages where topic_id=? order by timestamp",
                (topics["/tf_static"],)):
            for t in deserialize_message(data, TFMessage).transforms:
                static[t.child_frame_id] = (t.header.frame_id, matrix(t.transform))
        samples = {}
        for data, in connection.execute(
                "select data from messages where topic_id=? order by timestamp",
                (topics["/tf"],)):
            transforms = deserialize_message(data, TFMessage).transforms
            if not transforms:
                continue
            packet_stamps = {t.header.stamp.sec * 10**9 + t.header.stamp.nanosec
                             for t in transforms}
            if len(packet_stamps) != 1:
                raise ValueError("TF packet has mixed times; asynchronous TF needs a buffer")
            edges = dict(static)
            edges.update({t.child_frame_id: (t.header.frame_id, matrix(t.transform))
                          for t in transforms})
            child, pose, visited = "camera_optical_frame", np.eye(4), set()
            while child != "world":
                if child in visited or child not in edges:
                    raise ValueError("Incomplete or cyclic world-to-camera TF chain")
                visited.add(child)
                parent, edge = edges[child]
                pose = edge @ pose
                child = parent
            samples[next(iter(packet_stamps))] = pose
    times_ns = np.array(sorted(samples), dtype=np.int64)
    if stamps.min() < times_ns[0] or stamps.max() > times_ns[-1]:
        raise ValueError("Camera timestamps exceed TF coverage; refusing extrapolation")
    poses = np.array([samples[t] for t in times_ns])
    # Subtract the origin before converting to seconds to preserve timestamp precision.
    times = (times_ns - stamps[0]) * 1e-9
    target = (stamps - stamps[0]) * 1e-9
    reference = np.tile(np.eye(4), (len(stamps), 1, 1))
    reference[:, :3, :3] = Slerp(times, Rotation.from_matrix(poses[:, :3, :3]))(target).as_matrix()
    for axis in range(3):
        reference[:, axis, 3] = np.interp(target, times, poses[:, axis, 3])
    reference = np.linalg.inv(reference[0]) @ reference
    return reference, float(np.max(np.diff(times)))


def read_estimate(path, stamps):
    rows = np.atleast_2d(np.loadtxt(path))
    indices = {int(t): i for i, t in enumerate(stamps)}
    result = np.full((len(stamps), 4, 4), np.nan)
    for row in rows:
        stamp = int(round(row[0] * 1e9))
        if stamp not in indices:
            raise ValueError("Estimate timestamp absent from input manifest")
        i = indices[stamp]
        if np.isfinite(result[i]).any():
            raise ValueError("Duplicate estimate timestamp")
        result[i] = np.eye(4)
        result[i, :3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
        result[i, :3, 3] = row[1:4]
    if not np.isfinite(result[0]).all():
        raise ValueError("First input pose required for first-pose alignment")
    return np.linalg.inv(result[0]) @ result


def metrics(reference, estimate, mask):
    translation = np.linalg.norm(estimate[mask, :3, 3] - reference[mask, :3, 3], axis=1)
    relative_rotation = np.swapaxes(reference[mask, :3, :3], 1, 2) @ estimate[mask, :3, :3]
    angle = Rotation.from_matrix(relative_rotation).magnitude() * 180 / np.pi
    consecutive = np.flatnonzero(mask[1:] & mask[:-1]) + 1
    errors = []
    for i in consecutive:
        truth_delta = np.linalg.inv(reference[i - 1]) @ reference[i]
        estimated_delta = np.linalg.inv(estimate[i - 1]) @ estimate[i]
        errors.append(np.linalg.inv(truth_delta) @ estimated_delta)
    errors = np.array(errors)
    return {
        "evaluated_frames": int(mask.sum()),
        "position_rmse_m": float(np.sqrt(np.mean(translation**2))),
        "position_median_m": float(np.median(translation)),
        "position_max_m": float(translation.max()),
        "last_successful_position_error_m": float(translation[-1]),
        "rotation_rmse_deg": float(np.sqrt(np.mean(angle**2))),
        "last_successful_rotation_error_deg": float(angle[-1]),
        "adjacent_successful_pairs": len(consecutive),
        "adjacent_relative_translation_rmse_m": (
            float(np.sqrt(np.mean(np.sum(errors[:, :3, 3]**2, axis=1)))) if len(errors) else None),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("cpp", type=Path)
    parser.add_argument("python", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stamps = np.array([int(line.split()[0]) for line in args.manifest.read_text().splitlines()
                       if line.strip() and not line.startswith("#")], dtype=np.int64)
    reference, gap = reference_from_bag(args.bag, stamps)
    estimates = {name: read_estimate(folder / "trajectory_tum.txt", stamps)
                 for name, folder in (("C++", args.cpp), ("Python", args.python))}
    masks = {name: np.isfinite(pose).all(axis=(1, 2)) for name, pose in estimates.items()}
    common = np.logical_and.reduce(list(masks.values()))
    report = {"reference": "recorded world -> camera_optical_frame kinematic TF",
              "alignment": "first input pose only, fixed metric scale; no best-fit alignment",
              "input_frames": len(stamps), "duration_s": float((stamps[-1] - stamps[0]) * 1e-9),
              "maximum_tf_interval_s": gap,
              "reference_path_length_m": float(np.linalg.norm(np.diff(reference[:, :3, 3], axis=0), axis=1).sum()),
              "common_successful_frames": int(common.sum()), "estimators": {}}
    for name, estimate in estimates.items():
        report["estimators"][name] = {
            "own_successful_frames": metrics(reference, estimate, masks[name]),
            "common_successful_frames": metrics(reference, estimate, common)}
    (args.output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    quaternion = Rotation.from_matrix(reference[:, :3, :3]).as_quat()
    np.savetxt(args.output / "reference_tum.txt",
               np.column_stack((stamps * 1e-9, reference[:, :3, 3], quaternion)),
               header="simulation seconds; first-pose-normalized kinematic reference Twc")

    plt.rcParams.update({"font.size": 10})
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    colors = {"C++": "#d05a27", "Python": "#287bc1"}
    for ax, (a, b) in zip(axes[0], ((0, 1), (0, 2))):
        ax.plot(reference[:, a, 3], reference[:, b, 3], color="black", label="TF reference", lw=2)
        for name, estimate in estimates.items():
            ax.plot(estimate[:, a, 3], estimate[:, b, 3], color=colors[name], label=name, lw=1.4)
        ax.scatter([0], [0], marker="*", color="black", s=80, zorder=5)
        ax.set(xlabel="XYZ"[a] + " (m)", ylabel="XYZ"[b] + " (m)", title="Trajectory projection")
        ax.axis("equal")
        ax.legend()
    time = (stamps - stamps[0]) * 1e-9
    for name, estimate in estimates.items():
        mask = masks[name]
        position = np.linalg.norm(estimate[:, :3, 3] - reference[:, :3, 3], axis=1)
        angle = np.full(len(stamps), np.nan)
        angle[mask] = Rotation.from_matrix(
            np.swapaxes(reference[mask, :3, :3], 1, 2) @ estimate[mask, :3, :3]).magnitude() * 180 / np.pi
        axes[1, 0].plot(time, position * 100, color=colors[name], label=name)
        axes[1, 1].plot(time, angle, color=colors[name], label=name)
    axes[1, 0].set(xlabel="Elapsed simulation time (s)", ylabel="Position error (cm)",
                   title="Position error (gaps mean tracking loss)")
    axes[1, 1].set(xlabel="Elapsed simulation time (s)", ylabel="Orientation error (degrees)",
                   title="Orientation error")
    for ax in axes.flat:
        ax.grid(alpha=.25)
    fig.suptitle("RGB-D drift comparison | first-pose alignment, fixed scale\n"
                 "324 identical input frames | recorded kinematic TF reference", fontsize=14)
    fig.savefig(args.output / "trajectory_comparison.png", dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()