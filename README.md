# VisualSLAM

A C++17 visual SLAM project for personal learning.

## Versions

- `main`: RGB-D input and SLAM.
- `features-stereo`: stereo-to-depth input for the shared RGB-D pipeline, with both runners available.

## Build and test

Requires CMake 3.16+, a C++17 compiler, Eigen3, OpenCV (core, imgproc, features2d, calib3d, imgcodecs), Ceres Solver, and threads.

```sh
git clone git@github.com:sILVER-Cat-spec/VisualSLAM.git
cd VisualSLAM
git switch main
cmake -S . -B build/main -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/main -j2
ctest --test-dir build/main --output-on-failure
```

## Run RGB-D SLAM

Input images must be rectified, with depth registered to color. Configure camera intrinsics and `depth_scale` (meters per stored depth unit); the supplied configuration is a simulation example.

```sh
./build/main/run_dataset /path/to/manifest.txt config/rgbd_simulation.yaml results/rgbd_run
```

Manifest rows contain increasing timestamps in nanoseconds, RGB paths, and depth paths. Relative paths resolve from the manifest directory; `#` starts a comment.

```text
1000000000 rgb/000001.png depth/000001.exr
1033333333 rgb/000002.png depth/000002.exr
```

Choose a new output directory for each run. Additional examples: `rgbd_camera_example`, `core_data_example`, and `initial_mapping_example` (configuration, manifest, new output directory; its parent must exist).

The optional `VSLAM_TEST_PYTHON_BASELINE` check requires a separate Python reference project, NumPy, and OpenCV; it is disabled by default. Python utilities are in `tools/`.

Build products, datasets, logs, generated results, local agent instructions, and Windows download metadata are excluded from version control.

## Run stereo-to-depth SLAM (`features-stereo`)

```sh
git switch features-stereo
cmake -S . -B build/stereo -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/stereo -j2
ctest --test-dir build/stereo --output-on-failure
./build/stereo/run_stereo_dataset config/stereo_simulation.yaml /path/to/stereo_manifest.txt results/stereo_run
```

The runner uses rectified, synchronized left/right images. OpenCV SGBM estimates disparity, the adapter converts valid disparity to metric depth, and the shared RGB-D pipeline estimates the trajectory. The CMake configuration selects this route with `VSLAM_DEPTH_ROUTE=1`.

Set calibrated intrinsics, `right_cx`, and `baseline_m` in the stereo configuration. The runner requires schema version 2 and `rectified: 1`; raw images must be rectified before running it.

The manifest requires the exact first-line header below, followed by left/right timestamps in nanoseconds and image paths. Each timestamp sequence must increase, and paired timestamps must match. Relative image paths resolve from the manifest directory.

```text
# stereo_manifest_version=2
1000000000 1000000000 left/000001.png right/000001.png
1033333333 1033333333 left/000002.png right/000002.png
```

The RGB-D runner and examples remain available in `build/stereo/` with the same arguments documented above.
