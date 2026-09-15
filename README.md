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
./build/main/run_dataset config/rgbd_simulation.yaml /path/to/manifest.txt results/rgbd_run
```

Manifest rows contain increasing timestamps in nanoseconds, RGB paths, and depth paths. Relative paths resolve from the manifest directory; `#` starts a comment.

```text
1000000000 rgb/000001.png depth/000001.exr
1033333333 rgb/000002.png depth/000002.exr
```

Choose a new output directory for each run. Additional examples: `rgbd_camera_example`, `core_data_example`, and `initial_mapping_example` (configuration, manifest, new output directory; its parent must exist).

The optional `VSLAM_TEST_PYTHON_BASELINE` check requires a separate Python reference project, NumPy, and OpenCV; it is disabled by default. Python utilities are in `tools/`.

Build products, datasets, logs, generated results, local agent instructions, and Windows download metadata are excluded from version control.
