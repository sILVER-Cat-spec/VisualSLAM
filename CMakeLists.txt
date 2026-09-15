cmake_minimum_required(VERSION 3.16)
project(visual_slam_cpp VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d imgcodecs)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

function(vslam_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# ------ foundation ------
add_library(vslam_foundation
  src/sensor/camera_model.cpp
  src/sensor/rgbd_camera.cpp
  src/geometry/se3.cpp
  src/diagnostics/logger.cpp)
add_library(vslam::foundation ALIAS vslam_foundation)
target_compile_features(vslam_foundation PUBLIC cxx_std_17)
set_target_properties(vslam_foundation PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(vslam_foundation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vslam_foundation PUBLIC Eigen3::Eigen Threads::Threads)
vslam_warnings(vslam_foundation)

# ------ map ------
add_library(vslam_map
  src/core/feature_set.cpp
  src/core/frame.cpp
  src/core/keyframe.cpp
  src/core/map.cpp)
add_library(vslam::map ALIAS vslam_map)
target_link_libraries(vslam_map PUBLIC vslam::foundation opencv_core)
target_include_directories(vslam_map SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
set_target_properties(vslam_map PROPERTIES CXX_EXTENSIONS OFF)
vslam_warnings(vslam_map)

# ------ estimation ------
add_library(vslam_estimation src/optimization/solvers.cpp)
add_library(vslam::estimation ALIAS vslam_estimation)
target_link_libraries(vslam_estimation PUBLIC vslam::foundation PRIVATE Ceres::ceres)
vslam_warnings(vslam_estimation)

# ------ pipeline ------
add_library(vslam_pipeline
  src/sensor/image_preprocessor.cpp
  src/frontend/features.cpp
  src/initialization/rgbd_initializer.cpp
  src/tracking/frame_tracker.cpp
  src/mapping/local_mapper.cpp
  src/system/slam_system.cpp)
add_library(vslam::pipeline ALIAS vslam_pipeline)
target_link_libraries(vslam_pipeline PUBLIC vslam::map vslam::estimation
  PRIVATE opencv_imgproc opencv_features2d opencv_calib3d)
vslam_warnings(vslam_pipeline)

# ------ io ------
add_library(vslam_io
  src/io/dataset_io.cpp
  src/io/point_cloud_writer.cpp)
target_link_libraries(vslam_io PUBLIC vslam::pipeline PRIVATE opencv_imgcodecs)
vslam_warnings(vslam_io)

# ------ apps ------
add_executable(rgbd_camera_example apps/rgbd_camera_example.cpp)
target_link_libraries(rgbd_camera_example PRIVATE vslam::foundation)
vslam_warnings(rgbd_camera_example)

add_executable(core_data_example apps/core_data_example.cpp)
target_link_libraries(core_data_example PRIVATE vslam::map)
vslam_warnings(core_data_example)

add_executable(run_dataset apps/run_dataset.cpp)
target_link_libraries(run_dataset PRIVATE vslam_io)
vslam_warnings(run_dataset)

# ------ tests ------
include(CTest)
if(BUILD_TESTING)
  add_executable(dataset_io_test tests/unit/dataset_io_test.cpp)
  target_link_libraries(dataset_io_test PRIVATE vslam_io opencv_imgcodecs)
  add_test(NAME dataset_io COMMAND dataset_io_test)
  add_executable(rgbd_pipeline_test tests/integration/rgbd_pipeline_test.cpp)
  target_link_libraries(rgbd_pipeline_test PRIVATE vslam::pipeline opencv_imgproc)
  vslam_warnings(rgbd_pipeline_test)
  add_test(NAME rgbd_pipeline COMMAND rgbd_pipeline_test)
  add_executable(camera_model_test tests/unit/camera_model_test.cpp)
  target_link_libraries(camera_model_test PRIVATE vslam::foundation)
  add_test(NAME camera_model COMMAND camera_model_test)
  add_executable(core_data_test tests/unit/core_data_test.cpp)
  target_link_libraries(core_data_test PRIVATE vslam::map)
  add_test(NAME core_data COMMAND core_data_test)
  add_executable(map_allocation_test tests/unit/map_allocation_test.cpp)
  target_link_libraries(map_allocation_test PRIVATE vslam::map)
  add_test(NAME map_allocation_rollback COMMAND map_allocation_test)
  add_executable(diagnostics_test tests/unit/diagnostics_test.cpp)
  target_link_libraries(diagnostics_test PRIVATE vslam::foundation)
  add_test(NAME diagnostics COMMAND diagnostics_test)
  foreach(target camera_model_test core_data_test map_allocation_test diagnostics_test)
    vslam_warnings(${target})
  endforeach()
  add_executable(camera_baseline_probe tests/regression/camera_baseline_probe.cpp)
  target_link_libraries(camera_baseline_probe PRIVATE vslam::foundation)
  option(VSLAM_TEST_PYTHON_BASELINE "Compare with sibling VisualSLAM (requires Python numpy and cv2)" OFF)
  if(VSLAM_TEST_PYTHON_BASELINE)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME camera_python_baseline COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tests/regression/compare_camera_baseline.py
      $<TARGET_FILE:camera_baseline_probe>
      ${CMAKE_CURRENT_SOURCE_DIR}/../VisualSLAM)
  endif()
endif()

# Independent stereo input adapter; the RGB-D pipeline contract stays unchanged.
target_sources(vslam_foundation PRIVATE src/sensor/stereo_camera.cpp)
target_sources(vslam_pipeline PRIVATE src/sensor/stereo_preprocessor.cpp src/sensor/stereo_depth_adapter.cpp)
target_sources(vslam_io PRIVATE src/io/stereo_dataset_io.cpp)
add_executable(run_stereo_dataset apps/run_stereo_dataset.cpp)
target_link_libraries(run_stereo_dataset PRIVATE vslam_io)
target_compile_definitions(run_stereo_dataset PRIVATE VSLAM_DEPTH_ROUTE=1)
vslam_warnings(run_stereo_dataset)

if(BUILD_TESTING)
  add_executable(stereo_contract_test tests/unit/stereo_contract_test.cpp)
  target_link_libraries(stereo_contract_test PRIVATE vslam_io opencv_imgproc)
  add_test(NAME stereo_contract COMMAND stereo_contract_test)

  target_compile_definitions(stereo_contract_test PRIVATE VSLAM_DEPTH_ROUTE=1)

  add_executable(controlled_backend tests/regression/controlled_backend.cpp)
  target_link_libraries(controlled_backend PRIVATE vslam::estimation)
  target_compile_definitions(controlled_backend PRIVATE VSLAM_DEPTH_ROUTE=1)
  add_test(NAME controlled_backend COMMAND controlled_backend)

  add_executable(stereo_dataset_io_test tests/unit/stereo_dataset_io_test.cpp)
  target_link_libraries(stereo_dataset_io_test PRIVATE vslam_io)
  add_test(NAME stereo_dataset_io COMMAND stereo_dataset_io_test)
endif()

target_sources(vslam_pipeline PRIVATE src/sensor/stereo_rectification.cpp)

add_executable(initial_mapping_example apps/initial_mapping_example.cpp)
target_link_libraries(initial_mapping_example PRIVATE vslam_io)
vslam_warnings(initial_mapping_example)
