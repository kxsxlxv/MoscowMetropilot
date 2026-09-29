cmake_minimum_required(VERSION 3.16)
project(metropilot_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(METROPILOT_BUILD_REPLAY "Build the SQLite replay smoke executable" ON)

add_library(metropilot_core
  src/pointcloud_decoder.cpp
  src/evidence.cpp
  src/geometry.cpp
  src/sensor_mount_config.cpp
  src/sweep_geometry.cpp
  src/track_estimation.cpp
  src/scene_projection.cpp
  src/swept_volume.cpp
  src/vehicle_geometry_config.cpp
  src/sweep_evidence_gate.cpp
  src/structural_residual.cpp
  src/candidate_grouping.cpp
  src/current_scan_classifier.cpp
  src/current_scan_detector.cpp
  src/detector_pipeline.cpp
  src/precision_gate.cpp
)

target_include_directories(metropilot_core PUBLIC include)
target_compile_options(metropilot_core PRIVATE -Wall -Wextra -Wpedantic)

if(METROPILOT_BUILD_REPLAY)
  find_package(SQLite3 REQUIRED)
  add_executable(metropilot_precision_replay tools/precision_replay.cpp)
  target_link_libraries(metropilot_precision_replay PRIVATE metropilot_core SQLite::SQLite3)
  target_compile_options(metropilot_precision_replay PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_precision_gate tests/test_precision_gate.cpp)
  target_link_libraries(test_precision_gate PRIVATE metropilot_core)
  target_compile_options(test_precision_gate PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME precision_gate COMMAND test_precision_gate)

  add_executable(test_detector_pipeline tests/test_detector_pipeline.cpp)
  target_link_libraries(test_detector_pipeline PRIVATE metropilot_core)
  target_compile_options(test_detector_pipeline PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME detector_pipeline
    COMMAND test_detector_pipeline ${CMAKE_CURRENT_SOURCE_DIR}/../config/vehicle_geometry.yaml)
endif()
