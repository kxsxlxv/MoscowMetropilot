cmake_minimum_required(VERSION 3.16)
project(metropilot_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(METROPILOT_BUILD_REPLAY OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../cpp
  ${CMAKE_CURRENT_BINARY_DIR}/metropilot_core
  EXCLUDE_FROM_ALL
)

add_executable(obstacle_detector src/obstacle_detector.cpp)
target_link_libraries(obstacle_detector metropilot_core)
ament_target_dependencies(
  obstacle_detector
  ament_index_cpp
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
)
target_compile_options(obstacle_detector PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS obstacle_detector DESTINATION lib/${PROJECT_NAME})
install(
  FILES
    ../../config/sensor_mount_challenge_assumed.yaml
    ../../config/vehicle_geometry.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(PROGRAMS scripts/lidar_debug.py DESTINATION lib/${PROJECT_NAME} RENAME lidar_debug)
install(FILES scripts/bag_sequence.py DESTINATION lib/${PROJECT_NAME})

ament_package()
