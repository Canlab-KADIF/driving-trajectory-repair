# Copyright 2026 Korea Electronics Technology Institute (KETI)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(driving_trajectory_repair)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# Not publicly distributed; see README.md for the message field layout
find_package(cyber_perception_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Geometry, motion models and curve fitting. No ROS at all, so this target can
# be compiled and tested on its own; that is what the test target relies on.
# ---------------------------------------------------------------------------
add_library(driving_trajectory_repair_geometry SHARED
  src/geometry/box2d.cc
  src/geometry/polygon_iou.cc
  src/track/obstacle_pose_estimator.cc
  src/spline/spline_solver.cc
)
target_include_directories(driving_trajectory_repair_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(driving_trajectory_repair_geometry PUBLIC Eigen3::Eigen)

# ---------------------------------------------------------------------------
# Track repair and bag rewriting
# ---------------------------------------------------------------------------
add_library(driving_trajectory_repair_core SHARED
  src/ros/obstacle_conversion.cc
  src/track/track_repairer.cc
  src/bag_rewriter.cc
)
target_include_directories(driving_trajectory_repair_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(driving_trajectory_repair_core PUBLIC driving_trajectory_repair_geometry)
ament_target_dependencies(driving_trajectory_repair_core PUBLIC
  rclcpp rosbag2_cpp rosbag2_storage cyber_perception_msgs)

# ---------------------------------------------------------------------------
# rviz styling shared by the visualisation nodes
# ---------------------------------------------------------------------------
add_library(driving_trajectory_repair_viz SHARED
  src/viz/marker_style.cc
)
target_include_directories(driving_trajectory_repair_viz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(driving_trajectory_repair_viz PUBLIC driving_trajectory_repair_geometry)
ament_target_dependencies(driving_trajectory_repair_viz PUBLIC
  std_msgs builtin_interfaces visualization_msgs)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
add_executable(repair_trajectory src/repair_trajectory_main.cc)
target_link_libraries(repair_trajectory driving_trajectory_repair_core)
ament_target_dependencies(repair_trajectory rclcpp)

add_executable(estimation_visualizer src/viz/estimation_visualizer_main.cc)
target_link_libraries(estimation_visualizer driving_trajectory_repair_core driving_trajectory_repair_viz)
ament_target_dependencies(estimation_visualizer
  rclcpp visualization_msgs cyber_perception_msgs)

add_executable(repaired_trajectory_visualizer src/viz/repaired_trajectory_visualizer_main.cc)
target_link_libraries(repaired_trajectory_visualizer driving_trajectory_repair_viz)
ament_target_dependencies(repaired_trajectory_visualizer
  rclcpp visualization_msgs cyber_perception_msgs)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS repair_trajectory estimation_visualizer repaired_trajectory_visualizer
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS driving_trajectory_repair_geometry driving_trajectory_repair_core driving_trajectory_repair_viz
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(driving_trajectory_repair_geometry driving_trajectory_repair_core driving_trajectory_repair_viz)
ament_export_dependencies(rclcpp rosbag2_cpp cyber_perception_msgs Eigen3)

# ---------------------------------------------------------------------------
# Tests. Only the ROS-free target is covered, so the suite runs without a
# running graph and without the perception messages being available.
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_driving_trajectory_repair
    test/test_geometry.cc
    test/test_obstacle_pose_estimator.cc
    test/test_spline_solver.cc
  )
  target_link_libraries(test_driving_trajectory_repair driving_trajectory_repair_geometry)
endif()

ament_package()
