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

cmake_minimum_required(VERSION 3.0.2)
project(bag_modifier)

add_compile_options(-std=c++14 -Wall -Wextra)

# Message and utility packages that are not publicly distributed are listed
# here as well; see README.md for what they contain.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  tf
  std_msgs
  geometry_msgs
  visualization_msgs
  jsk_recognition_msgs
  cyber_perception_msgs
  utils
  planner
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bag_modifier_geometry bag_modifier_core
  CATKIN_DEPENDS roscpp rosbag tf cyber_perception_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${utils_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# Geometry and motion models. No ROS, no third-party geometry library, so this
# target can be compiled and tested on its own.
# ---------------------------------------------------------------------------
add_library(bag_modifier_geometry
  src/geometry/box2d.cc
  src/geometry/polygon_iou.cc
  src/track/obstacle_pose_estimator.cc
)

# ---------------------------------------------------------------------------
# Track repair. Depends on the perception messages and the spline solver.
# ---------------------------------------------------------------------------
add_library(bag_modifier_core
  src/ros/obstacle_conversion.cc
  src/spline/spline_solver.cc
  src/track/track_repairer.cc
  src/bag_rewriter.cc
)
add_dependencies(bag_modifier_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_modifier_core
  bag_modifier_geometry
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------
# rviz styling shared by the visualisation nodes
# ---------------------------------------------------------------------------
add_library(bag_modifier_viz
  src/viz/marker_style.cc
)
add_dependencies(bag_modifier_viz ${catkin_EXPORTED_TARGETS})
target_link_libraries(bag_modifier_viz
  bag_modifier_geometry
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
add_executable(bag_modifier src/bag_modifier_main.cc)
target_link_libraries(bag_modifier bag_modifier_core ${catkin_LIBRARIES})

add_executable(estimation_visualizer src/viz/estimation_visualizer_main.cc)
target_link_libraries(estimation_visualizer
  bag_modifier_core
  bag_modifier_viz
  ${catkin_LIBRARIES}
)

add_executable(repaired_bag_visualizer src/viz/repaired_bag_visualizer_main.cc)
target_link_libraries(repaired_bag_visualizer
  bag_modifier_viz
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS bag_modifier estimation_visualizer repaired_bag_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS bag_modifier_geometry bag_modifier_core bag_modifier_viz
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# ---------------------------------------------------------------------------
# Tests. Only the ROS-free targets are covered, so the suite runs without a
# roscore and without the perception messages being available.
# ---------------------------------------------------------------------------
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_bag_modifier_geometry
    test/test_geometry.cc
    test/test_obstacle_pose_estimator.cc
  )
  target_link_libraries(test_bag_modifier_geometry bag_modifier_geometry)
endif()
