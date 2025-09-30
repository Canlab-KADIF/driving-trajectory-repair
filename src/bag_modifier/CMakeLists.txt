cmake_minimum_required(VERSION 3.0.2)
project(bag_modifier)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  visualization_msgs
  control_msgs
  novatel_oem7_msgs
  perception_msgs
  cyber_perception_msgs  
  # morai_msgs
  rosbag
  map
  utils
  # ouster_ros
  tf
  planner
  can_msgs
)

# Find CGAL library to compute IOU
find_package(CGAL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${utils_INCLUDE_DIRS}
)

add_executable(estimation_visualizer src/estimation_visualizer.cpp)

target_link_libraries(estimation_visualizer
  ${catkin_LIBRARIES}
)

add_executable(modified_bag_visualizer src/modified_bag_visualizer.cpp)

target_link_libraries(modified_bag_visualizer
  ${catkin_LIBRARIES}
)

add_executable(bag_modifier
  src/bag_modifier/bag_modifier.cpp
  src/bag_modifier/spline_solver.cpp
  src/bag_modifier/modifying_factory.cpp
  src/bag_modifier/main.cpp
)

target_link_libraries(bag_modifier
  ${catkin_LIBRARIES}
  CGAL::CGAL
)