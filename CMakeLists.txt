cmake_minimum_required(VERSION 3.8)
project(passable_area)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
else()
  message(FATAL_ERROR "OpenMP not found")
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/common
  include/passable_area
)

add_executable(passable_area
  src/elevation_map.cpp
  src/elevation_solver.cpp
  src/layer_processing.cpp
  src/passable_area_node.cpp
  src/lidar_coverage.cpp
  src/traversal_cost.cpp
  src/passable_area_main.cpp
)

ament_target_dependencies(passable_area
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  grid_map_core
  grid_map_cv
  grid_map_ros
  nav_msgs
)

target_link_libraries(passable_area
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

install(TARGETS passable_area
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
