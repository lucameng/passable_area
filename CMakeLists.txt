cmake_minimum_required(VERSION 3.8)
project(passable_node)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
else()
  message(FATAL_ERROR "OpenMP not found")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    grid_map_core
    grid_map_cv
    grid_map_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/common
  include/passable_area
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    grid_map_core
    grid_map_cv
    grid_map_ros
)

add_executable(passable_area
  src/elevation_map.cpp
  src/elevation_solver.cpp
  src/layer_processing.cpp
  src/lidar_coverage.cpp
  src/traversal_cost.cpp
  src/passable_area_node.cpp
  src/passable_area_main.cpp
)

target_link_libraries(passable_area
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
)

add_dependencies(passable_area ${catkin_EXPORTED_TARGETS})
