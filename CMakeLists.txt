cmake_minimum_required(VERSION 3.8)
project(passable_area)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(msg_files
  "msg/TerrainObservability.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

set(core_sources
  src/core/preprocess/frame_preprocessor.cpp
  src/core/observability/frame_observability_estimator.cpp
  src/core/frontend/polar_frontend.cpp
  src/core/mapping/local_terrain_map.cpp
  src/core/mapping/dropout_aware_map_updater.cpp
  src/core/features/terrain_feature_updater.cpp
  src/core/traversability/traversability_solver.cpp
  src/core/pipeline/processor.cpp
)

add_library(passable_area_core STATIC ${core_sources})
target_link_libraries(passable_area_core ${OpenCV_LIBS})
target_include_directories(passable_area_core PUBLIC include)
ament_target_dependencies(passable_area_core Eigen3 OpenCV grid_map_core)

set(interface_sources
  src/interfaces/ros/converters/pointcloud_converter.cpp
  src/interfaces/ros/converters/odom_converter.cpp
  src/interfaces/ros/converters/output_converter.cpp
  src/interfaces/ros/params/ros_param_loader.cpp
  src/interfaces/ros/publishers/result_publishers.cpp
  src/interfaces/ros/publishers/debug_publishers.cpp
  src/interfaces/ros/status/watchdog_manager.cpp
  src/interfaces/ros/status/perf_stats.cpp
  src/interfaces/ros/nodes/passable_area_node.cpp
)

add_library(passable_area_ros STATIC ${interface_sources})
target_include_directories(passable_area_ros PUBLIC include)
ament_target_dependencies(passable_area_ros
  grid_map_core
  grid_map_msgs
  grid_map_ros
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
target_link_libraries(passable_area_ros passable_area_core)
rosidl_target_interfaces(passable_area_ros ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(passable_area_node src/main/passable_area_node_main.cpp)
target_link_libraries(passable_area_node passable_area_ros)
ament_target_dependencies(passable_area_node rclcpp)

add_executable(passable_area_benchmark test/core/benchmark_processor.cpp)
target_include_directories(passable_area_benchmark PUBLIC include)
target_link_libraries(passable_area_benchmark passable_area_core ${OpenCV_LIBS})
ament_target_dependencies(passable_area_benchmark Eigen3 OpenCV grid_map_core)

install(TARGETS
  passable_area_core
  passable_area_ros
  passable_area_node
  passable_area_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  ament_add_gtest(test_processor
    test/core/test_processor.cpp
    ${core_sources}
  )
  if(TARGET test_processor)
    target_include_directories(test_processor PUBLIC include)
    target_link_libraries(test_processor ${OpenCV_LIBS})
    ament_target_dependencies(test_processor Eigen3 OpenCV grid_map_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  grid_map_core
  grid_map_msgs
  grid_map_ros
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

ament_package()
