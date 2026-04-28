cmake_minimum_required(VERSION 3.8)
project(passable_area)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_path(LOG4CPLUS_DR_INCLUDE_DIR
  NAMES log4cplus_dr/dr_logger.hpp
  DOC "Directory containing log4cplus_dr/dr_logger.hpp"
)
if(NOT LOG4CPLUS_DR_INCLUDE_DIR)
  message(FATAL_ERROR
    "Required header log4cplus_dr/dr_logger.hpp was not found. "
    "Please install the development package resolved by rosdep key 'log4cplus'.")
endif()
find_library(LOG4CPLUS_LIBRARY NAMES log4cplus)
if(NOT LOG4CPLUS_LIBRARY)
  message(FATAL_ERROR
    "Required library log4cplus was not found. "
    "Please install the development package resolved by rosdep key 'log4cplus'.")
endif()
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
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
  ${PCL_INCLUDE_DIRS}
  include
)

set(core_sources
  src/core/config_validation.cpp
  src/core/frame_preprocessor.cpp
  src/core/frame_observability_estimator.cpp
  src/core/polar_frontend.cpp
  src/core/mapping/local_terrain_map.cpp
  src/core/mapping/dropout_aware_map_updater.cpp
  src/core/terrain_feature_updater.cpp
  src/core/traversability_solver.cpp
  src/core/obstacle_reasoner.cpp
  src/core/obstacle_publication.cpp
  src/core/processor.cpp
)

add_library(passable_area_core STATIC ${core_sources})
target_link_libraries(passable_area_core ${OpenCV_LIBS})
target_include_directories(passable_area_core PUBLIC include)
ament_target_dependencies(passable_area_core Eigen3 OpenCV grid_map_core)

add_library(log4cplus_dr_dependency INTERFACE)
target_include_directories(log4cplus_dr_dependency INTERFACE
  ${LOG4CPLUS_DR_INCLUDE_DIR}
)
target_link_libraries(log4cplus_dr_dependency INTERFACE
  ${LOG4CPLUS_LIBRARY}
)

set(interface_sources
  src/interfaces/ros/converters/pointcloud_converter.cpp
  src/interfaces/ros/converters/odom_converter.cpp
  src/interfaces/ros/converters/output_converter.cpp
  src/interfaces/ros/ros_param_loader.cpp
  src/interfaces/common/logging/log_bridge.cpp
  src/interfaces/common/logging/node_logger.cpp
  src/interfaces/ros/runtime/result_publishers.cpp
  src/interfaces/ros/runtime/debug_publishers.cpp
  src/interfaces/ros/runtime/status_code_manager.cpp
  src/interfaces/ros/runtime/watchdog_manager.cpp
  src/interfaces/ros/runtime/perf_stats.cpp
  src/interfaces/ros/passable_area_node.cpp
)

add_library(passable_area_ros STATIC ${interface_sources})
target_include_directories(passable_area_ros PUBLIC include)
ament_target_dependencies(passable_area_ros
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(passable_area_ros
  passable_area_core
  ${PCL_LIBRARIES}
  log4cplus_dr_dependency
)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(passable_area_ros ${cpp_typesupport_target})
else()
  rosidl_target_interfaces(passable_area_ros
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

add_executable(passable_area_node src/main/passable_area_node_main.cpp)
target_link_libraries(passable_area_node passable_area_ros)
ament_target_dependencies(passable_area_node rclcpp)

# Manual core throughput benchmark. This is intentionally not part of ament gtests.
add_executable(passable_area_benchmark benchmarks/benchmark_processor.cpp)
target_include_directories(passable_area_benchmark PUBLIC include)
target_link_libraries(passable_area_benchmark passable_area_core ${OpenCV_LIBS})
ament_target_dependencies(passable_area_benchmark Eigen3 OpenCV grid_map_core)

add_executable(passable_area_offline_replay
  tools/offline_replay.cpp
  src/tools/false_obstacle_analyzer.cpp
  src/tools/miss_obstacle_analyzer.cpp
)
target_include_directories(passable_area_offline_replay PUBLIC include)
target_link_libraries(passable_area_offline_replay
  passable_area_ros
  passable_area_core
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(passable_area_offline_replay
  Eigen3
  OpenCV
  grid_map_core
  nav_msgs
  pcl_conversions
  rclcpp
  rosbag2_cpp
  sensor_msgs
  tf2_ros
)

# Manual ROS end-to-end latency benchmark. This is intentionally not part of ament gtests.
add_executable(passable_area_e2e_benchmark benchmarks/benchmark_ros_e2e.cpp)
target_include_directories(passable_area_e2e_benchmark PUBLIC include)
target_link_libraries(passable_area_e2e_benchmark
  passable_area_ros
  passable_area_core
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(passable_area_e2e_benchmark
  Eigen3
  OpenCV
  grid_map_core
  grid_map_msgs
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

if(COMMAND rosidl_get_typesupport_target)
  target_link_libraries(passable_area_e2e_benchmark ${cpp_typesupport_target})
else()
  rosidl_target_interfaces(passable_area_e2e_benchmark
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

install(TARGETS
  passable_area_core
  passable_area_ros
  passable_area_node
  passable_area_benchmark
  passable_area_offline_replay
  passable_area_e2e_benchmark
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

  ament_add_gtest(test_obstacle_reasoner
    test/core/test_obstacle_reasoner.cpp
    ${core_sources}
  )
  if(TARGET test_obstacle_reasoner)
    target_include_directories(test_obstacle_reasoner PUBLIC include)
    target_link_libraries(test_obstacle_reasoner ${OpenCV_LIBS})
    ament_target_dependencies(test_obstacle_reasoner Eigen3 OpenCV grid_map_core)
  endif()

  ament_add_gtest(test_output_converter
    test/interfaces/ros/test_output_converter.cpp
  )
  if(TARGET test_output_converter)
    target_include_directories(test_output_converter PUBLIC include)
    target_link_libraries(test_output_converter passable_area_ros passable_area_core ${OpenCV_LIBS})
    ament_target_dependencies(test_output_converter
      Eigen3
      OpenCV
      grid_map_core
      grid_map_msgs
      grid_map_ros
      nav_msgs
      rclcpp
      std_msgs
    )
    if(COMMAND rosidl_get_typesupport_target)
      target_link_libraries(test_output_converter ${cpp_typesupport_target})
    else()
      rosidl_target_interfaces(test_output_converter
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()
  endif()

  ament_add_gtest(test_debug_publishers
    test/interfaces/ros/test_debug_publishers.cpp
  )
  if(TARGET test_debug_publishers)
    target_include_directories(test_debug_publishers PUBLIC include)
    target_link_libraries(test_debug_publishers passable_area_ros passable_area_core ${OpenCV_LIBS})
    ament_target_dependencies(test_debug_publishers
      Eigen3
      OpenCV
      nav_msgs
      pcl_conversions
      rclcpp
      sensor_msgs
      std_msgs
    )
    if(COMMAND rosidl_get_typesupport_target)
      target_link_libraries(test_debug_publishers ${cpp_typesupport_target})
    else()
      rosidl_target_interfaces(test_debug_publishers
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()
  endif()

  ament_add_gtest(test_status_code_manager
    test/interfaces/ros/test_status_code_manager.cpp
  )
  if(TARGET test_status_code_manager)
    target_include_directories(test_status_code_manager PUBLIC include)
    target_link_libraries(test_status_code_manager
      passable_area_ros
      passable_area_core
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(test_status_code_manager
      Eigen3
      OpenCV
      rclcpp
      std_msgs
    )
    if(COMMAND rosidl_get_typesupport_target)
      target_link_libraries(test_status_code_manager ${cpp_typesupport_target})
    else()
      rosidl_target_interfaces(test_status_code_manager
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()
  endif()

  ament_add_gtest(test_ros_param_loader
    test/interfaces/ros/test_ros_param_loader.cpp
  )
  if(TARGET test_ros_param_loader)
    target_include_directories(test_ros_param_loader PUBLIC include)
    target_link_libraries(test_ros_param_loader
      passable_area_ros
      passable_area_core
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(test_ros_param_loader
      Eigen3
      OpenCV
      rclcpp
      std_msgs
    )
    if(COMMAND rosidl_get_typesupport_target)
      target_link_libraries(test_ros_param_loader ${cpp_typesupport_target})
    else()
      rosidl_target_interfaces(test_ros_param_loader
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()
  endif()

  ament_add_gtest(test_passable_area_node_frames
    test/interfaces/ros/test_passable_area_node_frames.cpp
  )
  if(TARGET test_passable_area_node_frames)
    target_include_directories(test_passable_area_node_frames PUBLIC include)
    target_link_libraries(test_passable_area_node_frames
      passable_area_ros
      passable_area_core
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(test_passable_area_node_frames
      Eigen3
      OpenCV
      nav_msgs
      pcl_conversions
      rclcpp
      sensor_msgs
      std_msgs
      tf2_msgs
    )
    if(COMMAND rosidl_get_typesupport_target)
      target_link_libraries(test_passable_area_node_frames ${cpp_typesupport_target})
    else()
      rosidl_target_interfaces(test_passable_area_node_frames
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
    endif()
  endif()

  ament_add_gtest(test_false_obstacle_analyzer
    test/tools/test_false_obstacle_analyzer.cpp
    src/tools/false_obstacle_analyzer.cpp
  )
  if(TARGET test_false_obstacle_analyzer)
    target_include_directories(test_false_obstacle_analyzer PUBLIC include)
    target_link_libraries(test_false_obstacle_analyzer passable_area_core ${OpenCV_LIBS})
    ament_target_dependencies(test_false_obstacle_analyzer
      Eigen3
      OpenCV
      grid_map_core
    )
  endif()

  ament_add_gtest(test_miss_obstacle_analyzer
    test/tools/test_miss_obstacle_analyzer.cpp
    src/tools/miss_obstacle_analyzer.cpp
  )
  if(TARGET test_miss_obstacle_analyzer)
    target_include_directories(test_miss_obstacle_analyzer PUBLIC include)
    target_link_libraries(test_miss_obstacle_analyzer passable_area_core ${OpenCV_LIBS})
    ament_target_dependencies(test_miss_obstacle_analyzer
      Eigen3
      OpenCV
      grid_map_core
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

ament_package()
