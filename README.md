# Passable Area (ROS1)

ROS1 Noetic port of the Humble `passable_area` node. It evaluates ground traversability from a gravity-aligned cloud and IMU, publishes passable/impassable clouds, a `grid_map` with passability and coverage layers, and an optional traversal-cost `OccupancyGrid`. v0.0.4 adds raycast-based cliff detection, NaN-as-stiff handling, and reuses traversal cost to resolve unknown cells.

## Build
Dependencies: `roscpp`, `sensor_msgs`, `nav_msgs`, `grid_map_core`, `grid_map_cv`, `grid_map_ros`, `pcl_ros`, `pcl_conversions`, `Eigen3`, `OpenCV`, `OpenMP`.

Build inside your catkin workspace:
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Run
- Navigation preset:
  ```bash
  roslaunch passable_node passable_nav.launch
  ```
- Mapping preset:
  ```bash
  roslaunch passable_node passable_mapping.launch
  ```

Each launch loads three parameter files:
- `config/nav_params.yaml` or `config/mapping_params.yaml` (map size, thresholds, topics, traversal cost toggles)
- `config/body_params.yaml` (robot footprint per model `x30`/`m20`)
- `config/lidar_params.yaml` (per-lidar pose/FOV; used for coverage and blind-spot filling)

## Topics
- Subscribed: `accumulate_cloud_topic` (`sensor_msgs/PointCloud2`), `imu_topic` (`sensor_msgs/Imu`)
- Published: `passable_cloud_topic`, `impassable_cloud_topic` (`sensor_msgs/PointCloud2`), `grid_map_topic` (`grid_map_msgs/GridMap`), `traversal_cost_topic` (`nav_msgs/OccupancyGrid` when enabled), `body_visual` (`visualization_msgs/Marker`)
