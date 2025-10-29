# Passable Node

## Overview
`passable_node` evaluates passable ground in real time from a fused point cloud and IMU stream. The node ingests a gravity-aligned elevation map, classifies terrain into passable/impassable/unknown cells, and exposes both grid map and point cloud outputs that can be consumed by navigation stacks or mapping tools.

## Key Features
- Body-frame passability check that combines roughness and drop height tests to remain stable on slopes.
- Optional blind-spot compensation and lidar coverage analysis that leverages configurable sensor poses.
- Inpainting pipeline that fills elevation holes and re-injects padded points for consistent classification.
- Unified executable for mapping and navigation; launch files merely swap parameter presets and topic remaps.
- Publishes `grid_map_msgs/GridMap`, filtered `sensor_msgs/PointCloud2` clouds, and an RViz body marker for quick inspection.

## Build & Dependencies
The package targets ROS Noetic (Catkin workspace) and requires:
- `roscpp`, `sensor_msgs`, `pcl_ros`, `pcl_conversions`
- `grid_map_core`, `grid_map_cv`, `grid_map_ros`
- Eigen3, OpenCV, and OpenMP (detected automatically during configuration)

Build from the workspace root:

```bash
catkin_make
source devel/setup.bash
```

## Running
Two launch files ship with tuned presets:

- Navigation pipeline:
  ```bash
  roslaunch passable_node passable_nav.launch
  ```
  Subscribes to `/accumulate_cloud/cloud_cur` and `/imu/data`, loads `config/nav_params.yaml`.

- Mapping pipeline:
  ```bash
  roslaunch passable_node passable_mapping.launch
  ```
  Uses the same node executable but loads `config/mapping_params.yaml` and remaps the cloud topic to `/accumulate_cloud/cloud_body`.

The RViz layout in `config/passable_node.rviz` highlights the passable/impassable clouds alongside the grid map.

## Configuration
Both YAML files expose the same parameters. Important ones include:

| Name | Default (nav) | Description |
| ---- | ------------- | ----------- |
| `map_width`, `map_height` | `10.0`, `3.0` | XY span of the elevation map in metres. |
| `voxel_size` | `0.05` | Resolution of the grid cells; affects computation load. |
| `max_drop` | `0.25` | Maximum allowed drop between adjacent cells (metres). |
| `max_roughness` | `0.10` | Roughness threshold used when evaluating terrain variance. |
| `body_length`, `body_width` | `0.9`, `0.4` | Dimensions of the robot body for visualization and padding. |
| `world_frame`, `gravity_frame`, `body_frame`, `used_frame` | see YAML | Frame IDs used when projecting measurements and publishing outputs. |
| `enable_blind_check` | `true` | Toggles lidar blind-spot padding and coverage checks. |

Per-lidar blocks (`lidar_front_up`, `lidar_front_down`, etc.) declare:
- `pos_body`: `[x, y, z]` mounting position in the body frame (metres)
- `rpy_body`: Euler angles in degrees
- `fov_up_deg` / `fov_down_deg`: vertical field-of-view limits
- Optional `min_range` / `max_range` overrides may be added if required

Adjust these values to match the physical sensor layout before running the node. When `enable_blind_check` is disabled, lidar coverage computations are skipped to reduce CPU usage.

## ROS Interfaces
### Subscribed Topics
- `~cloud_topic` (`sensor_msgs/PointCloud2`): gravity-aligned point cloud used to build the elevation map.
- `~imu` (`sensor_msgs/Imu`): orientation source for transforming gravity measurements into the body frame.

### Published Topics
- `~passable_area` (`sensor_msgs/PointCloud2`): filtered points classified as traversable.
- `~impassable_area` (`sensor_msgs/PointCloud2`): obstructed or risky terrain points.
- `~grid_map` (`grid_map_msgs/GridMap`): elevation grid containing passability, coverage, padding, and point-count layers.
- `~body_visual` (`visualization_msgs/Marker`): footprint marker showing the robot body in RViz.
- `~expanded` (`sensor_msgs/PointCloud2`): optional debug cloud containing the padded samples (publish left disabled by default).

All topics are private to the node (`~`) so they will inherit the node name or remaps provided in launch files.

## Branches

These branches are currently maintained:
* ROS 1
  * [master](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/master)
* ROS 2
  * [foxy](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/foxy)
  * [humble](https://codeup.aliyun.com/deeprobotics/perception/passable_node/tree/humble)