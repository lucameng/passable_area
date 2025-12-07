# Passable Area

## Overview
`passable_area` evaluates ground traversability in real time using a fused point cloud and IMU stream. The node maintains a gravity-aligned elevation grid, classifies each cell as passable, impassable, or unknown based on roughness and drop thresholds, and publishes both grid map and point cloud outputs for navigation or mapping stacks.

## Key Features
- Body-frame safety checks that combine roughness and drop limits to stay stable on slopes.
- Optional blind-spot padding and lidar coverage analysis configured per-sensor pose.
- Inpainting step that fills holes in the elevation map and re-injects padded points for consistent labeling.
- One executable serves mapping and navigation scenarios; launch files simply swap parameter presets and topic remaps.
- Maintains both ground and peak elevation layers so obstacle point clouds omit low-lying floor returns.
- Publishes `grid_map_msgs/GridMap`, filtered `sensor_msgs/PointCloud2` clouds, and an RViz body marker for quick inspection.

## Build & Dependencies
This branch targets ROS 2 Humble with `ament_cmake`. Required dependencies:
- `rclcpp`, `sensor_msgs`, `pcl_ros`, `pcl_conversions`
- `grid_map_core`, `grid_map_cv`, `grid_map_ros`
- `Eigen3`, `OpenCV`, and `OpenMP`

Build from the workspace root:
```bash
colcon build --packages-select passable_area --symlink-install
source install/setup.bash
```
Make sure OpenMP support is installed (for example `sudo apt install libomp-dev`) or CMake will abort early.

## Running
Two launch files provide tuned presets:

- Navigation pipeline
  ```bash
  ros2 launch passable_area nav.launch.py
  ```
  Loads `config/nav_params.yaml` and subscribes to the node-local `cloud_topic` and `imu` inputs.

- Mapping pipeline
  ```bash
  ros2 launch passable_area mapping.launch.py
  ```
  Uses the same executable, pulls parameters from `config/mapping_params.yaml`, and allows remapping the cloud topic (for example to `/accumulate_cloud/cloud_body`).

The RViz layout in `config/passable.rviz` highlights passable/impassable clouds alongside the grid map.

For quick experimentation without launch files, run the node directly:
```bash
ros2 run passable_area passable_area
```

> **Note:** `third_party/` contains mirrored copies of this package for synchronization purposes. Make changes under `src/passable_area/` only.

## Configuration
`config/nav_params.yaml` and `config/mapping_params.yaml` expose the same parameter set. Key entries:

Use `map_length` / `map_width` to control the X/Y span and `map_height_min` / `map_height_max` to clamp vertical range.

| Name | Default (nav) | Description |
| --- | --- | --- |
| `map_length`, `map_width` | `10.0`, `10.0` | Map extents along X/Y in metres. |
| `map_height_min`, `map_height_max` | `-1.5`, `1.5` | Lower and upper Z bounds of the elevation map (metres). |
| `voxel_size` | `0.1` | Grid resolution; smaller values increase compute load. |
| `max_drop` | `0.3` | Maximum allowed drop between neighbouring cells (metres). |
| `max_roughness` | `0.1` | Roughness threshold used during terrain evaluation. |
| `treat_nan_as_stiff` | `true` | When enabled, NaN neighbours only block traversal if a forward/back lidar ray sees a finite→NaN→finite gap with a drop. |
| `clearance_threshold` | `0.05` | Height margin around ground/baseline used to classify obstacle points. |
| `baseline_radius` | `0.5` | Radius (m) around the robot used to estimate the baseline ground height. |
| `body_length`, `body_width` | `0.6`, `0.4` | Robot footprint for padding and visualization. |
| `world_frame`, `gravity_frame`, `body_frame`, `used_frame` | see YAML | Frame IDs for projecting measurements and publishing outputs. |
| `enable_blind_check` | `false` | Toggles lidar blind-spot padding and coverage checks. |

The `traversal_cost.*` namespace (see the YAML files) tunes the new geometric cost layer:
- `slope_*`, `rough_*`, and `step_*` thresholds describe when each feature transitions from easy → hard → impassable.
- `*_weight` terms weight the contribution of slope, roughness, and steps to the aggregate difficulty score.
- `easy_cost`, `hard_cost`, `max_cost`, and `curve_power` shape how the normalized difficulty is converted into the published cost.
- `terrain_sample_window` controls the kernel (in cell units) used when computing local roughness/slope.

Per-lidar blocks (for example `lidar_front_up`) define:
- `pos_body`: `[x, y, z]` mount position in the body frame (metres)
- `rpy_body`: Euler angles in degrees
- `fov_up_deg` / `fov_down_deg`: vertical field-of-view limits
- Optional `min_range` / `max_range` overrides for coverage trimming

Disable `enable_blind_check` if you do not require coverage analysis and want to save CPU cycles.

## ROS Interfaces
### Subscribed Topics
- `cloud_topic` (`sensor_msgs/msg/PointCloud2`): gravity-aligned cloud used to construct the elevation map.
- `imu` (`sensor_msgs/msg/Imu`): orientation source for the gravity-to-body transform.

### Published Topics
- `passable_area` (`sensor_msgs/msg/PointCloud2`): filtered points classified as traversable.
- `impassable_area` (`sensor_msgs/msg/PointCloud2`): points marked as obstructed or risky.
- `grid_map` (`grid_map_msgs/msg/GridMap`): grid containing passability, coverage, padding, and point-count layers.
- `body_visual` (`visualization_msgs/msg/Marker`): body footprint marker for RViz.
- `expanded_area` (`sensor_msgs/msg/PointCloud2`): optional debug cloud with padded samples (disabled by default).

All names are relative, so they inherit the node namespace and can be remapped in launch files.

## Branches
Maintained branches:
- ROS 1
  - [master](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/master)
- ROS 2
  - [foxy](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/foxy)
  - [humble](https://codeup.aliyun.com/deeprobotics/perception/passable_area/tree/humble)
