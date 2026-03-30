# Passable Area (ROS 2 Humble)

`passable_area` builds a gravity-aligned elevation grid from odometry-leveled point cloud data, classifies passability with BFS, and publishes both a `grid_map` and filtered point clouds for navigation or mapping. The single executable serves multiple scenarios; launch files and YAML presets pick the mode.

## Pipeline Highlights
- Frames and inputs: raw input cloud is expected in `body_frame`; cloud and odometry are paired by `ExactTime` on identical `header.stamp`, then `roll/pitch` are used to rotate the cloud into `gravity_frame`, voxel-downsample it, and crop it to `[map_length, map_width, map_height_min, map_height_max]` before writing into a `grid_map::GridMap`.
- Elevation solving: a histogram-based solver (configurable bin count and ROI) estimates `ground_height`, `ceiling_height`, and `clearance`; outside the ROI it falls back to the legacy min/max strategy.
- Map cleanup and features: hole inpainting with optional center padding, median filtering, roughness/slope/step features, and cliff detection keep BFS stable on noisy data.
- Passability and cost: BFS grows from the robot center using `max_drop`, `max_roughness`, and slope limits; `traversal_cost` blends slope/roughness/step into a continuous cost layer and is also published as an `OccupancyGrid`.
- Blind-spot handling (optional): when `enable_blind_check` is true, `LidarCoverage` projects per-lidar FOV cones from `lidar_params.*`, marks uncovered cells, and dilates blind spots so impassable points are not emitted from unseen areas.
- Outputs: passable/impassable point clouds, a full `grid_map` (passability, coverability, traversal_cost, elevation, etc.), a `body_visual` marker, and a traversal-cost occupancy grid.

## Build
- Dependencies: `rclcpp`, `sensor_msgs`, `nav_msgs`, `pcl_ros`, `pcl_conversions`, `grid_map_core`, `grid_map_cv`, `grid_map_ros`, `Eigen3`, `OpenCV`.
- From workspace root:
  ```bash
  colcon build --packages-select passable_area --symlink-install
  source install/setup.bash
  ```
  The navigation launch also starts `accumulate_cloud`, so ensure that package is installed if you use the nav preset.

## Run
- Navigation pipeline (starts `accumulate_cloud` and `passable_area`):
  ```bash
  ros2 launch passable_area nav.launch.py
  ```
  Loads `config/nav_params.yaml` plus `config/body_params.yaml` and `config/lidar_params.yaml`.
- Mapping pipeline:
  ```bash
  ros2 launch passable_area mapping.launch.py
  ```
  Expects `config/mapping_params.yaml`; create it (e.g., copy `nav_params.yaml`) before launching to avoid missing-file errors.
- Direct node for quick experiments:
  ```bash
  ros2 run passable_area passable_area --ros-args \
    --params-file src/passable_area/config/nav_params.yaml
  ```
- RViz: use `rviz/passable_area.rviz` to inspect passable/impassable clouds, grid-map layers, and the body footprint.
- Helpers: `scripts/passable_nav.sh` / `scripts/passable_mapping.sh` wrap the launches after sourcing the workspace; edit the paths at the top of each script before use.

## Configuration
Parameter files live in `config/`:
- `nav_params.yaml`: shipped nav defaults (used by `nav.launch.py`).
- `mapping_params.yaml`: user-provided mapping preset (referenced by `mapping.launch.py`).
- `body_params.yaml`: footprint dimensions per `dog_model` (`m20` and `x30` provided).
- `lidar_params.yaml`: per-model lidar poses and FOV for coverage.

Key nav defaults in `config/nav_params.yaml`:
| Group | Param | Default (nav) | Notes |
| --- | --- | --- | --- |
| Map geometry | `map_length` / `map_width` | `8.0 / 8.0` | Body-centered grid size (m). |
|  | `map_height_min` / `map_height_max` | `-1.0 / 0.5` | Z crop (m). |
|  | `voxel_size` | `0.05` | Grid resolution (m). |
| Passability | `max_drop` | `0.25` | Max allowed step between cells (m). |
|  | `max_roughness` | `0.10` | Roughness gate for BFS. |
|  | `max_slope_deg` | `89.0` | Slope limit (deg). |
|  | `treat_nan_as_stiff` | `true` | Only block on finite→NaN→finite drops. |
|  | `max_inpaint_pixels` | `200` | Max hole size for inpaint (cells). |
|  | `enable_center_padding` / `center_dist_thresh` | `true / 0.8` | Force-fill near-body holes. |
| Frames/topics | `input_cloud_topic` | `/LOC_BODY_POINTS` | Input cloud in `body_frame`. |
|  | `odom_topic` | `/ODOM` | Odometry used to remove `roll/pitch` and align the cloud to `gravity_frame`. |
|  | `imu_topic` | `/IMU` | Reserved parameter; not used for point-cloud leveling. |
|  | `passable_cloud_topic` / `impassable_cloud_topic` | `/passable_area` / `/impassable_area` | Output clouds. |
|  | `passable_status_code_topic` | `/passable_status_code` | `std_msgs/msg/Int32` status topic, currently publishes `100` each frame before passable/impassable clouds. |
|  | `grid_map_topic` / `traversal_cost_topic` | `/grid_map` / `/traversal_cost` | Map outputs. |
|  | `world_frame` / `gravity_frame` | `camera_init` / `base_gravity` | TF frames. |
|  | `body_frame` | `base_link` | Input body frame and body marker frame. |
|  | `dog_model` | `m20` | Selects footprint & lidar layout. |
| Downsample | `downsample.enable` / `downsample.voxel_size` | `true / 0.05` | Applied after the cloud is rotated into `gravity_frame`. |
| Sync | `sync.queue_size` | `10` | `ExactTime` synchronizer queue depth for cloud + odometry. |

Elevation solver defaults (ROI uses histogram solver; outside falls back to legacy):
| Param | Value |
| --- | --- |
| `use_histogram_solver` | `true` |
| `histogram_bins` | `20` |
| `region_enabled` | `true` |
| `region_min_x` / `region_max_x` | `-3.0 / 3.0` |
| `region_min_y` / `region_max_y` | `-1.0 / 1.0` |

Raycast & blind check defaults:
| Param | Value | Notes |
| --- | --- | --- |
| `raycast.enable` | `true` | Crops NaN gaps in rays before labeling cliffs. |
| `raycast.max_ray_distance` | `4.0` | Meters. |
| `raycast.max_nan_gap` | `3.0` | Cells. |
| `enable_blind_check` | `false` | Set `true` to compute coverability. |

Traversal cost defaults (published in `grid_map` and as `nav_msgs/OccupancyGrid`):
| Param | Value |
| --- | --- |
| `enable` | `true` |
| `slope_free_deg` / `slope_block_deg` | `10.0 / 80.0` |
| `rough_free` / `rough_block` | `0.01 / 0.04` |
| `step_free` / `step_block` | `0.08 / 0.35` |
| `slope_weight` / `roughness_weight` / `step_weight` | `0.3 / 0.3 / 0.4` |
| `easy_cost` / `hard_cost` / `max_cost` | `0.0 / 90.0 / 100.0` |
| `missing_cost` | `20.0` |
| `curve_power` | `1.0` |
| `low_cost_filter_ratio` | `0.1` |
| `terrain_sample_window` | `1` |
| `safe_zone_side_length` | `0.0` |

## ROS Interfaces
- Subscribed: `input_cloud_topic` (`sensor_msgs/msg/PointCloud2`) and `odom_topic` (`nav_msgs/msg/Odometry`) through `message_filters::Synchronizer` with `ExactTime`; only strictly equal stamps enter the main pipeline, so a single-sided dropped frame is not processed. `imu_topic` is retained as a parameter for compatibility but is not consumed in the current pipeline.
- Published: `passable_cloud_topic` and `impassable_cloud_topic` (`sensor_msgs/msg/PointCloud2`), `grid_map_topic` (`grid_map_msgs/msg/GridMap`), `traversal_cost_topic` (`nav_msgs/msg/OccupancyGrid`), `body_visual` (`visualization_msgs/msg/Marker`).
- Status code: `passable_status_code_topic` (`std_msgs/msg/Int32`), default `/passable_status_code`; currently publishes `100` every frame before passable/impassable clouds as a reserved interface for future status refinement.
- Passability, traversal cost, and output point clouds are published in `gravity_frame`.

## Development Notes
- Detailed algorithm notes: `docs/passable_area.md` (full pipeline) and `docs/traversal_cost.md` (cost layer math). Treat these as the source of truth when tuning.
- Tests are not yet wired; add gtests under `src/passable_area/test/` and register them in `CMakeLists.txt` when introducing new features.
- `third_party/passable_area/` mirrors this package—edit files only in `src/passable_area/` and use `scripts/sync_passable_node.sh` if you intentionally need to refresh the mirror.
