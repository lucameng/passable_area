# Passable Area

`passable_area` is now a single-node, dropout-aware local terrain evaluator for quadruped navigation. The node keeps ROS concerns in `interfaces/ros` and runs the terrain logic in a ROS-free `core` pipeline.

## Runtime model
- Inputs are strictly synchronized with `ExactTime`:
  - `/LOC_BODY_POINTS` (`sensor_msgs/msg/PointCloud2`)
  - `/ODOM` (`nav_msgs/msg/Odometry`)
- The preprocessor now produces two explicit views from the synchronized input cloud:
  - `cloud_in_base`: body-centric view for observability and sector semantics after body-box noise removal and local crop
  - `cloud_in_odom`: internal local-map view for mapping, features, and traversability
- Outputs are:
  - `/terrain_state` (`nav_msgs/msg/OccupancyGrid`)
  - `/terrain_cost` (`nav_msgs/msg/OccupancyGrid`)
  - `/terrain_debug/grid_map` (`grid_map_msgs/msg/GridMap`)
  - `/terrain_debug/base_gravity_cloud`
  - `/terrain_debug/support_points`
  - `/terrain_debug/obstacle_points`
  - `/terrain_debug/unknown_mask`
  - `/terrain_debug/observability` (`passable_area/msg/TerrainObservability`)

`terrain_state` uses:
- `0` = passable
- `100` = impassable
- `-1` = unknown

## Pipeline
Each synchronized frame is processed in this order:
1. `FramePreprocessor`
2. `FrameObservabilityEstimator`
3. `PolarFrontend`
4. `DropoutAwareMapUpdater`
5. `TerrainFeatureUpdater`
6. `TraversabilitySolver`

The current implementation already supports:
- support persistence
- sector-level observability
- dropout-aware map aging
- incremental feature recomputation on dirty cells
- three-state traversability output

`FrameObservabilityEstimator` only consumes `cloud_in_base`. It no longer relies on transforming points back from the debug `base_gravity` frame to recover front/rear/blind/dropout semantics.

`PolarFrontend` keeps body-centric observability semantics separate from odom-centric map projection: sector logic is derived from base-view samples, while candidate aggregation and map indexing use odom-view samples.

Frame semantics are intentionally split:
- `odom_frame`: internal local mapping frame used by `grid_map`, occupancy outputs, and map indexing
- `base_gravity_frame`: robot-centric gravity frame used by debug point clouds; origin is the current robot pose, roll/pitch are removed, yaw is preserved

Preprocessing semantics are intentionally split as well:
- `preprocess.body_filter.*` is interpreted in `base_link`
- `map_height_min/max` is interpreted as robot-relative height, matching `base_gravity` z semantics
- local XY crop is implemented internally against the current local map window

Current debug point semantics:
- `/terrain_debug/base_gravity_cloud`: full preprocessed algorithm cloud expressed in `base_gravity_frame`
- `/terrain_debug/support_points`: real input samples that land in support cells and remain close to the cell support height, expressed in `base_gravity_frame`
- `/terrain_debug/obstacle_points`: real input samples that land in obstacle cells and remain close to the cell `overhead_height`, so they currently represent overhead or upper-surface obstacle samples rather than every obstacle-labeled sample in the cell
- `/terrain_debug/unknown_mask`: unknown cell centers expressed in `base_gravity_frame`
- `TerrainObservability.odom_point_count`: number of samples in the internal `cloud_in_odom` mapping view

## Layout
- `include/passable_area/core`: ROS-free data types and processing modules
- `include/passable_area/interfaces/ros`: converters, params, publishers, watchdog, node
- `src/core`: algorithm implementation
- `src/interfaces/ros`: ROS integration
- `msg/TerrainObservability.msg`: observability debug contract
- `test/core`: gtests for processor behavior

## Build and test
From the workspace root:

```bash
colcon build --packages-select passable_area --symlink-install
source install/setup.bash
colcon test --packages-select passable_area --event-handlers console_direct+
```

## Launch
Use the single-node launch:

```bash
ros2 launch passable_area nav.launch.py
```

Config is split across:
- `config/passable_area.yaml`
- `config/sensors.yaml`
- `config/debug.yaml`

`config/passable_area.yaml` holds algorithm and preprocessing parameters. `config/debug.yaml` holds debug publishers and debug output topics.

Relevant debug controls:
- `debug.publish_base_gravity_cloud`: enable publishing the full preprocessed algorithm cloud in `base_gravity_frame`
- `output.base_gravity_cloud_topic`: topic name for that cloud

Relevant preprocessing controls:
- `preprocess.body_filter.*`: reject points inside a configured `base_link` box, intended for body-interior noise
- `preprocess.crop_to_map.enable`: reject points outside the current local map support window
- `preprocess.crop_to_map.xy_margin`: extra XY padding for the local crop window
