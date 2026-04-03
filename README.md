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
  - `/terrain_obstacle_points`
  - `/terrain_debug/grid_map` (`grid_map_msgs/msg/GridMap`)
  - `/terrain_debug/base_gravity_cloud`
  - `/terrain_debug/support_points`
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

`FrameObservabilityEstimator` only consumes `cloud_in_base`. It derives front/rear coverage and dropout semantics directly from body-frame sector coverage.
Rear-sector conservatism is now driven only by `rear_dropout / MissingByDropout`; there is no fixed rear blind sector.

`PolarFrontend` keeps body-centric observability semantics separate from odom-centric map projection: sector logic is derived from base-view samples, while candidate aggregation and map indexing use odom-view samples.
Obstacle formation is additionally guarded by local structure: a single-cell vertical span only marks the cell as suspicious, and the cell is promoted to an obstacle only when its fixed 3x3 neighborhood contains enough upper-support cells.

Frame semantics are intentionally split:
- `odom_frame`: internal local mapping frame used by map indexing and all core terrain reasoning
- `base_gravity_frame`: robot-centric gravity frame used by debug point clouds; origin is the current robot pose, roll/pitch are removed, yaw is preserved

Published map semantics are intentionally split from internal map semantics:
- `grid_map`, `terrain_state`, and `terrain_cost` are published as robot-centric `base_gravity` views
- internal map storage and all core outputs remain in `odom`

Preprocessing semantics are intentionally split as well:
- `preprocess.body_filter.*` is interpreted in `base_link`
- `map_height_min/max` is interpreted as robot-relative height, matching `base_gravity` z semantics
- local XY crop is implemented internally against the current local map window

Current debug point semantics:
- `/terrain_debug/base_gravity_cloud`: full preprocessed algorithm cloud expressed in `base_gravity_frame`
- `/terrain_debug/support_points`: real input samples that land in support cells and remain close to the cell support height, expressed in `base_gravity_frame`
- `/terrain_obstacle_points`: real input samples that land in cells whose `obstacle_evidence` is already high enough, expressed in `base_gravity_frame`
- `/terrain_debug/unknown_mask`: unknown cell centers expressed in `base_gravity_frame`
- these point outputs are intended to align directly with the robot-centric `grid_map`, `terrain_state`, and `terrain_cost` views in RViz
- `TerrainObservability.odom_point_count`: number of samples in the internal `cloud_in_odom` mapping view
- `/terrain_debug/observability`: non-geometric debug summary that still uses the same `base_gravity` debug header context

## Layout
- `include/passable_area/core`: ROS-free data types and processing modules
- `include/passable_area/interfaces/ros`: converters, params, publishers, watchdog, node
- `src/core`: algorithm implementation
- `src/interfaces/ros`: ROS integration
- `msg/TerrainObservability.msg`: observability debug contract
- `test/core`: gtests for processor behavior
- `config/offline_benchmark_bags.yaml`: batch false-obstacle benchmark manifest
- workspace-level `scripts/passable_obstacle_benchmark.sh`: runs the manifest bags and writes a timestamped Markdown summary under the workspace root

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

Relevant dynamic-obstacle persistence controls:
- `obstacle_clear_observed_decay`: faster obstacle evidence decay when ground is re-observed without a new obstacle
- `obstacle_clear_partial_decay_scale`: scales that faster decay under partially observed sectors
- `obstacle_height_clear_threshold`: clears `overhead_height` once obstacle evidence has fallen below this threshold

Relevant obstacle-formation controls:
- `upper_min_height_above_support`: minimum height above the support reference required to mark a cell as containing upper support; when historical `support_height` is missing, the current-frame `min_z` is used as fallback
- `min_neighbor_upper_support_cells`: minimum number of upper-support cells inside the fixed 3x3 neighborhood required to confirm a suspicious obstacle cell
