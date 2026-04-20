# Passable Area

`passable_area` is now a single-node, dropout-aware local terrain evaluator for quadruped navigation. The node keeps ROS concerns in `interfaces/ros` and runs the terrain logic in a ROS-free `core` pipeline.

## Runtime model
- Inputs are strictly synchronized with `ExactTime`:
  - `/LOC_BODY_POINTS` (`sensor_msgs/msg/PointCloud2`)
  - `/ODOM` (`nav_msgs/msg/Odometry`)
- The preprocessor now produces two explicit views from the synchronized input cloud:
  - `cloud_in_base`: body-centric view for observability and sector semantics after body-box noise removal and local crop
  - `cloud_in_map`: internal local-map view expressed in `map_frame`
- Outputs are:
  - `/terrain_state` (`nav_msgs/msg/OccupancyGrid`)
  - `/terrain_cost` (`nav_msgs/msg/OccupancyGrid`)
  - `/terrain_obstacle_points`
  - `/terrain_debug/grid_map` (`grid_map_msgs/msg/GridMap`)
  - `/terrain_debug/base_gravity_cloud`
  - `/terrain_debug/support_points`
- `/terrain_debug/unknown_mask`
- `/terrain_debug/observability` (`passable_area/msg/TerrainObservability`)
- `/passable_area/status_code` (`std_msgs/msg/Int32`)

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

`PolarFrontend` keeps body-centric observability semantics separate from map-centric map projection: sector logic is derived from base-view samples, while candidate aggregation and map indexing use the internal map-view samples.
Obstacle formation is additionally guarded by local structure: a single-cell vertical span only marks the cell as suspicious, and the cell is promoted to an obstacle only when its fixed 3x3 neighborhood contains enough upper-support cells.
For layered below-robot cells, `PolarFrontend` may temporarily lift a frontend-local `effective_support_ref` for the current cell explanation when the upper layer is better supported by neighboring anchored support than the lower layer; this does not rewrite map `support_height` and does not participate in ascending stair-trend counting.

Frame semantics are intentionally split:
- `map_frame`: external parent frame used by synchronized pose input, TF parent naming, and all documented local-map semantics
- `base_gravity_frame`: robot-centric gravity frame used by debug point clouds; origin is the current robot pose, roll/pitch are removed, yaw is preserved

Published map semantics are intentionally split from internal map semantics:
- `grid_map`, `terrain_state`, and `terrain_cost` are published as robot-centric `base_gravity` views
- internal map storage and all core outputs remain in the continuous local `map` frame

Preprocessing semantics are intentionally split as well:
- `preprocess.body_filter.*` is interpreted in `base_link`
- `map_height_min/max` is interpreted as robot-relative height, matching `base_gravity` z semantics
- local XY crop is implemented internally against the current local map window

Current debug point semantics:
- `/terrain_debug/base_gravity_cloud`: full preprocessed algorithm cloud expressed in `base_gravity_frame`
- `/terrain_debug/support_points`: real input samples that land in support cells and remain close to the cell support height, expressed in `base_gravity_frame`
- `/terrain_obstacle_points`: real input samples that land in obstacle cells whose `obstacle_evidence` is already high enough and whose height is at least `obstacle_points_min_height` relative to the cell support reference, not as an absolute z threshold, expressed in `base_gravity_frame`
- `/terrain_debug/unknown_mask`: unknown cell centers expressed in `base_gravity_frame`
- these point outputs are intended to align directly with the robot-centric `grid_map`, `terrain_state`, and `terrain_cost` views in RViz
- `TerrainObservability.map_point_count`: counts samples in the internal map-view cloud (`cloud_in_map`)
- `/terrain_debug/observability`: non-geometric debug summary that still uses the same `base_gravity` debug header context

## Layout
- `include/passable_area/core`: ROS-free data types and processing modules
- `include/passable_area/interfaces/ros`: converters, params, publishers, watchdog, node
- `include/passable_area/interfaces/common/logging`: shared logging adapters for ROS and future non-ROS hosts
- `src/core`: algorithm implementation
- `src/interfaces/ros`: ROS integration
- `msg/TerrainObservability.msg`: observability debug contract
- `test/`: automated gtests only
- `benchmarks/`: manual core and ROS performance benchmarks
- `config/offline_benchmark_bags.yaml`: shared offline benchmark manifest for obstacle and timing batch runs
- workspace-level `scripts/passable_obstacle_benchmark.sh`: obstacle-focused batch benchmark
- workspace-level `scripts/passable_timing_benchmark.sh`: timing-focused batch benchmark
- workspace-level `scripts/passable_benchmark.sh`: unified batch benchmark runner and final report entrypoint

## Build and test
From the workspace root:

```bash
colcon build --packages-select passable_area --symlink-install
source install/setup.bash
colcon test --packages-select passable_area --event-handlers console_direct+
```

Detailed test inventory, coverage matrix, and benchmark classification:

- `docs/test_guide.md`

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

Runtime status reporting:
- `output.status_code_topic`: status code topic name, default `/passable_area/status_code`
- published as `std_msgs/msg/Int32` at 10 Hz
- current codes:
  - `100` = `OK_RUNNING`
  - `1000` = `ERR_CLOUD_TIMEOUT`
  - `1001` = `ERR_ODOM_TIMEOUT`
  - `1002` = `ERR_SYNC_STALL`
  - `1004` = `ERR_OUTPUT_STALL`
  - `1902` = `FATAL_RUNTIME_EXCEPTION`

Runtime logging:
- `passable_area` now supports `dr_logger` with ROS fallback
- when `dr_logger` initializes successfully, messages are written only to `dr_logger`
- when `dr_logger` is disabled or initialization fails, logging falls back to ROS-only
- default `dr_logger` path: `/var/opt/robot/log`
- default properties path: `/var/opt/robot/conf/log.properties`
- environment overrides:
  - `PASSABLE_DR_LOGGER_ENABLE`
  - `PASSABLE_DR_LOGGER_NAME`
  - `PASSABLE_DR_LOGGER_PATH`
  - `PASSABLE_DR_LOGGER_PROPERTIES_PATH`

Relevant debug controls:
- `debug.publish_base_gravity_cloud`: enable publishing the full preprocessed algorithm cloud in `base_gravity_frame`
- `output.base_gravity_cloud_topic`: topic name for that cloud
- `debug.publish_map_to_base_gravity_tf`: whether to publish `map_frame -> base_gravity_frame`; default `false`

Relevant preprocessing controls:
- `preprocess.body_filter.*`: reject points inside a configured `base_link` box, intended for body-interior noise
- `preprocess.crop_to_map.enable`: reject points outside the current local map support window
- `preprocess.crop_to_map.xy_margin`: extra XY padding for the local crop window

Relevant dynamic-obstacle persistence controls:
- `obstacle_clear_observed_decay`: faster obstacle evidence decay when ground is re-observed without a new obstacle
- `obstacle_clear_partial_decay_scale`: scales that faster decay under partially observed sectors
- `obstacle_height_clear_threshold`: clears `overhead_height` once obstacle evidence has fallen below this threshold

Relevant obstacle-formation controls:
- `max_step_up`: the frontend uses `vertical_span > max_step_up * 0.75` as the direct obstacle trigger threshold

The current frontend has been rolled back to a simple per-cell baseline:
- support candidates come directly from the current cell `min_z`
- obstacle candidates form directly from the current cell `vertical_span`
- the old anchor/leak/neighbor-gate rule stack is no longer part of the active obstacle-formation path

Relevant obstacle-point output controls:
- `obstacle_points_min_evidence`: minimum obstacle evidence required before a cell can contribute to `/terrain_obstacle_points`
- `obstacle_points_min_height`: minimum height relative to the cell support reference required for a sample to be published to `/terrain_obstacle_points`, not an absolute z threshold; when historical `support_height` is missing, the current-frame `min_z` is used as fallback
- `obstacle_points_max_height_in_base_link`: maximum allowed sample height in `base_link` for `/terrain_obstacle_points`; samples above this ceiling are filtered at publish time even if they clear the support-relative height gate in `base_gravity`

## Offline analysis tools

`passable_area_offline_replay` currently provides three offline diagnosis modes:

- `--analyze-false-obstacles`: summarize why obstacle points appeared inside a robot-centric detection box
- `--analyze-missed-obstacles`: summarize why `/terrain_obstacle_points` did not appear inside a robot-centric ROI around a target start offset
- `--inspect-roi`: dump low-level per-cell ROI state for manual debugging

`--analyze-missed-obstacles` uses `base_gravity` ROI coordinates and only treats `/terrain_obstacle_points` as the detection signal. It does not use `terrain_state` as the primary pass/fail criterion.

See:
- `docs/false_obstacle_analysis_guide.md`
- `docs/miss_obstacle_analysis_guide.md`
- `docs/test_guide.md`
