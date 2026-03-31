# Passable Area

`passable_area` is now a single-node, dropout-aware local terrain evaluator for quadruped navigation. The node keeps ROS concerns in `interfaces/ros` and runs the terrain logic in a ROS-free `core` pipeline.

## Runtime model
- Inputs are strictly synchronized with `ExactTime`:
  - `/LOC_BODY_POINTS` (`sensor_msgs/msg/PointCloud2`)
  - `/ODOM` (`nav_msgs/msg/Odometry`)
- The preprocessor now produces two explicit views from the synchronized input cloud:
  - `cloud_in_base`: body-centric view for observability and sector semantics
  - `cloud_in_gravity`: gravity-aligned view for mapping, features, and traversability
- Outputs are:
  - `/terrain_state` (`nav_msgs/msg/OccupancyGrid`)
  - `/terrain_cost` (`nav_msgs/msg/OccupancyGrid`)
  - `/terrain_debug/grid_map` (`grid_map_msgs/msg/GridMap`)
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

`FrameObservabilityEstimator` only consumes `cloud_in_base`. It no longer relies on transforming points back from `gravity_frame` to recover front/rear/blind/dropout semantics.

`PolarFrontend` keeps body-centric observability semantics separate from gravity-centric map projection: sector logic is derived from base-view samples, while candidate aggregation and map indexing use gravity-view samples.

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
