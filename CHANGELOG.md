## [1.0.0] - 2026-04-14

### 🚀 Features

- Make passable status code topic configurable
- Level raw input clouds with odometry before passability
- Sync cloud and odometry with ExactTime for gravity leveling
- Separate odom mapping frame from base_gravity debug outputs
- Publish odom to base_gravity transform for debug outputs
- Add base_gravity debug cloud from preprocessed odom samples
- Add body-box filtering and local map cropping in preprocessing
- Publish all raw points from obstacle cells in debug cloud
- Clear dynamic obstacle overhead faster after ground is reobserved
- Publish map outputs as robot-centric base_gravity grids
- Add bag-level false obstacle analysis to offline replay
- Gate obstacle formation with local upper-support neighborhoods
- Filter terrain obstacle points to upper obstacle band by 0.2m
- Add anchored stair-mix obstacle filtering
- Add PCL dependency and link libraries to build targets
- Add warm-up windowed start-offset support for false-obstacle replay
- Implement neighbor consensus and validation utilities for elevated support surface estimation
- Add base_link max height gate for obstacle point publishing
- Size up max_step_up/down and min_clearance params
- Add rosbag2_storage dependency and support cross-version storage options in offline_replay tool
- Expand false obstacle analyzer with additional cell diagnostic metrics and support for effective elevation tracking
- Integrate dr_logger with ROS fallback for passable_area runtime logging
- Add runtime status code reporting for passable_area
- Log subscribed and output topics on startup

### 🐛 Bug Fixes

- Harden PointCloud2 layout validation before PCL conversion
- Shift traversal cost layer with explicit float unknown sentinel
- Fix watchdog timebase and stabilize dual-view map semantics
- Correct frame semantics for dual-view processing and debug outputs
- Align grid_map layer orientation with robot-centric outputs
- Gate debug obstacle points on configurable strong evidence threshold
- Warm up missed-obstacle replay and add unsupported-wall regression test
- Ignore stale support anchors for wall-only obstacle cells
- Ignore wall-like anchors with shallow upper bands
- Boost evidence for wall-like obstacle cells
- Scale wall-like obstacle evidence gain by cell geometry
- Add conditional support for rosidl_target_interfaces to maintain compatibility with different ROS 2 versions
- Fix Foxy/Humble build compatibility for offline replay and tests
- Reject below-robot ground layer mixes with minimal neighbor support
- Reject below-robot upstair ground layer mixes
- Decouple downstairs ground-mix from upstair trend checks
- Log initial input waiting state immediately

### 💼 Other

- Replace deprecated rosidl_target_interfaces usage

### 🚜 Refactor

- Rebuild module with core/interfaces architecture and new traversability pipeline
- Complete new pipeline baseline with replay and e2e benchmark
- Split preprocessed cloud into base and gravity views
- Remove fixed rear blind sector from observability
- Promote obstacle points config and default topic to runtime interface
- Improve test readability and standardize rosbag2 reader initialization in offline replay tool
- Modularize terrain support rejection logic and add non-collinear pattern detection for upper support cells
- Revert effective support elevation path while keep analyzer diagnostics
- Unify anchor validity and below-robot explanation gates
- Preserve stale-anchor filtering and split raw vs adjusted upper-support states
- Internalize effective support ref and make legacy upper-support semantics explicit
- Remove redundant common header files and update internal include paths
- Move aligned support from pre-veto to explanation stage
- Require explicit keep verdict for obstacle candidates
- Tighten anchor authority for pre-trigger leak suppression
- Gate pre-trigger leak suppression by anchor authority
- Flatten benchmarks and simplify ros interface layout

### 📚 Documentation

- Add doc of current algorithm scheme
- Add user guide for false obstacle offline analysis output
- Reorganize test assets and add test guide
- Document layered support logic in PolarFrontend
- Update algorithm scheme and add code walk-through doc
- Add polar frontend fused refactor plan

### ⚡ Performance

- Share robot-centric resampling across published map outputs

### 🎨 Styling

- Apply consistent clang-format styling across the codebase

### 🧪 Testing

- Load offline replay bag config from params file
- Format false obstacle replay output into readable frame blocks
- Polish false obstacle report text contrast and wording
- Rename suspicious frame summary labels and add rear dropout count
- Align report section headers with card width
- Report hotspot min and max obstacle point height in false obstacle analysis
- Report bag-relative start offset for false obstacle frames
- Add batch obstacle benchmark script and manifest
- Add 2 bags of upstairs scenario to benchmark
- Split offline benchmark and add timing metrics
- Add wall and below-robot obstacle regressions
- Add roi-based missed-obstacle analysis for terrain obstacle points
- Add downstairs bag which just resolved
- Add rosbag2_mtbf_x30_upstair_passage to offline benchmark configuration

### ⚙️ Miscellaneous Tasks

- Make passable_area output topics configurable via YAML
- Remove duplicated debug params from main config
- Adjust map boundary and body filter params
- Add rviz config for terrain debug only
- Update service launch target and remove redundant pass.launch.py file
- Add three new test bags to offline benchmark configuration
- Add new benchmark bag configurations to offline_benchmark_bags.yaml
- Decrease bags from benchmark for last revert
- Decrease obstacle_points_min_height in order to preserve the integrity of the obstacle point cloud
- Add comments to each function in polar frontend
- Expose neighbor-gate evidence in offline analysis
- Clarify facade evidence naming and demote legacy reject display
- Add shlibs.local override for log4cplus packaging dependency
## [0.0.7] - 2026-03-16

### 🚀 Features

- Publish placeholder /passable_status_code before passable clouds

### ⚙️ Miscellaneous Tasks

- Modify cpu affinity settings to 4,5
- Modify max height so as to align to input accumulate_cloud

## [0.0.6] - 2026-01-30

### 🚀 Features

- Add ground/ceiling neighbor gating and clarify solver params defaults
- Tighten solver parameters to prevent the inability to identify normal obstacbles

### 🐛 Bug Fixes

- Fill ground holes after neighbor gating using neighbor minima
- Relax the conditions for Impassable decided by traversal cost to resolve wrong Passable issue
- Reduce pre-enqueue seeds in flood-filling

## [0.0.5] - 2025-12-22

### 🚀 Features

- Add watchdog warnings for missing IMU/cloud data
- Add configurable missing traversal cost for holes and blind spots
- Add low-cost filtering for traversal cost layer

### 🐛 Bug Fixes

- Change the cost range to 0-100
- Fix coverability computation by using elevation layer
- Guard median filter against NaN propagation on ARM

## [0.0.4] - 2025-12-07

### 🚀 Features

- Make NaN stiff handling configurable
- Add lidar-based ray consistency for NaN cliffs and centralize param loading
- Simplify passability pipeline and seed BFS with front/back anchors
- For negative obstacles, set current cell to be Impassable as well
- Add reusable BFS seeding helper and extra centerline seeds
- Resolve Unknown cell with traversal cost
- Gate Unknown-to-obstacle flip by neighbors and cost
- Propagate raycast cliff drops into traversal cost

### 🚜 Refactor

- Pass passability params through a setter

## [0.0.3] - 2025-11-27

### 🚀 Features

- Max and min heights and length and width of the map can be set separately
- Refine obstacle filtering with baseline ground controls
- Check whether current cell is a cliff through various factors
- Align dog-specific body and lidar parameters, and enable blind check of m20
- Initially implement the filtering algorithm and ensure it can fall back to the result of original algorithm
- Modify functions and params to optimize filter
- Add regional fallback and legacy caching for elevation solver
- Add configurable center padding controls
- Parameterize topic names and log configured endpoints
- Add traversal cost layer and clean up roughness helpers
- Increase the limit on the maximum slope and decouple passability and traversal cost
- Add fallback processing logic for height difference and roughness
- Optimize slope computation via Local plane fitting
- Publish traversal cost via nav_msgs/OccupancyGrid
- Add traversal_cost OccupancyGrid publisher and safe zone tuning

### 🐛 Bug Fixes

- Fix wrong topic of launch files
- Fix inpaint center override and expose max pixels
- Fix refactor error that causes some fields of elevation params missing
- Align passability layers with traversal fallbacks
- NaN cell remains empty which leads to traversal cost of -1

### 🚜 Refactor

- Modify package name and executable
- Scope enums and centralize conversion helpers
- Extract layer processing helpers from ElevationMap
- Modularize elevation solver and add legacy toggle
- Refactor params loading process and rename histogram solver bins parameter
- Refactor traversal cost module
- Centralize shared types and sort out helpers
## [0.0.2] - 2025-10-13

### 🚀 Features

- Unify mapping and navigation program which differ from launch and config files
- Decide whether to enable blind check through the  field in the config file
## [0.0.1] - 2025-09-26

### 🚀 Features

- Subscribe and publish cloud data using SensorDataQoS

### 🐛 Bug Fixes

- Fix initialize bug and rewrite rviz2 config
- Fix topic error in launch file
- Fix occasional all-impassable issues
- Fix boundary impassable issues
- Fix impassable bug and remove height ratio calculation

### ⚡ Performance

- Spin the node using multi-threads executor
- Use queue instead of deque when BFS
