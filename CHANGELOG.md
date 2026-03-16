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
