## [0.0.3-ros2] - 2025-11-27

### 🚀 Features

- [feat] max and min heights and length and width of the map can be set separately
- [feat] refine obstacle filtering with baseline ground controls
- [feat] check whether current cell is a cliff through various factors
- [feat] align dog-specific body and lidar parameters, and enable blind check of m20
- [feat] initially implement the filtering algorithm and ensure it can fall back to the result of original algorithm
- [feat] modify functions and params to optimize filter
- [feat] add regional fallback and legacy caching for elevation solver
- [feat] add configurable center padding controls
- [feat] parameterize topic names and log configured endpoints
- [feat] add traversal cost layer and clean up roughness helpers
- [feat] increase the limit on the maximum slope and decouple passability and traversal cost
- [feat] add fallback processing logic for height difference and roughness
- [feat] optimize slope computation via Local plane fitting
- [feat] publish traversal cost via nav_msgs/OccupancyGrid
- [feat] add traversal_cost OccupancyGrid publisher and safe zone tuning

### 🐛 Bug Fixes

- [fix] fix wrong topic of launch files
- [fix] fix inpaint center override and expose max pixels
- [fix] fix refactor error that causes some fields of elevation params missing
- [fix] align passability layers with traversal fallbacks
- [fix] NaN cell remains empty which leads to traversal cost of -1

### 🚜 Refactor

- [refactor] modify package name and executable
- [refactor] scope enums and centralize conversion helpers
- [refactor] extract layer processing helpers from ElevationMap
- [refactor] modularize elevation solver and add legacy toggle
- [refactor] refactor params loading process and rename histogram solver bins parameter
- [refactor] refactor traversal cost module
- [refactor] centralize shared types and sort out helpers

## [0.0.2-ros2] - 2025-10-13

### 🚀 Features

- [feat] unify mapping and navigation program which differ from launch and config files
- [feat] decide whether to enable blind check through the  field in the config file

## [0.0.1-ros2] - 2025-09-26

### 🚀 Features

- [feat] subscribe and publish cloud data using SensorDataQoS

### 🐛 Bug Fixes

- [fix] fix initialize bug and rewrite rviz2 config
- [fix] fix topic error in launch file
- [fix] fix occasional all-impassable issues
- [fix] fix boundary impassable issues
- [fix] fix impassable bug and remove height ratio calculation

### ⚡ Performance

- [perf] spin the node using multi-threads executor
- [perf] use queue instead of deque when BFS
