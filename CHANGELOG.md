## [0.0.4] - 2025-12-09

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

## [0.0.2] - 2025-10-29

### 🚀 Features

- Implement lidar coverage detection and void-filling in specific areas
- Add inpainted points to original point cloud
- Comparison of height is converted to body frame
- Processing of blind spots of Lidars using dummy elevation layer
- Unify mapping and navigation program which differ from launch and config files
- Decide whether to enable blind check through the field in the config file

### 🚜 Refactor

- Modify namespaces and filenames
## [0.0.1] - 2025-09-16

### 🐛 Bug Fixes

- Fix initialization bug which causes crash

### 🚜 Refactor

- Refactor code and support ros1

### ⚡ Performance

- Remove height ratio calculation to reduce cpu usage
- Use queue instead of deque when BFS
