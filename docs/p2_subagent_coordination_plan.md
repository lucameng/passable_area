# P2 Subagent Coordination Plan

## Scope

This execution board tracks the P2 work that follows the clean P1 baseline.
It is not an authoritative design document; `docs/algorithm_scheme.md` and the
code/tests remain the behavior contract.

P2 must preserve the P0/P1 contracts:

- no obstacle point publication for `height <= max_step_up`
- no global `base_gravity` floor clamp
- no bag-, ROI-, frame-, or patch-specific fixes
- no benchmark or acceptance-preset pollution
- core logic remains ROS-free

## Progress

| Stage | Status | Write files | Implementation summary | Tests added/updated | Validation results | Strict review result | Blockers |
| --- | --- | --- | --- | --- | --- | --- | --- |
| P2-A Parameter and Hidden-Constant Contract | Committed as `3175a4f fix: name terrain geometry constants` | `include/passable_area/core/config_validation.hpp`, `src/core/config_validation.cpp`, `CMakeLists.txt`, `src/core/polar_frontend.cpp`, `src/core/obstacle_reasoner.cpp`, `src/core/traversability_solver.cpp`, `src/core/mapping/dropout_aware_map_updater.cpp`, `src/core/mapping/local_terrain_map.cpp`, `src/core/processor.cpp`, `src/interfaces/ros/ros_param_loader.cpp`, `test/core/test_processor.cpp`, `test/interfaces/ros/test_ros_param_loader.cpp`, `docs/algorithm_scheme.md`, `docs/p2_subagent_coordination_plan.md` | Names internal constants in the frontend, reasoner, solver, map updater, and processor; adds core config validation for stable physical/config invariants; makes `obstacle_clear_partial_decay_scale` control observed partial obstacle-evidence decay without adding a new public knob. | Added config invariant validation coverage, ROS loader invalid-config coverage, and a focused map-updater test proving `obstacle_clear_partial_decay_scale` controls observed partial obstacle decay. | `colcon build --packages-select passable_area --symlink-install`: pass. `colcon test --packages-select passable_area --event-handlers console_direct+`: pass. `colcon test-result --verbose`: `136 tests, 0 errors, 0 failures, 0 skipped`. False-obstacle benchmark: all six frozen bags pass with `suspicious_frames: 0`. Miss acceptance: Setup A left `1/4`, Setup A right `2/4`, Setup B left `2/3`, Setup B right `0/3`. | P2-R strict read-only review: no blocking findings. Non-blocking note: existing DenseProtrusionSource docs should be cleaned in P2-B/docs pass. | None. |
| P2-B Runtime/Offline Publication Decision Trace Unification | Commit pending | `include/passable_area/core/obstacle_publication.hpp`, `src/core/obstacle_publication.cpp`, `CMakeLists.txt`, `src/core/processor.cpp`, `src/tools/miss_obstacle_analyzer.cpp`, `src/tools/false_obstacle_analyzer.cpp`, `test/core/test_obstacle_reasoner.cpp`, `test/tools/test_miss_obstacle_analyzer.cpp`, `test/tools/test_false_obstacle_analyzer.cpp`, `docs/algorithm_scheme.md`, `docs/miss_obstacle_analysis_guide.md`, `docs/false_obstacle_analysis_guide.md`, `docs/p2_subagent_coordination_plan.md` | Adds a ROS-free shared obstacle-publication decision trace used by runtime processing and offline analyzers; removes duplicated analyzer height/bridge/status interpretation; updates diagnostic wording to the current support-relative publication contract; keeps publication gates correlated per runtime sample, including the geometry-failure base-gravity lower floor. | Added shared publication-trace boundary tests for `height <= max_step_up`, missing support, geometry-failure floor, no-sample finalization, and publish-path naming; updated miss/false analyzer tests to assert analyzer-facing status follows runtime publication semantics and cannot combine uncorrelated sample extrema. | `colcon build --packages-select passable_area --symlink-install`: pass. `ROS_LOG_DIR=/tmp colcon test --packages-select passable_area --event-handlers console_direct+`: pass. `colcon test-result --verbose`: `140 tests, 0 errors, 0 failures, 0 skipped`. False-obstacle benchmark: all six frozen bags pass with `suspicious_frames: 0`. Miss acceptance: Setup A left `1/4`, Setup A right `2/4`, Setup B left `2/3`, Setup B right `0/3`. `git diff --check`: pass. | P2-R strict read-only review rerun: no blocking or non-blocking findings. Prior blockers for geometry-failure helper parity and correlated analyzer samples were verified fixed. | None. |
| P2-C Explicit Support-Surface Contamination Eligibility | Pending | TBD | TBD | TBD | TBD | TBD | None. |
| P2-D Final P2 Documentation and Handover Consolidation | Pending | TBD | TBD | TBD | TBD | TBD | None. |
