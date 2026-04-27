# P1 Final Report

## Scope Status

P1-A/B/C are complete on top of the repaired P0 baseline
`fe6146f fix: prevent obstacle overlap from polluting support`. P1-D was
reopened after strict audit feedback because `67f15ee` only closed the diagonal
slope-distance subtask, not the full P1-4 terrain geometry item. The reopened
P1-D follow-up has passed build, tests, P0 benchmark validation, and strict
read-only review. A follow-up audit then found that the support-surface
eligibility filter was still coupled to aggregate `obstacle_evidence`; the
current refinement keeps overhead-only support neighbors eligible and uses only
active protrusion evidence as support-surface contamination. The refinement has
passed build, tests, P0 benchmark validation, and strict read-only review.

Completed P1 commits:

- P1-A: `a2c9d71 fix: add symmetric dropout sector detection`
- P1-B: `c61d5d1 fix: gate low clearance by overhead state`
- P1-C: `1e78565 fix: require finite support for obstacle publication`
- P1-D partial: `67f15ee fix: use diagonal distance for terrain slope`
- P1-D completion: `08e5410 fix: robustify support surface geometry`

## P1-B Low-Clearance Orthogonalization

Low-clearance geometry now remains separate from overhead evidence/state.
`ObstacleReasoner` only emits `LowClearance` / `Mixed` block reasons when
low-clearance geometry has an active overhead state. Obstacle point publication
continues to require its independent evidence and height gates, including the
`clearance > max_step_up` low-clearance bridge gate.

Validation:

- `colcon build --packages-select passable_area --symlink-install`: pass
- `colcon test --packages-select passable_area --event-handlers console_direct+`: pass
- `colcon test-result --verbose`: `126 tests, 0 errors, 0 failures, 0 skipped`
- false-obstacle benchmark: all six frozen bags passed with `suspicious_frames: 0`
- miss-obstacle acceptance: Setup A left `1/4`, Setup A right `2/4`, Setup B left `2/3`, Setup B right `0/3`
- strict read-only review: no blocking findings

## P1-C Support Reference Consistency

Native obstacle point publication no longer uses current-frame sample minimum as
a temporary support reference. Publication requires finite trusted map
`support_height`, and the miss-obstacle analyzer now mirrors the same finite
support and strict `max_sample_z > support_ref + max_step_up` publication
height gate.

Validation:

- `colcon build --packages-select passable_area --symlink-install`: pass
- `colcon test --packages-select passable_area --event-handlers console_direct+`: pass
- `colcon test-result --verbose`: `128 tests, 0 errors, 0 failures, 0 skipped`
- false-obstacle benchmark: all six frozen bags passed with `suspicious_frames: 0`
- miss-obstacle acceptance: Setup A left `1/4`, Setup A right `2/4`, Setup B left `2/3`, Setup B right `0/3`
- strict read-only review: no blocking findings

## P1-D Terrain Geometry Robustness

The original P1-D commit only made slope use actual neighbor planar distance.
The reopened follow-up extends this to the full P1-4 geometry requirement:

- active protrusion-evidence neighbors no longer participate in support-surface
  geometry, preventing wall-foot bleed into standable cells without excluding
  overhead-only low-clearance support neighbors
- slope and roughness are computed from a local support plane over the filtered
  support-surface neighborhood
- `step_up` / `step_down` are directional costs for entering the current cell
  from a neighboring support cell
- obstacle publication paths remain unchanged

Validation:

- `colcon build --packages-select passable_area --symlink-install`: pass
- `colcon test --packages-select passable_area --event-handlers console_direct+`: pass
- `colcon test-result --verbose`: `133 tests, 0 errors, 0 failures, 0 skipped`
- false-obstacle benchmark: all six frozen bags passed with `suspicious_frames: 0`
- miss-obstacle acceptance: Setup A left `1/4`, Setup A right `2/4`, Setup B left `2/3`, Setup B right `0/3`
- strict read-only review: no blocking findings

## Benchmark Summary

False-obstacle frozen bags all remained at `suspicious_frames: 0`:

- `rosbag2_mtbf_down_up_slope`
- `rosbag2_mtbf_upstair_and_downslope`
- `rosbag2_mtbf_upstair_and_downslope_2`
- `rosbag2_mtbf_short_upstair_1`
- `rosbag2_mtbf_long_corridor`
- `rosbag2_mtbf_long_passage`

Miss-obstacle acceptance stayed at the repaired P0/P1 baseline:

- Setup A `left_board`: `1 / 4` missed
- Setup A `right_board`: `2 / 4` missed
- Setup B `left_side_board`: `2 / 3` missed
- Setup B `right_side_board`: `0 / 3` missed

## Remaining Risks

None blocking.

Non-blocking follow-up candidates:

- Add a Processor-level regression proving native obstacle publication requires
  finite trusted `support_height`; P1-C locked the diagnostic analyzer and
  runtime code path, but the strict review noted this extra test would reduce
  future risk.
