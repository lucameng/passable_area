# P2 Final Report

## Scope

P2 closed the follow-up issues listed in `docs/v2_synthesized_authoritative_audit.md`:

- P2-A: Parameter and hidden-constant contract (`P2-1`)
- P2-B: Runtime/offline publication decision trace unification (`P2-2`)
- P2-C: Explicit support-surface contamination eligibility (`P2-3`)
- P2-D: Documentation and handover consolidation

P2 preserved the P0/P1 contracts: no obstacle point publication for `height <= max_step_up`, no restoration of the old global `base_gravity` floor clamp, no bag/ROI/frame-specific fixes, no benchmark config pollution, and ROS-free core behavior.

## Completed Changes

### P2-A Parameter and Hidden-Constant Contract

Commit: `3175a4f fix: name terrain geometry constants`

- Added core config validation for stable physical/config invariants.
- Named internal frontend/reasoner/solver/map-updater/processor constants without adding ad-hoc YAML knobs.
- Wired existing `obstacle_clear_partial_decay_scale` into observed partial obstacle-evidence decay.
- Added focused config and map-updater tests.

### P2-B Runtime/Offline Publication Decision Trace Unification

Commit: `97308f6 fix: unify publication decision trace`

- Added a ROS-free obstacle-publication helper and structured decision trace.
- Runtime publishing, miss analyzer, and false analyzer now use shared publication status/path semantics.
- Publication height gates are evaluated per correlated runtime sample, including support-relative strict `max_step_up`, finite support, base-link ceiling, and geometry-failure base-gravity floor.
- Removed duplicated offline bridge/status/height-gate inference and stale base-gravity/base-link wording.

### P2-C Explicit Support-Surface Contamination Eligibility

Commit: `9d0e9d9 fix: clarify support contamination eligibility`

- Added internal `support_surface_contaminated` terrain-layer semantics.
- `DropoutAwareMapUpdater` derives the layer from active protrusion evidence and clears it for overhead-only refreshes.
- `TerrainFeatureUpdater` now consumes explicit support-geometry eligibility instead of directly reading a publication-named threshold.
- Overhead-only low-clearance support remains eligible for support geometry; active protrusion/wall-foot contamination is excluded; inactive/residual evidence does not indefinitely exclude support geometry.
- Obstacle publication behavior is unchanged.

### P2-D Documentation Consolidation

Commit: this report is committed by the P2-D closeout commit (`docs: consolidate P2 handover`).

- Consolidated P2-A/B/C/D status, validation, review, and remaining-risk conclusions here.
- Kept `docs/p2_subagent_coordination_plan.md` as the execution board rather than long-term design authority.

## Validation Results

### P2-A

- `colcon build --packages-select passable_area --symlink-install`: pass
- `colcon test --packages-select passable_area --event-handlers console_direct+`: pass
- `colcon test-result --verbose`: `136 tests, 0 errors, 0 failures, 0 skipped`
- False-obstacle benchmark: all six frozen bags passed with `suspicious_frames: 0`
- Miss acceptance:
  - Setup A left: `1 / 4 missed`
  - Setup A right: `2 / 4 missed`
  - Setup B left: `2 / 3 missed`
  - Setup B right: `0 / 3 missed`

### P2-B

- `colcon build --packages-select passable_area --symlink-install`: pass
- `ROS_LOG_DIR=/tmp colcon test --packages-select passable_area --event-handlers console_direct+`: pass
- `colcon test-result --verbose`: `140 tests, 0 errors, 0 failures, 0 skipped`
- False-obstacle benchmark: all six frozen bags passed with `suspicious_frames: 0`
- Miss acceptance:
  - Setup A left: `1 / 4 missed`
  - Setup A right: `2 / 4 missed`
  - Setup B left: `2 / 3 missed`
  - Setup B right: `0 / 3 missed`

### P2-C

- `colcon build --packages-select passable_area --symlink-install`: pass
- `ROS_LOG_DIR=/tmp colcon test --packages-select passable_area --event-handlers console_direct+`: pass
- `colcon test-result --verbose`: `145 tests, 0 errors, 0 failures, 0 skipped`
- False-obstacle benchmark: all six frozen bags passed with `suspicious_frames: 0`
- Miss acceptance:
  - Setup A left: `1 / 4 missed`
  - Setup A right: `2 / 4 missed`
  - Setup B left: `2 / 3 missed`
  - Setup B right: `0 / 3 missed`

### P2-D

- Documentation-only stage.
- Validation was rerun after the final P2-D documentation edits.
- `colcon build --packages-select passable_area --symlink-install`: pass
- `ROS_LOG_DIR=/tmp colcon test --packages-select passable_area --event-handlers console_direct+`: pass
- `colcon test-result --verbose`: `145 tests, 0 errors, 0 failures, 0 skipped`
- Offline benchmarks were not rerun for P2-D because no code, config, analyzer, obstacle, dropout, terrain, support, publication, or acceptance semantics changed.

## Strict Review Results

- P2-A: strict read-only review found no blocking issues. One non-blocking DenseProtrusionSource documentation note was addressed in P2-B.
- P2-B: first review found blocking publication-trace parity issues; fixes were applied. Final strict read-only rerun found no blocking or non-blocking issues.
- P2-C: first review found blocking residual-contamination behavior under overhead-only refresh; fixes were applied. Final strict read-only rerun found no blocking or non-blocking issues.
- P2-D: initial strict read-only documentation review found documentation-completeness blockers only: validation evidence needed after doc edits and final review result still pending. Validation was rerun after the doc edits. Final strict read-only review rerun found no blocking findings; the only non-blocking note was to record the final review result before commit.

## Remaining Risks

None known for P2 scope after P2-A/B/C validation and strict review.

Operational notes:

- ROS-facing tests should be run with `ROS_LOG_DIR=/tmp` in this sandbox to avoid `/home/deep/.ros/log` write failures.
- The DDS socket warnings seen during offline replay are sandbox/network-interface warnings; the rosbag analyses completed successfully.

## Current P2 Commit Stack

- `3175a4f fix: name terrain geometry constants`
- `97308f6 fix: unify publication decision trace`
- `9d0e9d9 fix: clarify support contamination eligibility`
- `283200b docs: consolidate P2 handover`
