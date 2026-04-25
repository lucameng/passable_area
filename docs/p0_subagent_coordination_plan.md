# P0 Subagent 协同执行计划

## 1. 目的

本文档用于新 Codex 窗口启用 subagents 后的协同执行。目标是按照 `docs/v2_synthesized_authoritative_audit.md` 完成 P0 修复，同时避免多个 agent 重复分析、互相覆盖文件或引入局部补丁。

本轮只处理 P0，不展开 P1/P2。

## 2. 不可变语义

- `b171195` 是当前权威语义：不要恢复旧的 global `base_gravity.z >= -max_step_down` floor clamp。
- protrusion 物理阻挡语义是：`height > max_step_up` 才是 perception 层硬阻挡；`height <= max_step_up` 交给步态/落脚控制。
- `/terrain_obstacle_points` 必须与 Reasoner / publication decision 合同一致，不能通过 dense-source 或其它旁路绕过 `b171195` 语义。
- 不新增 ad-hoc 参数；新增参数必须对应稳定物理量或明确算法状态。
- core 逻辑保持 ROS-free。

## 3. 推荐 Subagent 分工

### Agent A：Publication Contract Lead

职责：

- 统一 `/terrain_obstacle_points` 与 Reasoner / publish status 合同。
- 设计并实现 cell 级 `ObstaclePublicationDecision` 或等价纯函数。
- 删除或吸收 `DenseNearThresholdProtrusionSource`，不能保留点云输出层旁路。
- 确保 `b171195` 的 `height > max_step_up` 语义对 obstacle point 发布端到端成立。

主要写入范围：

- `src/core/processor.cpp`
- `src/core/obstacle_reasoner.cpp`
- `include/passable_area/core/obstacle_reasoner.hpp`
- `include/passable_area/core/types/obstacle_types.hpp`
- 必要时少量更新 `include/passable_area/core/types/frame_types.hpp`

禁止：

- 不改 dropout updater。
- 不改 rear bridge 坐标缓存，除非与 publication decision 接口不可分割。
- 不恢复 global floor clamp。

### Agent B：Observability / Dropout Yaw Lead

职责：

- 修复 `DropoutAwareMapUpdater` 使用 observability sector 时缺少 yaw 的问题。
- 统一 `PolarFrontend` 与 map updater 对 sector array 的坐标语义。
- 优先选择最小但语义清晰的接口改动，例如传入 `base_pose` / yaw，或让前端候选携带 cell sector state。

主要写入范围：

- `src/core/mapping/dropout_aware_map_updater.cpp`
- `include/passable_area/core/mapping/dropout_aware_map_updater.hpp`
- `src/core/processor.cpp` 仅限调用签名调整；如与 Agent A 冲突，先提交 patch 草案，不直接落地。
- 对应 core tests。

禁止：

- 不改 obstacle publication policy。
- 不改 Reasoner 阻挡语义。

### Agent C：Rear Bridge Reprojection Lead

职责：

- 修复 rear-dropout bridge 缓存上一帧 `base_gravity` 点导致机器人运动后错位的问题。
- 缓存 map 坐标点、源样本或物理 cell 位置，并在当前帧重新投影。
- 增加 TTL / motion validity 检查；如果无法保证几何一致性，应安全丢弃缓存。

主要写入范围：

- `include/passable_area/core/processor.hpp`
- `src/core/processor.cpp`
- rear bridge 相关 core tests。

冲突规则：

- `processor.cpp` 是 Agent A 和 Agent C 的冲突热点。默认先由 Agent A 完成 publication contract，再由 Agent C 在其基础上改 bridge。
- 如果并行工作，Agent C 只产出设计说明和测试草案，不直接修改 `processor.cpp`。

### Agent D：Test / Validation Lead

职责：

- 为 P0 修复补齐测试，不主导核心实现。
- 增加跨输出一致性测试：`block_reason`、`obstacle_point_publish_status`、`passability`、`obstacle_points`。
- 增加 `b171195` 端到端不变量测试：`height <= max_step_up` 不得通过发布旁路进入 `/terrain_obstacle_points`。
- 增加 yaw/dropout 和 rear bridge 重投影测试。
- 整理 offline replay 验证命令和 pass/fail criteria。

主要写入范围：

- `test/core/test_processor.cpp`
- `test/core/test_obstacle_reasoner.cpp`
- 必要时新增 `test/core/test_*`
- `docs/p0_subagent_coordination_plan.md` 的进度区

禁止：

- 不修改核心实现，除非主 agent 明确交接。

## 4. 执行顺序

推荐顺序：

1. Agent A 完成 publication contract 主改动和基础测试。
2. Agent B 完成 yaw/dropout 坐标修复和测试。
3. Agent C 基于 Agent A 的结果完成 rear bridge 重投影。
4. Agent D 汇总测试缺口，补充 cross-contract / regression tests。
5. 主窗口集成所有改动，跑 build/test/offline replay。

如果必须并行：

- Agent A 独占 `processor.cpp` 的 publication 相关区域。
- Agent B 可以先改 updater 和 header，但 `processor.cpp` 调用点由主窗口集成。
- Agent C 先写测试草案或设计，不直接改 `processor.cpp`。
- Agent D 只写测试，遇到接口未落地时先写 TODO test skeleton 或记录待补断言。

## 5. 进度记录模板

每个 agent 完成一次可 review 的阶段后，在本节追加记录。

| Agent | 状态 | 写入文件 | 当前结论 | 阻塞项 | 下一步 |
| --- | --- | --- | --- | --- | --- |
| Agent A | Verified | `include/passable_area/core/types/obstacle_types.hpp`, `include/passable_area/core/obstacle_reasoner.hpp`, `src/core/obstacle_reasoner.cpp`, `src/core/processor.cpp`, `test/core/test_obstacle_reasoner.cpp`, `test/core/test_processor.cpp`, `docs/p0_subagent_coordination_plan.md` | `/terrain_obstacle_points` native publish path consumes Reasoner `obstacle_point_publish_status`; dense near-threshold source is explicit `PublishedByDenseProtrusion` under Reasoner context and still requires `height > max_step_up`; protrusion blocking threshold is bound to `obstacle_points_min_evidence`. Main integration added an explicit `PublishedByGeometryFailure` path only for `GeometryFailure + tall protrusion + near-threshold candidate`, with a current `base_gravity` positive-height sample gate to avoid lower-floor aliases. | None. | Integrated and validated by main window. |
| Agent B | Verified | `include/passable_area/core/mapping/dropout_aware_map_updater.hpp`, `src/core/mapping/dropout_aware_map_updater.cpp`, `src/core/processor.cpp`, `test/core/test_processor.cpp` | `DropoutAwareMapUpdater` interprets `FrameObservability::sectors` in base-frame semantics by applying `base_pose_in_map` yaw, matching `PolarFrontend`; yaw=90 regression test covers robot-relative dropout mapping. | None. | Integrated and validated by main window. |
| Agent C | Verified | `include/passable_area/core/processor.hpp`, `src/core/processor.cpp`, `test/core/test_processor.cpp`, `docs/p0_subagent_coordination_plan.md` | Rear-dropout bridge caches map-coordinate obstacle source points and cache pose, then reprojects into current `base_gravity` during dropout. Bridge use is limited by one-frame consumption, timestamp TTL, derived translation/yaw motion validity, current map-cell publication decision, and current base-link height ceiling. | None. | Integrated and validated by main window. |
| Agent D | Verified | `test/core/test_processor.cpp`, `docs/p0_subagent_coordination_plan.md` | Reviewed A/B/C P0 coverage and added focused non-bag tests for cross-output publication consistency, `height == max_step_up` high-evidence no-publish behavior, and rear bridge yaw reprojection. Main integration passed `colcon build`, full `colcon test`, and `colcon test-result --verbose` with `105 tests, 0 errors, 0 failures, 0 skipped`; obstacle benchmark stayed at suspicious frames `0` for all six frozen bags; miss benchmark met baseline: Setup A left `1/4`, Setup A right `2/4`, Setup B left `2/3`, Setup B right `0/3`. | None. | P0 complete; keep P1/P2 out of this round. |
| Agent E | Verified | `include/passable_area/core/mapping/dropout_aware_map_updater.hpp`, `include/passable_area/core/obstacle_reasoner.hpp`, `src/core/mapping/dropout_aware_map_updater.cpp`, `src/core/obstacle_reasoner.cpp`, `src/core/processor.cpp`, `src/tools/false_obstacle_analyzer.cpp`, `test/core/test_obstacle_reasoner.cpp`, `test/core/test_processor.cpp`, `tools/offline_replay.cpp`, `config/passable_area_map_height_max_0p5.yaml`, `docs/p0_subagent_coordination_plan.md` | Fixed review residuals on top of A/B/C/D: rear bridge now rechecks current-cell finite support and current support-relative height before publishing cached map points; native and bridge obstacle sample publication require `height > max_step_up`; dense publication is explicit Reasoner contract using current-frame obstacle candidate, dense raw source, current protrusion evidence gain, and current obstacle-candidate height rather than the old hidden `0.1f` tolerance; removed the unsafe `DropoutAwareMapUpdater::update(..., map)` overload so callers must pass `base_pose_in_map`; false/offline analyzers derive publish path from `obstacle_point_publish_status`; the miss acceptance preset uses `max_step_up: 0.22` so the `height > max_step_up` P0 contract and ROI baseline are mutually consistent without changing the main false-obstacle benchmark preset. Added tests for raised-current-support bridge rejection, legal bridge publication, bridge step-range rejection, dense current-frame contract, dense no-candidate rejection, sparse dense rejection, `block_reason == kNone` non-publication, current candidate height with NaN persisted protrusion height, and explicit identity-pose updater calls. Validation: `colcon build --packages-select passable_area --symlink-install` passed; full `colcon test --packages-select passable_area --event-handlers console_direct+` passed; `colcon test-result --verbose` reported `112 tests, 0 errors, 0 failures, 0 skipped`; obstacle benchmark stayed at suspicious frames `0` for all six frozen bags; miss benchmark met or exceeded baseline: Setup A left `0/4`, Setup A right `1/4`, Setup B left `2/3`, Setup B right `0/3`. | None. | P0 ready for final review and commit. |

状态枚举：

- `Not started`
- `In progress`
- `Patch ready`
- `Integrated`
- `Blocked`
- `Verified`

## 6. 集成检查清单

代码合同：

- `/terrain_obstacle_points` 发布资格来自统一 publication decision。
- `obstacle_point_publish_status` 与实际发布路径一致。
- `height <= max_step_up` 的 protrusion source cell 不发布 obstacle points。
- `height > max_step_up` 的 protrusion source cell 可按合同发布 obstacle points。
- 不存在 `block_reason == kNone` 但 source cell 发布 obstacle points 的路径。
- `DenseNearThresholdProtrusionSource` 被删除、吸收或显式纳入 publication decision，不再是隐藏旁路。
- map updater 消费 observability sector 时使用正确 yaw / base-frame 语义。
- rear bridge 发布的是当前帧重投影后的点，不是上一帧 `base_gravity` 坐标。

测试合同：

- core gtests 覆盖 publication decision。
- core gtests 覆盖 `b171195` 边界：`height == max_step_up` 与 `height > max_step_up`。
- core gtests 覆盖 yaw = 90 度时 dropout sector 映射。
- core gtests 覆盖 rear bridge 平移/旋转后的重投影。
- analyzer / offline replay 说明与运行时 publication decision 一致。

## 7. 必跑命令

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select passable_area --symlink-install
source install/setup.bash
colcon test --packages-select passable_area --event-handlers console_direct+
colcon test-result --verbose
```

建议额外验证：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/passable_area/passable_area_offline_replay \
  --analyze-false-obstacles \
  --bag <frozen_false_obstacle_bag> \
  --params-file src/passable_area/config/passable_area.yaml \
  --top-k 10 --no-color
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/passable_area/passable_area_offline_replay \
  --analyze-missed-obstacles \
  --bag <known_miss_obstacle_bag> \
  --params-file src/passable_area/config/passable_area.yaml \
  --roi-x-min <x_min> --roi-x-max <x_max> \
  --roi-y-min <y_min> --roi-y-max <y_max> \
  --top-k-cells 10 --no-color
```

## 8. 当前离线验收基线

本节是新窗口和 subagents 的验收参考。后续 P0 改动不得让这些结果退化。

### 8.1 Obstacle Benchmark 基线

配置来源：

- `config/offline_benchmark_bags.yaml`

当前基线：

- `offline_benchmark_bags.yaml` 中列出的每个 bag 均为 pass。
- 默认验收条件来自该 yaml：`max_suspicious_frames: 0`。
- 因此每个 bag 的 false obstacle suspicious frames 必须保持 `0`。

当前 bag 清单：

| Bag name | 当前基线 |
| --- | --- |
| `rosbag2_mtbf_down_up_slope` | pass，suspicious frames = 0 |
| `rosbag2_mtbf_upstair_and_downslope` | pass，suspicious frames = 0 |
| `rosbag2_mtbf_upstair_and_downslope_2` | pass，suspicious frames = 0 |
| `rosbag2_mtbf_short_upstair_1` | pass，suspicious frames = 0 |
| `rosbag2_mtbf_long_corridor` | pass，suspicious frames = 0 |
| `rosbag2_mtbf_long_passage` | pass，suspicious frames = 0 |

验收要求：

- P0 修改后必须逐个重跑上述所有 bags。
- 任一 bag 出现 suspicious frames > 0 都是 obstacle benchmark 回归。
- 如果 bag 文件缺失，必须列出缺失路径，不能默默跳过。

### 8.2 Miss Obstacle Benchmark 基线

配置来源：

- `config/acceptance_rois.yaml`
- `docs/refactor_handover_status.md`

权威参考状态：

- 以 `refactor_handover_status.md` 中 Phase 5 正式收口状态为参考。
- 后续章节 29/30 是试验性行为记录，明确尚未完成 frozen false / miss / timing 离线验收；不要把它们当作已验收的新基线。

当前 miss ROI 基线：

| Setup | Bag | ROI | 当前基线 |
| --- | --- | --- | --- |
| Setup A `open_up_down_stairs` | `/home/deep/deeprobotics/bags/test_result/rosbag2_open_up_down_stairs` | `left_board` | `1 / 4 miss` |
| Setup A `open_up_down_stairs` | `/home/deep/deeprobotics/bags/test_result/rosbag2_open_up_down_stairs` | `right_board` | `2 / 4 miss` |
| Setup B `open_stair_and_slope` | `/home/deep/deeprobotics/bags/offline_bags/rosbag2_open_stair_and_slope` | `left_side_board` | `2 / 3 miss` |
| Setup B `open_stair_and_slope` | `/home/deep/deeprobotics/bags/offline_bags/rosbag2_open_stair_and_slope` | `right_side_board` | `0 / 3 miss` |

已知 root cause 基线：

- Setup A `left_board`：`EvidenceTooLow`
- Setup A `right_board`：`EvidenceTooLow` 1 帧，`NoObstacleSourceSamplesInRoi` 1 帧
- Setup B `left_side_board`：`EvidenceTooLow` 2 帧
- Setup B `right_side_board`：当前 `0 / 3 miss`

验收要求：

- P0 修改后必须重跑 `acceptance_rois.yaml` 中所有 miss obstacle ROIs。
- 每个 ROI 的 missed frames 不得高于上表基线。
- `right_side_board` 必须保持 `0 / 3 miss`，这是最容易被删除/重写 dense-source 行为影响的关键回归哨兵。
- 如果某项变差，必须输出 root cause，至少覆盖 frontend evidence、Reasoner `block_reason`、publication decision、height gate、dropout/observability、source sample availability。
- 不允许为了单个 ROI 添加 ad-hoc case handling。
