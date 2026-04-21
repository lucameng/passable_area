# V2 Obstacle Frontend 重构实施计划

基于 `docs/refactor_handover_status.md` 方案评审结论 + 用户决策。

---

## 前提决策记录

| 决策项 | 决定 |
|--------|------|
| `obstacle_point_publish_status` enum 定义时机 | Phase 1 |
| Low Clearance → obstacle_points 规则定义时机 | Phase 1 |
| `kPartiallyObserved` 删除后合并方向 | → `kObserved` |
| `PolarFrontend` 接口签名收窄 | 接受，不再暴露 `LocalTerrainMap` layers |
| False obstacle 验收 bags | `offline_benchmark_bags.yaml` 全部（除 x30_upstair_passage） |
| Miss obstacle 验收 ROIs | Setup A + Setup B（见下文冻结的 ROI 定义） |
| Low clearance 验收 bag | 暂不冻结 |

---

## Phase 1：冻结类型 / 枚举 / 层合同 / 验收 ROI

**目标**：在不改行为的前提下，把 V2 所需的全部类型、枚举、层定义冻结下来，同时冻结验收资产。

### 涉及文件

| 动作 | 文件 |
|------|------|
| MODIFY | `include/passable_area/core/types/state_types.hpp` |
| MODIFY | `include/passable_area/core/types/frame_types.hpp` |
| MODIFY | `include/passable_area/core/candidate_types.hpp` |
| MODIFY | `include/passable_area/core/mapping/terrain_layers.hpp` |
| NEW | `include/passable_area/core/types/obstacle_types.hpp` |
| NEW | `config/acceptance_rois.yaml` |
| MODIFY | `test/core/test_processor.cpp` — 删除 AmbiguousCandidate 相关 case |
| MODIFY | `src/core/polar_frontend.cpp` — 删除 ambiguous 生成 |
| MODIFY | `src/core/mapping/dropout_aware_map_updater.cpp` — 消除 kPartiallyObserved 分支 |
| MODIFY | `src/core/frame_observability_estimator.cpp` — 不再产生 kPartiallyObserved |

### 具体变更

#### 1.1 删除 `ObservabilityState::kPartiallyObserved`

`state_types.hpp`：
```cpp
enum class ObservabilityState : uint8_t {
  kObserved = 0,
  // kPartiallyObserved removed — merged into kObserved
  kMissingByDropout = 2,  // 保持数值不变，避免 TerrainObservability.msg 编码断裂
};
```

`frame_types.hpp`：
```cpp
struct SectorObservability {
  ObservabilityState state = ObservabilityState::kObserved;  // 默认值改为 kObserved
  float coverage_confidence = 0.0f;
};
```

`frame_observability_estimator.cpp`：原来赋值 `kPartiallyObserved` 的所有路径改为赋值 `kObserved`。

`dropout_aware_map_updater.cpp`：删除 `kPartiallyObserved` 分支，只保留 `kObserved` 和 `kMissingByDropout` 两级衰减。

#### 1.2 删除 `AmbiguousCandidate`

`candidate_types.hpp`：删除 `AmbiguousCandidate` struct，删除 `FrontendOutput::ambiguous_candidates`。

`polar_frontend.cpp`：删除 L149-152 的 ambiguous 生成逻辑。

`test_processor.cpp`：删除 `PolarFrontendEmitsAmbiguousCandidateForPartiallyObservedNonTriggerCell`。

#### 1.3 新增 `obstacle_types.hpp`

```cpp
enum class BlockReason : uint8_t {
  kNone = 0,
  kProtrusion,
  kLowClearance,
  kGeometryFailure,
  kMixed,
};

enum class ObstaclePointPublishStatus : uint8_t {
  kNotApplicable = 0,
  kPublishedByProtrusion,
  kPublishedByOverhead,
  kGatedByEvidence,
  kGatedByHeight,
  kBlockedButNoSamples,
};
```

#### 1.4 新增 TerrainLayers 字段（预留，初始化为 NaN/0）

`terrain_layers.hpp`：
```cpp
std::vector<float> protrusion_height;     // 新增
std::vector<float> protrusion_evidence;   // 新增
std::vector<float> overhead_evidence;     // 新增（与现有 obstacle_evidence 独立）
```

> [!IMPORTANT]
> Phase 1 只新增字段并初始化为 NaN/0。不改变任何运行时行为。现有 `obstacle_evidence` 暂时保留不动，直到 Phase 2 由 protrusion/overhead 分别维护。

#### 1.5 冻结验收 ROI

创建 `config/acceptance_rois.yaml`：

```yaml
false_obstacle:
  bags_source: config/offline_benchmark_bags.yaml
  excluded_bags:
    - rosbag2_mtbf_x30_upstair_passage
  detection_box:
    x_min: 0.0
    x_max: 1.4
    y_min: -0.25
    y_max: 0.25

miss_obstacle:
  setups:
    - name: open_up_down_stairs
      bag: /home/deep/deeprobotics/bags/test_result/rosbag2_open_up_down_stairs
      config_overrides:
        map_height_max: 0.5
      window:
        start_offset_sec: 6.7
        time_window_sec: 0.4
      rois:
        - name: left_board
          x_min: 0.8
          x_max: 2.7
          y_min: 0.4
          y_max: 0.6
        - name: right_board
          x_min: 0.8
          x_max: 2.7
          y_min: -0.6
          y_max: -0.4

    - name: open_stair_and_slope
      bag: /home/deep/deeprobotics/bags/offline_bags/rosbag2_open_stair_and_slope
      config_overrides:
        map_height_max: 0.5
      window:
        start_offset_sec: 5.7
        time_window_sec: 0.3
      rois:
        - name: left_side_board
          x_min: 1.5
          x_max: 2.5
          y_min: 0.4
          y_max: 0.6
        - name: right_side_board
          x_min: 1.5
          x_max: 2.5
          y_min: -0.6
          y_max: -0.4

low_clearance:
  # TBD — pending bag asset
```

#### 1.6 文档对齐

在 `docs/refactor_handover_status.md` 末尾追加 Phase 1 完成状态。

### 验收标准

- [ ] `colcon build` 通过
- [ ] `colcon test` 通过（删除 1 个旧测试，无新增测试失败）
- [ ] `ObservabilityState` enum 只剩 `kObserved` 和 `kMissingByDropout`
- [ ] `AmbiguousCandidate` 从代码中完全消失
- [ ] `BlockReason` / `ObstaclePointPublishStatus` enum 存在于新头文件
- [ ] `TerrainLayers` 新增 3 个 layer vector，初始化为 NaN/0
- [ ] `acceptance_rois.yaml` 存在且格式正确
- [ ] 运行时行为与 Phase 1 前完全一致（通过现有 80 个测试 - 1 个被删 = 79 个测试）

---

## Phase 2：Frontend V2 双层摘要 + MapUpdater 三证据链

**目标**：实现核心算法变更，使前端输出 `SupportCandidate / ProtrusionCandidate / OverheadCandidate`，map updater 分别维护三条证据链。

### 涉及文件

| 动作 | 文件 |
|------|------|
| MODIFY | `include/passable_area/core/polar_frontend.hpp` — 收窄接口签名 |
| MODIFY | `src/core/polar_frontend.cpp` — 双层摘要实现 |
| MODIFY | `include/passable_area/core/candidate_types.hpp` — 新 candidate 类型 |
| MODIFY | `src/core/mapping/dropout_aware_map_updater.cpp` — 三证据链 |
| MODIFY | `src/core/processor.cpp` — 适配新 candidate 类型 |
| MODIFY | `test/core/test_processor.cpp` — 替换前端行为测试 |

### 具体变更

#### 2.1 收窄 Frontend 接口

```cpp
// polar_frontend.hpp
struct MapGeometry {
  int rows, cols, size;
  float resolution;
  Eigen::Vector2f origin;
  // mapToIndex / indexToMap as methods
};

class PolarFrontend {
public:
  FrontendOutput run(const ProcessedFrame &frame,
                     const FrameObservability &observability,
                     const MapGeometry &geo) const;
};
```

这从类型系统层面阻止前端访问 `map.layers()` 历史数据。

#### 2.2 双层摘要

前端 per-cell 逻辑：
1. 收集 cell 内所有 z 值
2. 按 `profile_split_gap` 做 band split
3. 确定 `support_band`（下层）和 `upper_band`（上层）
4. band 内用 trimmed bounds（10%/90% 分位）
5. 根据 band 结构输出：
   - `SupportCandidate`（来自 support_band.bottom）
   - `ProtrusionCandidate`（上下层间距构成近地凸起）
   - `OverheadCandidate`（上层构成低净空威胁）
   - 单 tall band 无 split 时，直接作为 `ProtrusionCandidate`

#### 2.3 MapUpdater 三证据链

```
support_candidates → support_height, support_confidence
protrusion_candidates → protrusion_height, protrusion_evidence
overhead_candidates → overhead_height, overhead_evidence
```

现有 `obstacle_evidence` 在过渡期保留为 `max(protrusion_evidence, overhead_evidence)` 的兼容别名，直到 Phase 3 reasoner 接管后移除。

#### 2.4 测试覆盖

新增合成 case 覆盖：
- 普通地面 → support_band only
- 墙面 → single tall band → protrusion
- 低天花板 → support + upper → overhead
- 楼梯混样 → 两层 band split，不应触发 protrusion
- 墙脚噪点 → trimmed bounds 排除少量 lower leak
- Dropout → support candidate 抑制

### 验收标准

- [ ] `colcon build` + `colcon test` 通过
- [ ] 前端不再接收 `LocalTerrainMap` 引用
- [ ] `ProtrusionCandidate` / `OverheadCandidate` 在合成测试中正确生成
- [ ] 三证据链在 map updater 中独立衰减
- [ ] 现有 wall / low ceiling / dropout 相关测试通过或被合理替换

---

## Phase 3a：Shadow ObstacleReasoner + Analyzer 更新

**目标**：引入 ObstacleReasoner 作为 shadow path，与当前 solver 并行对比；同步更新 analyzer root cause 集合。

### 涉及文件

| 动作 | 文件 |
|------|------|
| NEW | `include/passable_area/core/obstacle_reasoner.hpp` |
| NEW | `src/core/obstacle_reasoner.cpp` |
| MODIFY | `src/core/processor.cpp` — 调用 reasoner 但不替换 solver |
| MODIFY | `src/tools/miss_obstacle_analyzer.cpp` — 更新 root cause |
| MODIFY | `src/tools/false_obstacle_analyzer.cpp` — 区分 protrusion/overhead |
| NEW | `test/core/test_obstacle_reasoner.cpp` |

### 具体变更

- ObstacleReasoner 输入：`TerrainLayers`（support / protrusion / overhead 证据 + terrain geometry）
- ObstacleReasoner 输出：`block_reason` / `protrusion_stage` / `overhead_stage` / `obstacle_point_publish_status`
- Processor 同时调用 solver 和 reasoner，在 `FrameOutput` 中暴露两套结果供对比
- Miss analyzer root cause 更新为：`kNoFrontendCandidate / kEvidenceTooLow / kPublishHeightGated / kReasonerNotBlocked / kNoSamplesInRoi / kUnknownOrMixed`

### 验收标准

- [ ] Shadow reasoner 逐帧输出 `block_reason`
- [ ] 在 false obstacle bags 上，reasoner 与 solver 的 `Impassable` 判定一致率 > 95%
- [ ] Miss analyzer 新 root cause 能正确分类 Setup A + B 中的漏检帧

---

## Phase 3b：Low Clearance → obstacle_points 桥

**目标**：让低净空阻挡在 `/terrain_obstacle_points` 中可见。

### 涉及文件

| 动作 | 文件 |
|------|------|
| MODIFY | `src/core/processor.cpp` — overhead cell 的 sample 筛选发布 |
| MODIFY | `test/core/test_processor.cpp` — 新增 low clearance obstacle point 测试 |

### 具体变更

当 reasoner 给出 `block_reason == kLowClearance` 且 `overhead_evidence >= threshold` 时，从该 cell 的当前帧 sample 中选取高于 `support_ref` 的 sample 作为 obstacle point 发布。

### 验收标准

- [ ] 新增测试：低净空 cell 产生 obstacle point
- [ ] 现有 `LowCeilingCreatesImpassableCells` 继续通过
- [ ] `ObstaclePointsExcludeGroundSamplesUnderLowCeiling` 语义保持（地面点不进 obstacle_points）

---

## Phase 4：Bag 结构验收 + 切 solver 主判定权

**目标**：在冻结的 ROI 上跑全量验收，确认 recall 改善且无大面积 false obstacle 回潮后，切换 solver 消费 reasoner 输出。

### 操作

1. `offline_replay --analyze-false-obstacles` on all frozen false obstacle bags
2. `offline_replay --analyze-missed-obstacles` on Setup A + Setup B
3. Performance benchmark: `offline_replay --benchmark-timing`
4. 确认后：solver 从直接看 `obstacle_evidence + continuity` 改为消费 `block_reason`

### 验收标准

- [ ] False obstacle bags: candidate frame 数量不超过 baseline 的 120%
- [ ] Miss obstacle Setup A + B: ROI 内 obstacle_points > 0 的帧占比明显改善
- [ ] Timing: per-frame p99 不超过 baseline 的 150%
- [ ] Dropout / sparse coverage case 无回退

---

## Phase 5：旧兼容字段 / 测试 / 文档清理

**目标**：移除所有旧前端兼容字段，对齐 analyzer、文档、grid_map 发布。

### 涉及文件

| 动作 | 文件 |
|------|------|
| MODIFY | `include/passable_area/core/types/frame_types.hpp` — 删除旧字段 |
| MODIFY | `include/passable_area/core/candidate_types.hpp` — 删除旧字段 |
| MODIFY | `src/interfaces/ros/converters/output_converter.cpp` — 删除旧 grid_map 层 |
| MODIFY | `test/tools/test_false_obstacle_analyzer.cpp` |
| MODIFY | `test/tools/test_miss_obstacle_analyzer.cpp` |
| MODIFY | `docs/algorithm_scheme.md` |
| MODIFY | `docs/refactor_handover_status.md` |
| MODIFY | `AGENTS.md`（工作区根） |

### 待删除的旧字段

- `support_anchor_used`, `support_anchor_origin`, `support_anchor_authority`
- `anchor_leak_suppression_enabled`, `sub_support_leak_count`
- `anchor_below_observation_count`, `stale_anchor_residual_filtered_count`
- `raw_upper_support_cell`, `explanation_adjusted_upper_support_cell`, `upper_support_cell`
- `obstacle_local_triggered`, `obstacle_upper_patch_confirmed`, `obstacle_explanation_rejected`
- `obstacle_rejected_by_neighbor_support`
- `neighbor_upper_support_count`, `aligned_neighbor_support_count`
- `explanation_decision`
- `facade_lower_upper_coexisting`, `facade_upper_edge_aligned_with_supported_neighbors`

### 验收标准

- [ ] `FrameOutput` / `FrontendOutput` 不再包含上述旧字段
- [ ] Analyzer 与新 contract 完整对齐
- [ ] 文档无旧 explanation 语义残留
- [ ] `colcon build` + `colcon test` 全通过

---

## Verification Plan

### Automated Tests

每个 Phase 完成后：
```bash
colcon build --packages-select passable_area --symlink-install
colcon test --packages-select passable_area --event-handlers console_direct+
colcon test-result --verbose
```

### Offline Validation

Phase 4：
```bash
# False obstacle
ros2 run passable_area offline_replay --analyze-false-obstacles \
  --bags-config config/offline_benchmark_bags.yaml \
  --exclude rosbag2_mtbf_x30_upstair_passage

# Miss obstacle Setup A
ros2 run passable_area offline_replay --analyze-missed-obstacles \
  --bag /home/deep/deeprobotics/bags/test_result/rosbag2_open_up_down_stairs \
  --map-height-max 0.5 --start-offset 6.7 --time-window 0.4 \
  --roi 0.8,2.7,0.4,0.6

# Timing
ros2 run passable_area offline_replay --benchmark-timing
```

### Manual Verification

- 用户在实机或 bag 回放时通过 RViz 确认：
  - `/terrain_obstacle_points` 在 wall / box / pillar 前有点
  - Low ceiling 场景有 obstacle point 输出
  - 楼梯场景无大面积 false obstacle
