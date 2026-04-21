# Phase 4.1 — 原理性修复方案

## 根因分析

### False Obstacle（`mtbf_upslope_and_downstair` 1 帧误报）

诊断数据：
- Cell 13513, pos=(0.33, -0.19), support=-1.10, overhead=-0.74, clearance=0.36
- `block_reason = kLowClearance`（0.36 < min_clearance 0.5）
- `protrusion_evidence = 0.51` ≥ threshold 0.4
- `source_publish_path = ProtrusionEvidence`
- `low_clearance_bridge_hit = false`（overhead_evidence 0.13 < 0.4）

根因：**Processor 与 Reasoner 的判定权不一致。** Reasoner 对 `kLowClearance` 类型的 cell 只通过 overhead bridge 路径发布，正确阻止了该 cell 的发布（overhead_evidence 不足）。但 `HasObstaclePointPublishEvidence()` 完全不检查 `block_reason`，独立地判定 `protrusion_evidence >= 0.4` 即发布。这是一个架构缺口：protrusion 证据从台阶落差中累积，但本 cell 的几何语义是"低净空台阶投影"而非"竖向障碍物"。

### Miss Obstacle（Setup B 侧板 3/3 全漏）

ROI 诊断数据（以 right_side_board 一个典型 cell 为例）：
- base_x=1.553, base_y=-0.467, clearance=inf, block_reason=None
- sample: min_z=0.714, max_z=1.203, support_h=0.771
- `height_above_support ≈ 0.43m`
- `protrusion_evidence = 0.243`（未达 0.4 阈值）
- `local_trigger=1, upper_patch_confirmed=1`（前端已检出）

evidence 增长计算：
- 当前公式：`candidate.evidence = clamp(height, 0, 1) = 0.43`
- `overhead_triggered = true`（0.43m < min_clearance 0.5m）→ `gain_scale = 1.0`
- 每帧增益：`0.25 × 1.0 × 0.43 = 0.1075`
- 3 帧窗口：`0.32`（不及阈值 0.4）

根因：**证据公式将高度线性映射为置信度，语义不正确。** 0.43m 高度已超过 max_step_up (0.32m)，是明确的不可通行障碍物，物理置信度应为 1.0 而非 0.43。

## 修复方案（两处修改，相互独立）

### 修改 1：证据公式改革（解决 miss）

**文件**：[polar_frontend.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/src/core/polar_frontend.cpp)

将 protrusion evidence 公式从：
```cpp
std::clamp(height_above_support, 0.0f, 1.0f)
```
改为：
```cpp
std::clamp(height_above_support / std::max(config_.geometry.max_step_up, 1e-3f), 0.0f, 1.0f)
```

物理含义：以 `max_step_up` 为归一化基准。高度达到 max_step_up 即为"确定障碍物"（evidence = 1.0），低于此值按比例线性缩放。

效果测算：
- 0.43m 侧板：`0.43/0.32 = 1.34 → clamp → 1.0`，每帧增益 0.25，2 帧达 0.50 ✓
- 0.20m 台阶踢面：`0.20/0.32 = 0.625`，每帧 0.156（但由修改 2 保护）

### 修改 2：Processor 对齐 Reasoner 判定权（解决 false）

**文件**：[processor.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/src/core/processor.cpp)

在 `HasObstaclePointPublishEvidence()` 中增加检查：当 `block_reason == kLowClearance` 时，抑制 protrusion 路径发布。

```cpp
const bool is_low_clearance_only =
    index < output.block_reason.size() &&
    output.block_reason[index] == static_cast<uint8_t>(BlockReason::kLowClearance);
const bool protrusion_publish =
    !is_low_clearance_only &&
    index < layers.protrusion_evidence.size() &&
    layers.protrusion_evidence[index] >= config.obstacle_points_min_evidence;
```

物理含义：`kLowClearance` 意味着"有限净空 + 连续支撑"→ 台阶踏面模式。该几何语义下的 protrusion 证据来自台阶落差，不应通过独立于净空分析的 protrusion 路径发布。这些 cell 仍可通过 bridge（有 clearance > max_step_up 门控）正确发布。

为什么 kMixed 不受影响：kMixed = 低净空 + 不连续支撑（continuity < 0.3），表示真实障碍物界面，protrusion 发布正确。

## 场景验证矩阵

| 场景 | clearance | block_reason | 修改 1 效果 | 修改 2 效果 | 预期结果 |
|------|-----------|-------------|-----------|-----------|---------|
| 台阶踢面 0.18m | 0.18 | kLowClearance | evidence↑ 但无影响 | protrusion 被抑制 | bridge 也被 kGatedByHeight 阻止 → 不发布 ✓ |
| 台阶踢面 0.36m | 0.36 | kLowClearance | evidence↑ 但无影响 | protrusion 被抑制 | bridge 检查 overhead_evidence → 仅强 evidence 发布 ✓ |
| 侧板 0.43m | ∞ | None | evidence 1.0 → 2 帧达阈值 | 不触发（非 kLowClearance）| 正常发布 ✓ |
| 真实墙面 | ∞ | kProtrusion | evidence↑ | 不触发 | 正常发布 ✓ |
| 低管道 0.40m | 0.40 | kLowClearance | 无影响 | protrusion 被抑制 | bridge: 0.40>0.32 → 检查 overhead_evidence → 正确发布 ✓ |
| 混合界面 | <0.5 | kMixed | evidence↑ | 不触发（kMixed≠kLowClearance) | protrusion 正常发布 ✓ |

## 需同步的文件

| 文件 | 改动 |
|------|------|
| [polar_frontend.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/src/core/polar_frontend.cpp) | evidence 公式 |
| [processor.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/src/core/processor.cpp) | block_reason 检查 |
| [offline_replay.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/tools/offline_replay.cpp) | 诊断 InferPublishPath 同步 |
| [false_obstacle_analyzer.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/src/tools/false_obstacle_analyzer.cpp) | 诊断 InferPublishPath 同步 |
| [test_processor.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/test/core/test_processor.cpp) | 补充 kLowClearance suppression 测试 |

## 验证计划

1. `colcon build && colcon test` — 全量通过
2. `--analyze-false-obstacles` — 全部 frozen bags 归零（含 mtbf_upslope_and_downstair）
3. `--inspect-roi` Setup B right_side_board — protrusion_evidence ≥ 0.4
