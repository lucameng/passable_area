# Phase 4 修复：第三版根因与方案

## 前两版方案为什么失败

### False Obstacle — protrusion_evidence 共存门控无效

楼梯台阶在 V2 frontend 中的实际路径：

```
cell samples: 当前台阶 (z≈0.0) + 下一台阶 (z≈0.17)
                    ↓
SplitHeightBands: gap=0.17 > split_gap=0.10 → 产生 2 个 band
                    ↓
overhead_band check: band[1].bottom - support_z = 0.17 < min_clearance(0.5)
  → overhead_triggered = TRUE
                    ↓
protrusion check: height_above_support = 0.17 < suspicious(0.24)
  → protrusion_triggered = FALSE
                    ↓
只发 OverheadCandidate → protrusion_evidence 始终为 0
  → 共存门控永远不命中
```

**核心错误**：我假设楼梯 cell 会同时有 protrusion evidence。但台阶高差（0.15~0.25m）低于 `suspicious_vertical_span`（0.24m），frontend 不会为它们产生 ProtrusionCandidate。楼梯 cell 是**纯 overhead**。

### Miss Obstacle — gentle decay 方向不对

Frozen results 证明：切换到 gentle decay 后 miss 率不变甚至更差。这说明 evidence 在帧间的衰减并不是瓶颈 — **单帧累积量本身就不够**。

---

## 第三版方案

### False Obstacle 修复：clearance 范围门控

**第一性原理**：`max_step_up` 定义了机器人能够跨越的最大地形高差。如果 `clearance <= max_step_up`，这个间隙在机器人的台阶处理能力范围内 — 它是台阶地形，不是天花板。

```cpp
bool HasLowClearanceObstaclePointBridge(const TerrainLayers &layers,
                                        const FrameOutput &output, int cell,
                                        const Config &config) {
  const auto index = static_cast<size_t>(cell);
  if (index >= output.block_reason.size() ||
      index >= layers.overhead_evidence.size()) {
    return false;
  }
  if (output.block_reason[index] !=
      static_cast<uint8_t>(BlockReason::kLowClearance)) {
    return false;
  }

  // Clearance-range gate: if the gap is within one step height, it's
  // terrain geometry (staircase), not a ceiling. The solver already handles
  // step traversability via step_up/step_down checks.
  const float clearance = index < layers.clearance.size()
                              ? layers.clearance[index]
                              : std::numeric_limits<float>::quiet_NaN();
  if (!std::isfinite(clearance) || clearance <= config.geometry.max_step_up) {
    return false;
  }

  return layers.overhead_evidence[index] >= config.obstacle_points_min_evidence;
}
```

**为什么是 `max_step_up` 而不是魔法数字**：

| 属性 | 说明 |
|------|------|
| 来源 | 已有配置参数，已针对具体机器人标定 |
| 物理含义 | 机器人能跨越的最大高差（0.32m），天然区分台阶与天花板 |
| 覆盖范围 | 所有台阶高度（标准楼梯 0.15~0.20m）都 < max_step_up |
| 边界安全 | bridge 只在 `(max_step_up, min_clearance)` 范围内开启，即 (0.32, 0.50) |

验证楼梯场景：
- 台阶 clearance ≈ 0.17m ≤ max_step_up(0.32) → **bridge 抑制** ✅
- 真实低天花板 clearance ≈ 0.40m > max_step_up(0.32) → **bridge 开放** ✅
- 极低天花板 clearance ≈ 0.25m ≤ max_step_up(0.32) → bridge 抑制，但 solver 已标记 kImpassable(cost=100)，下游规划不会穿越 → **安全** ✅

### Miss Obstacle 修复：结构信噪比驱动的 gain scale

**第一性原理**：单 band 高大垂直结构（wall-like obstacle）在结构上是无歧义的 — 所有 sample 一致指向"这是一面墙"。多 band 场景（地面 + 上方分离结构）存在歧义（可能是台阶、平台、分层地面）。gain_scale 应反映这种**观测确信度**差异。

当前 gain_scale 分配：

| 场景 | overhead_triggered | protrusion_triggered | 当前 gain_scale | evidence/帧 |
|------|:---:|:---:|:---:|:---:|
| 多 band + overhead | ✅ | ✅ | 1.0 | 0.095 |
| 多 band, 无 overhead | ❌ | ✅ | 1.5 | 0.143 |
| 单 band, 高大 (wall-like) | ❌ | ✅ | 1.5 | 0.143 |

问题：单 band wall-like 和多 band protrusion-only 共用 1.5，但它们的结构确信度不同。

改进后的 gain_scale 分配：

```cpp
if (protrusion_triggered) {
  float gain_scale = 1.0f;
  if (!overhead_triggered) {
    gain_scale = 1.5f;
    // Single tall band with sufficient samples and height clearly
    // exceeding step range: structurally unambiguous vertical
    // obstacle. Higher confidence warrants faster accumulation.
    if (!has_upper_band && stats.count >= 3 &&
        height_above_support > config_.geometry.max_step_up) {
      gain_scale *= 1.5f;  // → 2.25
    }
  }
  output.protrusion_candidates.push_back(ProtrusionCandidate{
      cell, obstacle_z, std::clamp(height_above_support, 0.0f, 1.0f),
      gain_scale});
}
```

| 场景 | gain_scale | evidence/帧 | 到 0.4 阈值需要帧数 |
|------|:---:|:---:|:---:|
| 多 band + overhead | 1.0 | 0.095 | ~5 帧 |
| 多 band, 无 overhead | 1.5 | 0.143 | ~3 帧 |
| **单 band wall-like** | **2.25** | **0.214** | **~2 帧** |

> [!IMPORTANT]
> 2.25x 并不是拍脑袋数字：它是 1.5（non-overhead base）× 1.5（single-band confidence boost）的组合。每个 1.5x 都有独立的物理依据。

**为什么不用 3.0x**：
- 3.0x 对所有 non-overhead protrusion 统一加速，包括多 band 场景 → false obstacle 风险
- 2.25x 只对满足三个条件的 candidate 加速：single band + 多 sample + 超过 step 高度
- 条件本身就是结构确信度的编码，不是 case-specific tuning

### Map Updater 衰减：回退到 Phase 2 版本

既然 gentle decay 没有改善 Setup A/B（frozen 结果已证明），衰减侧的改动应该**全部回退**。保持 Phase 2 的干净衰减逻辑，避免引入无效复杂度。

---

## 完整变更清单

| 文件 | 改动 |
|------|------|
| `processor.cpp` | 删除 protrusion_evidence 共存门控，换为 clearance ≤ max_step_up 门控 |
| `polar_frontend.cpp` | 恢复 Phase 2 的 gain_scale 基线 (1.0/1.5)，增加单 band wall-like 1.5x boost |
| `dropout_aware_map_updater.cpp` | 回退 gentle-decay 改动，恢复到 Phase 2 版本 |
| `test_processor.cpp` | 更新/新增对应测试 |

## 验证计划

```bash
# 单元测试
colcon test --packages-select passable_area --event-handlers console_direct+

# False obstacle frozen bags
ros2 run passable_area offline_replay --analyze-false-obstacles \
  --bag .../rosbag2_b1_upstairs
ros2 run passable_area offline_replay --analyze-false-obstacles \
  --bag .../rosbag2_open_short_upstairs

# Miss obstacle frozen ROIs
# Setup A
ros2 run passable_area offline_replay --analyze-missed-obstacles \
  --bag .../rosbag2_open_up_down_stairs \
  --params-file config/passable_area_map_height_max_0p5.yaml \
  --start-offset 6.7 --time-window 0.4 \
  --roi 0.8,2.7,0.4,0.6
# (repeat for right board, Setup B left/right)
```

预期：
- False obstacle: b1_upstairs 和 open_short_upstairs 均归零或接近零（clearance gate 直接命中台阶几何）
- Miss obstacle: Setup A/B 改善到 ≤ patch 版本水平（2.25x gain 使 2 帧即可达标）
