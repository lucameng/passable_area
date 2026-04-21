# Phase 4 修复评审：Patch vs 原则性方案

---

## 评审结论：两个修复都应替换

### 修复 A：`IsSparseStepLikeLowClearanceProjection` — ❌ 多条件 patch

**问题 1：魔法数字堆砌**

```cpp
step_like_height > config.geometry.max_step_up + 0.05f  // 为什么是 0.05？
output.raw_sample_count[index] > 4U                      // 为什么是 4？
layers.support_continuity[index] >= 0.8f                  // 为什么是 0.8？
```

这三个阈值没有从物理语义或系统参数推导来，而是针对两个失败 bag 调出来的。换一个楼梯（台阶更高、点更密、连续性更低），这些阈值就可能失效。

**问题 2：作用点错误**

该 check 被放在了 `HasObstaclePointPublishEvidence()` 内部，这意味着它不仅抑制了 low-clearance bridge，**还抑制了 legacy `obstacle_evidence` 路径**。如果一个合法障碍恰好满足"稀疏 + 步高量级 + 高连续性"条件，它的 obstacle points 会被错误抑制 → 潜在 miss obstacle。

**问题 3：本质上是 explanation 规则树的重新长出**

`IsSparseStepLikeLowClearanceProjection` 的结构是 `if (条件1 && 条件2 && 条件3 && 条件4) → suppress`。这恰好是 V2 重构要消除的旧 explanation 规则树模式。

### 修复 B：`protrusion_gain_scale = 3.0f` — ❌ 用 gain 对抗 decay

**问题 1：没有解决根因**

根因是 `obstacle_clear_observed_decay = 0.20`  在 wall-like cell "漏扫帧 + 地面 sample overlay" 时把 evidence 清零。3.0x gain 只是让累积速度更快以尝试跑赢衰减，但如果连续 2 帧漏扫，evidence 仍然归零。

**问题 2：全局副作用**

3.0x 适用于**所有** protrusion-only candidate，不仅是 wall-like 侧板。一个本应需要 3-4 帧确认的动态近地障碍（如临时放置的箱子被移走后残影），现在 1-2 帧就能达到发布阈值 → false obstacle 风险增加。

**问题 3：Gain 和 decay 是对称设计**

`obstacle_evidence_gain` 和 `obstacle_clear_observed_decay` 的比值决定了"确认一个障碍需要多少帧 / 清除一个障碍需要多少帧"。单方面把 gain 提 3 倍而不改 decay，破坏了这个平衡。

---

## 原则性替代方案

### 替代 A：protrusion_evidence 共存门控（替换 IsSparseStepLikeLowClearanceProjection）

**第一性原理**：如果一个 cell 已经有 protrusion evidence，说明该 cell 存在垂直结构（台阶/墙面/障碍物），low-clearance bridge 不应该在此基础上再叠加 overhead obstacle 发布。让 protrusion 路径自己决定是否发布。

```cpp
bool HasLowClearanceObstaclePointBridge(const TerrainLayers &layers,
                                        const FrameOutput &output, int cell,
                                        const Config &config) {
  const auto index = static_cast<size_t>(cell);
  if (index >= output.block_reason.size() ||
      index >= layers.overhead_evidence.size()) {
    return false;
  }

  // Only pure kLowClearance. kMixed already has protrusion_blocking, let
  // the protrusion evidence path handle obstacle publishing.
  if (output.block_reason[index] !=
      static_cast<uint8_t>(BlockReason::kLowClearance)) {
    return false;
  }

  // If the cell carries non-trivial protrusion evidence, the vertical
  // structure is protrusion-like (step, wall), not a pure overhead ceiling.
  const float protrusion_ev =
      index < layers.protrusion_evidence.size()
          ? layers.protrusion_evidence[index] : 0.0f;
  if (protrusion_ev > 0.05f) {
    return false;
  }

  return layers.overhead_evidence[index] >= config.obstacle_points_min_evidence;
}
```

**为什么比 patch 更好**：
- 单一判据（protrusion_evidence）而非 4 个魔法数字
- 从证据链语义出发：protrusion 和 overhead 是互斥的发布意图
- 不干扰 legacy `obstacle_evidence` 路径（从 `HasObstaclePointPublishEvidence` 中移除 suppress check）
- 楼梯、分层地面、台阶等所有"上方结构同时构成 protrusion"的场景都自动覆盖

### 替代 B：衰减侧保护已建立的 protrusion evidence（替换 3.0x gain）

**第一性原理**：`obstacle_clear_observed_decay` 的语义是"看到了干净地面 → 障碍可以消退"。但对于已建立 protrusion evidence 的 cell，单帧无 protrusion candidate 不代表障碍消失——可能只是该帧漏扫了墙面。

修改 `DropoutAwareMapUpdater::update()` 的衰减计算：

```cpp
// 现有逻辑
if (support_reobserved && sector.state == ObservabilityState::kObserved) {
  obstacle_decay = config_.persistence.obstacle_clear_observed_decay;  // 0.20
}

// 替换为
if (support_reobserved && sector.state == ObservabilityState::kObserved) {
  const float existing_protrusion =
      layers.protrusion_evidence[cell];
  if (existing_protrusion > config_.obstacle_points_min_evidence * 0.5f) {
    // Established protrusion evidence: use gentle decay instead of
    // aggressive clear. The protrusion will clear naturally through
    // the regular evidence_decay path if the obstacle truly disappears.
    obstacle_decay = config_.persistence.obstacle_evidence_decay * 0.4f;
  } else {
    obstacle_decay = config_.persistence.obstacle_clear_observed_decay;
  }
}
```

**为什么比 patch 更好**：
- 直接解决根因（clear decay 太激进，而非 gain 太低）
- 不改变 gain/decay 平衡比
- 只影响已有 protrusion 信念的 cell，不影响全局
- protrusion_gain_scale 恢复为 1.0（或保留 Phase 2 的 `overhead_triggered ? 1.0 : 1.5`）
- 动态障碍（protrusion_evidence 从未达到 `min_evidence * 0.5`）仍然走原来的快速清理路径

### 恢复项

- 删除 `IsSparseStepLikeLowClearanceProjection()` 整个函数
- 从 `HasObstaclePointPublishEvidence()` 中移除对它的调用
- `protrusion_gain_scale` 恢复为 `overhead_triggered ? 1.0f : 1.5f`（Phase 2 值）

---

## 验证计划

与之前相同的 frozen bags / ROI 验证，期望：

| 指标 | 当前 patch 结果 | 原则性方案预期 |
|------|:---:|:---:|
| rosbag2_open_short_upstairs | 0/120 | 0/120 |
| rosbag2_b1_upstairs | 3/156 | ≤ 3/156 |
| Setup A left_board | 1/4 miss | ≤ 1/4 miss |
| Setup A right_board | 2/4 miss | ≤ 2/4 miss |
| Setup B left_side_board | 2/3 miss | ≤ 2/3 miss |
| Setup B right_side_board | 0/3 miss | 0/3 miss |

如果原则性方案的结果不劣于 patch 结果，就替换；如果有劣化，分析具体帧再决定。
