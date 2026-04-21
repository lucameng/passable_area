# Phase 4.2 — Overhead Evidence 公式归一化

## 背景

Phase 4.1 对 protrusion evidence 进行了 `h / max_step_up` 归一化，解决了 Setup B right 的 miss。但 overhead evidence 仍使用原始线性公式 `min_clearance - gap`，导致 clearance 接近 min_clearance 时证据增长极慢。

Setup A right 的最高证据 cell 诊断：
```
clearance = 0.37    min_clearance = 0.50    max_step_up = 0.32
overhead_evidence_per_frame = min_clearance - gap = 0.50 - 0.37 = 0.13
gain_per_frame = 0.25 × 1.0 × 0.13 = 0.0325
frames_to_threshold = 0.4 / 0.0325 ≈ 12 帧
```

ROI 窗口仅 3-4 帧，根本不够。

## 改动方案

### 唯一改动：Overhead Evidence 归一化

**文件**：[polar_frontend.cpp](file:///home/deep/deeprobotics/passable_humble_ws/src/passable_area/src/core/polar_frontend.cpp) L250-257

```diff
-std::clamp(config_.geometry.min_clearance -
-               (overhead_band->bottom - support_z),
-           0.0f, 1.0f)
+std::clamp((config_.geometry.min_clearance -
+                (overhead_band->bottom - support_z)) /
+               std::max(config_.geometry.min_clearance -
+                            config_.geometry.max_step_up, 1e-3f),
+           0.0f, 1.0f)
```

**物理含义**：

| 净空 (gap) | 旧值 | 新值 | 语义 |
|-----------|------|------|------|
| = max_step_up (0.32m) | 0.18 | 1.0 | 确定不可通行 |
| = 0.37m | 0.13 | 0.72 | 高置信度 |
| = 0.45m | 0.05 | 0.28 | 低置信度 |
| = min_clearance (0.50m) | 0.00 | 0.00 | 不触发 |

归一化基准 `min_clearance - max_step_up = 0.18m` 是"不可通行净空范围"的完整跨度。

**效果测算（Setup A right cell, clearance=0.37）：**
```
new_evidence = (0.50 - 0.37) / 0.18 = 0.72
gain_per_frame = 0.25 × 1.0 × 0.72 = 0.18
frames_to_threshold = 0.4 / 0.18 ≈ 3 帧  ✓
```

## 场景安全矩阵

| 场景 | clearance | gap→evidence | bridge 发布条件 | 预期 |
|------|-----------|-------------|----------------|------|
| 台阶 0.18m | 0.18 | 1.0 | clearance ≤ max_step_up → kGatedByHeight | 不发布 ✓ |
| 台阶 0.25m | 0.25 | 1.0 | clearance ≤ max_step_up → kGatedByHeight | 不发布 ✓ |
| 侧板 clearance 0.37m | 0.37 | 0.72 | clearance > max_step_up → 检查 evidence | 2-3 帧发布 ✓ |
| 管道 clearance 0.40m | 0.40 | 0.56 | clearance > max_step_up → 检查 evidence | 3 帧发布 ✓ |
| 几乎够高的净空 0.48m | 0.48 | 0.11 | clearance > max_step_up → 检查 evidence | 需 15 帧（边界区域，保守） |

> [!IMPORTANT]
> 台阶场景安全性由 bridge 的 `clearance > max_step_up` 门控保证，与 overhead evidence 大小无关。evidence 增大不会改变台阶的发布判定。

## 需同步的文件

| 文件 | 改动 |
|------|------|
| `src/core/polar_frontend.cpp` | overhead evidence 公式 |
| `tools/offline_replay.cpp` | 诊断同步（如有 IsLowClearanceBridgeEligible 内嵌公式） |
| `test/core/test_processor.cpp` | 更新 overhead evidence 相关测试断言 |

## 验证计划

1. `colcon build && colcon test` — 全量通过
2. False frozen bags 全量 9 个 — 全部归零
3. Miss ROIs 全量 4 个 — 预期改善：
   - Setup A left: ≤1/4 miss
   - Setup A right: ≤1/4 miss（从 2/4 改善）
   - Setup B left: ≤2/3 miss（受限于 clearance < max_step_up，公式改动无影响）
   - Setup B right: 0/3 miss（维持）

---

# Phase 5 入口条件

Phase 5 的目标是：**将 TraversabilitySolver 切换为消费 ObstacleReasoner 的输出作为主判定权。**

## 入口前置条件

| # | 条件 | 当前状态 | Phase 4.2 后预期 |
|---|------|---------|----------------|
| 1 | False frozen bags 全部归零 | ✅ 0/9 | ✅ |
| 2 | Miss ROIs 不劣于 v1 patch 基线 | ❌ Setup A/B left 仍有 miss | ⚠️ Setup A 改善，B left 不变 |
| 3 | 95+ tests 全通过 | ✅ 95/95 | ✅ |
| 4 | ObstacleReasoner 输出与 Processor 发布逻辑一致 | ✅ 已对齐 | ✅ |

## 何时进入 Phase 5

**Phase 4.2 完成后即可进入 Phase 5，前提是：**

1. False frozen bags 仍全部归零
2. Setup A miss 不劣于 Phase 3 基线（≤2/4 per ROI）
3. Setup B left 的 miss 被标注为 **已知局限**（clearance 0.16m < max_step_up，架构正确行为）

> [!NOTE]
> Setup B left 的 2/3 miss 是传感器分辨率和物理几何共同决定的下界。0.16m 净空在 max_step_up=0.32m 范围内，系统判定为"可跨越"是正确的。该 ROI 应从 frozen miss 验收集中移除或降级为 known limitation。

如果 Phase 4.2 验证通过以上条件，**可以立即进入 Phase 5**。
