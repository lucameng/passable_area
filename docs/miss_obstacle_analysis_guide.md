# 漏检障碍离线分析使用说明

这份文档说明如何使用 `offline_replay --analyze-missed-obstacles`。

当前版本有一个重要前提：

- `PolarFrontend` 已回退到简单基线
- 前端不再通过 3x3 upper-support neighbor gate 和 explanation 分支来决定障碍是否成立
- 当前主链更接近：
  - 有样本吗
  - `vertical_span` 触发前端障碍了吗
  - `obstacle_evidence` 累起来了吗
  - obstacle point 发布门槛过了吗

所以这份排查指南也按这条链来读。

## 1. 如何运行

先在工作空间根目录执行：

```bash
source install/setup.bash
build/passable_area/passable_area_offline_replay \
  --analyze-missed-obstacles \
  --bag /你的/bag/路径 \
  --params-file /home/deep/deeprobotics/passable_humble_ws/src/passable_area/config/passable_area.yaml \
  --start-offset-sec 15.0 \
  --roi-x-min -3.3 --roi-x-max -2.6 \
  --roi-y-min -0.55 --roi-y-max -0.45 \
  --time-window-sec 0.4 \
  --top-k-cells 3
```

## 2. 坐标系

ROI 使用 `base_gravity` 机器人相对坐标：

- `x > 0`：前方
- `x < 0`：后方
- `y > 0`：左侧
- `y < 0`：右侧

## 3. 判定口径

这个工具用 `/terrain_obstacle_points` 判断“是否检测到障碍”：

- ROI 内有 obstacle points：不是 miss obstacle
- ROI 内 obstacle points 数量为 0：进入漏检分析

`terrain_state` 可以辅助人工排查，但不是这个工具的主判定信号。

## 4. root cause 怎么来

root cause 不是猜的，而是沿着当前控制链往后推：

1. ROI 内是否有输入样本
2. 前端是否形成 `obstacle_suspicious / obstacle_candidate`
3. 地图里的 `obstacle_evidence` 是否过阈值
4. 当前样本是否满足 `/terrain_obstacle_points` 的发布高度门槛

所以报告里会直接给出：

- `roi_sample_count`
- `obstacle_suspicious_cells`
- `obstacle_candidate_cells`
- `rejected_suspicious_cells`
- `neighbor_upper_support_count`
- `aligned_neighbor_support_count`
- `explanation_decision`
- `max_obstacle_evidence`
- `support_ref`
- `max_sample_z_minus_support_ref`

当前版本里建议这样理解：

- `obstacle_suspicious_cells`
  - 当前帧 `vertical_span` 已经过阈值的 cell 数
- `obstacle_candidate_cells`
  - 在当前简单前端里，通常与 `obstacle_suspicious_cells` 同步出现
- `max_obstacle_evidence`
  - 前端候选进入地图后，最终有没有积累到足够障碍证据
- `max_sample_z_minus_support_ref`
  - obstacle point 发布时，当前 ROI 样本相对支撑参考的最高高度差

这里的“发布高度门槛”包含两部分：

- 样本相对 `support_ref` 的高度要达到 `obstacle_points_min_height`
- 样本在 `base_link` 下的高度不能高于 `obstacle_points_max_height_in_base_link`

## 5. 当前版本里最常见的 root cause

当前简单前端基线下，优先关注这些原因：

- `NoSamplesInRoi`
- `NoFrontendObstacleSuspicion`
- `ObstacleEvidenceTooLow`
- `OutputHeightGateNotMet`
- `NoObstacleSourceSamplesInRoi`

它们大致对应：

- ROI 根本没样本
- 当前 cell 的原始高度跨度不够，前端没触发障碍
- 前端触发了，但地图证据还不够
- 地图证据够了，但 obstacle point 发布高度门槛没过
- 地图里有强 evidence cell，但当前 ROI 内没有对应 source sample

## 6. 关于兼容性字段怎么理解

报告里仍然可能出现：

- `RejectedByNeighborSupport`
- `LeakFilteredToNoCandidate`
- `neighbor_upper_support_count`
- `aligned_neighbor_support_count`
- `explanation_decision`

这些字段和 root cause 之所以还保留，是为了：

- 不打破 `FrameOutput` 契约
- 不打破 analyzer 输出格式
- 兼容历史数据和历史调试心智

但对当前前端实现来说，它们已经不是主路径。

换句话说：

- 现在如果你看到 `neighbor_upper_support_count = 0`
- `aligned_neighbor_support_count = 0`
- `explanation_decision = None`

这通常不是异常，而是当前简单前端的正常表现。

## 7. 当前排查顺序建议

遇到漏检时，建议按这个顺序看：

1. `roi_sample_count`
2. `obstacle_suspicious_cells`
3. `obstacle_candidate_cells`
4. `max_obstacle_evidence`
5. `max_sample_z_minus_support_ref`

对应判断：

- `roi_sample_count = 0`
  - 先排 ROI、时间窗、bag 对齐
- `obstacle_suspicious_cells = 0`
  - 先排 `vertical_span` 是否根本不够
- `obstacle_candidate_cells > 0` 但 `max_obstacle_evidence` 低
  - 先排地图证据累计是否不够
- `max_obstacle_evidence` 高但 `max_sample_z_minus_support_ref` 低
  - 先排 `obstacle_points_min_height`
- `max_obstacle_evidence` 高，但点都高于 base 下上限
  - 先排 `obstacle_points_max_height_in_base_link`

## 8. 一个最典型的当前版漏检结论

如果你看到：

- ROI 内有样本
- `obstacle_suspicious=true`
- `obstacle_candidate_cell=true`
- `max_obstacle_evidence` 还不够

那么更接近这类结论：

> 前端已经把它当成障碍候选，但地图层还没把证据累计到 obstacle point 的发布阈值。

这不是 neighbor gate 问题，也不是 explanation reject 问题。

如果你看到：

- ROI 内有样本
- `obstacle_suspicious=true`
- `obstacle_candidate_cell=true`
- `max_obstacle_evidence` 足够
- 但 `max_sample_z_minus_support_ref < obstacle_points_min_height`

那么更接近这类结论：

> 障碍已经成立，但发布层没有找到足够高于 `support_ref` 的样本，因此 `/terrain_obstacle_points` 仍为空。

## 9. 结论

当前版本的 miss-obstacle 排查主线很简单：

- 先看样本有没有进 ROI
- 再看 `vertical_span` 有没有形成前端障碍
- 再看地图证据够不够
- 最后看 obstacle point 发布门槛

不要默认把漏检归因到 neighbor gate 或 explanation 分支。

在当前实现里，那已经不是主路径了。
