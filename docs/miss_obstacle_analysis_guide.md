# 漏检障碍离线分析使用说明

这份文档说明如何使用 `offline_replay --analyze-missed-obstacles`。

它回答的问题是：

> 在给定的 `base_gravity` ROI 和 `start_offset` 附近，为什么 `/terrain_obstacle_points` 没有出现？

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

这个工具只用 `/terrain_obstacle_points` 判断“是否检测到障碍”：

- ROI 内有 obstacle points：不是 miss obstacle
- ROI 内 obstacle points 数量为 0：进入漏检分析

`terrain_state` 可以继续配合人工排查，但不是这个工具的主判定信号。

## 4. root cause 怎么来

root cause 不是猜的，而是沿着实际控制链往后推：

1. ROI 内是否有输入样本
2. 前端是否形成 `obstacle_suspicious / obstacle_candidate`
3. 地图里的 `obstacle_evidence` 是否过阈值
4. 当前样本是否满足 `/terrain_obstacle_points` 的发布高度门槛

所以报告里会直接给出：

- `roi_sample_count`
- `obstacle_suspicious_cells`
- `obstacle_candidate_cells`
- `rejected_suspicious_cells`
- `max_obstacle_evidence`
- `support_ref`
- `max_sample_z_minus_support_ref`

这里的“发布高度门槛”现在包含两部分：

- 样本相对 `support_ref` 的高度要达到 `obstacle_points_min_height`
- 样本在 `base_link` 下的高度不能高于 `obstacle_points_max_height_in_base_link`

## 5. 常见 root cause

- `NoSamplesInRoi`
- `NoFrontendObstacleSuspicion`
- `RejectedByNeighborSupport`
- `LeakFilteredToNoCandidate`
- `ObstacleEvidenceTooLow`
- `OutputHeightGateNotMet`
- `NoObstacleSourceSamplesInRoi`

这些名称都对应实际字段，不是场景标签。
