# Passable Area 算法方案说明

## 1. 文档目的

本文档描述当前仓库中 `passable_area` 的**实际实现**，用于统一下面这些事情的理解口径：

- 模块到底解决什么问题
- 整体处理链路和模块分工是什么
- 输入输出合同、坐标系语义、调试输出语义是什么
- 地图层、前端解释层、最终通行性判定层分别在做什么
- 当前实现有哪些边界、取舍和容易误解的点

本文档以代码为准，主要对应下面这些路径：

- `include/passable_area/core`
- `src/core`
- `include/passable_area/interfaces/ros`
- `src/interfaces/ros`

如果后续代码实现变化，本文档也应一起更新。

---

## 2. 系统目标

`passable_area` 的目标不是长期全局建图，而是做一个**高频、局部、短时稳定**的地形可通行判定器。

它的核心特征是：

- 输入是严格时间同步的点云和里程计
- 内部维护一个对齐 `map` 的局部栅格地图
- 先在地图上建立支撑面、上层结构、观测覆盖和障碍证据
- 再输出三态通行性和通行代价
- 最终发布时，把内部结果重表达成以机器人为中心的 `base_gravity` 栅格

当前实现不是纯单帧算法，而是一个带短时记忆的局部地形系统。短时记忆主要体现在：

- `support_confidence` 跨帧累积和衰减
- `obstacle_evidence` 跨帧累积和衰减
- 地图随机器人平移，但不会每帧完全清空
- `dropout` 会直接影响衰减与负更新策略；普通覆盖不足通过 `frame_partial` 记录，但 sector state 并入 `Observed`

因此更准确的描述是：

> `passable_area` 是一个使用局部连续地图、观测性评估和短时证据记忆的实时地形可通行判定系统。

---

## 3. 软件架构总览

当前架构分成两层。

### 3.1 `core`

路径：

- `include/passable_area/core`
- `src/core`

职责：

- 不依赖 ROS 消息类型
- 完成点云预处理、观测性估计、前端候选生成、地图更新、地形特征计算和通行性判定
- 提供统一入口：

```cpp
Processor::update(const FrameInput&) -> FrameOutput
```

### 3.2 `interfaces/ros`

路径：

- `include/passable_area/interfaces/ros`
- `src/interfaces/ros`

职责：

- ROS 参数加载
- `PointCloud2` / `Odometry` 转换
- cloud + `/ODOM` 的 `ExactTime` 严格同步
- watchdog 和 perf stats
- 发布 `terrain_state` / `terrain_cost` / `grid_map` / debug 点云 / `TerrainObservability`
- 可选发布 `map -> base_gravity` TF

### 3.3 运行时入口

主节点入口：

- `src/passable_area/src/main/passable_area_node_main.cpp`

主节点类：

- `passable_area::interfaces::ros::PassableAreaNode`

运行时只有这一个 ROS 2 节点入口。

---

## 4. 当前处理链路

每个同步后的输入帧，按固定顺序经过下面这些阶段：

1. `FramePreprocessor`
2. `FrameObservabilityEstimator`
3. `PolarFrontend`
4. `DropoutAwareMapUpdater`
5. `TerrainFeatureUpdater`
6. `ObstacleReasoner`
7. `TraversabilitySolver`
8. `Processor::buildOutput`
9. ROS 输出转换与发布

这条主链由：

- `src/passable_area/src/core/processor.cpp`

串起来。

---

## 5. 输入输出合同

### 5.1 输入订阅

默认订阅：

- `/LOC_BODY_POINTS` `sensor_msgs/msg/PointCloud2`
- `/ODOM` `nav_msgs/msg/Odometry`

配置来源：

- `src/passable_area/config/sensors.yaml`
- `src/passable_area/include/passable_area/interfaces/ros/ros_param_loader.hpp`

这两路输入通过 `message_filters::Synchronizer<ExactTime>` 严格同步。

当前实现含义是：

- 时间戳不严格匹配，就不会进入主处理链
- 没有 `ApproximateTime` 兼容路径

### 5.2 输出发布

主要输出：

- `/terrain_state` `nav_msgs/msg/OccupancyGrid`
- `/terrain_cost` `nav_msgs/msg/OccupancyGrid`
- `/terrain_debug/grid_map` `grid_map_msgs/msg/GridMap`
- `/terrain_debug/base_gravity_cloud` `sensor_msgs/msg/PointCloud2`
- `/terrain_debug/support_points` `sensor_msgs/msg/PointCloud2`
- `/terrain_obstacle_points` `sensor_msgs/msg/PointCloud2`
- `/terrain_debug/unknown_mask` `sensor_msgs/msg/PointCloud2`
- `/terrain_debug/observability` `passable_area/msg/TerrainObservability`

TF：

- `map_frame -> base_gravity_frame`，默认关闭

### 5.3 `terrain_state` 语义

- `0` = passable
- `100` = impassable
- `-1` = unknown

### 5.4 `terrain_cost` 语义

- `-1` = unknown
- `100` = impassable
- `1~99` = passable cell 的几何代价

### 5.5 `TerrainObservability` 当前字段

消息定义位于：

- `src/passable_area/msg/TerrainObservability.msg`

字段：

- `std_msgs/Header header`
- `bool frame_partial`
- `bool rear_dropout`
- `uint16 sector_count`
- `uint32 base_point_count`
- `uint32 map_point_count`
- `uint8[] sector_states`
- `float32[] sector_coverage_confidence`

`sector_states` 编码：

- `0` = `Observed`
- `1` = 保留编码空位，当前实现不再产生
- `2` = `MissingByDropout`

说明：

- ROS 消息里的 `rear_dropout` 为历史兼容字段，仅表示后向 dropout。
- 前向/后向及其它方向的 dropout 统一通过 `sector_states == MissingByDropout` 表达。

### 5.6 输入假设

当前实现默认依赖以下前提：

- 输入点云在 `body_frame`，默认 `base_link`
- `/ODOM` 提供的姿态消息当前以 `map` 作为 `header.frame_id`
- `map` 足够连续，适合作为局部地图累积系
- `map` 在高度语义上可作为重力参考
- 输入 cloud 和 `/ODOM` 时间严格同步

---

## 6. 坐标系定义与真实发布语义

当前系统里最重要的三个坐标系是：

- `base_link`
- `map`
- `base_gravity`

### 6.1 `base_link`

定义：

- 原始输入点云所在的机器人机体系
- 保留完整姿态，包含 roll / pitch / yaw

用途：

- 原始观测
- 观测性与扇区语义分析

### 6.2 `map`

定义：

- 外部 `/ODOM` 提供的实际父坐标系，当前系统里它的 `header.frame_id` 是 `map`
- 当前实现把这套 `map` 语义当作内部建图和跨帧累积坐标系

用途：

- `cloud_in_map`
- `map_samples`
- `LocalTerrainMap`
- 各种内部地图层

说明：

- `cloud_in_map` / `map_samples` / `base_pose_in_map` 仍是历史字段名
- 本次修正后，这些字段的数值语义统一按 `map` 理解

### 6.3 `base_gravity`

定义：

- 原点在当前机器人位置
- 平移跟随机器人
- z 轴保持重力方向
- 旋转只保留 yaw，不保留 roll / pitch

用途：

- 调试点云的机器人中心表达
- 最终对外地图结果的 robot-centric 发布表达

### 6.4 一个非常重要的实现事实

当前代码里：

- **内部主处理在 `map` 中进行**
- **对外发布的 `terrain_state` / `terrain_cost` / `grid_map` 会重采样成 `base_gravity` 机器人中心栅格**

这点来自：

- `PassableAreaNode::onSynced()` 向发布器传入的是 `base_gravity_header`
- `OutputConverter::toMapOutputs()` 显式执行了从内部 `map` 地图到 robot-centric `base_gravity` 栅格的重采样

因此要区分：

- 内部地图语义：`map`
- 最终发布语义：`base_gravity`

### 6.5 变换关系

#### `base_link -> map`

在 `FramePreprocessor` 中完成：

`p_map = R(map<-base) * p_base + t(map<-base)`

来源：

- 位置：`base_pose_in_map.position`（历史字段名，语义已是 `map`）
- 姿态：`base_pose_in_map.orientation`（历史字段名，语义已是 `map`）

#### `map -> base_gravity`

用于调试点云和输出重表达：

1. 减去机器人在 `map` 下的位置
2. 只保留 yaw 旋转
3. 不保留 roll / pitch

这保证：

- 点云和地图结果围绕机器人中心表达
- 机器人俯仰或侧倾时，可视化不会整体歪斜

### 6.6 为什么主处理不用 `base_gravity`

因为 `base_gravity` 是 robot-centric 坐标系，点会随着机器人运动而改变坐标，不能稳定地做跨帧地图累计。

内部主处理使用 `map` 的原因是：

- 同一环境位置在连续帧中仍然落在稳定栅格
- support / obstacle / coverage 可以跨帧累积
- dropout / persistence 才有意义

---

## 7. Core 数据结构

### 7.1 `FrameInput`

定义在：

- `include/passable_area/core/types/frame_types.hpp`

关键字段：

- `stamp`
- `base_pose_in_map`
- `input_cloud_in_base`
- `processing_enabled`

作用：

- 表示送入 `Processor` 的一帧原始输入

### 7.2 `ProcessedFrame`

关键字段：

- `stamp`
- `base_pose_in_map`
- `cloud_in_base`
- `cloud_in_map`
- `map_samples`

其中：

- `cloud_in_base` 用于观测性分析
- `cloud_in_map` 和 `map_samples` 用于建图与几何推理

### 7.3 `FrameObservability`

关键字段：

- `frame_partial`
- `front_dropout`
- `rear_dropout`
- `base_point_count`
- `map_point_count`
- `sectors`

作用：

- 描述这一帧在机器人各方向上的观测完整性

### 7.4 `FrameOutput`

关键字段包括：

- 地图尺寸、分辨率、`origin`
- `passability`
- `traversal_cost`
- `support_height`
- `overhead_height`
- `protrusion_height`
- `support_confidence`
- `protrusion_evidence`
- `overhead_evidence`
- `obstacle_evidence`
- `coverage_confidence`
- `slope`
- `step_up`
- `step_down`
- `roughness`
- `clearance`
- `support_continuity`
- `block_reason`
- `protrusion_stage`
- `overhead_stage`
- `obstacle_point_publish_status`
- `obstacle_suspicious`
- `obstacle_candidate_cell`
- `support_state`
- `base_gravity_cloud_points`
- `support_points`
- `obstacle_points`
- `unknown_points`
- `observability`

### 7.5 `LocalTerrainMap`

作用：

- 维护一个在 `map` 中随机器人平移的局部二维栅格地图

关键接口：

- `recenter()`
- `mapToIndex()`
- `indexToMap()`
- `ageCells()`

### 7.6 `TerrainLayers`

作用：

- 存放每个 cell 的所有地图层

当前主要 layer：

- `support_height`
- `support_confidence`
- `overhead_height`
- `overhead_confidence`
- `protrusion_height`
- `protrusion_evidence`
- `overhead_evidence`
- `obstacle_evidence`
- `coverage_confidence`
- `slope`
- `step_up`
- `step_down`
- `roughness`
- `clearance`
- `support_continuity`
- `support_state`
- `passability_state`
- `traversal_cost`
- `last_sector_state`
- `last_observed_age`
- `last_reliable_age`

---

## 8. 各模块详细说明

### 8.1 `FramePreprocessor`

实现：

- `src/passable_area/src/core/frame_preprocessor.cpp`

职责：

- 重置输出结构并复制位姿
- 对输入点做有限值检查
- 在 `base_link` 下按 `preprocess.body_filter.*` 剔除车体内部点
- 用 `base_pose_in_map` 把点从 `base_link` 变换到 `map`
- 按当前局部地图窗口裁剪
- 用相对机器人高度窗口过滤 z
- 按配置做体素降采样

输出保留三份关键表达：

- `cloud_in_base`
- `cloud_in_map`
- `map_samples`

实现特点：

- `cloud_in_base` 保留给观测性分析
- `cloud_in_map` / `map_samples` 保留给建图与前端解释
- z 裁剪使用：

```cpp
relative_z = point_in_map.z - base_pose_in_map.position.z()
```

这意味着高度窗口是相对当前机器人高度解释的。

### 8.2 `FrameObservabilityEstimator`

实现：

- `src/passable_area/src/core/frame_observability_estimator.cpp`

职责：

- 只使用 `cloud_in_base`
- 按 360 度扇区统计点数
- 输出每个扇区的覆盖置信度和观测状态
- 输出 `frame_partial`、内部 `front_dropout` 和兼容字段 `rear_dropout`

当前状态分类：

- `Observed`
- `MissingByDropout`

实现特点：

- 点数为 0 或覆盖不足的扇区会设置 `frame_partial`，但扇区状态默认仍并入 `Observed`
- 如果前方或后方整体点数相对另一半视野明显不足，且对应半区连续空扇区超过阈值，就把该空洞升级成 `MissingByDropout`
- `rear_dropout` 只保留后向布尔兼容语义；前向 dropout 由内部 `front_dropout` 和 `sector_states` 表达

这一步的结果直接影响后续地图层的衰减和负更新强度。

### 8.3 `PolarFrontend`

实现：

- `src/passable_area/src/core/polar_frontend.cpp`

职责：

- 基于 `map_samples` 逐 cell 聚合当前帧点
- 推导本帧支撑候选和障碍候选

输出：

- `support_candidates`
- `protrusion_candidates`
- `overhead_candidates`
- sample 统计以及轻量 stage 字段

#### 8.3.1 核心思路

当前主线已经从简单 `min_z / max_z` baseline 升级为 V2 轻量双层摘要。

它的核心步骤是：

1. 按 cell 聚合当前帧样本，统计 `min_z / max_z / count`
2. 按 `profile_split_gap` 对 cell 内 z profile 做 band split
3. 用 band 内 10% / 90% trimmed bounds 形成稳定高度摘要
4. 对非 dropout cell 输出 `support_candidates(cell, support_band.bottom, coverage)`
5. 根据上层结构分别输出：
   - `protrusion_candidates`
   - `overhead_candidates`

当前不再在前端主链里做：

- 支撑锚点借用与拒绝
- leak suppression
- upper-support 邻域确认
- explanation reject / keep
- `effective_support_ref` 这类临时抬高参考面的解释链

#### 8.3.2 支撑相关的几个关键概念

当前真正参与主链判定的量是：

- `raw_min_z`
  - 当前 cell 本帧样本的最低 z，仅作为 debug 原始统计
- `vertical_span`
  - 当前 cell 本帧样本的竖直跨度 `max_z - min_z`，用于单 tall band / protrusion 触发
- `support_band`
  - 当前 cell 的支撑层高度摘要
- `upper_band`
  - support 之上的上层结构摘要

它们的用法是：

- `support_band.bottom`
  - 作为 `support_candidates` 的高度来源
- `vertical_span`
  - 参与 protrusion trigger
- `upper_band.bottom`
  - 参与 overhead / low-clearance trigger

当前 `FrameOutput` / `FrontendOutput` 已经不再携带旧 explanation / anchor 兼容字段。

当前主线只保留：

- 原始/过滤后的 sample 统计
- `obstacle_suspicious`
- `obstacle_candidate_cell`
- reasoner 的 `block_reason`
- `protrusion_stage` / `overhead_stage`
- `obstacle_point_publish_status`

#### 8.3.3 历史解释逻辑已退出主线

当前主线前端已经不做锚点解析，相关兼容字段也已移除。

也就是说：

- 不再使用历史 `support_height` 给当前帧借锚
- 不再区分 local / borrowed support anchor
- 不再做 stale / wall-only anchor reject
- analyzer / grid_map 也不再暴露这些历史锚点状态

当前支撑候选的来源就是：

- 当前 cell 的本帧 `support_band.bottom`
- 如果最低 band 只有 1 个样本、且下一个 band 有至少 3 个样本，则把最低 band 视作 sparse lower leak，并用下一 band 作为 support

#### 8.3.4 suspicious obstacle 只是第一步

当前实现会用 band 摘要触发可疑：

- `vertical_span > max_step_up * 0.75`
- 或 support 上方存在低于 `min_clearance` 的 upper band

这时只会把 cell 标成：

- `obstacle_suspicious`

在当前 V2 前端里，`obstacle_suspicious` 与 `obstacle_candidate_cell` 仍作为兼容 stage 字段同步形成，不再经过旧邻域门控。

#### 8.3.5 当前障碍形成的真实门控

当前没有额外门控。

主线判断就是：

- 如果 support 上方高度差超过 `max_step_up * 0.75`
  - 输出 `protrusion_candidates`
  - candidate evidence 使用 `height_above_support / max_step_up` 归一化并 clamp 到 `[0, 1]`
  - 无 overhead 共存的 pure protrusion 使用 `1.5` 的证据累积倍率；有 overhead 共存时保持 `1.0`
- 如果 support 上方存在 `upper_band.bottom - support_ref < min_clearance`
  - 输出 `overhead_candidates`
  - candidate evidence 使用 `(min_clearance - clearance_gap) / max(min_clearance - max_step_up, 1e-3)` 归一化并 clamp 到 `[0, 1]`
  - 这表示在 `max_step_up..min_clearance` 这段不可通行净空区间内归一化证据；是否发布到 `/terrain_obstacle_points` 仍由后续 low-clearance bridge 的 `clearance > max_step_up` 和 evidence 阈值共同决定
- 任一候选形成时：
  - 设置 `obstacle_suspicious = 1`
  - 设置 `obstacle_candidate_cell = 1`

因此当前已经没有“固定 `3x3` upper-support neighborhood gate”。

#### 8.3.6 `effective_support_ref` 的定位

当前主线前端已经不再使用 `effective_support_ref`，它也不是现役 contract 的一部分。

#### 8.3.7 `ambiguous_candidates`

当前前端已经删除 `ambiguous_candidates` 输出。

历史版本中它用于标记 partial observed 且未触发 obstacle 的 cell，但该分支没有进入地图更新闭环。Phase 1 后，缺测下的保守性只由 `Observed / MissingByDropout` 二态观测、证据衰减和 persistence 机制承担。

### 8.4 `DropoutAwareMapUpdater`

实现：

- `src/passable_area/src/core/mapping/dropout_aware_map_updater.cpp`

职责：

- 把前端候选写进地图层
- 维护 support / protrusion / overhead 证据的累积与衰减
- 根据观测状态决定负更新强度

核心思想：

- `Observed`：可以较正常地衰减和更新
- `MissingByDropout`：极弱衰减，避免因为掉点把地图刷空

support 更新：

- 写 `support_height`
- 增加 `support_confidence`
- 更新 `coverage_confidence`
- 维护 `support_state`
- 更新 `last_observed_age` / `last_reliable_age`

obstacle 更新：

- `protrusion_candidates` 写 `protrusion_height` / `protrusion_evidence`
- `overhead_candidates` 写 `overhead_height` / `overhead_confidence` / `overhead_evidence`
- 过渡期 `obstacle_evidence = max(protrusion_evidence, overhead_evidence)`，继续作为 solver 的兼容证据层
- `/terrain_obstacle_points` 发布侧拆成两条证据入口：
  - `protrusion_evidence` 达阈值的 protrusion 发布路径；连续支撑上的纯 `LowClearance` 投影不会按普通 protrusion 阈值直接发布
  - 当前 source cell 有足够密集的真实样本，且 `protrusion_evidence` 只差一个很小的 evidence quantum 时，允许 `DenseProtrusionSource` 近阈值发布，用于对齐侧板 ROI 内 candidate 累积和当前 source samples
  - reasoner 低净空相关阻挡且 `overhead_evidence` 达阈值的 low-clearance bridge；该 bridge 还要求 `clearance > max_step_up`，避免把台阶量级的低净空地形投影发布到外部障碍点云
- `DenseProtrusionSource` 不是新的障碍真值，也不改变 `terrain_state` / `terrain_cost`：
  - 只在非 `LowClearance` source cell 上生效
  - 必须已有本帧 frontend obstacle candidate
  - `raw_sample_count >= min_points_per_sector - 1`，要求当前帧 ROI/source 样本足够密集
  - 近阈值容差是 `obstacle_evidence_gain * 0.1`；这不是物理高度阈值，而是单帧证据量化容差，默认 `0.25 * 0.1 = 0.025`，即把默认发布阈值 `0.4` 临时放宽到 `0.375`
  - 该容差用于覆盖短 ROI 窗口中 dense source cell 落在阈值下方一个很小证据 quantum 的情况；如果后续 evidence 累积/归一化架构能消除此量化误差，应优先删除该路径
- obstacle point 的样本高度门控同时要求：
  - 相对 support reference 高于 `obstacle_points_min_height`
  - 在 `base_link` 中不高于 `obstacle_points_max_height_in_base_link`

当前实现不再按旧 explanation / 邻域否决字段清理障碍层。障碍层清理由证据衰减、support 重观测和 support 失效后的阈值判断触发。

### 8.5 `TerrainFeatureUpdater`

实现：

- `src/passable_area/src/core/terrain_feature_updater.cpp`

职责：

- 对 dirty cells 及其邻域增量更新几何特征

当前特征：

- `slope`
- `step_up`
- `step_down`
- `roughness`
- `clearance`
- `support_continuity`

实现特点：

- 使用 3x3 support-surface 邻域统计
- 达到发布证据阈值的 active `protrusion_evidence` 邻居不参与 support-surface 几何特征，避免墙脚或实体障碍 bleed 到可站立 cell；`overhead_evidence` 不通过聚合的 `obstacle_evidence` 间接剔除地面支撑邻居
- slope / roughness 来自局部支撑面平面拟合；点数不足以拟合平面时，按邻居实际平面距离计算退化坡度
- `step_up` / `step_down` 表示“从邻居进入当前 cell”的方向性跨越代价
- 偏向实时性和稳定性

### 8.6 `TraversabilitySolver`

实现：

- `src/passable_area/src/core/traversability_solver.cpp`

职责：

- 基于 support / coverage / feature 层和 `ObstacleReasoner` 输出的 `block_reason` 输出三态可通行性
- 同步生成 `terrain_cost`

当前判定策略：

1. 先判 `UNKNOWN`
2. 再判 `IMPASSABLE`
3. 剩余可靠 cell 直接判 `PASSABLE`

`UNKNOWN` 的典型条件：

- `coverage_confidence` 太低
- `support_state == None`
- `support_confidence` 太低
- `last_reliable_age` 超过 stale 阈值

`IMPASSABLE` 的典型条件：

- `block_reason == LowClearance`
- `block_reason == Protrusion`
- `block_reason == Mixed`
- `block_reason == GeometryFailure`

`PASSABLE` 的典型条件：

- `block_reason == None`
- 且 cell 已经通过 `UNKNOWN` gate

实现说明：

- `UNKNOWN` 优先级仍高于 `block_reason`，无可靠 support / coverage / stale cell 不会被 reasoner 硬判成 `IMPASSABLE`
- solver 不再直接用 `obstacle_evidence + support_continuity` 解释障碍阻挡；障碍阻挡语义先由 `ObstacleReasoner` 给出
- solver 也不再重复执行几何阻挡阈值判定；`GeometryFailure` 等阻挡语义由 `ObstacleReasoner` 提供，solver 只负责 `UNKNOWN / IMPASSABLE / PASSABLE` 三态整合与 cost 输出
- 当前主链没有 BFS、可达域扩张或图搜索求解
- 当前版本是逐 cell 判通行性，再给 passable cell 生成代价

### 8.7 `ObstacleReasoner`

实现：

- `src/passable_area/src/core/obstacle_reasoner.cpp`

职责：

- 在 `TraversabilitySolver` 前解释当前地图层中的阻挡原因
- 向 solver 提供 `block_reason`，作为 `IMPASSABLE` 判定的主障碍语义输入
- 把 protrusion / overhead / geometry 相关证据整理成可审计输出

当前输出：

- `block_reason`
  - `None`
  - `Protrusion`
  - `LowClearance`
  - `GeometryFailure`
  - `Mixed`
- `protrusion_stage`
- `overhead_stage`
- `obstacle_point_publish_status`

当前边界：

- `block_reason` 是内部通行性判定和诊断信号，不是新的外部障碍真值。
- 对下游导航仍以 `/terrain_obstacle_points` 为唯一外部障碍输出合同。
- `LowClearance` 阻挡由几何净空不足和仍然有效的 overhead 状态共同决定；`clearance` 只表达几何高度差，`overhead_confidence` / `overhead_evidence` 决定该低净空结构是否仍有效。
- Phase 3b 后，`LowClearance` 和包含低净空的 `Mixed` cell 在 overhead evidence 达到发布阈值时，可驱动 `/terrain_obstacle_points` 发布。

### 8.8 `Processor::buildOutput`

实现：

- `src/passable_area/src/core/processor.cpp`

职责：

- 把内部地图层整理成 `FrameOutput`
- 生成调试点云

调试点包括：

- `base_gravity_cloud_points`
- `support_points`
- `obstacle_points`
- `unknown_points`

其中障碍调试点还会额外经过 evidence / reasoner / 高度门控：

- `protrusion_evidence` 必须足够高，或 reasoner 给出低净空相关阻挡且 `overhead_evidence` 达到发布阈值
- 点相对当前地图中可信 `support_height` 的高度要够高；没有有限 `support_height` 的 cell 不会用当前帧最低样本临时生成发布支撑参考
- 点在 `base_link` 下的 z 又不能太高

这使得障碍调试点更偏向“与机器人近地通行有关的障碍样本”。

---

## 9. 局部地图机制

`LocalTerrainMap` 是当前实现稳定性的关键。

### 9.1 作用

- 维护局部二维栅格和多层地形属性
- 跟随机器人在 `odom` 中平移
- 保留短时历史

### 9.2 特性

- 地图不是长期全局图
- 是固定尺寸的局部连续缓冲区
- 机器人移动时，通过 `recenter()` 和 `shiftLayers()` 平移历史层

### 9.3 为什么需要它

如果完全纯单帧：

- 局部缺点会直接变 unknown
- support / obstacle 结论更不稳定
- 局部覆盖不足会导致帧间翻转

局部地图让系统更稳，但仍保持近场实时判定属性。

---

## 10. 输出与调试语义

### 10.1 `/terrain_debug/base_gravity_cloud`

语义：

- 当前实际参与主链的预处理后点云
- 来源是 `map_samples`
- 再显式转换到 `base_gravity`

用途：

- 看算法真正吃进去的点
- 对照 support / obstacle / unknown 结果

### 10.2 `/terrain_debug/support_points`

语义：

- 落在 support cell，且与 `support_height` 足够接近的真实样本点
- 发布在 `base_gravity`

### 10.3 `/terrain_obstacle_points`

语义：

- 落在 `protrusion_evidence` 已足够高的 cell 内，落在 dense near-threshold protrusion source cell 内，或落在 reasoner 低净空相关阻挡 cell 内且 `overhead_evidence` 已足够高的真实样本点
- 连续支撑上的纯 `LowClearance` cell 默认视作低净空地形投影，不通过普通 protrusion 阈值发布
- low-clearance bridge 只用于 `clearance > max_step_up` 的低净空障碍，不用于台阶量级投影
- 当前还额外保留一个最小版 rear-dropout obstacle-point bridge：
  - 仅当 `rear_dropout == true` 且当前帧后方没有原生 obstacle points 时，允许复用上一帧已发布的后方 obstacle points
  - 该 bridge 最多只续 1 帧，不可连续续命
  - 当前只启用“rear dropout / 上一帧缓存 / 单帧生存期 / source cell index 仍在局部地图内”这组最小条件
  - 基于当前 `block_reason`、`passability` 或 evidence 的额外 gate 仍保留为后续可选约束，暂未启用
- 要求相对支撑参考高度至少为 `obstacle_points_min_height`
- 同时要求样本在 `base_link` 下 z 不高于 `obstacle_points_max_height_in_base_link`
- 当前 cell 必须有有限可信的地图 `support_height`；当前帧 `min_z` 不得作为 fallback 支撑参考授予障碍点发布资格
- 发布在 `base_gravity`

### 10.4 `/terrain_debug/unknown_mask`

语义：

- unknown cell 的地图状态点
- 不是原始观测点
- 发布在 `base_gravity`

### 10.5 `/terrain_debug/grid_map`

语义：

- 局部地图调试输出
- 内部真值来自 `odom` 地图
- 发布阶段重采样成 `base_gravity` robot-centric 栅格
- 与 `/terrain_state` 和 `/terrain_cost` 共用同一套发布 geometry

### 10.6 `/terrain_debug/observability`

语义：

- 当前帧观测完整性和扇区状态摘要
- 不是几何点云
- header 使用 `base_gravity` 发布上下文

---

## 11. ROS 接口层实现说明

当前 ROS 接口层负责：

- 参数声明和装配
- `PointCloud2` / `Odometry` 转换
- `ExactTime` 同步
- watchdog
- perf stats
- 结果与 debug 输出发布
- TF 发布

### 11.1 同步策略

- 使用 `message_filters::Synchronizer`
- cloud 和 odom 采用 `ExactTime`
- 只有严格同步的消息对才进入处理

### 11.2 Watchdog

职责：

- 记录 cloud 输入是否正常
- 记录 odom 输入是否正常
- 记录同步回调是否持续触发

### 11.3 Perf stats

职责：

- 统计输入频率
- 统计处理耗时

### 11.4 `base_gravity` TF

当前节点在参数打开时会发布：

- `map_frame -> base_gravity_frame`

定义：

- 平移直接使用当前 `base_pose_in_map.position`
- 旋转只保留 yaw

这保证 RViz 在 `map` 固定系下也能正确显示围绕机器人中心的 debug 点云和地图。

---

## 12. 参数说明

### 12.1 基础地图参数

来自：

- `src/passable_area/config/passable_area.yaml`

参数：

- `map_length`
- `map_width`
- `map_resolution`
- `map_height_min`
- `map_height_max`

说明：

- `map_height_min/max` 是相对当前机器人高度的窗口
- XY 裁切依赖当前局部地图窗口

### 12.2 几何参数

- `max_support_slope`
- `max_step_up`
- `max_step_down`
- `max_support_roughness`
- `min_clearance`
- `profile_split_gap`

### 12.3 观测参数

- `dropout_sector_gap_threshold`
- `min_support_confidence`
- `stale_to_unknown_time`
- `sector_count`
- `min_points_per_sector`

### 12.4 持续性参数

- `support_persistence_frames`
- `obstacle_clear_observed_decay`
- `obstacle_clear_partial_decay_scale`
- `obstacle_height_clear_threshold`

### 12.5 障碍调试点参数

- `obstacle_points_min_evidence`
- `obstacle_points_min_height`
- `obstacle_points_max_height_in_base_link`

### 12.6 坐标系参数

- `map_frame`
- `base_gravity_frame`
- `body_frame`
- `debug.publish_map_to_base_gravity_tf`

### 12.7 预处理参数

- `preprocess.body_filter.enable`
- `preprocess.body_filter.x_min`
- `preprocess.body_filter.x_max`
- `preprocess.body_filter.y_min`
- `preprocess.body_filter.y_max`
- `preprocess.body_filter.z_min`
- `preprocess.body_filter.z_max`
- `preprocess.crop_to_map.enable`
- `preprocess.crop_to_map.xy_margin`
- `downsample.enable`
- `downsample.voxel_size`

### 12.8 调试参数

来自：

- `src/passable_area/config/debug.yaml`

参数：

- `debug.publish_grid_map`
- `debug.publish_points`
- `debug.publish_base_gravity_cloud`
- `debug.publish_observability`

当前代码事实需要特别说明：

- `publish_base_gravity_cloud` 真的控制了 `base_gravity_cloud` 是否发布
- `publish_grid_map` / `publish_points` / `publish_observability` 当前已经声明并加载，但发布逻辑没有完全按这些开关分支控制

换句话说，当前实现里这些调试开关的行为比参数名字看起来更“弱”。

### 12.9 调试输出 topic 参数

- `output.terrain_state_topic`
- `output.terrain_cost_topic`
- `output.debug_grid_map_topic`
- `output.base_gravity_cloud_topic`
- `output.support_points_topic`
- `output.obstacle_points_topic`
- `output.unknown_mask_topic`
- `output.observability_topic`

---

## 13. 设计取舍与已知边界

### 13.1 为什么不是简单高度阈值法

因为当前模块需要处理的不只是“平地上立着一个障碍物”，还包括：

- 楼梯
- 斜坡
- 分层地面
- 机器人下方还能看到更低层地面
- 垂直墙面落入单格造成大高度跨度
- 后向或局部掉点

如果只做“最低点当地面，高于阈值当障碍”，这些场景里会有大量假障碍和错误未知。

### 13.2 当前实现特别照顾的场景

从整条链路的职责分工可以看出，当前实现仍然特别关注：

- 楼梯边缘误判为墙
- 机器人脚下以下的低层地面混入当前层
- 墙面型高跨度 cell 造成的假障碍

区别在于：

- 这些场景不再通过复杂 `PolarFrontend` 规则栈处理
- 当前通过轻量双层摘要表达 support / protrusion / overhead
- 稳定性主要靠 observability、地图更新、证据累计和输出门槛维持

### 13.3 工程化取舍

为了实时性和稳定性，当前实现做了这些折中：

- 使用固定尺寸局部地图，不做全局图
- 特征只在 dirty cells 及邻域增量更新
- 坡度、粗糙度等用 3x3 邻域统计近似
- 不做复杂地面拟合和全局优化
- 把复杂前端解释整体拿掉，保持 `PolarFrontend` 轻量且可维护

### 13.4 当前主链没有做的事情

当前版本没有：

- BFS 可达域扩张
- 全局 reachable set 求解
- 多层地图持久建模
- `ambiguous_candidates`

### 13.5 后续维护时最需要小心的地方

- `support_height`、`support_ref` 和一批历史兼容字段不要混淆
- 当前前端里 `vertical_span` 只参与 protrusion trigger，低净空由 `overhead_candidates` 表达
- 内部地图是 `map`，对外发布结果是 `base_gravity`
- 不要再把复杂场景特化重新堆回 `PolarFrontend`

---

## 14. 构建方式

在工作区根目录执行：

```bash
colcon build --packages-select passable_area --symlink-install
source install/setup.bash
```

注意：

- 本包使用 `ament_cmake`
- 含自定义消息 `TerrainObservability.msg`
- 节点和消息在同包内构建

---

## 15. 测试与验证方式

### 15.1 单元测试

执行：

```bash
colcon test --packages-select passable_area --event-handlers console_direct+
```

当前测试路径包括：

- `src/passable_area/test/core/`
- `src/passable_area/test/interfaces/ros/`
- `src/passable_area/test/tools/`

### 15.2 Benchmark 与离线工具

可用工具：

- `benchmarks/benchmark_processor.cpp`
- `benchmarks/benchmark_ros_e2e.cpp`
- `tools/offline_replay.cpp`

典型用途：

- 观察不同点云规模下处理耗时
- 离线回放 bag 检查 unknown 比例和 dropout 行为
- 回归性能趋势

### 15.3 运行时联调检查

建议至少做以下检查：

1. topic 是否存在

```bash
ros2 topic list | grep terrain
```

2. topic 类型是否正确

```bash
ros2 topic info /terrain_debug/observability
ros2 topic info /terrain_debug/base_gravity_cloud
```

3. TF 是否正确

```bash
ros2 run tf2_ros tf2_echo map base_gravity
```

4. debug frame 是否正确

```bash
ros2 topic echo /terrain_debug/base_gravity_cloud --once
```

确认：

- `header.frame_id == base_gravity`

---

## 16. 启动方式

当前 launch 文件：

- `launch/nav.launch.py`
- `launch/passable_area.launch.py`
- `launch/mapping.launch.py`

常用方式：

```bash
ros2 launch passable_area nav.launch.py
```

运行前建议：

```bash
source install/setup.bash
```

---

## 17. RViz 调试建议

### 17.1 看机器人中心视角点云和地图

设置：

- `Fixed Frame = base_gravity`

适合观察：

- `/terrain_debug/base_gravity_cloud`
- `/terrain_debug/support_points`
- `/terrain_obstacle_points`
- `/terrain_debug/unknown_mask`
- `/terrain_state`
- `/terrain_cost`
- `/terrain_debug/grid_map`

### 17.2 看 `map` 下的相对关系

设置：

- `Fixed Frame = map`

前提：

- 节点已打开 `debug.publish_map_to_base_gravity_tf`
- 节点已发布 `map -> base_gravity` TF

适合观察：

- 机器人在局部地图中的运动
- debug 点云和机器人之间的相对关系
- robot-centric 地图随机器人重表达的结果

---

## 18. 总结

理解当前 `passable_area` 实现时，最重要的一条主线是：

**它不是直接从点云里“找障碍”，而是先在局部地图里建立可靠支撑，再在支撑参考上谨慎地区分上层结构、观测缺失和真正障碍。**

顺着这条主线去理解，就能把下面这些设计连起来：

- 为什么同时保留 `cloud_in_base` 和 `cloud_in_map`
- 为什么要先做 `FrameObservabilityEstimator`
- 为什么 support 和 obstacle 都用证据累积
- 为什么这次把 `vertical_span` 恢复成直接障碍形成
- 为什么复杂解释不再放在前端
- 为什么 `UNKNOWN` 的优先级这么高
- 为什么内部在 `map` 建图，但最终发布为 `base_gravity` 机器人中心地图

如果后续要改算法行为，最需要确认的三个问题是：

1. 这次改动改变的是当前帧解释，还是长期地图记忆？
2. 这次改动会不会把楼梯、分层地面或墙面重新误解释成障碍？
3. 这次改动在 dropout、覆盖不足、短时失观测时是否仍然稳定？

只要这三件事始终盯住，基本就不会偏离当前实现的核心设计方向。
