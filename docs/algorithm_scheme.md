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
- 内部维护一个对齐 `odom` 的局部栅格地图
- 先在地图上建立支撑面、上层结构、观测覆盖和障碍证据
- 再输出三态通行性和通行代价
- 最终发布时，把内部结果重表达成以机器人为中心的 `base_gravity` 栅格

当前实现不是纯单帧算法，而是一个带短时记忆的局部地形系统。短时记忆主要体现在：

- `support_confidence` 跨帧累积和衰减
- `obstacle_evidence` 跨帧累积和衰减
- 地图随机器人平移，但不会每帧完全清空
- `dropout` 和 `partial observability` 会直接影响衰减与负更新策略

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
- cloud + odom 的 `ExactTime` 严格同步
- watchdog 和 perf stats
- 发布 `terrain_state` / `terrain_cost` / `grid_map` / debug 点云 / `TerrainObservability`
- 发布 `odom -> base_gravity` TF

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
6. `TraversabilitySolver`
7. `Processor::buildOutput`
8. ROS 输出转换与发布

这条主链由：

- `src/passable_area/src/core/pipeline/processor.cpp`

串起来。

---

## 5. 输入输出合同

### 5.1 输入订阅

默认订阅：

- `/LOC_BODY_POINTS` `sensor_msgs/msg/PointCloud2`
- `/ODOM` `nav_msgs/msg/Odometry`

配置来源：

- `src/passable_area/config/sensors.yaml`
- `src/passable_area/include/passable_area/interfaces/ros/params/ros_param_loader.hpp`

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

- `odom_frame -> base_gravity_frame`

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
- `uint32 odom_point_count`
- `uint8[] sector_states`
- `float32[] sector_coverage_confidence`

`sector_states` 编码：

- `0` = `Observed`
- `1` = `PartiallyObserved`
- `2` = `MissingByDropout`

### 5.6 输入假设

当前实现默认依赖以下前提：

- 输入点云在 `body_frame`，默认 `base_link`
- 里程计姿态可用于 `base_link -> odom` 的完整 6DoF 变换
- `odom` 足够连续，适合作为局部地图累积系
- `odom` 在高度语义上可作为重力参考
- 输入 cloud 和 odom 时间严格同步

---

## 6. 坐标系定义与真实发布语义

当前系统里最重要的三个坐标系是：

- `base_link`
- `odom`
- `base_gravity`

### 6.1 `base_link`

定义：

- 原始输入点云所在的机器人机体系
- 保留完整姿态，包含 roll / pitch / yaw

用途：

- 原始观测
- 观测性与扇区语义分析

### 6.2 `odom`

定义：

- 外部 `/ODOM` 提供的局部连续父坐标系
- 当前实现把它当作内部建图和跨帧累积坐标系

用途：

- `cloud_in_odom`
- `odom_samples`
- `LocalTerrainMap`
- 各种内部地图层

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

- **内部主处理在 `odom` 中进行**
- **对外发布的 `terrain_state` / `terrain_cost` / `grid_map` 会重采样成 `base_gravity` 机器人中心栅格**

这点来自：

- `PassableAreaNode::onSynced()` 向发布器传入的是 `base_gravity_header`
- `OutputConverter::toMapOutputs()` 显式执行了从内部 `odom` 地图到 robot-centric `base_gravity` 栅格的重采样

因此要区分：

- 内部地图语义：`odom`
- 最终发布语义：`base_gravity`

### 6.5 变换关系

#### `base_link -> odom`

在 `FramePreprocessor` 中完成：

`p_odom = R(odom<-base) * p_base + t(odom<-base)`

来源：

- 位置：`base_pose_in_odom.position`
- 姿态：`base_pose_in_odom.orientation`

#### `odom -> base_gravity`

用于调试点云和输出重表达：

1. 减去机器人在 `odom` 下的位置
2. 只保留 yaw 旋转
3. 不保留 roll / pitch

这保证：

- 点云和地图结果围绕机器人中心表达
- 机器人俯仰或侧倾时，可视化不会整体歪斜

### 6.6 为什么主处理不用 `base_gravity`

因为 `base_gravity` 是 robot-centric 坐标系，点会随着机器人运动而改变坐标，不能稳定地做跨帧地图累计。

内部主处理使用 `odom` 的原因是：

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
- `base_pose_in_odom`
- `input_cloud_in_base`
- `processing_enabled`

作用：

- 表示送入 `Processor` 的一帧原始输入

### 7.2 `ProcessedFrame`

关键字段：

- `stamp`
- `base_pose_in_odom`
- `cloud_in_base`
- `cloud_in_odom`
- `odom_samples`

其中：

- `cloud_in_base` 用于观测性分析
- `cloud_in_odom` 和 `odom_samples` 用于建图与几何推理

### 7.3 `FrameObservability`

关键字段：

- `frame_partial`
- `rear_dropout`
- `base_point_count`
- `odom_point_count`
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
- `support_confidence`
- `obstacle_evidence`
- `coverage_confidence`
- `slope`
- `step_up`
- `step_down`
- `roughness`
- `clearance`
- `support_continuity`
- `support_anchor_used`
- `sub_support_leak_count`
- `upper_support_cell`
- `obstacle_suspicious`
- `obstacle_candidate_cell`
- `obstacle_rejected_by_neighbor_support`
- `neighbor_upper_support_count`
- `effective_support_ref_elevated`
- `support_state`
- `base_gravity_cloud_points`
- `support_points`
- `obstacle_points`
- `unknown_points`
- `observability`

### 7.5 `LocalTerrainMap`

作用：

- 维护一个在 `odom` 中随机器人平移的局部二维栅格地图

关键接口：

- `recenter()`
- `odomToIndex()`
- `indexToOdom()`
- `ageCells()`

### 7.6 `TerrainLayers`

作用：

- 存放每个 cell 的所有地图层

当前主要 layer：

- `support_height`
- `support_confidence`
- `overhead_height`
- `overhead_confidence`
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

- `src/passable_area/src/core/preprocess/frame_preprocessor.cpp`

职责：

- 重置输出结构并复制位姿
- 对输入点做有限值检查
- 在 `base_link` 下按 `preprocess.body_filter.*` 剔除车体内部点
- 用 `base_pose_in_odom` 把点从 `base_link` 变换到 `odom`
- 按当前局部地图窗口裁剪
- 用相对机器人高度窗口过滤 z
- 按配置做体素降采样

输出保留三份关键表达：

- `cloud_in_base`
- `cloud_in_odom`
- `odom_samples`

实现特点：

- `cloud_in_base` 保留给观测性分析
- `cloud_in_odom` / `odom_samples` 保留给建图与前端解释
- z 裁剪使用：

```cpp
relative_z = point_in_odom.z - base_pose_in_odom.position.z()
```

这意味着高度窗口是相对当前机器人高度解释的。

### 8.2 `FrameObservabilityEstimator`

实现：

- `src/passable_area/src/core/observability/frame_observability_estimator.cpp`

职责：

- 只使用 `cloud_in_base`
- 按 360 度扇区统计点数
- 输出每个扇区的覆盖置信度和观测状态
- 输出 `frame_partial` 和 `rear_dropout`

当前状态分类：

- `Observed`
- `PartiallyObserved`
- `MissingByDropout`

实现特点：

- 点数为 0 的扇区先视作 `PartiallyObserved`
- 如果后方整体点明显比前方少，且后向连续空扇区超过阈值，就升级成 `MissingByDropout`

这一步的结果直接影响后续地图层的衰减和负更新强度。

### 8.3 `PolarFrontend`

实现：

- `src/passable_area/src/core/frontend/polar_frontend.cpp`

职责：

- 基于 `odom_samples` 逐 cell 聚合当前帧点
- 推导本帧支撑候选、障碍候选和解释辅助层

输出：

- `support_candidates`
- `obstacle_candidates`
- `ambiguous_candidates`
- 一系列前端解释层

#### 8.3.1 核心思路

当前前端不是“每格取最低点当地面、取最高点当障碍”的简单版本。

它更接近：

1. 先给每个 cell 找一个合理的支撑参考
2. 再判断当前 cell 是否存在上层结构、分层结构或楼梯状结构
3. 最后只把通过邻域支撑门控的可疑 cell 形成障碍候选

#### 8.3.2 支撑相关的几个关键概念

最容易混淆的是这四个量：

- `raw_min_z`
- `support_anchor`
- `support_ref`
- `effective_support_ref`

含义分别是：

- `raw_min_z`
  - 当前 cell 本帧样本的最低 z
- `support_anchor`
  - 当前前端从历史地图借来的支撑锚点
- `support_ref`
  - 当前 cell 本帧解释真正使用的基础支撑参考
- `effective_support_ref`
  - 当前前端为了避免误判，对当前 cell 临时抬高后的解释参考

其中：

- `support_height` 是地图长期层
- `effective_support_ref` 只是前端局部解释量，不写回地图 `support_height`

#### 8.3.3 锚点的来源与拒绝逻辑

前端先尝试为当前 cell 找支撑锚点：

- 优先使用当前 cell 自身已有的历史 `support_height`
- 如果本 cell 无法用，再尝试 3x3 邻域里有效支撑的中位数

但前端不会盲信历史锚点，还会拒绝一些不可信情况，例如：

- `reject_stale_anchor`
- `reject_wall_only_anchor`

这些规则的目的是防止历史支撑参考在分层结构、墙面或旧地图残留的情况下把当前解释带偏。

#### 8.3.4 suspicious obstacle 只是第一步

当前实现会先用单格内部竖向跨度触发可疑：

- `vertical_span > max_step_up * 0.75`

这时只会把 cell 标成：

- `obstacle_suspicious`

它还不是最终障碍。

#### 8.3.5 当前障碍形成的真实门控

最终障碍候选必须经过邻域支撑门控。

当前实现逻辑是：

- 先保留 `vertical_span` 的 suspicious trigger
- 再要求 3x3 邻域里有足够多的 `upper_support_cell`
- 同时要排除被当前前端解释成“楼梯/上下层混合/前缘重解释”的情况

只有这样才会形成：

- `obstacle_candidate_cell`
- `obstacle_candidates`

这就是当前版本的“固定 `3x3` upper-support neighborhood gate”。

#### 8.3.6 `effective_support_ref` 的定位

当前代码明确把它当作：

> frontend-local temporary explanation ref

也就是：

- 只服务于当前 cell 的本帧解释
- 只影响 `upper_support` / obstacle explanation
- 不写回地图的 `support_height`
- 不是新的长期支撑估计

它主要用于避免下面这类典型误判：

- 机器人下方还能看到更低一层，导致 `support_ref` 被拉到过低层
- 结果把更合理的上层踏面和其上的局部结构误打成 obstacle

#### 8.3.7 `ambiguous_candidates`

当前前端仍会输出 `ambiguous_candidates`，但当前 `DropoutAwareMapUpdater` 并没有使用它。

因此在当前版本里它不是主链核心输入，更像预留结构和调试痕迹。

### 8.4 `DropoutAwareMapUpdater`

实现：

- `src/passable_area/src/core/mapping/dropout_aware_map_updater.cpp`

职责：

- 把前端候选写进地图层
- 维护 support / obstacle 证据的累积与衰减
- 根据观测状态决定负更新强度

核心思想：

- `Observed`：可以较正常地衰减和更新
- `PartiallyObserved`：只做保守衰减
- `MissingByDropout`：极弱衰减，避免因为掉点把地图刷空

support 更新：

- 写 `support_height`
- 增加 `support_confidence`
- 更新 `coverage_confidence`
- 维护 `support_state`
- 更新 `last_observed_age` / `last_reliable_age`

obstacle 更新：

- 写 `overhead_height`
- 增加 `overhead_confidence`
- 增加 `obstacle_evidence`

当前实现还会显式清理两类旧障碍：

- 本帧被“抬高有效支撑参考”重新解释的 cell
- 本帧被“邻域支撑否决为障碍”的 cell

### 8.5 `TerrainFeatureUpdater`

实现：

- `src/passable_area/src/core/features/terrain_feature_updater.cpp`

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

- 使用 3x3 邻域统计
- 不做复杂曲面拟合
- 偏向实时性和稳定性

### 8.6 `TraversabilitySolver`

实现：

- `src/passable_area/src/core/traversability/traversability_solver.cpp`

职责：

- 基于 support / obstacle / coverage / feature 层输出三态可通行性
- 同步生成 `terrain_cost`

当前判定策略：

1. 先判 `UNKNOWN`
2. 再判 `IMPASSABLE`
3. 满足几何约束再判 `PASSABLE`

`UNKNOWN` 的典型条件：

- `coverage_confidence` 太低
- `support_state == None`
- `support_confidence` 太低
- `last_reliable_age` 超过 stale 阈值

`IMPASSABLE` 的典型条件：

- `clearance < min_clearance`
- `support_continuity` 差且 `obstacle_evidence` 明显

`PASSABLE` 的典型条件：

- `slope <= max_support_slope_deg`
- `step_up <= max_step_up`
- `step_down <= max_step_down`
- `roughness <= max_support_roughness`
- `clearance` 充足

实现说明：

- 当前主链没有 BFS、可达域扩张或图搜索求解
- 当前版本是逐 cell 判通行性，再给 passable cell 生成代价

### 8.7 `Processor::buildOutput`

实现：

- `src/passable_area/src/core/pipeline/processor.cpp`

职责：

- 把内部地图层整理成 `FrameOutput`
- 生成调试点云

调试点包括：

- `base_gravity_cloud_points`
- `support_points`
- `obstacle_points`
- `unknown_points`

其中障碍调试点还会额外经过高度门控：

- 障碍证据必须足够高
- 点相对支撑参考的高度要够高
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
- 来源是 `odom_samples`
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

- 落在障碍证据已足够高的 cell 内的真实样本点
- 要求相对支撑参考高度至少为 `obstacle_points_min_height`
- 同时要求样本在 `base_link` 下 z 不高于 `obstacle_points_max_height_in_base_link`
- 当历史 `support_height` 不可用时，用当前帧该 cell 的 fallback `min_z` 作为支撑参考
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

当前节点会发布：

- `odom_frame -> base_gravity_frame`

定义：

- 平移直接使用当前 `base_pose_in_odom.position`
- 旋转只保留 yaw

这保证 RViz 在 `odom` 固定系下也能正确显示围绕机器人中心的 debug 点云和地图。

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
- `upper_min_height_above_support`
- `sub_support_leak_tolerance`
- `support_anchor_reobserve_tolerance`
- `min_neighbor_upper_support_cells`

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

- `odom_frame`
- `base_gravity_frame`
- `body_frame`

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

从前端规则可以明确看出，当前实现特别关注：

- 楼梯边缘误判为墙
- 楼梯上行时前沿台阶误判为障碍
- 机器人脚下以下的低层地面混入当前层
- 历史支撑参考和当前观测发生冲突时的保守重解释
- 墙面型高跨度 cell 造成的假障碍

### 13.3 工程化取舍

为了实时性和稳定性，当前实现做了这些折中：

- 使用固定尺寸局部地图，不做全局图
- 特征只在 dirty cells 及邻域增量更新
- 坡度、粗糙度等用 3x3 邻域统计近似
- 不做复杂地面拟合和全局优化
- 用 `effective_support_ref` 做前端局部解释，但不直接污染长期地图支撑层

### 13.4 当前主链没有做的事情

当前版本没有：

- BFS 可达域扩张
- 全局 reachable set 求解
- 多层地图持久建模
- 把 `ambiguous_candidates` 纳入主链更新

### 13.5 后续维护时最需要小心的地方

- `support_height`、`support_anchor`、`support_ref`、`effective_support_ref` 不要混淆
- `vertical_span` 只是 suspicious trigger，不是障碍最终判定
- 内部地图是 `odom`，对外发布结果是 `base_gravity`
- 前端条件耦合很强，改一个阈值可能连带影响楼梯解释和假障碍抑制

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

- `benchmarks/core/benchmark_processor.cpp`
- `benchmarks/interfaces/ros/e2e_benchmark.cpp`
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
ros2 run tf2_ros tf2_echo odom base_gravity
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

### 17.2 看 `odom` 下的相对关系

设置：

- `Fixed Frame = odom`

前提：

- 节点已发布 `odom -> base_gravity` TF

适合观察：

- 机器人在局部地图中的运动
- debug 点云和机器人之间的相对关系
- robot-centric 地图随机器人重表达的结果

---

## 18. 总结

理解当前 `passable_area` 实现时，最重要的一条主线是：

**它不是直接从点云里“找障碍”，而是先在局部地图里建立可靠支撑，再在支撑参考上谨慎地区分上层结构、观测缺失和真正障碍。**

顺着这条主线去理解，就能把下面这些设计连起来：

- 为什么同时保留 `cloud_in_base` 和 `cloud_in_odom`
- 为什么要先做 `FrameObservabilityEstimator`
- 为什么 support 和 obstacle 都用证据累积
- 为什么 `vertical_span` 只能做 suspicious trigger
- 为什么需要 `effective_support_ref`
- 为什么 `UNKNOWN` 的优先级这么高
- 为什么内部在 `odom` 建图，但最终发布为 `base_gravity` 机器人中心地图

如果后续要改算法行为，最需要确认的三个问题是：

1. 这次改动改变的是当前帧解释，还是长期地图记忆？
2. 这次改动会不会把楼梯、分层地面或墙面重新误解释成障碍？
3. 这次改动在 dropout、partial observability、短时失观测时是否仍然稳定？

只要这三件事始终盯住，基本就不会偏离当前实现的核心设计方向。
