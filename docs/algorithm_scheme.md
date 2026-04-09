# Passable Area 算法方案说明

## 1. 文档目的

本文档描述当前 `passable_area` 包的实际算法方案、软件架构、输入输出合同、坐标系定义、调试输出、运行方式、测试方式和已知边界。内容以当前仓库实现为准，目标是为后续维护、调参、联调和二次开发提供统一参考。

本文档覆盖：

- 系统整体职责和边界
- `core` 与 `interfaces/ros` 的职责划分
- 当前处理链路和各模块作用
- `odom` / `base_link` / `base_gravity` 三套坐标语义
- 当前对外 ROS 接口和调试话题语义
- 构建、测试、启动、联调建议
- 典型验证方法和性能观察点

## 2. 系统目标

`passable_area` 的目标不是做长期全局建图，而是做一个高频、局部、短时稳定的可通行区域判定器。它的核心特点是：

- 输入是严格时间同步的点云和里程计
- 内部维护一个局部连续的 `odom` 对齐地图缓冲区
- 在地图上做 support / obstacle / coverage / feature / traversability 推理
- 输出三态通行性和地形代价，并在发布阶段重表达为 `base_gravity` 机器人中心栅格
- 提供一组围绕 `base_gravity` 的调试点云，便于从机器人视角观察结果

当前实现不是纯单帧算法。它带有短时记忆和局部融合能力，主要体现在：

- `support_confidence` / `obstacle_evidence` 会跨帧累积和衰减
- 地图会随机器人位置重心平移，但不会每帧完全清空
- dropout / partial observability 会影响地图负更新和结果稳定性

因此，更准确的描述是：

> `passable_area` 是一个实时局部地形判定系统，内部使用短时局部地图和观测可信度机制来提升输出稳定性。

## 3. 架构总览

当前架构分为两层：

### 3.1 `core`

路径：

- `include/passable_area/core`
- `src/core`

职责：

- 不依赖 ROS 消息类型
- 完成点云预处理、观测性估计、候选聚合、局部地图更新、地形特征计算和通行性判定
- 提供统一的 `Processor::update(const FrameInput&) -> FrameOutput`

### 3.2 `interfaces/ros`

路径：

- `include/passable_area/interfaces/ros`
- `src/interfaces/ros`

职责：

- ROS 参数加载
- PointCloud2 / Odometry 转换
- cloud + odom ExactTime 同步
- watchdog 和 perf stats
- 发布 occupancy grid、grid map、debug clouds、observability 消息
- 发布 `odom -> base_gravity` TF

### 3.3 节点入口

当前主节点：

- `interfaces/ros/nodes/passable_area_node`

该节点是系统唯一运行时入口，负责：

- 订阅输入
- 调用 `core::Processor`
- 发布结果

## 4. 当前处理链路

每个同步后的输入帧按固定顺序处理：

1. `FramePreprocessor`
2. `FrameObservabilityEstimator`
3. `PolarFrontend`
4. `DropoutAwareMapUpdater`
5. `TerrainFeatureUpdater`
6. `TraversabilitySolver`
7. ROS 输出转换与发布

## 5. 输入输出合同

### 5.1 订阅

- `/LOC_BODY_POINTS` `sensor_msgs/msg/PointCloud2`
- `/ODOM` `nav_msgs/msg/Odometry`

这两路输入通过 `ExactTime` 严格同步。不同步时不会进入主处理链。

### 5.2 发布

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

### 5.4 `TerrainObservability` 当前字段

当前消息定义：

- `std_msgs/Header header`
- `bool frame_partial`
- `bool rear_dropout`
- `uint16 sector_count`
- `uint32 base_point_count`
- `uint32 odom_point_count`
- `uint8[] sector_states`
- `float32[] sector_coverage_confidence`

`sector_states` 编码：

- `0` Observed
- `1` PartiallyObserved
- `2` MissingByDropout

## 6. 坐标系定义

当前系统存在三套关键坐标系。

### 6.1 `base_link`

定义：

- 原始输入点云所在的机器人机体系
- 保留机器人完整姿态语义，包含 roll / pitch / yaw

用途：

- 原始观测
- 可观测性和扇区语义分析

### 6.2 `odom`

定义：

- 由外部 `/ODOM` 提供的局部连续父坐标系
- 当前代码将其视为内部建图和跨帧累积坐标系
- 用于局部地图定位、栅格索引和地形层维护

当前假设：

- `odom` 至少在高度解释上可视作重力对齐参考
- 系统不自己重建全局世界系，而是信任输入里程计

用途：

- `cloud_in_odom`
- 局部地图 `LocalTerrainMap`
- `grid_map`
- `terrain_state`
- `terrain_cost`

### 6.3 `base_gravity`

定义：

- 原点在当前机器人位置
- 平移跟随机器人
- z 轴对齐重力方向
- 仅保留 yaw
- 去掉 roll / pitch

用途：

- 调试点云的机器人中心视角可视化

当前实现中，`base_gravity` 有两部分：

- 点云数值由 `odom -> base_gravity` 显式变换得到
- 节点同时发布 `odom -> base_gravity` TF，保证 RViz 在 `odom` 固定系下也能正确显示

## 7. 坐标变换关系

### 7.1 `base_link -> odom`

在 `FramePreprocessor` 中，输入点先从 `base_link` 变换到 `odom`。

形式上：

`p_odom = R(odom<-base) * p_base + t(odom<-base)`

对应当前实现：

- 位置来自 `base_pose_in_odom.position`
- 姿态来自 `base_pose_in_odom.orientation`
- 使用完整 6DoF 变换

### 7.2 `odom -> base_gravity`

对于 debug 点云，当前实现会把 `odom` 点重新表达成机器人中心局部坐标：

1. 减去机器人在 `odom` 下的位置
2. 只按 yaw 旋回到机器人朝向
3. 不保留 roll / pitch

因此：

- 调试点云随机器人移动
- 调试点云不会随机器人俯仰侧倾而整体歪斜

### 7.3 为什么算法主处理不直接使用 `base_gravity`

因为 `base_gravity` 是 robot-centric 的，点会随机器人运动而改变坐标，不适合做跨帧地图累计。

算法主处理必须使用一个局部稳定的父系，也就是当前的 `odom`，才能保证：

- 同一环境位置在连续帧中仍能落到稳定栅格
- support / obstacle / coverage 可以跨帧累积
- dropout / persistence 有意义

## 8. Core 数据结构

### 8.1 `FrameInput`

当前关键字段：

- `stamp`
- `base_pose_in_odom`
- `input_cloud_in_base`
- `processing_enabled`

### 8.2 `ProcessedFrame`

关键字段：

- `base_pose_in_odom`
- `cloud_in_base`
- `cloud_in_odom`
- `odom_samples`

其中：

- `cloud_in_base` 用于观测性语义
- `cloud_in_odom` 和 `odom_samples` 用于建图和后续推理

### 8.3 `FrameOutput`

关键字段包括：

- 地图尺寸和分辨率
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
- `support_state`
- `base_gravity_cloud_points`
- `support_points`
- `obstacle_points`
- `unknown_points`
- `observability`

## 9. 各模块详细说明

### 9.1 `FramePreprocessor`

职责：

- 清空输出结构并复制基础位姿
- 对输入点做有限值检查
- 在 `base_link` 下按 `preprocess.body_filter.*` 剔除车体内部噪点
- 由 `base_link` 变换到 `odom`
- 按当前局部地图窗口裁剪
- 高度窗口按相对当前机器人高度解释，即对应 `base_gravity` 的 z 语义
- 按配置决定是否做 voxel downsample

当前保留：

- `cloud_in_base`：机体系点
- `cloud_in_odom`：建图系点
- `odom_samples`：同时保留 `point_in_base` 和 `point_in_odom`

其中：

- `cloud_in_base` 仍服务于观测性分析，但已经去除了车体包围盒内噪点
- `cloud_in_odom` 的 XY 裁切依赖内部局部地图窗口
- `cloud_in_odom` 的 Z 裁切使用 `relative_z = point_in_odom.z - base_pose_in_odom.position.z()`

### 9.2 `FrameObservabilityEstimator`

职责：

- 仅使用 `cloud_in_base`
- 按扇区统计覆盖情况
- 输出每个扇区的观测状态
- 输出 `frame_partial` 和 `rear_dropout`

当前状态分类：

- `Observed`
- `PartiallyObserved`
- `MissingByDropout`

### 9.3 `PolarFrontend`

职责：

- 遍历 `odom_samples`
- 以 `point_in_base` 计算扇区语义
- 以 `point_in_odom` 聚合到地图 cell
- 先按单格竖向跨度产生 suspicious obstacle，再用固定 3x3 邻域确认 obstacle
- 输出：
  - `support_candidates`
  - `obstacle_candidates`
  - `ambiguous_candidates`

设计目标：

- 把“机体观测语义”和“地图投影语义”解耦
- 避免孤立高点直接形成 obstacle candidate，要求局部最小空间支持

当前前端还会维护一个 frontend-local temporary explanation ref: `effective_support_ref`。

- 它只用于当前 cell 的 obstacle explanation
- 只影响当前 cell 的 `upper_support` / candidate explanation
- 不写回地图 `support_height`
- 不是新的 support estimate
- 不参与 `ascending_stair_support_count`

`effective_support_ref` 仅在下面这类 case 才允许高于原始 `support_ref`：

- 当前 cell 位于机器人下方
- 当前 cell 同时存在下层 support 和上层分层样本
- 当前 cell 有有效 `support_anchor`
- 上层候选与邻域/历史 anchored support 一致性更强
- 当前 case 不属于已有 stair/downstairs/upstairs ground-mix 解释

这条解释链的目的不是“过滤草”或“过滤软障碍”，而是避免镂空地面把 `support_ref` 拉到过低层后，再把上层踏面和其上的弱上部结构一起误解释成 obstacle。

### 9.4 `DropoutAwareMapUpdater`

职责：

- 更新支持面、上方障碍和覆盖相关地图层
- 维护 support / obstacle 证据的累积和衰减
- 根据观测状态决定是否允许负更新

核心思想：

- `Observed` 可以正常更新
- `PartiallyObserved` 只做保守更新
- `MissingByDropout` 禁止激进负更新

### 9.5 `TerrainFeatureUpdater`

职责：

- 仅对 dirty cells 及邻域增量更新地形特征

当前特征：

- `slope`
- `step_up`
- `step_down`
- `roughness`
- `clearance`
- `support_continuity`

### 9.6 `TraversabilitySolver`

职责：

- 基于 support / obstacle / coverage / feature 层输出三态可通行性
- 同步生成 `terrain_cost`

当前判定思想：

- 证据不足或覆盖不足优先 unknown
- 净空不足优先 impassable
- 支撑不连续且障碍明显则 impassable
- 支撑连续且特征达标则 passable
- 剩余保守处理为 unknown

## 10. 局部地图机制

`LocalTerrainMap` 是当前算法稳定性的关键。

### 10.1 作用

- 维护局部二维栅格和多层地形属性
- 跟随机器人在 `odom` 中平移重心
- 保留短时历史

### 10.2 特性

- 地图不是长期全局图
- 是一个 robot-following 的局部连续缓冲区
- 当机器人移动时，地图通过 `recenter()` 和 `shiftLayers()` 平移已有层

### 10.3 为什么需要它

因为纯单帧会明显更抖：

- 局部缺点会直接变 unknown
- support / obstacle 结论不稳定
- 低覆盖区域会在帧间剧烈翻转

局部地图让系统在高频条件下更稳，但仍保持实时近场判定属性。

## 11. 调试输出语义

### 11.1 `/terrain_debug/base_gravity_cloud`

语义：

- 当前算法实际使用的预处理后点云
- 来源是 `odom_samples`
- 每个点再转换到 `base_gravity`

用途：

- 看算法真正吃进去的点长什么样
- 看点云与 `support / obstacle / unknown` 结果是否对齐

### 11.2 `/terrain_debug/support_points`

语义：

- 落在 support cell 且与 `support_height` 接近的真实样本点
- 发布在 `base_gravity`

### 11.3 `/terrain_obstacle_points`

语义：

- 落在 `obstacle_evidence` 已足够高的 obstacle cell 内的真实样本点
- 仅发布相对该 cell support 参考面高度至少为 `obstacle_points_min_height` 的上部障碍样本，不按绝对 z 阈值解释
- 当历史 `support_height` 不可用时，使用当前帧该 cell 的 `min_z` 作为 support 参考
- 发布在 `base_gravity`

### 11.4 `/terrain_debug/unknown_mask`

语义：

- unknown cell center 转换到 `base_gravity`
- 它不是原始观测点，而是地图状态点
- 应与 robot-centric `grid_map / terrain_state / terrain_cost` 在 RViz 中直接对齐

### 11.5 `/terrain_debug/grid_map`

语义：

- 局部地图的调试输出
- 内部真值来自 `odom` 地图
- 发布阶段重表达为 `base_gravity` robot-centric 栅格
- 与 `/terrain_state` 和 `/terrain_cost` 共用同一套发布 geometry

### 11.6 `/terrain_debug/observability`

语义：

- 当前帧观测完整性和扇区状态摘要
- 不是几何点云
- 继续使用 `base_gravity` 调试 header 上下文，但不参与 grid geometry 对齐

## 12. ROS 接口层实现说明

当前 ROS 接口层负责：

- 参数声明和装配
- PointCloud2 / Odometry 转换
- `ExactTime` 同步
- watchdog
- perf stats
- 结果和 debug 输出
- TF 发布

### 12.1 同步策略

- 使用 `message_filters::Synchronizer`
- cloud 和 odom 采用 `ExactTime`
- 只有严格同步的消息对才进入处理

### 12.2 Watchdog

职责：

- 检测 cloud 是否长时间没有输入
- 检测 odom 是否长时间没有输入
- 检测同步回调是否长时间没有触发

### 12.3 性能统计

职责：

- 统计输入频率
- 统计处理耗时
- 以日志方式输出性能窗口信息

### 12.4 `base_gravity` TF

当前节点会发布：

- `odom_frame -> base_gravity_frame`

定义：

- 平移直接使用当前 `base_pose_in_odom.position`
- 旋转只保留 yaw

这保证了 debug 点云在 `odom` 固定系下也能正确显示，不会出现“点云不跟机器人下沉/抬升”的漂移问题。

## 13. 参数说明

### 13.1 基础参数

来自 `config/passable_area.yaml`：

- `map_length`
- `map_width`
- `map_resolution`
- `map_height_min`
- `map_height_max`

说明：

- `map_height_min/max` 表示相对当前机器人高度的窗口，语义对应 `base_gravity` z 轴
- 当前实现仅把 XY 裁切放在内部 `odom` 局部地图窗口上完成

### 13.2 几何参数

- `max_support_slope`
- `max_step_up`
- `max_step_down`
- `max_support_roughness`
- `min_clearance`
- `upper_min_height_above_support`
- `sub_support_leak_tolerance`
- `support_anchor_reobserve_tolerance`
- `min_neighbor_upper_support_cells`

### 13.3 观测参数

- `dropout_sector_gap_threshold`
- `min_support_confidence`
- `stale_to_unknown_time`
- `sector_count`
- `min_points_per_sector`

### 13.4 持续性参数

- `support_persistence_frames`
- `obstacle_clear_observed_decay`
- `obstacle_clear_partial_decay_scale`
- `obstacle_height_clear_threshold`

### 13.4A 镂空楼梯抑制说明

当前 `PolarFrontend` 在 obstacle 形成前额外做两步前端解释收敛：

- 如果本 cell 历史 support 有效，优先使用该 support 作为 `support_anchor`
- 仅当本 cell 历史 support 无效时，才使用 `3x3` 邻域内有效 support 的中位数作为 fallback anchor
- 当样本点低于 `support_anchor - sub_support_leak_tolerance` 时，这些点会被当作下层泄漏点，不参与本帧 `support / min_z / max_z / vertical_span / upper_support / obstacle_candidate`
- 当可疑 cell 的“上层样本”仍低于机器人、并且与邻域 support 高度对齐时，前端会把它视为楼梯相邻支撑层混叠，而不是 overhead obstacle
- 当本 cell 出现 below-robot 分层，且上层候选比下层 `support_ref` 更符合邻域/历史 anchored support 一致性时，前端只会在当前 cell explanation 中临时提升 `effective_support_ref`
- 这个 `effective_support_ref` 只参与当前 cell 的 `upper_support` / obstacle explanation，不写回地图，也不参与 `ascending_stair_support_count`

这两步都只影响 `PolarFrontend` 的当前帧解释，不改变地图主语义，地图仍保持单层 `support_height + obstacle_evidence` 设计。

### 13.5 坐标系参数

- `odom_frame`
- `base_gravity_frame`
- `body_frame`

### 13.6 预处理参数

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

### 13.7 下采样参数

- `downsample.enable`
- `downsample.voxel_size`

### 13.7 调试参数

来自 `config/debug.yaml`：

- `debug.publish_grid_map`
- `debug.publish_points`
- `debug.publish_base_gravity_cloud`
- `debug.publish_observability`

### 13.8 调试输出 topic 参数

来自 `config/debug.yaml`：

- `output.terrain_state_topic`
- `output.terrain_cost_topic`
- `output.debug_grid_map_topic`
- `output.base_gravity_cloud_topic`
- `output.support_points_topic`
- `output.obstacle_points_topic`
- `output.unknown_mask_topic`
- `output.observability_topic`

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

## 15. 测试方式

### 15.1 单元测试

执行：

```bash
colcon test --packages-select passable_area --event-handlers console_direct+
```

当前核心 gtest 覆盖了：

- 平地可通行
- 覆盖不足导致 unknown
- 斜坡通过
- 楼梯场景仍保留通行带
- 低净空导致 impassable
- rear dropout 标记与 support 保持
- rear gap 触发 dropout 扇区
- 局部空洞不立即清空稳定 support
- 稀疏覆盖导致 partial 但不误判 dropout
- sample 顺序变化不改变 PolarFrontend cell 判定
- 地图 recenter 会平移历史层
- support observed 状态不会在同一帧被 persistent 覆盖
- `base_gravity` debug 点云方向符合机器人局部重力系

### 15.2 Benchmark

可用工具：

- `benchmarks/core/benchmark_processor.cpp`
- `benchmarks/interfaces/ros/e2e_benchmark.cpp`
- `tools/offline_replay.cpp`

典型用途：

- 观察 8 万到 16 万点点云下的处理耗时
- 离线重放 bag 检查 unknown 比例和 dropout 行为
- 回归性能趋势
- workspace 级 batch benchmark 可分别输出 obstacle profile 与 timing profile

### 15.3 运行时联调检查

建议最少做以下检查：

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

## 16. 启动方式

当前 launch 文件：

- `launch/nav.launch.py`
- `launch/pass.launch.py`
- `launch/mapping.launch.py`

单节点启动常用方式：

```bash
ros2 launch passable_area nav.launch.py
```

运行前建议：

```bash
source install/setup.bash
```

## 17. RViz 调试建议

### 17.1 看机器人中心视角点云

设置：

- `Fixed Frame = base_gravity`

适合观察：

- `/terrain_debug/base_gravity_cloud`
- `/terrain_debug/support_points`
- `/terrain_obstacle_points`
- `/terrain_debug/unknown_mask`

### 17.2 看地图与机器人相对关系

设置：

- `Fixed Frame = odom`

前提：

- 节点已发布 `odom -> base_gravity` TF

适合观察：

- `grid_map`
- `terrain_state`
- `terrain_cost`
- 同时叠加 `base_gravity` debug 点云

## 18. 当前已知边界

### 18.1 `odom` 质量直接影响算法结果

系统当前默认信任外部 odom：

- 如果 odom 的 z 轴不适合做重力解释
- 或姿态明显异常

则高度、坡度、台阶、净空解释都会受影响。

### 18.2 `obstacle_points` 不是“所有障碍点”

当前它更接近：

- `obstacle_evidence` 足够高的 obstacle cell 内、相对 support 参考面达到最小高度门槛的上部样本点

因此它仍不是“所有 impassable cell 的点”，而是 obstacle 分支对应的 cell 内上部障碍样本。

### 18.3 局部地图不是全局地图

当前地图设计目标是：

- 近场
- 短时稳定
- 跟随机器人

不是长期全局建图系统。

## 19. 维护建议

后续新增功能时，建议遵守以下原则：

1. 算法主处理继续放在 `core`
2. ROS 相关逻辑继续留在 `interfaces/ros`
3. 地图类输出保持在 `odom`
4. 点云调试输出保持在 `base_gravity`
5. 新增参数时同步：
   - `config/passable_area.yaml`
   - `config/debug.yaml`
   - README / docs

其中：

- 算法、建图、预处理参数放在 `config/passable_area.yaml`
- debug 开关和 debug 输出 topic 放在 `config/debug.yaml`
6. 新增重要行为时同步补：
   - gtest
   - 运行验证步骤
   - RViz 观察说明

## 20. 推荐的日常验证流程

每次修改后建议最少执行：

```bash
colcon build --packages-select passable_area --symlink-install
source install/setup.bash
colcon test --packages-select passable_area --event-handlers console_direct+
ros2 launch passable_area nav.launch.py
```

运行时建议重点观察：

- `/terrain_state`
- `/terrain_cost`
- `/terrain_debug/grid_map`
- `/terrain_debug/base_gravity_cloud`
- `/terrain_debug/support_points`
- `/terrain_obstacle_points`
- `/terrain_debug/unknown_mask`
- `/terrain_debug/observability`
- `odom -> base_gravity` TF

## 21. 一句话总结

当前 `passable_area` 的核心方案可以概括为：

> 输入点云在 `base_link` 下进入系统，先变换到内部 `odom` 建图系完成局部地图推理，再把关键 debug 点云重新表达为机器人中心的 `base_gravity`，最终输出实时、稳定、可解释的局部可通行区域判定结果。
