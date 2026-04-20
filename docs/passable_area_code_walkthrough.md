# passable_area 代码解读：

## 1. 先用一句话说清这套模块在干什么

`passable_area` 不是一个“把点云投到地面上做高度阈值判断”的小工具，也不是一个长期全局建图模块。

它更像是一个**高频、局部、带短时记忆的地形可通行判定器**：

- 输入是一帧严格同步的 `PointCloud2` 和 `Odometry`
- 内部在 `map` 坐标系里维护一个随机器人平移的局部栅格地图
- 每一帧会尝试从点云里恢复“支撑面 / 上方结构 / 可观测性 / 障碍证据”
- 再把每个栅格判成三态：
  - `PASSABLE`
  - `IMPASSABLE`
  - `UNKNOWN`

它要解决的核心问题是：

- 单帧点云里，最低点不一定就是“可踩的地面”
- 同一个栅格可能同时出现下层、上层、墙面、楼梯边缘、悬空障碍等混合结构
- 传感器后方可能掉点、局部区域可能半观测，如果直接负更新，很容易把地图刷坏
- 机器人运行时需要的是**局部、实时、相对稳定**的可通行结果，而不是每帧都剧烈跳变的即时判断

这也是为什么代码里会出现这些关键词：

- `support`：把“当前格子的支撑参考面”单独建模
- `overhead / obstacle`：把上方结构和障碍证据单独建模
- `observability`：把“这帧到底看清楚了没有”单独建模
- `persistence`：把短时历史保留下来，提高稳定性
- `UNKNOWN`：宁可保守未知，也不随便判 passable

---

## 2. 模块整体输入输出是什么

### 2.1 输入

运行时入口在：

- `src/passable_area/src/main/passable_area_node_main.cpp`
- `src/passable_area/src/interfaces/ros/passable_area_node.cpp`

节点类是 `passable_area::interfaces::ros::PassableAreaNode`。

它订阅两路输入：

- 点云：`sensor_msgs/msg/PointCloud2`
- 里程计：`nav_msgs/msg/Odometry`

默认 topic 在 `src/passable_area/include/passable_area/interfaces/ros/ros_param_loader.hpp` 和配置文件里定义：

- 点云：`/LOC_BODY_POINTS`
- 里程计：`/ODOM`

对应配置文件：

- `src/passable_area/config/sensors.yaml`

两路消息通过 `message_filters::Synchronizer<ExactTime>` 严格同步。也就是说：

- 时间戳不严格对齐，就不会进入主处理链
- 当前实现没有 ApproximateTime 的兜底路径

### 2.2 输入坐标语义

从代码实现看，系统依赖三套坐标语义：

#### `base_link`

- 原始输入点云所在坐标系
- 保留机器人完整姿态
- 主要用于本帧观测性判断和“从机器人视角看哪个方向缺点”

#### `map`

- 外部 `/ODOM` 实际提供的局部连续坐标系；当前系统里其 `header.frame_id` 是 `map`
- 用于内部局部地图、跨帧累积、栅格索引
- `FramePreprocessor` 会把点从 `base_link` 变换到这套 `map` 语义

#### `base_gravity`

- 原点跟随机器人当前位置
- 朝向只保留 yaw，不保留 roll/pitch
- 用于对外发布调试点云和最终地图的机器人中心表达

这里有一个非常重要的点：

**内部算法主处理在 `map` 中进行，但公开发布出来的地图结果会被重采样成 `base_gravity` 机器人中心栅格。**

当前核心数据表达已经统一为：
- `base_pose_in_map`
- `cloud_in_map`
- `map_samples`

这个点和 `algorithm_scheme.md` 里的部分表述并不完全一致，后面会专门展开。

### 2.3 输出

主要输出由两部分组成。

#### 结果地图

由 `ResultPublishers` 发布：

- `terrain_state`：`nav_msgs/msg/OccupancyGrid`
- `terrain_cost`：`nav_msgs/msg/OccupancyGrid`
- `grid_map`：`grid_map_msgs/msg/GridMap`

默认 topic：

- `/terrain_state`
- `/terrain_cost`
- `/terrain_debug/grid_map`

语义：

- `terrain_state`
  - `0` = passable
  - `100` = impassable
  - `-1` = unknown
- `terrain_cost`
  - `-1` 表示未知
  - `100` 表示不可通行
  - `1~99` 表示可通行区域上的代价

#### 调试输出

由 `DebugPublishers` 发布：

- `/terrain_debug/base_gravity_cloud`
- `/terrain_debug/support_points`
- `/terrain_obstacle_points`
- `/terrain_debug/unknown_mask`
- `/terrain_debug/observability`

其中：

- `base_gravity_cloud`：把当前输入点云转成 `base_gravity` 视角后的调试点
- `support_points`：接近支撑高度的点
- `obstacle_points`：通过障碍证据和高度门控后的障碍调试点
- `unknown_mask`：当前 `UNKNOWN` 栅格的点表达
- `observability`：每个扇区的观测状态

另外节点在参数打开时会发布一个 TF：

- `map -> base_gravity`

发布逻辑在 `PassableAreaNode::publishMapToBaseGravityTransform()`。

---

## 3. 代码结构怎么分工

先不要按文件名机械罗列，先按职责看。

### 3.1 ROS 接线层：`interfaces/ros`

路径：

- `include/passable_area/interfaces/ros`
- `src/interfaces/ros`

负责：

- 加载 ROS 参数
- 订阅点云和里程计
- ExactTime 同步
- ROS 消息和内部数据结构转换
- 发布 OccupancyGrid / GridMap / 调试点云 / TF

这层不做核心地形算法，只负责“把数据接进来”和“把结果发出去”。

### 3.2 核心算法层：`core`

路径：

- `include/passable_area/core`
- `src/core`

负责完整主流水线：

1. `FramePreprocessor`
2. `FrameObservabilityEstimator`
3. `PolarFrontend`
4. `DropoutAwareMapUpdater`
5. `TerrainFeatureUpdater`
6. `TraversabilitySolver`

这条链由 `core::Processor` 串起来。

### 3.3 数据表示层：`types` + `mapping`

这里有两类非常重要的东西：

- 帧级数据：`FrameInput` / `ProcessedFrame` / `FrameOutput`
- 地图级数据：`LocalTerrainMap` + `TerrainLayers`

可以把它理解成：

- `Frame*` 负责“这一帧怎么流动”
- `TerrainLayers` 负责“地图上每个栅格长期维护哪些状态”

---

## 4. 从入口到输出，完整主链路怎么走

这一节是整份文档最重要的主线。

### 4.1 ROS 节点入口

入口非常简单，在 `src/passable_area/src/main/passable_area_node_main.cpp`：

1. `rclcpp::init`
2. 构造 `PassableAreaNode`
3. `rclcpp::spin`

真正的逻辑都在 `PassableAreaNode` 里。

### 4.2 `PassableAreaNode` 初始化时做了什么

构造函数在：

- `src/passable_area/src/interfaces/ros/passable_area_node.cpp`

初始化顺序大致是：

1. 用 `RosParamLoader` 加载参数
2. 构造 `core::Config`
3. 构造 `core::Processor`
4. 初始化结果发布器和调试发布器
5. 初始化 watchdog / perf stats
6. 建立点云订阅和 odom 订阅
7. 建立 `ExactTime` 同步器
8. 绑定同步回调 `onSynced`

### 4.3 每帧处理的真正入口：`onSynced`

主入口函数：

- `PassableAreaNode::onSynced(...)`

它做的事情很直接：

1. 把 ROS 点云转成内部 `PointCloud`
2. 把 ROS 里程计转成内部 `Pose3D`
3. 组装 `FrameInput`
4. 调用 `processor_.update(input)`
5. 如果 `FrameOutput.valid == true`，发布所有结果

这里可以把 `FrameInput` 理解成“送进核心算法的这一帧原材料”。

---

## 5. 主链路详解：从输入到输出

下面按照 `Processor::update()` 的真实顺序讲。

实现位置：

- `src/passable_area/src/core/processor.cpp`

主流程非常清晰：

```cpp
preprocessor_.process(...)
map_.recenter(...)
observability_estimator_.estimate(...)
frontend_.run(...)
map_updater_.update(...)
feature_updater_.update(...)
traversability_solver_.update(...)
buildOutput(...)
```

### 5.1 第一步：`FramePreprocessor`

文件：

- `src/passable_area/src/core/frame_preprocessor.cpp`

它做的不是复杂语义理解，而是为后续阶段准备两份“同一帧的不同表达”。

#### 它的输入

- `FrameInput`
  - 时间戳
  - `base_pose_in_map`
  - 原始点云 `input_cloud_in_base`

#### 它的输出

- `ProcessedFrame`
  - `cloud_in_base`
  - `cloud_in_map`
  - `map_samples`

#### 它做了什么

1. 过滤非法点
2. 可选机身包围盒过滤 `body_filter`
3. 用 `base_pose_in_map` 把点从 `base_link` 变换到 `map`
4. 可选按地图范围裁剪 `crop_to_map`
5. 可选体素降采样

#### 为什么会同时保留 `cloud_in_base` 和 `cloud_in_map`

这是这套实现里一个非常关键的设计：

- `cloud_in_base` 用来做观测性分析，因为“前后左右是否掉点”应该站在机器人自身视角看
- `cloud_in_map` 用来做地图更新，因为跨帧累积必须放在相对稳定的参考系里

#### `map_samples` 的作用

`map_samples` 不是多余拷贝，它把一一对应关系保留下来了：

- `point_in_base`
- `point_in_map`

后续很多调试输出会同时利用这两种表达。

### 5.2 第二步：地图重心平移 `LocalTerrainMap::recenter`

文件：

- `src/passable_area/src/core/mapping/local_terrain_map.cpp`

预处理之后，`Processor` 会先让局部地图跟着机器人位置移动：

```cpp
map_.recenter(preprocessed.base_pose_in_map.position.head<2>())
```

这个地图的关键特点是：

- 尺寸固定，由 `map_length / map_width / resolution` 决定
- 中心会跟着机器人在 `odom` 中的位置移动
- 但不是每帧清空，而是做“栅格平移”

也就是说：

- 机器人如果只移动了几个格子，已有地图层会整体搬移过去
- 超出边界的新区域才会被清空

这是它能做“短时稳定局部融合”的基础。

#### `LocalTerrainMap` 里维护了什么

真正的数据都在 `TerrainLayers` 里，例如：

- `support_height`
- `support_confidence`
- `overhead_height`
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

它本质上就是一个“按 cell 存很多并行 layer 的二维局部地图”。

### 5.3 第三步：`FrameObservabilityEstimator`

文件：

- `src/passable_area/src/core/frame_observability_estimator.cpp`

这一步只看 `cloud_in_base`，不看地图。

#### 它在干什么

它把机器人周围 360 度分成若干扇区（默认 72 个），统计每个扇区的点数，然后判断：

- 这个扇区是否观测充分
- 是部分观测还是明显掉点
- 整帧是否是 partial frame
- 后方是否出现 rear dropout

#### 判断逻辑

每个扇区会得到：

- `coverage_confidence`
- `state`

状态枚举定义在 `state_types.hpp`：

- `kObserved`
- `kPartiallyObserved`
- `kMissingByDropout`

其中：

- 点数为 0 的扇区，先记成 `PartiallyObserved`
- 如果后方整体点数明显少于前方，且出现连续后向空扇区超过阈值，就把那段后向空扇区升级成 `MissingByDropout`

#### 这一步为什么重要

因为后面地图更新时，不同可观测性会决定：

- 支撑置信度怎么衰减
- 障碍证据怎么衰减
- 某些 support 候选要不要采信

也就是说，这套代码明确区分了：

- “我看到了这里没有障碍”
- “我其实没看清”

这对掉点、遮挡、后视盲区非常关键。

### 5.4 第四步：`PolarFrontend`

文件：

- `src/passable_area/src/core/polar_frontend.cpp`

当前版本已经把前端回退到接近 `f92759c` 的简单语义。

如果只记一句话，可以记成：

**`PolarFrontend` 现在只负责把“当前帧每个 cell 的原始高度分布”翻译成 support / obstacle / ambiguous 三类单帧候选，不再承担复杂的历史解释。**

#### 5.4.1 它先按 cell 聚合点

对于每个 `map_sample`：

1. 根据 `point_in_map.x/y` 找到地图 cell
2. 更新这个 cell 的 `min_z / max_z / count`

这一步结束后，每个有样本的 cell 只有一份非常直接的统计：

- `min_z`
- `max_z`
- `count`
- `vertical_span = max_z - min_z`

当前前端不再在这一层做样本级过滤、anchor 借用、邻域锚点共识或 explanation-only 支撑抬高。

#### 5.4.2 support candidate 怎么来

对每个有样本、且该扇区不是 `MissingByDropout` 的 cell：

- 直接把当前帧 `min_z` 作为 `support_height`
- 生成一个 `SupportCandidate`

因此当前前端里的 support 候选是纯单帧、纯观测驱动的：

- 不依赖历史 `support_height`
- 不依赖 `support_anchor`
- 不引入 `effective_support_ref`

历史支撑的保留、衰减和状态转换都留给后面的 `DropoutAwareMapUpdater` 处理。

#### 5.4.3 obstacle candidate 怎么来

当前障碍形成规则也回到了直接判定：

```cpp
vertical_span > max_step_up * 0.75f
```

只要当前 cell 的原始高度跨度超过这个阈值，就会：

- 标记 `obstacle_local_triggered = 1`
- 标记 `obstacle_suspicious = 1`
- 标记 `obstacle_candidate_cell = 1`
- 生成一个 `ObstacleCandidate`

也就是说，当前版本里：

- `obstacle_suspicious`
- `obstacle_candidate_cell`
- `obstacle_upper_patch_confirmed`

三者本质上已经合并成同一阶段的兼容性表达，不再是多级门控链。

#### 5.4.4 `upper_support_cell` 和 explanation 字段现在是什么地位

当前实现仍然保留了很多历史调试字段，是为了不打破：

- `FrameOutput`
- offline replay
- false / miss analyzer
- 已有 ROS debug 输出契约

但这些字段在当前简单前端里大多只是兼容位：

- `raw_upper_support_cell = 0`
- `explanation_adjusted_upper_support_cell = 0`
- `upper_support_cell = 0`
- `obstacle_rejected_by_neighbor_support = 0`
- `neighbor_upper_support_count = 0`
- `aligned_neighbor_support_count = 0`
- `explanation_decision = None`

因此现在看到这些字段，不应该再把它们理解成“当前主判定链正在经过这些分支”。

#### 5.4.5 `ambiguous_candidates` 在当前实现里的位置

这部分仍然保留，但语义非常简单：

- cell 所在扇区是 `PartiallyObserved`
- 当前 cell 本身没有触发 obstacle candidate

满足这两个条件时，前端会生成一个 `AmbiguousCandidate`。

它仍然不是地图更新主链的核心输入，但保留这个输出可以继续表达：

- 这里不是完全看清
- 这里也不是前端明确障碍
- 它更像“观测不充分下的待定区域”

#### 5.4.6 当前复杂度被主动收回到了哪里

这次前端回退的核心边界是：

- `PolarFrontend` 只做单帧、per-cell、高度跨度解释
- `Processor` 继续保留当前处理链编排
- 地图记忆、证据累计、通行性求解都不回退
- `/terrain_obstacle_points` 仍然保留当前两个发布门槛

所以现在维护这层代码时，首先要把它当成：

**“当前帧候选生成器”，而不是“带大量场景特化解释的前端专家系统”。**

### 5.5 第五步：`DropoutAwareMapUpdater`

文件：

- `src/passable_area/src/core/mapping/dropout_aware_map_updater.cpp`

这一层负责把前端输出真正写进地图，并处理跨帧记忆和衰减。

可以把它理解成：

**前端负责“这一帧怎么看”，MapUpdater 负责“地图该怎么记”。**

#### 5.5.1 support 怎么更新

对每个 `SupportCandidate`：

- 直接写 `support_height`
- 增加 `support_confidence`
- 更新 `coverage_confidence`
- 把 `support_state` 设成 `Observed`
- 更新 `last_observed_age`
- 如果该扇区是 `Observed`，还会把 `last_reliable_age` 清零

#### 5.5.2 obstacle 怎么更新

对每个 `ObstacleCandidate`：

- 写 `overhead_height`
- 增加 `overhead_confidence`
- 增加 `obstacle_evidence`
- 更新 `coverage_confidence`

注意这里不是简单布尔值，而是证据累计。

#### 5.5.3 为什么叫 dropout-aware

因为它对不同观测状态的衰减策略不一样。

对于没被本帧 touch 到的 cell：

- `Observed`：可以更积极地衰减
- `PartiallyObserved`：衰减较弱
- `MissingByDropout`：几乎不衰减

这就避免了“传感器掉点导致地图被负更新清空”的问题。

#### 5.5.4 support persistence 是怎么实现的

如果某个 cell 本帧没有新的 support，但：

- `support_confidence` 还够高
- `last_reliable_age` 还没超过 `support_persistence_frames`

那么它会把 `support_state` 维持成 `Persistent`。

如果置信度太低或太久没可靠重观测：

- `support_state` 变成 `None`
- `support_height` 清空

所以这套地图不是“有支撑就一直留着”，而是明确带时效。

#### 5.5.5 哪些情况会主动清除旧障碍

如果某个 cell：

- 本帧经过“抬高有效支撑参考”重新解释
- 或者本帧被显式判成“障碍被邻域支撑否决”

那就会把旧的 `obstacle_evidence / overhead_*` 清掉。

这一步很关键，因为它保证“被重新解释成楼梯/层状支撑”的格子，旧障碍记忆不会一直残留。

### 5.6 第六步：`TerrainFeatureUpdater`

文件：

- `src/passable_area/src/core/terrain_feature_updater.cpp`

这一步只做几何特征更新，不做通行性决策。

它对 dirty cell 及其 8 邻域重新计算：

- `step_up`
- `step_down`
- `roughness`
- `slope`
- `clearance`
- `support_continuity`

#### 每个特征的直观意义

- `step_up`：周围邻居比当前格子高多少
- `step_down`：周围邻居比当前格子低多少
- `roughness`：邻居高度差的均方根
- `slope`：用最大上升量近似坡度
- `clearance`：上方障碍高度减去支撑高度
- `support_continuity`：邻居支撑连续程度，乘上当前 `support_confidence`

这里没有做复杂曲面拟合，而是采用非常直接、便宜的 3x3 邻域统计。

这是一个明显的实时性取舍。

### 5.7 第七步：`TraversabilitySolver`

文件：

- `src/passable_area/src/core/traversability_solver.cpp`

这一步负责把地图层转成最终的：

- `passability_state`
- `traversal_cost`

#### 先判 `UNKNOWN`

只要有任何一种情况不满足，就优先判未知：

- `coverage_confidence < 0.15`
- `support_state == None`
- `support_confidence < min_support_confidence`
- `last_reliable_age` 超过按时间换算出的 stale 阈值

也就是说，这个模块明显偏保守：

- 先确认“有可靠支撑”
- 再谈 passable / impassable

#### 再判 `IMPASSABLE`

典型条件包括：

- `clearance < min_clearance`
- `support_continuity < 0.3 && obstacle_evidence > 0.4`

#### 满足几何约束才判 `PASSABLE`

需要同时满足：

- `slope <= max_support_slope_deg`
- `step_up <= max_step_up`
- `step_down <= max_step_down`
- `roughness <= max_support_roughness`
- `clearance` 充足

#### traversal cost 怎么算

对于 `PASSABLE` 的格子，再根据：

- `slope`
- `step_up/step_down`
- `roughness`

按加权比例算出 `1~99` 的代价。

所以当前实现不是基于 BFS 或连通扩张来算通行性，它是：

**先逐栅格判三态，再给可通行格一个几何代价。**

如果你是带着“哪里做了 BFS / reachable expansion”的预期来读代码，这里要及时修正认知：

**当前版本没有 BFS 主链。**

### 5.8 第八步：`Processor::buildOutput`

文件：

- `src/passable_area/src/core/processor.cpp`

这一步把内部地图层和调试信息打包成 `FrameOutput`。

这里做了几件容易被忽略、但对理解输出很重要的事。

#### 5.8.1 直接把地图层拷到输出

例如：

- `passability`
- `traversal_cost`
- `support_height`
- `overhead_height`
- `support_confidence`
- `obstacle_evidence`

#### 5.8.2 生成调试点云

`buildOutput()` 会从 `map_samples` 和地图层里生成：

- `base_gravity_cloud_points`
- `support_points`
- `obstacle_points`
- `unknown_points`

这些点最终都是转换到 `base_gravity` 表达后发布的。

#### 5.8.3 调试障碍点不是所有高点都会发

障碍调试点还有额外门控：

- `obstacle_evidence` 必须超过阈值
- 点相对支撑面高度要够高
- 但在 `base_link` 里 z 又不能太高

也就是说，它更偏向发布“靠近地面、对机器人实际行进更相关的障碍点”，而不是所有上方结构。

---

## 6. 输出结果是怎么从内部地图变成 ROS 消息的

这一层在：

- `src/passable_area/src/interfaces/ros/converters/output_converter.cpp`

### 6.1 一个非常关键的事实：最终地图会重采样成机器人中心地图

`OutputConverter::toMapOutputs()` 不会直接把内部 `map` 栅格原样发布。

它会：

1. 以 `FrameOutput.base_pose_in_map` 为当前机器人位姿
2. 构造一个以机器人为中心、朝向跟随 yaw 的 `base_gravity` 栅格
3. 对这个机器人中心栅格的每个 cell，反查内部 `map` 地图对应的 source cell
4. 重采样 `passability / traversal_cost / 各种 debug layer`

因此：

- 内部地图是 `map` 语义
- 对外 `OccupancyGrid` / `GridMap` 是 `base_gravity` 语义

### 6.2 这和 `algorithm_scheme.md` 的差异

`algorithm_scheme.md` 里把内部地图语义解释得很清楚，但对对外地图的表述更容易让人以为：

- `terrain_state`
- `terrain_cost`
- `grid_map`

仍然直接在内部连续地图坐标系下发布。

**实际代码不是这样。**

从 `PassableAreaNode::onSynced()` 可以看到，发布时传给 `result_publishers_` 和 `debug_publishers_` 的 header 都是 `base_gravity_header`，而且 `OutputConverter` 也显式做了 robot-centric resampling。

甚至 `onSynced()` 里还创建了一个 `map_header`，但当前代码并没有使用它。

所以如果你在 RViz 里看结果，应该按下面理解：

- 内部计算和累积：`map`
- 对外地图表达：`base_gravity`
- TF：参数打开时发布 `map -> base_gravity`

### 6.3 `GridMap` 里有哪些 layer

当前 `OutputConverter` 会发布这些主要 layer：

- `support_height`
- `support_confidence`
- `overhead_height`
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
- `passability`

如果想做调参或问题定位，`grid_map` 是最值得看的输出。

---

## 7. 核心数据结构、关键类、关键函数

这一节只挑真正影响主线理解的内容。

### 7.1 `FrameInput`

文件：

- `include/passable_area/core/types/frame_types.hpp`

作用：

- 表示送入 `Processor` 的一帧原始输入

关键字段：

- `stamp`
- `base_pose_in_map`
- `input_cloud_in_base`
- `processing_enabled`

说明：

- `processing_enabled` 是预留开关，当前 ROS 节点没有动态使用它

### 7.2 `ProcessedFrame`

作用：

- 表示完成预处理后的帧数据

关键字段：

- `cloud_in_base`
- `cloud_in_map`
- `map_samples`

理解重点：

- 这是“单帧前端阶段”的标准输入
- 后续观测性和前端都围绕它展开

### 7.3 `FrameObservability`

作用：

- 表达这一帧从机器人视角看，哪些方向看清了，哪些方向没看清

关键字段：

- `frame_partial`
- `rear_dropout`
- `sectors`

理解重点：

- 这是地图更新衰减策略的重要输入，不只是一个 debug 消息

### 7.4 `LocalTerrainMap`

文件：

- `include/passable_area/core/mapping/local_terrain_map.hpp`
- `src/core/mapping/local_terrain_map.cpp`

作用：

- 在 `map` 中维护一个随机器人移动的局部地图

关键能力：

- `recenter()`
- `mapToIndex()`
- `indexToMap()`
- `ageCells()`

理解重点：

- 这是跨帧记忆的载体
- 不理解它，就不理解为什么这套算法不是纯单帧

### 7.5 `TerrainLayers`

作用：

- 存地图上每个 cell 的所有 layer

理解重点：

- 所有“support / obstacle / feature / passability”最终都落在这里
- 后续很多 bug 其实都是 layer 之间的语义不一致

### 7.6 `Processor`

文件：

- `include/passable_area/core/processor.hpp`
- `src/core/processor.cpp`

作用：

- 串起整条核心流水线

如果你第一次读代码，`Processor::update()` 是最值得先看的函数。

### 7.7 `PolarFrontend::run()`

作用：

- 从当前帧 `map_samples` 里提取单帧候选

输出：

- `FrontendOutput`

关键字段：

- `support_candidates`
- `obstacle_candidates`
- `ambiguous_candidates`
- `raw_sample_min_z / raw_sample_max_z / raw_sample_count`
- `filtered_sample_min_z / filtered_sample_max_z / filtered_sample_count`
- `obstacle_suspicious`
- `obstacle_candidate_cell`

理解重点：

- 当前版本的主逻辑只有两件事：
  - 非 dropout cell 用 `min_z` 形成 support candidate
  - `vertical_span` 超阈值时直接形成 obstacle candidate
- 很多历史 debug layer 仍然存在，但主要是兼容字段，不再驱动主判定

### 7.8 `DropoutAwareMapUpdater::update()`

作用：

- 把 `FrontendOutput` 写进地图层
- 做证据累计、衰减、持久化和清理

理解重点：

- 它决定了地图是“稳”还是“抖”

### 7.9 `TerrainFeatureUpdater::update()`

作用：

- 基于支撑面更新局部几何特征

理解重点：

- 这是 `TraversabilitySolver` 的直接上游

### 7.10 `TraversabilitySolver::update()`

作用：

- 最终把各 layer 归结成 `UNKNOWN / PASSABLE / IMPASSABLE`

理解重点：

- 当前版本没有复杂图搜索，这里是逐 cell 决策

---

## 8. 读代码时最容易混淆的几个概念

### 8.1 `support_height`、`support_ref` 和历史兼容字段

这是最容易绕晕的一组。

可以这样记：

- `support_height`
  - 地图里长期保存的支撑高度
- `support_ref`
  - 当前输出里用于解释 obstacle point 发布门槛的支撑参考
  - 在当前简单前端语义下，基本就是该 cell 的当前 `min_z` 或地图层里的有效支撑高度

当前需要特别注意的是：

- `support_height` 是地图长期层
- `support_ref` 是输出/调试视角下的参考高度
- `support_anchor_*`、`effective_support_ref_*` 相关字段现在主要是兼容性遗留，不应再按旧复杂前端心智去理解

### 8.2 `upper_support_cell` 现在不是主链判定量

当前版本里它默认保持不激活。

保留它的原因不是前端还在靠它判障碍，而是：

- analyzer 还会读取这类字段
- `FrameOutput` 结构没有被缩减
- 诊断输出仍然需要稳定契约

### 8.3 `obstacle_suspicious` 不等于 `obstacle_candidate_cell`

这句话对历史版本成立，但对当前版本不再成立。

当前简单前端里：

- `obstacle_suspicious`：表示 `vertical_span` 已过阈值
- `obstacle_candidate_cell`：同一个事件的兼容性映射

也就是说，在当前实现里它们通常同时出现。

### 8.4 `Observed` / `PartiallyObserved` / `MissingByDropout`

这三个状态不是“障碍类别”，而是观测质量类别。

它们影响：

- 支撑置信度衰减
- 障碍证据衰减
- 结果稳定性

### 8.5 `support_state`

它的值在 `state_types.hpp` 里：

- `kNone`
- `kObserved`
- `kPersistent`

它不是 passability。

它表达的是：

- 当前 cell 有没有被认可的支撑面，以及这个支撑是本帧看到的还是历史保留下来的

---

## 9. 推荐的新手读码顺序

如果你第一次接手这套模块，不建议直接扎进 `polar_frontend.cpp`。

推荐顺序如下。

### 9.1 第一轮：先建立大框架

先看这几个文件：

1. `src/passable_area/docs/algorithm_scheme.md`
2. `src/passable_area/src/main/passable_area_node_main.cpp`
3. `src/passable_area/src/interfaces/ros/passable_area_node.cpp`
4. `src/passable_area/src/core/processor.cpp`

第一轮的目标不是看细节，而是回答这几个问题：

- 从哪里进来
- 主链有哪些阶段
- 哪些在 ROS 层，哪些在 core 层
- 内部地图在哪个坐标系里

### 9.2 第二轮：先看数据结构，再看算法

建议接着看：

1. `include/passable_area/core/types/basic_types.hpp`
2. `include/passable_area/core/types/frame_types.hpp`
3. `include/passable_area/core/types/state_types.hpp`
4. `include/passable_area/core/mapping/terrain_layers.hpp`
5. `src/passable_area/src/core/mapping/local_terrain_map.cpp`

这一步的目标是先建立“数据在怎么流、地图里存了什么”的认知。

### 9.3 第三轮：看前半链，理解输入是怎么变成候选的

建议顺序：

1. `src/core/frame_preprocessor.cpp`
2. `src/core/frame_observability_estimator.cpp`
3. `src/core/polar_frontend.cpp`

其中读 `polar_frontend.cpp` 时，不要一开始就试图把所有 helper 细节全记住。

先抓三件事：

- cell 内 `min_z / max_z / count` 是怎么聚合的
- 哪些 cell 会形成 support candidate
- `vertical_span` 什么时候直接形成 obstacle candidate

### 9.4 第四轮：看地图怎么记、结果怎么判

建议顺序：

1. `src/core/mapping/dropout_aware_map_updater.cpp`
2. `src/core/terrain_feature_updater.cpp`
3. `src/core/traversability_solver.cpp`

这一步的目标是回答：

- 为什么结果不会完全跟单帧抖动
- UNKNOWN 是怎么来的
- IMPASSABLE 是怎么来的

### 9.5 第五轮：看输出层

最后再看：

1. `src/interfaces/ros/converters/output_converter.cpp`
2. `src/interfaces/ros/runtime/result_publishers.cpp`
3. `src/interfaces/ros/runtime/debug_publishers.cpp`

这一步最容易发现“文档理解”和“实际发布行为”之间的偏差，尤其是坐标系。

---

## 10. 当前实现里的重要设计取舍

### 10.1 为什么它不是简单高度阈值法

如果只是简单高度阈值法，通常会默认：

- 最低点就是地面
- 高于地面某阈值就是障碍

这种做法在下面几类场景里很容易坏：

- 楼梯
- 斜坡过渡
- 分层地面
- 机器人下方还能看到更低层时
- 垂直墙面占据一个栅格，导致 min/max 跨度很大
- 后方或局部掉点

当前实现的应对方式是：

- 单独建 `support`
- 单独建 `obstacle evidence`
- 单独建 `observability`
- 让地图层负责短时记忆和稳定性
- 把前端收回到简单、直接、单帧的候选生成

### 10.2 它试图特别照顾哪些场景

从整条链路的职责分工看，当前实现仍然在意这些场景：

- 机器人脚下以下还能看到更低层地面
- 上楼梯时台阶前缘
- 垂直墙面造成的高跨度假障碍

区别在于：

- 这些场景不再通过 `PolarFrontend` 里的大量特化 helper 来解释
- 当前先回到简单障碍形成，再依赖地图更新、证据累计和离线工具去重新建立可维护基线

### 10.3 为了实时性做了哪些折中

比较明显的折中有：

- 地图是固定尺寸局部栅格，不做长期全局图
- 特征只在 dirty cell 及其小邻域更新
- 坡度、粗糙度等用 3x3 统计近似
- 没有做复杂地面拟合或全局优化
- 前端重新回到纯 per-cell 统计，避免规则栈继续膨胀

### 10.4 为了稳定性做了哪些折中

- support / obstacle 用置信度和证据累积，而不是单帧覆盖
- support 有 persistence
- dropout 区域衰减极弱
- UNKNOWN 优先级很高
- 当前版本把“复杂前端解释”整体拿掉，稳定性主要转交给地图层和发布门槛保证

### 10.5 哪些地方后续维护时最要小心

#### 不要再把复杂规则堆回前端

这次回退的目的之一，就是把前端从难以维护的规则集合重新缩回简单基线。

如果后续又把下面这些能力继续塞回 `PolarFrontend`：

- anchor 借用
- leak 抑制
- stale anchor 拒绝
- explanation-only 支撑抬高
- 邻域 upper-support gate

那复杂度会快速回到这次决定放弃维护的状态。

因此新特性默认应该优先考虑：

- 是否能放到地图层做时序稳定
- 是否能放到离线诊断里做解释
- 是否能用更少的字段表达，而不是继续叠加前端状态位

#### 输出层坐标语义很容易误解

内部是 `odom`，对外地图是 `base_gravity` 重采样。

如果调试时忘了这一点，就会误判“地图怎么在抖”或者“为什么图层和调试点对不上”。

#### 某些 debug 参数当前没有真正生效

这是一个值得明确指出的代码事实。

在参数加载里声明了：

- `debug.publish_grid_map`
- `debug.publish_points`
- `debug.publish_observability`

但从当前实现看：

- `ResultPublishers` 总会初始化并发布 `grid_map`
- `DebugPublishers` 也总会发布 `support_points / obstacle_points / unknown_mask / observability`
- 只有 `publish_base_gravity_cloud` 真的控制了 `base_gravity_cloud` 是否发布

也就是说，当前代码里这些 debug 开关和直觉不完全一致。写文档、做联调、后续改参数逻辑时都要注意。

#### `ambiguous_candidates` 当前不参与地图更新

如果以后有人看到这个结构，以为它已经进入主链，需要特别小心。当前实现里它不是核心决策输入。

---

## 11. 和 `algorithm_scheme.md` 对照时，最值得注意的差异

### 11.1 以代码为准，结果地图当前是 robot-centric 发布

这是最重要的一条差异。

`algorithm_scheme.md` 对内部连续地图的描述是对的，但如果读者顺着文档以为发布的 `terrain_state / terrain_cost / grid_map` 还是原始 `map` 栅格，那会和实际代码不符。

实际代码里：

- `PassableAreaNode::onSynced()` 发布时传的是 `base_gravity_header`
- `OutputConverter` 会做 `map -> base_gravity` 的重采样

所以这部分必须以代码为准。

### 11.2 debug 发布开关的文档语义比代码更“理想化”

文档里会让人以为 debug 发布项可以按开关细分控制，但当前代码真正按开关控制的只有 `base_gravity_cloud`。

### 11.3 当前主线没有 BFS / 可达域扩张求解

如果带着“通行区域模块通常会做可达域扩张”的经验来读，容易脑补出一条并不存在的逻辑。当前实现的主链是逐 cell 几何判定，不是搜索式扩张。

---

## 12. 总结：理解这套代码时最重要的主线是什么

如果要把整套 `passable_area` 压缩成一条最该抓住的主线，我会这样说：

**它的核心不是“从点云里找障碍”，而是“先在局部地图里建立可靠支撑，再在此基础上谨慎地区分上层结构、掉点未知和真正障碍”。**

沿着这条主线去读，很多实现细节就顺了：

- 为什么先保留 `cloud_in_base` 和 `cloud_in_map` 两种视图
- 为什么观测性要单独估计
- 为什么 support 要有 confidence / persistence
- 为什么这次把 `vertical_span` 恢复成直接障碍形成
- 为什么复杂解释不再放在前端
- 为什么 UNKNOWN 的优先级这么高
- 为什么内部地图在 `odom`，但对外结果转成 `base_gravity`

如果你第一次维护这套代码，我建议始终围绕下面三个问题来判断改动是否合理：

1. 这次改动影响的是“当前帧解释”，还是“地图长期记忆”？
2. 这次改动是不是又在把复杂场景特化重新堆回前端？
3. 这次改动在掉点、部分观测、短时丢失观测时是否还稳定？

只要这三件事始终看住，后续维护就不容易偏离当前实现的真正设计方向。
