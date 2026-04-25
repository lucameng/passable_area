# `passable_area` V2 综合权威审计与修改计划

## 1. 审计范围与结论

本报告基于以下材料重新审计当前工作区实现：

- `docs/deep-research-report.md`
- `docs/v2_audit_report.md`
- `docs/v2_architecture_guide.md`
- `docs/algorithm_scheme.md`
- 当前源码主链路：`PolarFrontend -> DropoutAwareMapUpdater -> TerrainFeatureUpdater -> ObstacleReasoner -> TraversabilitySolver -> Processor::buildOutput`
- 当前 `HEAD`：`b171195 fix: ground protrusion_blocking in physical height only`

结论很明确：V2 的架构方向是正确的，但当前实现仍不能按“四足机器人真实导航可放行”的标准签字。主链路已经从 V1 式补丁逻辑收敛到证据、短时地图、Reasoner、Solver 的单向结构，`TraversabilitySolver` 也基本完成了“UNKNOWN gate + 信任 Reasoner”的职责收缩。但是，外部障碍点云发布合同、dropout 坐标语义、参数不变量和测试闭环仍存在硬缺口。

必须先澄清一个关键语义：**“低于机器人可达地面表面的点不得发布”不是要求恢复全局 `base_gravity.z >= -max_step_down` floor clamp。** `b171195` 的提交说明已经明确：旧 floor clamp 是为弥补 `continuity < 0.3` 误把下行楼梯立面/低位地面 alias 成障碍而引入的补偿性补丁；最新语义是用 `height > max_step_up` 的物理闭合规则决定 protrusion 是否 blocking。本文档据此修正旧审计口径：不能再把“缺少全局 floor clamp”本身列为 bug。仍需审计的是发布路径是否完全继承这条最新物理语义。

### 1.1 当前状态判断

架构状态：

| 领域 | 状态 | 审计结论 |
| --- | --- | --- |
| 主处理链路 | 基本成型 | `core` 与 ROS wrapper 分层清楚，单向管线可审计。 |
| Reasoner 主判定 | 部分达标 | `block_reason` 已集中到 `ObstacleReasoner`，但阈值和发布合同仍不统一。 |
| Solver 职责 | 基本达标 | Solver 不再重复 slope/step/roughness/clearance 阈值判定，只做 UNKNOWN gate 和三态合成。 |
| 外部障碍点云 | 未达标 | `/terrain_obstacle_points` 仍由 `Processor` 独立判定，不是 `obstacle_point_publish_status` 的函数。 |
| 低于可达地面过滤 | 部分达标 | `b171195` 已在 Reasoner 层移除 floor clamp 依赖，改用 `height > max_step_up`；但发布层仍有独立 evidence 旁路，需要证明不会绕过该语义。 |
| dropout 鲁棒性 | 未达标 | rear bridge 坐标缓存错误，front dropout 没有等价语义，map updater 扇区映射缺 yaw。 |
| 参数契约 | 未达标 | 多个硬编码常数和死参数仍影响行为。 |
| 测试覆盖 | 不足 | 现有 gtest 锁住了一些行为，但缺少跨合同一致性、坐标运动、`b171195` 端到端不变量和 bag 级回归。 |

### 1.2 已解决或不再成立的早期担忧

- 旧 V1 的 `support_anchor_used`、`explanation_decision`、neighbor reject 类字段已经不在当前 `FrontendOutput` / `FrameOutput` 主合同中，不能再作为当前失效主因分析。
- `TraversabilitySolver` 里重复执行几何阈值判定的旧问题已经解决；当前 Solver 只根据 `block_reason` 决定阻挡。
- 早期关于 `support_continuity < 0.3` 直接参与 `ObstacleReasoner` protrusion 分类的担忧在当前 `HEAD` 已过时。当前 `protrusion_blocking` 只依赖 `protrusion_evidence > 0.4f && tall_protrusion`，不再用 continuity 抑制高墙。
- “连续支撑把高墙压成低净空”的墙体碎裂问题，在有有限 `support_height` 且 `protrusion_height - support_height > max_step_up` 的场景下，已由 `tall_protrusion` 方向修正。

### 1.3 仍真实存在的风险分类

正确性问题：

- `/terrain_obstacle_points` 与 `block_reason` / `obstacle_point_publish_status` 不一致；这也是 `b171195` 端到端语义的主要剩余风险，因为低位地面不发布语义取决于发布层是否会绕过 `height > max_step_up` 规则。
- `ObstacleReasoner` 的 protrusion 阻挡阈值硬编码为 `0.4f`，与配置语义和发布阈值脱钩。
- `DropoutAwareMapUpdater` 应用扇区状态时缺少 base yaw，机器人转向后 coverage/dropout 衰减可能作用到错误方向。

鲁棒性问题：

- rear-dropout bridge 缓存的是上一帧 `base_gravity` 点，不是可重投影的 map 点或源样本。
- 当前 observability 只显式建模 rear dropout，front LiDAR 掉线没有对称保守语义。
- 前端 cell-center sector 代表角存在 aliasing；同一 cell 混入不同传感器方向样本时，只继承一个中心角扇区。
- `support_band.bottom` 与 `fallback_support_ref_by_cell = min_z` 具有明确偏置，在多层地形和楼梯下视场景中会放大高度门风险。

可维护性问题：

- 隐藏常数仍较多：`0.75f`、`1.5f`、`0.4f`、`0.15f`、`0.92f`、`0.85f`、`0.03f` 等未形成稳定物理合同。
- `obstacle_clear_partial_decay_scale` 已声明、配置和文档化，但当前核心衰减公式未使用它。
- 发布高度门逻辑在 `Processor` 和 `miss_obstacle_analyzer` 中重复，且分析器文本仍沿用旧 floor-clamp 时代的“base_gravity + base_link gates”表述，容易误导排查。

可观测性与调试问题：

- `obstacle_point_publish_status` 不是实际发布结果，debug layer 可能误导排查。
- `kGeometryFailure` 会让 `terrain_state` 变成 impassable，但没有对应障碍点发布路径；如果下游只消费 `/terrain_obstacle_points`，这是隐性漏检。
- false/miss analyzer 的 publish path 推断仍复制运行时规则，缺少单一源实现。

测试覆盖缺口：

- 没有硬性测试锁住 `b171195` 的端到端语义：`height <= max_step_up` 的低位/下行楼梯 alias 不得通过发布旁路进入 `/terrain_obstacle_points`。
- 没有测试 `block_reason`、`obstacle_point_publish_status`、`terrain_state`、`/terrain_obstacle_points` 的跨输出一致性。
- 没有测试 rear bridge 在机器人平移/旋转和 map recenter 后的位置正确性。
- 没有测试机器人 yaw 非零时 dropout sector 到 map cell 的映射正确性。

## 2. Issue-by-Issue 分析

### P0-1：外部障碍点云发布合同未闭合，`b171195` 端到端语义缺少强约束

问题陈述：

V2 声称 `ObstacleReasoner` 是 `block_reason` 单一权威，但 `Processor::buildOutput()` 并没有把 `reasoner_output.obstacle_point_publish_status` 当作实际发布真值。它重新调用 `HasObstaclePointPublishEvidence()`，其中包含：

- `layers.protrusion_evidence >= obstacle_points_min_evidence`
- `HasDenseNearThresholdProtrusionSource()`
- `HasLowClearanceObstaclePointBridge()`

`DenseNearThresholdProtrusionSource` 允许 `protrusion_evidence` 低于 `obstacle_points_min_evidence` 时发布点。`test_processor.cpp::DenseNearThresholdProtrusionSourcePublishesObstaclePoints` 已经把这种行为锁成测试。

这同时影响 `b171195` 的端到端语义。该提交明确接受：`Height > max_step_up -> impassable wall`，`Height <= max_step_up -> traversable terrain`，并据此消除全局 floor clamp 的必要性。Reasoner 层当前确实实现了这一方向：`protrusion_blocking = protrusion_evidence > 0.4f && tall_protrusion`，其中 `tall_protrusion` 基于 `protrusion_height - support_height > max_step_up`。但发布层仍有独立 evidence / dense-source 旁路，因此代码合同还没有证明所有 obstacle point 发布路径都服从这条物理闭合规则。

为什么重要：

导航系统通常把 `/terrain_obstacle_points` 当作外部障碍真值。若同一 cell 的 `block_reason == kNone`、`terrain_state == PASSABLE`，但点云发布了障碍点，规划器和 RViz 调试会看到互相矛盾的世界。反过来，如果 `kGeometryFailure` 让 `terrain_state` 阻挡但没有点云，消费点云的下游会漏掉内部已经认定的不可通行区域。

对 `b171195` 而言，四足机器人站在楼梯上方、坡顶或平台边缘时，前/后 LiDAR 可能看到下层地面或下行台阶立面。该提交的核心价值正是把 `height <= max_step_up` 的结构交给步态/落脚控制，而不是让感知层作为硬障碍 veto。若发布层绕过 Reasoner，仅凭 raw evidence 或 near-threshold dense source 发布点云，下游仍可能看到一堵“红点墙”，这会抵消该提交的设计收益。

涉及代码路径：

- `src/core/obstacle_reasoner.cpp`
- `src/core/processor.cpp::HasObstaclePointPublishEvidence`
- `src/core/processor.cpp::HasDenseNearThresholdProtrusionSource`
- `src/core/processor.cpp::HasLowClearanceObstaclePointBridge`
- `src/core/processor.cpp::PassesObstaclePointPublishHeightGates`
- `src/core/processor.cpp::Processor::buildOutput`
- `include/passable_area/core/types/obstacle_types.hpp`

期望语义：

必须明确一个合同：`/terrain_obstacle_points` 要么是 Reasoner / PublicationDecision 的函数，要么是独立外部合同。但如果选择后者，就不能再宣称 Reasoner 是输出单一权威。

本系统更合理的方向是前者：外部障碍点云发布必须服从同一物理闭合规则。只有 Reasoner 判定为 blocking protrusion / mixed / 明确定义的 low-clearance 发布资格时，才允许从对应 source samples 产生 `/terrain_obstacle_points`。`height <= max_step_up` 的低位结构不得因为 dense source 或 raw protrusion evidence 旁路被发布。

当前差距：

当前实现处在中间状态：Reasoner 给出 `obstacle_point_publish_status`，但运行时发布并不以它为准。`HasObstaclePointPublishEvidence()` 不直接检查 `block_reason` 是否为 `kProtrusion/kMixed`，`DenseNearThresholdProtrusionSource()` 还允许 evidence 未达发布阈值时发点。这不等于已经证明会违反 `b171195`，但它使该提交的端到端语义无法从代码合同上闭合。

推荐修复方向：

不要恢复旧的全局 floor clamp 作为默认修复方向。应把 cell 级发布决策上移为统一的 `ObstaclePublicationDecision`，包含 `block_reason`、`publish_status`、`publish_path`、`height_gate_status` 和 `requires_source_samples`。`Processor` 只负责把通过决策的 source samples 转换到 `base_gravity`。删除或吸收 `DenseNearThresholdProtrusionSource`，不要保留“点云输出层另开小灶”的旁路。`Processor`、miss analyzer、false analyzer 和测试都必须复用同一语义，避免运行时和离线诊断再次漂移。

### P0-2：`ObstacleReasoner` 的 protrusion 阻挡阈值硬编码

问题陈述：

当前 `src/core/obstacle_reasoner.cpp` 第 56-57 行：

```cpp
const bool protrusion_blocking =
    protrusion_evidence > 0.4f && tall_protrusion;
```

`0.4f` 与 `Config::obstacle_points_min_evidence` 默认值相同，但不是同一个变量。`test_obstacle_reasoner.cpp::EvidenceBelowPublishThresholdIsReportedAsGated` 甚至显式验证了当 `obstacle_points_min_evidence = 0.8f` 且 `protrusion_evidence = 0.5f` 时，`block_reason` 仍为 `kProtrusion`，只是发布状态为 `kGatedByEvidence`。

为什么重要：

如果 `0.4f` 是阻挡阈值，就必须命名为阻挡阈值并说明它和发布阈值的关系。如果它本应等于发布阈值，当前代码会在调参时产生不可预期分叉。四足机器人在 stair/side-board/low obstacle 边界上最怕这种“调 YAML 没有调到真正判定阈值”的隐式行为。

涉及代码路径：

- `src/core/obstacle_reasoner.cpp`
- `include/passable_area/core/types/config_types.hpp`
- `test/core/test_obstacle_reasoner.cpp`

期望语义：

阻挡阈值、发布阈值和 evidence accumulation 模型应有清晰关系。若为了安全允许“内部阻挡更敏感，外部点云发布更保守”，也必须显式建模，而不是用硬编码常数实现。

当前差距：

配置项名称是 `obstacle_points_min_evidence`，但 Reasoner 用固定 `0.4f` 决定 `block_reason`。这会让参数含义和行为不一致。

推荐修复方向：

优先减少自由参数：将 protrusion 阻挡阈值直接绑定到一个稳定语义，或者从 `max_step_up` 归一化证据和期望累积帧数派生。若确实需要内部阻挡阈值与点云发布阈值不同，应新增清晰命名的合同字段，并增加参数不变量测试。

### P0-3：dropout / coverage 扇区在地图更新阶段缺少 yaw 语义

问题陈述：

`PolarFrontend` 计算 cell 扇区时会用 `base_pose_in_map` 的 yaw 把 map cell 中心转回 base angle。但 `DropoutAwareMapUpdater::sector_state_for_cell()` 只用 `atan2(cell_center - map.center())`，没有 yaw 输入。`FrameObservabilityEstimator` 的扇区状态来自 `cloud_in_base`，语义在 base frame；map updater 却按 map frame 角度解释它。

为什么重要：

机器人转向 90 度后，base 前方与 map x 方向不再一致。此时 rear dropout、coverage confidence、support/obstacle 衰减可能作用到错误 cell。窄 FOV 双 LiDAR 场景下，这会直接导致旧障碍保留在错误方向，或真实 dropout 区域被按 observed 衰减。

涉及代码路径：

- `src/core/frame_observability_estimator.cpp`
- `src/core/polar_frontend.cpp`
- `src/core/mapping/dropout_aware_map_updater.cpp`
- `src/core/processor.cpp`

期望语义：

所有消费 `FrameObservability::sectors` 的模块必须使用同一坐标语义。扇区要么始终是 base frame，要么显式携带 transform 到 map 的方式。

当前差距：

同一帧内，前端和 map updater 使用不同角度语义解释同一个 sector array。

推荐修复方向：

把 base pose 或 yaw 传入 `DropoutAwareMapUpdater::update()`，或让 `PolarFrontend` 在 candidate 中携带 cell 的观测扇区/coverage 聚合结果，map updater 不再自行按 cell center 重新查扇区。更好的方向是样本级扇区聚合，而不是 cell-center 代表角。

### P0-4：rear-dropout bridge 缓存坐标不可重投影

问题陈述：

`Processor` 的 `rear_obstacle_point_cache_` 存储上一帧已经转换到 `base_gravity` 的 `CellDebugPoint`，dropout 时直接 push 到当前输出。缓存中虽有 `stamp`，但发布时没有时间约束；`source_cell` 只检查范围，没有检查 map recenter 后的物理一致性。

为什么重要：

机器人在 dropout 期间移动或转向时，上一帧 base-relative 点的位置已经失效。直接重发旧点会制造虚假障碍位置，尤其是后方避障和倒退动作中风险很高。

涉及代码路径：

- `src/core/processor.cpp::Processor::buildOutput`
- `test/core/test_processor.cpp::RearDropoutBridgesPreviousRearObstaclePointsForOneFrame`
- `test/core/test_processor.cpp::RearDropoutBridgeExpiresAfterOneFrame`

期望语义：

bridge 只能短时保持物理世界中的同一障碍位置，而不是保持上一帧屏幕坐标。

当前差距：

当前测试只验证点数一致，不验证坐标重投影。

推荐修复方向：

缓存 map 坐标点、源样本或 cell 的绝对 map 中心，并在当前帧重新投影到 `base_gravity`。加入时间 TTL、运动距离/角度阈值和 map recenter 后的有效性检查。若无法保证几何一致性，应宁可输出 UNKNOWN/保守状态，而不是发布错位障碍点。

### P1-1：front dropout 没有与 rear dropout 对称的保守语义

问题陈述：

`FrameObservabilityEstimator` 只产生 `rear_dropout`，且只把 rear gap sectors 标成 `kMissingByDropout`。前方 LiDAR 掉线或前方连续扇区空洞只会表现为 `frame_partial`，sector state 仍多为 `kObserved`。

为什么重要：

平台是前后双窄 FOV LiDAR。前方 dropout 对导航比后方 dropout 更直接：机器人可能继续朝旧 passable 区域前进，而前方实测已经缺失。

涉及代码路径：

- `src/core/frame_observability_estimator.cpp`
- `src/core/mapping/dropout_aware_map_updater.cpp`
- `src/core/traversability_solver.cpp`

期望语义：

观测性应描述所有方向的 dropout，而不是只特殊照顾后方。前方连续缺失应触发保守衰减或 UNKNOWN。

当前差距：

front dropout 不具备单独状态，无法建立明确的前向安全策略。

推荐修复方向：

把 dropout 从 `rear_dropout` 扩展为方向无关的 missing-sector 集合，并按传感器 FOV/安装位姿建模。若暂时不支持 per-sensor 模型，也应至少定义 front missing sector 的 UNKNOWN gate，不要让它长期继承旧 support。

### P1-2：低净空阻挡缺少 evidence/state 门控

问题陈述：

`ObstacleReasoner` 中 `low_clearance = isfinite(clearance) && clearance < min_clearance`。只要 `overhead_height` 有限且 `support_height` 有限，就可能阻挡；它不要求 `overhead_evidence` 高于阈值。另一方面，当 `support_height` 失效为 NaN 时，`clearance` 也失效，低净空阻挡又会突然消失。

为什么重要：

低净空是典型需要短时记忆的障碍。它既不能因为支撑面短暂丢失就消失，也不能在 overhead evidence 已衰减后仍无限阻挡。否则动态环境和窄 FOV dropout 下会出现低净空残留或误清。

涉及代码路径：

- `src/core/obstacle_reasoner.cpp`
- `src/core/terrain_feature_updater.cpp`
- `src/core/mapping/dropout_aware_map_updater.cpp`

期望语义：

低净空阻挡应由“几何 clearance 不足 + overhead evidence/state 仍有效”共同决定。支撑失效、overhead 失效、观测 dropout 下的状态迁移必须显式。

当前差距：

当前 `clearance` 同时承担几何高度差和证据有效性含义，状态不正交。

推荐修复方向：

引入明确的 overhead obstacle state 或 confidence gate。`clearance` 只表达几何量；是否阻挡由 `overhead_evidence`、`overhead_confidence`、support validity 和观测年龄共同决定。

### P1-3：支撑参考存在偏置且 fallback 不安全

问题陈述：

`PolarFrontend` 以 support band 的 10% 分位 `bottom` 作为 `support_z`。这不是必然错误，但会让 `height_above_support` 与 `clearance_gap` 系统性偏大。更严重的是，`Processor::buildOutput` 在没有历史 `support_height` 时使用当前帧 cell 内 `min_z` 作为 `fallback_support_ref_by_cell`。

为什么重要：

楼梯缝、下层地面、栅格多层混叠、远处地面投影进同一 cell 时，`min_z` 往往不是机器人可站立支撑，而是最危险的离群低点。它会人为增大 `sample.z - support_ref`，增加误发布障碍点的概率。

涉及代码路径：

- `src/core/polar_frontend.cpp`
- `src/core/processor.cpp`

期望语义：

支撑参考应代表“机器人足端可接触的局部地面表面”，而不是 cell 内最低点。

当前差距：

当前底部分位和 fallback min_z 都偏向低估支撑高度。该偏置会放大 `sample.z - support_ref`，使发布层更容易绕过 `b171195` 的“只发布物理阻挡结构”意图。

推荐修复方向：

先文档化 `support_band.bottom` 的安全取舍，并在输出发布层禁止把 `min_z` fallback 当作可达支撑。更稳健的方向是用 band 代表值、局部支撑 plane、median/trimmed quantile 或邻域可信 support 做参考；fallback 只能辅助 sample selection，不能独立授予发布资格。

### P1-4：地形几何特征仍可能对斜坡/楼梯产生边界误判

问题陈述：

`TerrainFeatureUpdater` 的 slope 使用 `atan(max_up / resolution)`，没有区分对角邻居距离。`step_up = max(neighbor_height - current_height)` 会让墙脚附近地面 cell 受到邻居高墙影响而变成 `GeometryFailure`。

为什么重要：

硬约束 1 要求楼梯和斜坡不得稳定误判为障碍。当前 3x3 max 差分特征在噪声、斜坡方向、cell aliasing 和墙脚边界上容易产生局部过估。

涉及代码路径：

- `src/core/terrain_feature_updater.cpp`
- `src/core/obstacle_reasoner.cpp`

期望语义：

斜率、台阶、粗糙度应描述 support surface 本身的可行走性，而不是把邻近实体障碍 bleed 到可站立地面 cell。

当前差距：

当前特征是简单邻域极值，缺少平面拟合、方向性和边界分离。

推荐修复方向：

用 3x3 或 5x5 支撑点拟合局部平面，并按实际邻居距离计算 slope；step_up/step_down 明确定义为“从邻居进入当前 cell”的可跨越代价。`GeometryFailure` 应限制在支撑面几何失败，不应用于替代障碍物发布。

### P2-1：参数和隐藏常数没有形成稳定合同

问题陈述：

当前代码仍有多处未命名常数：

- `PolarFrontend`: `0.75f` suspicious span，`1.5f` pure protrusion gain，`bands[0].count == 1 && bands[1].count >= 3`
- `ObstacleReasoner`: `0.4f`
- `TraversabilitySolver`: `coverage < 0.15f`，cost 权重 `0.4/0.35/0.25`
- `DropoutAwareMapUpdater`: `0.92f/0.85f/0.8f/0.4f/0.03f`
- `Processor`: dense source tolerance `obstacle_evidence_gain * 0.1f`，support debug tolerance `voxel_size * 1.5f`

同时 `obstacle_clear_partial_decay_scale` 在 config 和 ROS 参数中存在，但核心更新器未使用。

为什么重要：

系统硬约束要求参数数量最小、无魔法常数、物理可解释。隐藏常数不是“参数少”，而是“行为不可调且不可审计”。

涉及代码路径：

- `src/core/polar_frontend.cpp`
- `src/core/obstacle_reasoner.cpp`
- `src/core/traversability_solver.cpp`
- `src/core/mapping/dropout_aware_map_updater.cpp`
- `src/core/processor.cpp`
- `include/passable_area/core/types/config_types.hpp`
- `src/interfaces/ros/ros_param_loader.cpp`

期望语义：

每个阈值要么来自机器人几何、传感器噪声、地图分辨率、时间常数，要么应删除。

当前差距：

多个关键行为由经验常数决定，且没有启动期参数不变量校验。

推荐修复方向：

优先删除可由证据模型吸收的常数，而不是新增 YAML 参数。必须保留的常数要变成命名语义，并验证：

- `min_clearance > max_step_up`
- `0 <= obstacle_points_min_height <= max_step_down`
- `sector_count > 0`
- `map_resolution > 0`
- evidence gain/decay 与阈值满足可达稳态

### P2-2：离线诊断与运行时发布规则重复且漂移

问题陈述：

`miss_obstacle_analyzer.cpp` 复制了发布高度门，并且解释文本仍写“passes the publish height gates in both base_gravity and base_link”。在 `b171195` 之后，这种表述会把读者带回“恢复全局 floor clamp”的旧思路。`false_obstacle_analyzer.cpp` 也复制了 dense source 和 low-clearance bridge 推断逻辑。

为什么重要：

离线 replay 是当前系统判断 false/miss 的主要工具。如果工具和运行时不同步，bag 级结论会误导调参和修复优先级。

涉及代码路径：

- `src/tools/miss_obstacle_analyzer.cpp`
- `src/tools/false_obstacle_analyzer.cpp`
- `tools/offline_replay.cpp`
- `src/core/processor.cpp`

期望语义：

运行时发布规则和离线解释应共享同一决策函数或至少共享同一结构化 decision trace。

当前差距：

诊断工具复制逻辑，且已有文本/实现不一致。

推荐修复方向：

把发布决策抽成 core 纯函数并返回结构化 trace。Analyzer 只读取 trace，不重新推断。

## 3. 最佳实践修改计划

本节只给通用架构和算法改进方向，不建议为某个 bag 或某个 isolated failure 添加窄补丁。

### 3.1 先定义输出合同，再改代码

必须明确四个输出之间的关系：

- `block_reason`
- `passability_state`
- `obstacle_point_publish_status`
- `/terrain_obstacle_points`

推荐合同：

1. `ObstacleReasoner` 只决定 cell 是否因物理语义阻挡以及阻挡类型。
2. `ObstaclePublicationPolicy` 决定该阻挡是否需要外部点云、点云使用哪类 source samples、失败原因是什么。
3. `TraversabilitySolver` 只消费 `block_reason` 和 UNKNOWN gate。
4. `Processor::buildOutput` 不再重新推理，只执行 sample 选择、坐标转换和输出组装。

这样既保留 `terrain_state` 内部通行性，又让 `/terrain_obstacle_points` 作为外部导航合同具备可审计来源。

### 3.2 将 `b171195` 的物理闭合规则做成不可绕过的不变量

该规则不应实现为全局 `base_gravity.z` floor clamp，而应成为发布资格的统一前置条件：只有符合 Reasoner 物理阻挡语义的 cell 才能进入 obstacle point source sample 选择。建议输出以下 decision reason：

- `kReasonerNotBlocking`
- `kHeightWithinStepCapability`
- `kBelowSupportRelativeHeight`
- `kAboveBaseLinkCeiling`

所有 obstacle point 发布路径，包括 protrusion、low-clearance bridge、dense source、rear bridge，都必须通过同一个 publication decision。rear bridge 缓存点在重新投影后也必须重新检查该 decision。

### 3.3 统一观测性坐标系

观测扇区是 base frame 语义，地图 cell 是 map frame 语义。两个方向只能选一个统一：

- 方案 A：把 base yaw 传给所有消费 observability 的模块。
- 方案 B：前端按样本聚合得到每个 cell 的 sector state，map updater 直接使用 candidate 携带的观测语义。

更推荐方案 B，因为它同时解决 cell-center aliasing，避免同一 cell 里不同来源样本被一个中心角代表。

### 3.4 把 dropout 从 rear 特例升级为通用缺失扇区语义

不要只维护 `rear_dropout`。应维护：

- missing sector mask
- 每个 sector 的 coverage confidence
- 可选 per-sensor source / FOV metadata

front、rear、side dropout 都应通过同一机制影响 support persistence、obstacle decay 和 UNKNOWN gate。

### 3.5 让证据、几何和状态正交

建议将以下概念分开：

- 几何量：support height、overhead height、clearance、slope、step
- 证据量：protrusion evidence、overhead evidence、coverage confidence
- 状态量：support observed/persistent/none、obstacle active/decaying/cleared
- 输出决策：blocking reason、publish status、gate reason

当前 `clearance` 同时表达几何和低净空有效性，`obstacle_evidence` 又是兼容别名，容易产生状态漂移。正交化后，清除逻辑、dropout 保守逻辑和调试层都会更稳定。

### 3.6 简化而不是叠加特殊路径

优先删除或吸收到主语义中的逻辑：

- `DenseNearThresholdProtrusionSource`
- `fallback_support_ref_by_cell = min_z`
- 前端 `pure protrusion gain_scale = 1.5`
- 未使用的 `obstacle_clear_partial_decay_scale`

如果某条路径确实必要，应把它改成物理/统计概念，例如“sampling confidence”或“source sample availability”，而不是用 `min_points_per_sector - 1` 这类跨语义耦合。

## 4. 必须新增的单元与集成测试

### 4.1 `b171195` 端到端发布不变量

核心单元测试：

- 构造一个 cell：`support_height` 有效，`protrusion_height - support_height == max_step_up`，`protrusion_evidence` 充足，断言 `block_reason == kNone` 或非 protrusion blocking，且不得发布 obstacle point。
- 构造 `protrusion_height - support_height > max_step_up` 的相邻 case，断言 Reasoner 阻挡且发布资格按合同成立。
- 构造 dense near-threshold case，断言它不能绕过 `height <= max_step_up` 的非阻挡语义。
- 确认 protrusion、low-clearance bridge、dense source、rear bridge 都不能发布 Reasoner 判为非 blocking 的低位/下行楼梯 alias。

回归测试：

- 楼梯顶端俯看楼梯底地面时，`height <= max_step_up` 的下行结构不得进入 `/terrain_obstacle_points`；不要用全局 `base_gravity.z` clamp 作为断言依据。
- `miss_obstacle_analyzer` 与 `Processor` 对同一 ROI 的 publication decision 结论一致。

### 4.2 输出合同一致性测试

核心单元测试：

- 对每个 cell 断言：实际 `obstacle_points.source_cell` 必须对应明确的 publish decision。
- 构造 near-threshold protrusion，验证不会出现 `obstacle_point_publish_status == kGatedByEvidence` 但点云已发布，除非新增状态显式表达该路径。
- 构造 `block_reason == kNone` 的 cell，断言不能发布障碍点。
- 构造 `kGeometryFailure`，明确断言它是否应该发布点；若不发布，则必须有测试证明下游不会只靠点云漏掉该阻挡。

### 4.3 Reasoner 阈值与参数合同测试

核心单元测试：

- 将 `obstacle_points_min_evidence` 设置为 `0.3` 和 `0.8`，验证 `block_reason` 与 `publish_status` 符合文档合同。
- 测试 `protrusion_height - support_height == max_step_up` 边界。
- 测试 `support_height` 为 NaN 时 tall protrusion 的预期行为。

参数校验测试：

- `min_clearance <= max_step_up` 应启动失败或返回明确错误。
- `obstacle_points_min_height > max_step_down` 应启动失败或至少触发配置错误。
- `sector_count <= 0`、`map_resolution <= 0` 应被拒绝。

### 4.4 observability / dropout 坐标测试

核心单元测试：

- base yaw = 90 度时，front sector 缺失应作用到机器人前方，而不是 map x 方向。
- 同一个 cell 内混合来自不同 base angles 的样本，不得只按 cell center 抑制 support。
- front-only、rear-only、empty-sector 都应产生可解释的 missing sector mask。

回归测试：

- 前 LiDAR dropout 多帧时，前方旧 passable 不得长期保持为 PASSABLE；应进入 UNKNOWN 或明确保守状态。
- rear dropout 下支撑/障碍衰减速度符合 missing sector 策略。

### 4.5 rear bridge 几何正确性测试

核心单元测试：

- 源帧建立 rear obstacle，下一帧机器人平移后 rear dropout，断言 bridged points 在当前 `base_gravity` 下被重新投影。
- 同样测试 yaw 旋转。
- map recenter 后 `source_cell` 变化时，桥接仍指向同一 map 物理位置，或被安全丢弃。
- 超过 TTL 后不再 bridge。

### 4.6 地形几何与斜坡/楼梯测试

核心单元测试：

- 25 度以内斜坡沿 x、y、对角方向都不得稳定 `kGeometryFailure`。
- 连续楼梯踏面和立面不得在踏面区域形成稳定 false obstacle。
- 墙脚相邻地面 cell 不应因邻居高差产生过宽 impassable bleeding。

边界测试：

- `max_step_up`、`max_step_down`、`max_support_slope_deg` 的等号边界必须固定。
- 噪声小于 voxel / profile split gap 的单层地面不得被切成多层障碍。

### 4.7 evidence decay 和死参数测试

核心单元测试：

- 修改 `obstacle_clear_partial_decay_scale` 后，partial observed 场景的 obstacle evidence 衰减必须产生可测差异；如果设计决定不用该参数，则删除参数和文档。
- overhead evidence 衰减到阈值以下后，low-clearance block 应按状态机预期降级。
- support 暂时失效时，低净空障碍不得无条件消失，也不得无限残留。

### 4.8 ROS 接口与 offline replay 测试

ROS 接口测试：

- `grid_map` 中 `block_reason`、`obstacle_point_publish_status`、`protrusion_evidence`、`overhead_evidence` 与 core output 对齐。
- `/terrain_obstacle_points` 的 source cell 必须对应明确 publication decision；`height <= max_step_up` 的低位/下行楼梯 alias 不得发布。
- ExactTime 输入丢帧时 observability 输出不会伪造 observed。

offline replay 回归：

- frozen stairs/slopes bags：持续 false positive = 0。
- known miss bags：不因删除 dense bypass 退化为持续漏检；若有漏检，root cause 必须可由 evidence/source sample 指标解释。
- dropout bags：前/后 dropout 下 obstacle cloud 不闪烁、不错位。

## 5. 实施后的验证流程

### 5.1 必需构建与单测

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select passable_area --symlink-install
source install/setup.bash
colcon test --packages-select passable_area --event-handlers console_direct+
colcon test-result --verbose
```

最低通过标准：

- 所有 core gtest 通过。
- 所有 ROS interface gtest 通过。
- 新增的 `b171195` 端到端发布不变量测试必须通过。
- 新增的输出合同一致性测试必须通过。

### 5.2 offline replay 建议命令

以下 bag 路径按当前工作区常见验证路径列出，若本机不存在，应替换为等价 frozen validation bags。

false obstacle 分析：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/passable_area/passable_area_offline_replay \
  --analyze-false-obstacles \
  --bag /home/deep/deeprobotics/bags/offline_bags/rosbag2_mtbf_upslope_and_downstair \
  --params-file src/passable_area/config/passable_area.yaml \
  --range-x-min 0.0 --range-x-max 1.4 \
  --range-y-min -0.25 --range-y-max 0.25 \
  --top-k 5 --no-color
```

楼梯/下层地面重点验证：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/passable_area/passable_area_offline_replay \
  --analyze-false-obstacles \
  --bag /home/deep/deeprobotics/bags/test_result/rosbag2_open_up_down_stairs \
  --params-file src/passable_area/config/passable_area.yaml \
  --range-x-min 0.0 --range-x-max 2.7 \
  --range-y-min -0.6 --range-y-max 0.6 \
  --top-k 10 --no-color
```

miss obstacle 分析：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/passable_area/passable_area_offline_replay \
  --analyze-missed-obstacles \
  --bag <known_miss_obstacle_bag> \
  --params-file src/passable_area/config/passable_area.yaml \
  --roi-x-min <x_min> --roi-x-max <x_max> \
  --roi-y-min <y_min> --roi-y-max <y_max> \
  --top-k-cells 10 --no-color
```

timing 验证：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/passable_area/passable_area_offline_replay \
  --benchmark-timing \
  --bag <representative_high_density_bag> \
  --params-file src/passable_area/config/passable_area.yaml
```

### 5.3 必看指标

正确性指标：

- `/terrain_obstacle_points` 不得来自 `block_reason == kNone` 或 `height <= max_step_up` 的 protrusion source cell。
- frozen stairs/slopes bags 上 sustained false positive 必须为 0。
- `block_reason == kNone` 的 source cell 不应产生 obstacle points。
- `obstacle_point_publish_status` 必须与实际发布路径一致。

鲁棒性指标：

- rear dropout bridge 后的点与 map 物理位置重投影误差应小于一个 grid resolution。
- front dropout 后前方 passable 不得无界保持；应按合同进入 UNKNOWN 或保守状态。
- map recenter 后 obstacle cache 不得投到错误 cell。

性能指标：

- 80k-160k 点输入下，核心算法路径平均频率 >= 10 Hz。
- 单核 CPU budget 内 p95/p99 不应因新增合同明显退化。
- 如果引入平面拟合或 sample-sector 聚合，应比较修改前后 `offline_replay --benchmark-timing`。

### 5.4 RViz / topic 检查

必须检查以下输出：

- `/terrain_state`
- `/terrain_cost`
- `/terrain_obstacle_points`
- `/terrain_debug/grid_map`
- `/terrain_debug/support_points`
- `/terrain_debug/unknown_mask`
- `/terrain_debug/base_gravity_cloud`
- `/terrain_debug/observability`

重点 debug layers：

- `block_reason`
- `obstacle_point_publish_status`
- `protrusion_evidence`
- `overhead_evidence`
- `support_confidence`
- `coverage_confidence`
- `clearance`
- `support_continuity`

预期现象：

- 楼梯底地面不应出现红色 obstacle points。
- 楼梯踏面和可走斜坡不应稳定显示 impassable。
- dropout 区域应表现为 UNKNOWN 或短时保守保持，不应错位发布旧点。
- `obstacle_point_publish_status` 与实际红点来源一致。

### 5.5 如何确认不是只修一个 bag

修改后必须跨场景验证：

- 上楼梯、下楼梯、坡道、长走廊、室内窄门、动态障碍清除、前/后 LiDAR dropout。
- 同一规则在不同 `map_resolution`、不同 `voxel_size`、不同 yaw 下行为一致。
- 对每个失败样本，不只看最终红点数量，还要看 source cell 的 `block_reason`、publish decision、height gate reason 和 evidence。
- 如果某项修复只改善一个 ROI，但导致其他 frozen false bags 出现持续红点，应判定为失败。

## 6. 最终优先级

P0 必须先做：

1. 统一 `/terrain_obstacle_points` 与 Reasoner / publish status 合同，把 `b171195` 的 `height > max_step_up` 物理闭合规则实现为不可绕过的 publication decision，并删除或吸收 dense near-threshold 旁路。
2. 去掉 `ObstacleReasoner` protrusion 阈值硬编码，建立阻挡阈值与发布阈值的明确关系。
3. 修复 observability sector 到 map cell 的 yaw 语义不一致。
4. 修复 rear-dropout bridge 的坐标缓存与重投影。

P1 应紧随其后：

1. 扩展 front/rear 对称 dropout 语义。
2. 正交化 low-clearance 几何、evidence 和状态。
3. 替换不安全的 `min_z` fallback 支撑参考。
4. 改进 slope/step 几何特征，避免斜坡/楼梯边界 false positive。
5. 补齐跨输出一致性、dropout、`b171195` 端到端语义和 bag 回归测试。

P2 可作为清理：

1. 清理未使用参数和隐藏常数。
2. 统一 offline analyzer 与运行时决策 trace。
3. 更新文档，把旧 V1 字段和过时解释从当前算法说明中移除。

最终判断：**V2 架构可以继续演进，但当前实现的发布合同和 dropout 坐标语义还没有达到真实机器人导航系统应有的严谨程度。下一轮工作不应继续叠加局部启发式，而应先把“物理语义、输出合同、观测坐标、测试不变量”四件事统一。**
