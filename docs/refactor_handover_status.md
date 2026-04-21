# passable_area 重构交接文档

## 1. 文档目的与使用方式
这份文档是给“下一位继续做障碍判定重构的 agent / 工程师”看的交接材料。目标不是复述产品背景，而是把当前分支的真实代码状态、历史演化、踩过的坑、已经做完的收缩动作，以及下一步该怎么继续，尽量一次讲清楚。

建议把这份文档和下面几类材料配合着看：

- 总体说明：`docs/algorithm_scheme.md`
- 代码导览：`docs/passable_area_code_walkthrough.md`
- 当前测试资产：`docs/test_guide.md`
- 当前实现真值：`src/core/*.cpp`、`include/passable_area/core/*.hpp`
- 当前行为合同：`test/core/test_processor.cpp`
- 误检/漏检工具：`tools/offline_replay.cpp`、`src/tools/*analyzer.cpp`

本轮核对还发现一个实际情况：

- 你要求阅读的 `src/passable_area/AGENTS.md` 在仓库内不存在。
- 当前可见的 `AGENTS.md` 位于工作区根 `/home/deep/deeprobotics/passable_humble_ws/AGENTS.md`。
- 这份 `AGENTS.md` 对整体工程组织仍有参考价值，但其中关于 `PolarFrontend` 仍使用固定 `3x3` 邻域 gate、仍存在旧参数的描述已经落后于当前代码，后续以代码和测试为准。

这份文档里，“当前版本”指当前分支 `revert/base-line` 上、`16b43ed` 回退和 `8053800` 清理旧参数之后的状态。

更新说明（2026-04-21）：本文末尾已追加 Phase 1 / Phase 2 / Phase 3a / Phase 3b 完成状态。阅读时以后续完成状态为准；前文中关于“当前简单前端 / ObstacleCandidate / ambiguous_candidates / ObstacleReasoner 尚不存在 / Low Clearance bridge 未完成”的描述保留为重构背景和历史基线说明，不再代表 Phase 3b 后的最新 runtime 事实。

## 2. 项目当前总体状态
当前 `passable_area` 已经完成了比较大的工程化重构：主链是一个 **ROS-free core + 薄 ROS 接口** 的单节点 ROS 2 包，核心处理链是：

`FramePreprocessor -> FrameObservabilityEstimator -> PolarFrontend -> DropoutAwareMapUpdater -> TerrainFeatureUpdater -> TraversabilitySolver -> Processor::buildOutput`

当前障碍判定处于一个**有意收缩后的简单基线阶段**，不是终态：

- `PolarFrontend` 已经回退成接近 `f92759c` 时期的简单 per-cell 语义。
- 复杂的 support anchor、leak suppression、neighbor upper-support gate、explanation reject / keep、facade evidence 等旧前端规则树已经不再参与当前主链判定。
- 但 `Processor`、`DropoutAwareMapUpdater`、`TraversabilitySolver`、offline analyzers、ROS 输出合同没有整体回退，而是保留了当前框架和一部分后续能力。

已经完成的关键重构有：

- `core` / `interfaces/ros` 分层和单节点运行形态
- 外部 frame 语义从历史 `odom` 口径清理到当前 `map` 口径
- 障碍前端回退到简单 baseline
- 4 个旧前端参数彻底删除：
  - `upper_min_height_above_support`
  - `sub_support_leak_tolerance`
  - `support_anchor_reobserve_tolerance`
  - `min_neighbor_upper_support_cells`

还没开始或只停留在方案阶段的重构有：

- `ObstacleReasoner` 这个独立模块还不存在
- Ground Protrusion / Low Clearance 双链路还没有落地
- `PolarFrontend` 还没有升级成计划中的轻量双层摘要
- `FrameOutput` / analyzer / debug contract 还没有从旧 explanation 字段债务里脱身

当前最值得注意的风险点有 5 个：

1. 当前简单前端只是“基线”，表达力明显不够，不是最终设计。
2. `/terrain_obstacle_points` 是当前下游导航模块消费的唯一障碍指标，miss / false analyzer 把它当成真值是有意设计；真正的风险是内部 `passability / clearance / obstacle_evidence` 和最终 `obstacle_points` 之间还没有形成足够清晰、可审计的一致性合同。
3. `FrameOutput`、`FrontendOutput`、analyzers 仍然背着大量旧前端兼容字段。
4. 一部分文档已经更新到简单基线，但少数段落和工作区根 `AGENTS.md` 仍残留旧语义。
5. 目前 `map updater` 和 `solver` 仍把近地凸起、低净空、发布筛选混在同一套 `overhead_height / obstacle_evidence` 表达里。

## 3. 当前代码与目录结构速览
### 3.1 `docs/`
放设计说明、代码导览、测试说明和离线分析使用说明。

当前最关键的文档是：

- `docs/algorithm_scheme.md`
  - 对整体链路和术语做了较系统的整理
  - 已经写入“前端回退到简单 baseline”这件事
  - 但个别段落仍残留旧清障语义，后文会点名
- `docs/passable_area_code_walkthrough.md`
  - 适合快速建立当前代码心智模型
  - 也存在少量滞后描述
- `docs/test_guide.md`
  - 目前最接近“现状 contract 摘要”

### 3.2 `include/`
放核心类型、模块接口和 ROS 接口头文件。

关键路径：

- `include/passable_area/core/types/`
  - `Config`、`FrameInput`、`FrameOutput`、状态枚举
- `include/passable_area/core/mapping/`
  - `LocalTerrainMap`、`TerrainLayers`
- `include/passable_area/core/`
  - `polar_frontend.hpp`
  - `processor.hpp`
  - `frame_preprocessor.hpp`
  - `frame_observability_estimator.hpp`
  - `terrain_feature_updater.hpp`
  - `traversability_solver.hpp`
- `include/passable_area/tools/`
  - false / miss obstacle analyzer 的公共合同

### 3.3 `src/`
放核心实现、ROS 接口和工具实现。

本轮重构最关键的文件及职责：

- `src/core/polar_frontend.cpp`
  - 当前单帧前端基线
  - 只做 per-cell 原始统计、support / obstacle / ambiguous candidate 生成
  - 注意：`ambiguous_candidates` 是当前代码里仍存在的过渡输出，后续 V2 倾向删除
- `src/core/processor.cpp`
  - 串主链，组织 `FrameOutput`
  - 负责 `support_points` / `obstacle_points` / `unknown_points` 的最终生成
- `src/core/mapping/dropout_aware_map_updater.cpp`
  - 维护 support / obstacle 证据的累计、衰减、dropout-aware 记忆
- `src/core/terrain_feature_updater.cpp`
  - 计算 slope / step / roughness / clearance / support_continuity
- `src/core/traversability_solver.cpp`
  - 把 coverage / support / clearance / obstacle_evidence / geometry 合并成三态通行性
- `src/interfaces/ros/ros_param_loader.cpp`
  - 当前 YAML 参数和 `Config` 的映射真值
- `src/interfaces/ros/converters/output_converter.cpp`
  - 把内部 map 语义结果重采样到对外的 `base_gravity` 机器人中心栅格
- `src/interfaces/ros/runtime/debug_publishers.cpp`
  - 调试点云和 observability 发布逻辑

### 3.4 `test/`
当前最重要的是：

- `test/core/test_processor.cpp`
  - 这是当前真实行为的第一真值
  - 看它比看旧文档更快
- `test/tools/test_false_obstacle_analyzer.cpp`
- `test/tools/test_miss_obstacle_analyzer.cpp`
  - 说明当前离线工具还依赖哪些字段

### 3.5 `tools/`
目前只有一个核心入口：

- `tools/offline_replay.cpp`

它支持 4 类模式：

- `--analyze-false-obstacles`
- `--analyze-missed-obstacles`
- `--inspect-roi`
- `--benchmark-timing`

### 3.6 `config/`
当前参数入口分成：

- `config/passable_area.yaml`
  - 地图、几何阈值、observability、persistence、preprocess
- `config/sensors.yaml`
  - 输入 topic
- `config/debug.yaml`
  - 输出 topic 和调试 publish 开关
- `config/offline_benchmark_bags.yaml`
  - bag 级 benchmark / 回放资产清单

## 4. 这一路重构的背景与动机
### 4.1 最早的简单障碍形成逻辑为什么不够用
当前回退后的前端其实能在旧历史里找到对应原型。仓库在 `f92759c8d6bfd8a15355afdc672a478623c4dcfd` 这个时间点，旧路径下的：

- `src/core/frontend/polar_frontend.cpp`
- `src/core/pipeline/processor.cpp`

还是很简单的版本：

- 按 cell 聚合 `min_z / max_z / count`
- 非 dropout cell 直接用 `min_z` 形成 `SupportCandidate`
- `vertical_span > max_step_up * 0.75f` 直接形成 `ObstacleCandidate`
- `PartiallyObserved` 且未触发时生成 `AmbiguousCandidate`
- `Processor` 当时连 `obstacle_points_min_height` / `obstacle_points_max_height_in_base_link` 这两个发布门槛都还没有

这个简单版本的优点很明显：

- 好懂
- 好测
- 行为短链路
- 很少出现“到底是哪条规则把它压掉了”这种解释困难

但它也很快撞到了真实场景问题：

- 楼梯和开口楼梯里，上下层混样会直接把 `vertical_span` 顶起来
- 墙边、墙脚、立面附近容易形成高跨度 cell
- 同一个 cell 里混入少量低层点会把 support 拉低
- 低净空和近地凸起共用一套 `overhead_height / obstacle_evidence` 表达，解释上很别扭

### 4.2 为什么后来在 `PolarFrontend` 里堆了很多复杂规则
从 `git log --follow src/core/polar_frontend.cpp` 能看得很清楚，这条线是逐步加厚的，而不是一次性设计出来的。

几个关键节点：

- `4570c96 feat: gate obstacle formation with local upper-support neighborhoods`
  - 加了固定 `3x3` upper-support 邻域 gate
- `2659a9a feat: add anchored stair-mix obstacle filtering`
  - 开始显式处理楼梯/层状地面混样
- `268ccaf` / `a370f71` / `78bb7e0`
  - 分别处理 below-robot stair / ground-layer / upstair-ground mix
- `0368e15 feat: implement neighbor consensus and validation utilities for elevated support surface estimation`
  - 更明确地引入 upper-layer 邻域共识
- `6a6dcec refactor: revert effective support elevation path while keep analyzer diagnostics`
  - 说明当时已经开始觉得“effective support elevation”太重，但又不敢直接把 analyzer 契约打断
- `2440c43`、`f1863d8`、`9e45a22`
  - support anchor、raw vs adjusted upper-support、effective support ref 等语义继续细化
- `9e40829 refactor: require explicit keep verdict for obstacle candidates`
  - 复杂度进一步升级：不是“触发就算障碍”，而是要拿到显式 keep verdict
- `a53723a`、`613a10f`
  - pre-trigger leak suppression 也被 anchor authority 绑定

这些规则不是“无意义的过度设计”，它们各自都在解决真实问题：

- stair mix / layered ground mix
- support sparse leak
- stale anchor 残留
- wall-only cell
- wall-like obstacle 证据涨太慢
- facade lower/upper coexistence

对应地，测试里也长出了一整片复杂 case，尤其是旧版 `test_processor.cpp` 中 600 行到 2800 行附近的大量 `PolarFrontend*` case。

### 4.3 那些规则想解决什么问题
一句话概括：**想在前端就把“看起来像障碍、但实际更像楼梯/层状地面/墙脚/支撑泄漏”的情况尽可能解释掉。**

当时的思路是：

- 先把 support anchor 建起来
- 再用 upper-support 和邻域一致性看它是不是“真上层”
- 如果像楼梯/分层地面，就 explanation reject
- 如果像 wall/facade，再显式 keep
- 之后再进入地图证据累计

这条路的副作用，是前端不再只是“单帧候选生成器”，而变成了一个场景化专家系统。

### 4.4 为什么后来会认为这条路越走越重、难以维护、难以解释
问题不是某一条规则错了，而是**整个复杂度已经跨过了维护阈值**。

具体表现：

- 一个 cell 同时有：
  - `support_anchor_used`
  - `support_anchor_origin`
  - `support_anchor_authority`
  - `raw_upper_support_cell`
  - `explanation_adjusted_upper_support_cell`
  - `upper_support_cell`
  - `neighbor_upper_support_count`
  - `aligned_neighbor_support_count`
  - `explanation_decision`
  - `facade_*`
- “障碍为什么没成”不再是一个阶段，而是多阶段组合：
  - local trigger
  - upper patch confirm
  - explanation reject / keep
  - explicit keep verdict
  - map evidence
  - obstacle_points publish gates
- 行为分散在多个模块：
  - `PolarFrontend`
  - `DropoutAwareMapUpdater`
  - `Processor::buildOutput`
  - offline analyzers
- 测试爆炸式增长，且大量测试在守同一套旧 explanation contract，而不是守最终用户关心的 blocking semantics

更关键的是：这条线解决了很多误检，但也开始出现“本该检出的障碍被前端规则树压掉”的问题。用户这次明确提出不满意点就在这里：**误检是降了，但漏检又上来了。**

### 4.5 为什么先做了一次“回退到简单基线”的重构
回退动作不是简单 `git revert` 整条历史，而是一次有选择的收缩：

- `16b43ed revert: roll obstacle frontend back to simple vertical-span baseline`
  - 直接把前端收回到 per-cell `min_z / max_z / vertical_span` 基线
  - 删掉绝大部分旧前端回归测试
  - 文档和测试说明同步切换到 baseline 口径
- `8053800 chore: remove retired obstacle frontend parameters`
  - 把 4 个已经不再生效的前端参数从 YAML、`Config`、参数加载和测试里彻底移除

这次回退有两个边界非常重要：

1. **回退的是前端障碍形成语义，不是整个处理链。**
2. **保留了当前 `Processor` 的 obstacle point publish gates。**

所以现在的版本是：

- 前端更接近 `f92759c` 时期的简单语义
- 但不是完全回到 `f92759c` 的整个系统
- 特别是 `Processor` 的 obstacle point 发布层仍然保留了后来引入的高度门槛和 source cell 语义

## 5. 重构前后遇到/解决过的关键 case
这一节按“问题现象 -> 当时怀疑原因 -> 采取过的方案 -> 结果/副作用 -> 当前结论”整理。

### 5.1 楼梯 / 开口楼梯场景
问题现象：

- `vertical_span` 很容易在楼梯边缘、开口楼梯上下层混样时直接超阈值
- 原本应当保留的楼梯可通行带被打成障碍

当时怀疑原因：

- cell 内同时看到了下层和上层
- 单看 `min_z` 和 `max_z` 区分不了“真障碍”还是“楼梯结构”

采取过的方案：

- 固定 `3x3` upper-support 邻域 gate
- 引入 support anchor，本地锚点不够时借邻居锚点
- stair mix / ground-layer mix / upstair-ground mix explanation reject
- 相关旧测试包括：
  - `PolarFrontendRejectsBelowRobotStairMixDespiteNeighborUpperSupport`
  - `PolarFrontendRejectsBelowRobotStairMixFromObservationWithoutLeakExecution`
  - `PolarFrontendRejectsBelowRobotGroundLayerMixWithOnlyMinimalNeighborSupport`
  - `PolarFrontendRejectsBelowRobotUpstairGroundMixWithoutSupportAnchor`
  - `PolarFrontendRejectsBelowRobotUpstairGroundMixWithBelowRobotSupportAnchor`
  - `StairSceneKeepsTraversableBand`

结果 / 副作用：

- 楼梯误障碍问题确实被压下去了一部分
- 但“这是 stair mix 还是障碍”被编码成了大量 explanation 分支
- 真障碍如果形态和楼梯混样相似，也会被误压掉

当前结论：

- 楼梯/开口楼梯是必须覆盖的真实 case
- 但不再建议用旧版那种基于 `upper_support_cell + neighbor count + explanation_decision` 的规则树继续修
- V2 应该用更稳定的单 cell 垂向摘要 + 历史 support 跟踪来解决

### 5.2 墙边、墙脚、立面附近误障碍
问题现象：

- 墙脚附近容易出现高跨度 cell
- 墙面本体有时应当算阻挡，但墙脚低层噪点又会把 obstacle_points 搞脏

当时怀疑原因：

- 墙脚低点和上方立面混在同一 cell
- 单纯按 `vertical_span` 触发会同时吃到“立面是真障碍”和“墙脚噪点是假障碍”这两类情况

采取过的方案：

- wall-like obstacle evidence gain
- ignore wall-like anchors with shallow upper bands
- ignore stale support anchors for wall-only obstacle cells
- facade keep verdict
- 相关旧测试 / 当前仍保留的测试包括：
  - 旧：`PolarFrontendKeepsBorrowedAnchorWallOnlyCellWithExplicitKeepVerdict`
  - 旧：`PolarFrontendKeepsWallOnlyCellWithoutSupportBandWithExplicitKeepVerdict`
  - 当前：`ObstaclePointsExcludeWallBaseNoise`
  - 当前：`WallWithBaseNoiseStillProducesImpassableCells`
  - 当前：`WallWithoutGroundSupportStillPublishesObstaclePoints`

结果 / 副作用：

- 某些墙面 case 能保下来，墙脚噪点的 obstacle_points 也能过滤掉
- 但前端里因此长出了 wall-like/facade 专用逻辑，复杂度继续上升

当前结论：

- “墙脚噪点不应污染 obstacle_points”和“unsupported wall 仍可视为障碍”是应该保留的行为目标
- 但不应再靠前端 facade 解释树来维持
- 这类 case 更适合归入未来的 Ground Protrusion 证据链

### 5.3 分层地面、上下层混样
问题现象：

- 同一个 cell 里上下两层都被看到时，容易既像上层结构，又像可继续行走的分层地面

当时怀疑原因：

- 需要知道 upper layer 是否有邻域支撑一致性
- 需要区分 raw upper band 和 explanation-adjusted upper band

采取过的方案：

- raw / adjusted / legacy `upper_support_cell`
- 邻域 upper layer 共识
- aligned neighbor support
- 相关旧测试包括：
  - `PolarFrontendUsesNeighborUpperLayerConsensusForLayeredGroundExplanation`
  - `PolarFrontendKeepsLowObstacleWhenUpperLayerLacksNeighborSupportConsistency`

结果 / 副作用：

- 某些分层地面误障碍能被解释掉
- 但语义变成了：
  - “有 raw upper band”
  - “adjusted 后还有没有”
  - “legacy alias 是什么”
- 调试和 analyzer 都开始严重依赖这些中间字段

当前结论：

- 分层 ground / upper layer 共识是 V2 仍然要覆盖的 case
- 旧的 upper-support 三件套不再适合作为长期 contract

### 5.4 support 被低层 sparse leak 拉走
问题现象：

- 少量低层点落进 cell，会把 `min_z` 拉得很低
- support_ref 被拉走后，后续 relative height、clearance、obstacle publish 都会被带偏

当时怀疑原因：

- 低层 sparse leak 混入 support band
- 历史 support 可用性和当前观测之间缺少更稳健的选择机制

采取过的方案：

- `sub_support_leak_tolerance`
- `support_anchor_reobserve_tolerance`
- local / borrowed anchor
- `SupportAnchorAuthority::kExplanationOnly / kLeakEligible`
- stale anchor residual filtering
- 相关旧测试包括：
  - `PolarFrontendFiltersSubSupportLeakUsingValidHistoricalAnchor`
  - `PolarFrontendIgnoresFiniteButInvalidHistoricalSupportForLeakFiltering`
  - `PolarFrontendUsesNeighborMedianAnchorWhenLocalAnchorIsUnavailable`
  - `PolarFrontendSeparatesBorrowedAnchorObservationFromHardLeakSuppression`
  - `PolarFrontendRejectsStaleLowerAnchorWhenHigherBandDominates`

结果 / 副作用：

- 支撑泄漏的一部分误障碍被压下去了
- 但 support anchor 成为了前端复杂度的主要来源之一
- 这 4 个参数最终全部在 `8053800` 被删除，说明这条实现路径已经正式放弃

当前结论：

- 这个问题是真问题，不是误会
- 只是旧版靠 anchor/leak 规则树的解法已经被判定为不值得继续维护
- V2 需要换成更直接的 support profile / support_ref 选择机制

### 5.5 overhead / 顶障 / 低净空
问题现象：

- 低净空本应阻挡，但不一定会在 `obstacle_points` 里表现成“明显的近地障碍点”
- 有时 cell 已经 `Impassable`，但 `/terrain_obstacle_points` 仍然为空
- 由于下游导航当前只消费 `/terrain_obstacle_points`，这种“内部已阻挡、外部无障碍点”的状态对下游等价于漏检

当时怀疑原因：

- 内部 `Impassable / clearance` 判定和对外 obstacle point 发布合同之间缺少明确桥接
- 低净空本质上是 clearance 问题，不一定对应近地 obstacle point

采取过的方案：

- `TraversabilitySolver` 直接用 `clearance < min_clearance` 判 `Impassable`
- `Processor` 继续对 obstacle_points 做额外高度门槛
- 当前测试包括：
  - `LowCeilingCreatesImpassableCells`
  - `ObstaclePointsExcludeGroundSamplesUnderLowCeiling`
  - `ObstaclePointsExcludeSamplesAboveBaseLinkHeightCeiling`
  - `StaticLowCeilingStillRemainsImpassable`
- false analyzer 里也有 `FalseObstacleRootCause::kClearanceDriven`

结果 / 副作用：

- 低净空路径在 solver 里是有效的
- 但如果 `/terrain_obstacle_points` 为空，下游导航仍然看不到这个阻挡
- miss analyzer 用 ROI 内 `obstacle_points == 0` 来定义“漏检”是有意设计，因为它对齐的是下游真正消费的外部指标
- 因此低净空 case 暴露的不是 analyzer 口径错误，而是内部 clearance blocking 是否需要、以及如何驱动 obstacle point 发布的问题

当前结论：

- 低净空和近地凸起应该拆成两条内部证据链，避免在 `overhead_height / obstacle_evidence` 里继续混用
- 但只要某条链的结论要影响下游导航，就必须最终收敛到 `/terrain_obstacle_points` 发布，或明确标注为当前对下游不可见
- V2 不能把内部 `block_reason` 做成绕开 obstacle_points 的第二套外部真值

### 5.6 false obstacle 和 missed obstacle 的典型 bag / ROI
问题现象：

- 单靠 RViz 很难知道问题到底出在：
  - 前端没触发
  - 地图证据没起来
  - obstacle_points 发布被 gate 掉
  - 还是纯 observability / dropout 问题

当时怀疑原因：

- 主链太长，单看结果图看不出是哪一层出错

采取过的方案：

- `tools/offline_replay.cpp`
  - `--analyze-false-obstacles`
  - `--analyze-missed-obstacles`
  - `--inspect-roi`
- `config/offline_benchmark_bags.yaml`
  - 当前内置的 bag 集合主要是 stairs / slope / corridor / passage 类
  - 例如：
    - `rosbag2_mtbf_long_corridor`
    - `rosbag2_mtbf_upslope_and_downstair`
    - `rosbag2_mtbf_x30_upstair_passage`
    - `rosbag2_open_short_upstairs`

结果 / 副作用：

- 这套工具链很有价值，确实把“看起来不对”拆成了可排查的阶段
- 但 miss analyzer 的 root cause 设计是沿着旧前端 explanation contract 长出来的
- 现在 `RejectedByNeighborSupport`、`LeakFilteredToNoCandidate` 这类原因更多是在守兼容，而不是在解释当前 runtime 主路径

当前结论：

- 离线 replay / analyzer 必须保留
- 但其解释 contract 未来要跟着 V2 一起重做
- 目前 repo 里没有一份“固定权威 ROI 清单”作为正式 acceptance 资产，后续需要补

### 5.7 dropout / partially observed / missing by dropout 下的保守策略
问题现象：

- 后方掉点、局部缺测时，如果负更新太激进，会把已有支撑/障碍刷空

当时怀疑原因：

- 不能把“没看到”直接当“没有”

采取过的方案：

- `FrameObservabilityEstimator` 区分：
  - `Observed`
  - `PartiallyObserved`
  - `MissingByDropout`
- `DropoutAwareMapUpdater` 按观测状态分不同衰减强度
- 当前测试包括：
  - `RearDropoutIsFlaggedAndSupportPersists`
  - `RearGapTriggersMissingByDropoutSectors`
  - `SparseCoverageCreatesUnknownButNotDropout`
  - `ObstacleNotClearedAggressivelyDuringDropout`
  - `PolarFrontendSuppressesSupportCandidateInMissingByDropoutSector`
  - `PolarFrontendEmitsAmbiguousCandidateForPartiallyObservedNonTriggerCell`

结果 / 副作用：

- dropout-aware 这条线整体是成功的
- 当前 `ambiguous_candidates` 还保留着，但 `DropoutAwareMapUpdater` 实际并没有消费它

当前结论：

- 观测状态建模和 dropout-aware persistence 是应该保留的成熟部分
- 但 `ambiguous` 的用途在当前版本里没有闭环，后续不应再围绕它扩展
- V2 倾向删除 `AmbiguousCandidate`，并简化扇区状态，不再保留当前这个对主链没有实质消费的 `kPartiallyObserved` 分支
- dropout / 缺测下的保守策略应继续由 map updater 的衰减、持久化和清空策略承担，而不是靠 ambiguous candidate

### 5.8 obstacle_points 外部合同与内部判定语义不一致
问题现象：

- 下游导航当前把 `/terrain_obstacle_points` 当作唯一障碍输入
- false / miss analyzer 因此也有意把 obstacle points 当作检测真值
- 但当前内部还有 solver `Impassable`、clearance、obstacle evidence 等状态，这些状态并不总能解释为“最终是否发布了 obstacle point”

当时怀疑原因：

- `Processor::buildOutput` 把 obstacle point 发布层做成了额外筛选
- solver 又在另一条链上判 `Impassable`

采取过的方案：

- 加入 `obstacle_points_min_evidence`
- 加入 `obstacle_points_min_height`
- 加入 `obstacle_points_max_height_in_base_link`
- 对无支撑墙面再加 fallback `support_ref`

结果 / 副作用：

- `obstacle_points` 成为了“下游实际能看到的障碍”
- 内部 `Impassable` 和对外 obstacle point 发布之间可能出现断层
- 这会导致同一个 case 在内部看像 blocking，在下游和 analyzer 视角却是漏检

当前结论：

- 这是当前版本最核心的合同债之一
- V2 不应把 `obstacle_points` 降格为普通 debug 链；在现有下游接口不变的前提下，它必须继续作为外部障碍输出合同
- 需要补齐的是内部 `candidate / evidence / block_reason / publish gate` 到最终 obstacle point 的可审计链路

## 6. 已经做完的关键重构
这一节只写当前代码里真实存在的东西。

### 6.1 已完成且稳定
1. `core` / `interfaces/ros` 分层已经成型

- 主链完全 ROS-free
- ROS 层只做参数、同步、转换、发布、watchdog、perf stats

2. 外部 frame 语义已经清理到 `map`

- `FrameInput::base_pose_in_map`
- `ProcessedFrame::cloud_in_map`
- `ProcessedFrame::map_samples`
- 旧字段名仍保留，但数值语义按 `map` 理解

3. `PolarFrontend` 已经回退成简单基线

- 已删掉或失活的旧逻辑包括：
  - local / borrowed support anchor 解析
  - anchor validity / authority 主判定逻辑
  - leak suppression
  - stale anchor residual filtering
  - upper-support 邻域确认主逻辑
  - explanation reject / keep
  - facade evidence
  - wall-like gain path

4. 4 个旧前端参数已经从代码和 YAML 中去掉

- `upper_min_height_above_support`
- `sub_support_leak_tolerance`
- `support_anchor_reobserve_tolerance`
- `min_neighbor_upper_support_cells`

### 6.2 已完成但只是过渡形态
1. `Processor` 没有回退到旧 `f92759c` 的 processor

- 当前仍保留 obstacle point publish gates：
  - `obstacle_points_min_evidence`
  - `obstacle_points_min_height`
  - `obstacle_points_max_height_in_base_link`
- 还保留了 source-cell 语义和 unsupported wall 的 fallback `support_ref`

2. `DropoutAwareMapUpdater`、`TerrainFeatureUpdater`、`TraversabilitySolver` 没有重做

- 这几层仍是当前系统稳定性的主要承担者
- 说明这次回退不是“回到旧系统”，而是“回到旧前端语义”

3. offline analyzers 没有跟着清 contract

- 仍然读取大量历史 explanation / anchor / upper-support 字段
- 只是当前真实 runtime 大多把这些字段置 0 或无效

4. debug / output contract 仍保留大量历史兼容痕迹

- `FrontendOutput` / `FrameOutput` 里仍有：
  - `support_anchor_*`
  - `anchor_leak_*`
  - `raw_upper_support_cell`
  - `explanation_adjusted_upper_support_cell`
  - `upper_support_cell`
  - `neighbor_upper_support_count`
  - `aligned_neighbor_support_count`
  - `explanation_decision`
  - `facade_*`
- `output_converter.cpp` 也还在 `grid_map` 中发布：
  - `support_anchor_used`
  - `sub_support_leak_count`

## 7. 当前版本的真实语义（非常重要）
这一节只按当前代码实际行为写。

### 7.1 当前 `PolarFrontend` 到底输出什么
当前 `PolarFrontend::run()` 的行为非常直接：

1. 先给所有兼容字段分配默认值

- `support_anchor_used = NaN`
- `support_anchor_origin = kNone`
- `support_anchor_authority = kInvalid`
- `anchor_leak_suppression_enabled = 0`
- `sub_support_leak_count = 0`
- `raw_upper_support_cell = 0`
- `explanation_adjusted_upper_support_cell = 0`
- `upper_support_cell = 0`
- `neighbor_upper_support_count = 0`
- `aligned_neighbor_support_count = 0`
- `explanation_decision = kNone`
- `facade_* = 0`

2. 对 `frame.map_samples` 按 cell 聚合原始统计

- `raw_sample_min_z`
- `raw_sample_max_z`
- `raw_sample_count`
- `filtered_sample_*` 直接等于 raw，对当前前端没有额外过滤

3. 对每个有样本的 cell，根据其代表扇区状态输出 candidate

### 7.2 support / obstacle / ambiguous 的当前形成规则
当前只有三条核心规则：

1. `SupportCandidate`

- 条件：cell 有样本，且对应扇区不是 `ObservabilityState::kMissingByDropout`
- 高度：当前帧该 cell 的 `min_z`
- 置信度：对应 sector 的 `coverage_confidence`

2. `ObstacleCandidate`

- 条件：`vertical_span = max_z - min_z > max_step_up * 0.75f`
- 障碍高度：当前帧该 cell 的 `max_z`
- evidence：`vertical_span`，再 `clamp(0, 1)`
- `gain_scale`：固定为 `1.0f`
- 同时会把以下 stage 兼容字段一起置位：
  - `obstacle_local_triggered = 1`
  - `obstacle_upper_patch_confirmed = 1`
  - `obstacle_suspicious = 1`
  - `obstacle_candidate_cell = 1`

3. `AmbiguousCandidate`

- 条件：
  - 没有触发 `ObstacleCandidate`
  - 且扇区状态是 `ObservabilityState::kPartiallyObserved`
- 高度：当前帧该 cell 的 `min_z`

这只是当前代码的真实行为，不是 V2 目标。基于当前讨论，`AmbiguousCandidate` 和 `ObservabilityState::kPartiallyObserved` 都倾向在 V2 中删除；后续不要在这条分支上继续叠加新语义。

### 7.3 当前前端不会做什么
当前前端**不会**做这些事情：

- 不读取地图历史 support 作为当前 support_ref
- 不借邻居支撑锚点
- 不做 leak suppression
- 不做 stale anchor invalidation
- 不做 upper-support neighbor gate
- 不做 explanation reject / keep
- 不做 facade keep verdict

因此当前 runtime 里如果你看到：

- `explanation_decision != kNone`
- `neighbor_upper_support_count > 0`
- `upper_support_cell != 0`

那通常不是当前前端自然产生的，而是：

- analyzer 测试里手动构造的兼容场景
- 或旧分析口径残留

### 7.4 当前 `DropoutAwareMapUpdater` 实际在做什么
当前 map updater 只真正消费两类东西：

- `support_candidates`
- `obstacle_candidates`

`ambiguous_candidates` 目前没有被它使用。

具体行为：

1. support candidate

- 写 `support_height`
- 增加 `support_confidence`
- 更新 `coverage_confidence`
- 置 `support_state = Observed`
- 置 `last_observed_age = 0`
- 如果 sector 是 `Observed`，则 `last_reliable_age = 0`

2. obstacle candidate

- 写 `overhead_height`
- 增加 `overhead_confidence`
- 增加 `obstacle_evidence`

3. 对未 touched 的 cell 做按 observability 区分的衰减

- support：
  - `Observed` 衰减较强
  - `PartiallyObserved` 较弱
  - `MissingByDropout` 极弱
- obstacle：
  - support 重观测到时清得更快
  - `Observed` 普通衰减
  - `PartiallyObserved` 弱衰减
  - `MissingByDropout` 极弱衰减

4. 清理旧障碍的真实条件

- 当前代码里**并不存在**“被 explanation 重新解释后显式清障”那条逻辑
- 真正清理 `overhead_*` 的条件是：
  - 本帧 support 被 touch 且该 cell 当前没有新 obstacle，且 `obstacle_evidence <= obstacle_height_clear_threshold`
  - 或 support 失效且 `obstacle_evidence` 也降到阈值以下

这一点和 `algorithm_scheme.md` / `passable_area_code_walkthrough.md` 中个别仍在提“effective_support_ref 重解释清障”的段落不一致，以当前代码为准。

### 7.5 当前 `TraversabilitySolver` 的障碍语义
当前 solver 的判断顺序是：

1. 先判 `Unknown`

- `coverage_confidence < 0.15`
- `support_state == None`
- `support_confidence < min_support_confidence`
- `last_reliable_age` 超过 stale 阈值

2. 再判 `Impassable`

- `clearance < min_clearance`
- 或 `support_continuity < 0.3 && obstacle_evidence > 0.4`

3. 否则如果几何指标合格，则 `Passable`

- `slope <= max_support_slope_deg`
- `step_up <= max_step_up`
- `step_down <= max_step_down`
- `roughness <= max_support_roughness`

这意味着当前“障碍是否成立”并不等于“前端有没有 obstacle candidate”：

- 前端只是单帧触发
- 地图层要先累计证据
- solver 还会结合 `clearance` 和 `support_continuity`

### 7.6 当前 obstacle_points 最终是如何被发布出来的
`Processor::buildOutput()` 的 obstacle point 逻辑分两步：

1. 先为“evidence 足够但没有有限 support_height 的 cell”准备 fallback `support_ref`

- 条件：
  - `layers.obstacle_evidence[cell] >= obstacle_points_min_evidence`
  - `layers.support_height[cell]` 不是有限值
- fallback 值：
  - 当前帧该 cell 所有 map sample 的最小 `point_in_map.z`

2. 再逐个 sample 过滤并发布

一个 sample 会进入 `output.obstacle_points`，必须同时满足：

- 该 cell 的 `layers.obstacle_evidence[cell] >= obstacle_points_min_evidence`
- sample 相对 `support_ref` 的高度满足：
  - `sample.point_in_map.z >= support_ref + obstacle_points_min_height`
- sample 在 `base_link` 下的高度满足：
  - `sample.point_in_base.z <= obstacle_points_max_height_in_base_link`

最终发布时，点会被转换到 `base_gravity`，并带上 `source_cell`。

这几个行为后果非常关键：

- unsupported wall 仍然可以发布 obstacle points
- low ceiling 的地面点会被滤掉，只保留高于 `support_ref` 的上方样本
- gate 是在 `base_link` 下判的，但发布出去的是 `base_gravity` 点，所以最终点云里的 `z` 可以高于 `obstacle_points_max_height_in_base_link`

### 7.7 当前 `obstacle_points`、内部 passability 与下游真值的关系
这里需要区分内部状态和外部合同。

更准确地说：

- `Impassable` 是 solver 基于地图层综合判出来的
- `obstacle_points` 是从“evidence 已经够强的 cell”里再筛出的一部分可发布样本
- 下游导航当前只消费 `/terrain_obstacle_points`，所以对下游障碍检测来说，`obstacle_points` 就是有效真值
- false / miss analyzer 以 `obstacle_points` 为判断依据是有意设计，而不是工具误用

因此当前系统里完全可能出现：

- cell 已经 `Impassable`，但 `obstacle_points` 为空
  - 例如低净空
  - 例如 evidence 有了，但没通过 publish height gate
- `obstacle_points` 有输出，但不等于整个 blocking semantics 都解释清楚了

这意味着 V2 如果引入 `block_reason` 或 Low Clearance 独立链路，必须同时定义它和 obstacle point 发布之间的关系。否则内部阻挡结论即使正确，也不会被下游导航看到。

### 7.8 当前 offline analyzer 还能解释什么，哪些字段已经没有实质语义
当前 false / miss analyzer 还有效的部分：

- false analyzer 仍能解释：
  - `kClearanceDriven`
  - `kObstacleEvidenceDriven`
  - `kObstacleEvidencePlusLowContinuity`
  - `kObservabilityInfluenced`
- miss analyzer 仍然最有参考价值的 root cause 是：
  - `kNoFrontendObstacleSuspicion`
  - `kObstacleEvidenceTooLow`
  - `kOutputHeightGateNotMet`
  - `kNoObstacleSourceSamplesInRoi`
  - `kNoSamplesInRoi`

当前已经没有实质 runtime 主语义、但 analyzer 仍保留读取的字段：

- `support_anchor_*`
- `anchor_leak_suppression_enabled`
- `sub_support_leak_count`
- `raw_upper_support_cell`
- `explanation_adjusted_upper_support_cell`
- `upper_support_cell`
- `neighbor_upper_support_count`
- `aligned_neighbor_support_count`
- `explanation_decision`
- `facade_*`

这些字段现在的地位是：

- 仍然是 analyzer / 测试的兼容合同
- 但不应再被理解为“当前前端主链正在经过这些阶段”

## 8. 当前测试与工具链状态
### 8.1 当前最关键的测试
当前工作区 `colcon test-result --verbose` 显示：

- `Summary: 80 tests, 0 errors, 0 failures, 0 skipped`

最关键的仍然是 `test/core/test_processor.cpp`。

它当前锁住的重点行为包括：

- preprocessor：
  - body filter
  - crop to map
  - downsample / map semantic
- observability / dropout：
  - rear dropout
  - `MissingByDropout`
  - sparse coverage -> `Unknown`
- map updater：
  - support persistence
  - recenter 后历史层平移
  - dropout 下 obstacle 不被激进清空
  - 动态障碍可被地面重观测清掉
- 当前简单前端：
  - `PolarFrontendFormsObstacleCandidateFromVerticalSpanWithoutNeighborGates`
  - `PolarFrontendIgnoresHistoricalSupportAndUsesFrameMinZForSupportCandidate`
  - `PolarFrontendEmitsAmbiguousCandidateForPartiallyObservedNonTriggerCell`
  - `PolarFrontendSuppressesSupportCandidateInMissingByDropoutSector`
  - `PolarFrontendCopiesRawStatsIntoFilteredStatsWithoutFrontendFiltering`
- obstacle point 发布门槛：
  - `ObstaclePointsExcludeGroundSamplesUnderLowCeiling`
  - `ObstaclePointsExcludeSamplesAboveBaseLinkHeightCeiling`
  - `ObstaclePointsKeepSamplesAtOrBelowBaseLinkHeightCeiling`
  - `ObstaclePointPublishUsesBaseLinkCeilingButPublishesInBaseGravity`
  - `ObstaclePointsExcludeWallBaseNoise`
  - `WallWithoutGroundSupportStillPublishesObstaclePoints`
- low ceiling / wall / dropout 稳定性：
  - `WallWithBaseNoiseStillProducesImpassableCells`
  - `ObstacleNotClearedAggressivelyDuringDropout`
  - `StaticLowCeilingStillRemainsImpassable`

如果新窗口只想最快理解当前行为，优先读这几个测试名对应的 case。

### 8.2 false / miss obstacle analyzer 现在依赖哪些字段
`test_false_obstacle_analyzer` 和 `test_miss_obstacle_analyzer` 说明得很清楚：它们仍然依赖一大批历史字段，即使这些字段当前 runtime 主路径已经不再自然产生。

特别是 miss analyzer 仍保留这些 root cause：

- `kRejectedByNeighborSupport`
- `kLeakFilteredToNoCandidate`

但当前简单前端不会主动走出这两条路径；这些测试更多是在守兼容，不是在守当前主语义。

### 8.3 offline_replay 目前适合怎么用
当前 `tools/offline_replay.cpp` 最适合做 3 类事：

1. `--analyze-false-obstacles`

- 在一个 base_gravity detection box 里找 `obstacle_points`
- 聚热点
- 看 clearance / evidence / observability

2. `--analyze-missed-obstacles`

- 在一个 ROI 里，如果 `obstacle_points` 为 0，就进入漏检分析
- 看：
  - 有没有样本
  - 前端有没有 suspicion
  - evidence 是否够
  - publish height gate 是否挡掉了

3. `--inspect-roi`

- 适合做更细的样本级检查

要注意：

- miss analyzer 当前判断“是否漏检”的真值仍是 ROI 里有没有 `obstacle_points`，这是为了对齐下游导航实际消费的唯一障碍指标
- 它不等价于“cell 是否已经被判成 `Impassable`”，因此报告中后续应同时暴露内部阻挡但未发布 obstacle point 的情况，帮助定位 publish-layer loss

### 8.4 哪些测试或工具已经反映旧语义、后续大概率要改
以下内容后续大概率要重写：

- miss analyzer 的 root cause 集合
- false / miss analyzer 对旧 explanation 字段的读取合同
- `test_false_obstacle_analyzer.cpp` 和 `test_miss_obstacle_analyzer.cpp` 中大量手工构造旧字段的 case
- `FrameOutput` 上那些只为 analyzer 兼容存在的旧字段

重写工具时不要丢掉一个原则：`obstacle_points` 仍应作为外部检测是否成功的主判断口径。需要升级的是 root cause 解释，从“旧前端 explanation 字段”转向“候选、证据、reasoner、publish gate 为什么最终产生或没有产生 obstacle point”。

### 8.5 当前 acceptance 常用的 bag / case 类型
repo 里能确认的 acceptance 资产主要是：

- `config/offline_benchmark_bags.yaml`
  - stairs / upslope / downslope / corridor / passage 类 bag
- `docs/false_obstacle_analysis_guide.md`
- `docs/miss_obstacle_analysis_guide.md`
  - 提供了 ROI 分析工作流

但当前没有一份“正式、固定、权威”的：

- false obstacle 回归 bag + ROI 列表
- missed obstacle 回归 bag + ROI 列表

这一点后续仍需补齐。

## 9. 当前存在的主要问题与技术债
按重要性排序如下。

### 9.1 当前简单回退版只是基线，不是终态
当前 `PolarFrontend` 只看 `min_z / max_z / vertical_span`。

它的优势是简单，但缺点也很明显：

- 不能表达稳定 support band 和 upper band
- 不能区分 single tall wall 和 layered ground
- 不能稳健处理 sparse lower leak
- 不能把楼梯 / 开口楼梯 / 分层地面这些真实难例解释清楚

所以这版只适合作为“先把复杂前端规则树拆掉后的重新起点”，不适合作为终态方案。

### 9.2 `obstacle_points` 外部合同与内部阻挡状态尚未对齐
这件事当前已经是结构性问题：

- solver 在判 `Impassable`
- processor 在筛 `obstacle_points`
- miss analyzer 按 `obstacle_points` 缺失判“漏检”，这是对齐下游导航合同的正确口径

后果是：

- 低净空、height gate、unsupported wall 这些 case 很容易出现“内部阻挡”和“外部点云输出”不一致
- 用户从 RViz 和下游导航看到的是 obstacle points，而不是内部 blocking semantics
- 当前缺少一条明确的、可审计的桥：从 frontend candidate、map evidence、reasoner / solver 结论，到最终 obstacle point 是否发布以及为什么没有发布

### 9.3 旧兼容字段债还很重
当前 `FrameOutput` / `FrontendOutput` / analyzers / `grid_map` 里仍有很多旧字段：

- 它们会误导阅读者
- 也会让后续类型重构的改动面很大

这类债不是“看着别扭”而已，而是直接影响：

- 代码可读性
- analyzer 解释口径
- 新 agent 建立心智模型的速度

### 9.4 文档和代码存在局部失配
已确认的几处问题：

- 工作区根 `AGENTS.md` 仍写着当前前端有固定 `3x3` neighbor gate，已过时
- `algorithm_scheme.md` 和 `passable_area_code_walkthrough.md` 里关于 `DropoutAwareMapUpdater` “显式清理被 explanation 重解释或邻域否决的旧障碍”的段落，和当前代码不一致

这意味着后续任何设计讨论，都不能只看文档。

### 9.5 当前前端 / 地图 / solver 的职责边界仍不够理想
虽然前端已经被收缩了，但整体上仍然没有达到理想边界：

- 前端过于简单，表达力不足
- map updater 仍只维护一套 `obstacle_evidence`
- solver 还在直接解释 `obstacle_evidence + support_continuity`
- 没有一个独立的 reasoner 来说清楚“这个 cell 为什么 blocked”

### 9.6 `ambiguous_candidates` / `kPartiallyObserved` 是待删除的过渡债务
前端还会产生 `ambiguous_candidates`，但 map updater 不消费它。

这意味着：

- 当前系统已经有“这里不够明确”的表达，但没有真正进入 anti-clear / uncertainty 管理闭环
- 继续保留它会诱导后续设计在一条无消费路径上加语义
- V2 倾向删除 `AmbiguousCandidate`，并把扇区状态简化掉 `kPartiallyObserved`
- 缺测和不确定性下的保守策略应保留在 `DropoutAwareMapUpdater` 的衰减、持久化、清空策略中

### 9.7 部分 debug publish 参数目前只是名义存在
从当前实现看：

- `debug.publish_grid_map`
- `debug.publish_points`
- `debug.publish_observability`

并没有像参数名暗示的那样严格控制发布行为。

实际有效的主要是：

- `debug.publish_base_gravity_cloud`
- `debug.publish_map_to_base_gravity_tf`

这也是当前 ROS 接口层的一笔小但真实的债。

## 10. 已确定的下一步架构方向（V2 方案）
这一节写的是已经讨论清楚、准备往下做的方向，不是随意 brainstorming。

### 10.1 保留 2.5D、robot-centric、本地栅格地图主链
后续不打算回到重型 3D voxel，也不打算推翻当前 `LocalTerrainMap` 主干。

原因：

- 当前地图规模和实时性要求更适合 2.5D 局部图
- 现有工程链条、测试和 ROS 接口都围绕这套结构
- 需要重构的是障碍表达和判定边界，不是整套工程基础设施

### 10.2 漏检优先：前端更容易触发，稳定性和误检抑制下放到地图证据层和 reasoner
这条是明确的方向选择。

原因：

- 当前用户最不满意的是漏检
- 旧前端规则树正是“为了压误检而把漏检推上来”的典型例子

因此 V2 的思路不是“前端再变聪明”，而是：

- 前端更容易、也更透明地触发候选
- 稳定性、证据累计、阻挡确认放到地图层和独立 reasoner

### 10.3 明确拆分 Ground Protrusion 与 Low Clearance 两条链
这件事已经形成共识。

原因：

- 近地凸起和低净空本来就是两类不同问题
- 现在两者都被塞进 `overhead_height / obstacle_evidence / obstacle_points`
- 这会直接导致解释混乱

后续应明确分成：

- `Ground Protrusion`
  - 箱体、墙脚、竖直立面、近地凸起等
- `Low Clearance`
  - 顶障、悬挑、低净空

### 10.4 不新增 ROS topic，`/terrain_obstacle_points` 继续作为下游唯一障碍输出合同
当前共识是：

- 不新增新的 ROS debug topic
- 保留 `/terrain_obstacle_points`
- 在现有下游接口不变的前提下，继续把它作为下游导航消费的唯一障碍输出合同
- Ground Protrusion 和 Low Clearance 可以在内部拆成两条链，但只要它们要影响下游导航，就必须明确如何驱动 obstacle point 发布

这样做的优点是：

- 不破坏现有 ROS 接口面
- 在线调试和离线工具仍有可用抓手
- false / miss analyzer 的主评价口径可以继续和下游保持一致

风险是：

- 需要在 `FrameOutput` / analyzer / docs 里把内部 `block_reason` 与最终 `obstacle_points` 发布之间的关系讲清楚
- 不能让 `block_reason` 变成下游看不到的第二套障碍真值

### 10.5 新增 ROS-free 的 `ObstacleReasoner`
V2 将新增独立 `ObstacleReasoner` 模块。

它的职责不是接原始点，而是接：

- 地图层中的 support / protrusion / overhead 证据
- terrain geometry 特征

然后给出：

- `block_reason`
- `protrusion_stage`
- `overhead_stage`
- `obstacle_point_publish_status`
- obstacle point 发布意图或不发布原因

这样做的优点：

- 把“候选生成”和“阻挡确认”彻底拆开
- 不再让 solver 直接解释 raw obstacle evidence
- 让“内部为什么认为该阻挡”和“最终为什么有/没有 obstacle point”能在同一个合同里被审计

### 10.6 `PolarFrontend` 升级为轻量双层摘要，而不是再长出复杂 explanation 树
V2 不是回到旧 explanation tree，而是：

- 在每个 cell 内做轻量垂向摘要
- 明确 support band / upper band / single tall band
- 用简洁规则产生 candidate

这条路的核心价值在于：

- 仍然可解释
- 表达力明显强于 `min/max`
- 又不会重新长出旧版那种场景化规则森林

### 10.7 `DropoutAwareMapUpdater` 只维护 support / protrusion / overhead 三条证据链
V2 中 map updater 的目标边界是：

- 只做记忆、衰减、hysteresis
- 不做复杂场景解释

这有两个好处：

- 让 observability / persistence 逻辑继续留在它最适合的地方
- 不再让地图层承担旧 explanation 语义

### 10.8 `TraversabilitySolver` 后续消费 `block_reason + terrain geometry`，但不成为新的外部真值
V2 里 solver 不再直接解释“某个 evidence 多高、某个 continuity 多低，就算 obstacle”。

而是：

- `ObstacleReasoner` 先给出 blocking semantics
- solver 再把：
  - `Unknown`
  - `block_reason`
  - terrain geometry constraints
  合并成最终 passability

需要注意：solver 的 `passability` 仍是内部地形状态，不等于下游障碍输出。只要下游仍只消费 `/terrain_obstacle_points`，任何应影响导航避障的阻挡原因都必须和 obstacle point 发布合同对齐。

## 11. V2 方案的关键设计点
### 11.1 轻量双层摘要
当前共识是每个 cell 只保留轻量垂向结构，而不是完整 3D 体素。

核心对象：

- `support_band`
- `upper_band`
- `single_tall_band`

目标是覆盖：

- 普通地面
- 有上方结构的双层 cell
- 没有明确两层但本身就是高立面的 tall cell

### 11.2 `profile_split_gap`、trimmed bounds、raw min/max 只做 debug
V2 里会按 cell 收集 `z`，再用 `profile_split_gap` 做 band split。

band 内的代表高度不再直接拿 raw `min/max`，而是用更稳健的 trimmed bounds，例如：

- `bottom = 10% 分位`
- `top = 90% 分位`

而 raw `min/max/count` 只保留为 debug 字段。

这样做是为了：

- 降低少量离群点对 support / upper 判定的影响
- 同时保留原始观测证据，便于离线核查

### 11.3 `support_ref` 的选择优先级与历史 support 跟踪
V2 不会回到旧 anchor tree，但会保留“历史 support 跟踪”这个必要能力。

目前已讨论清楚的优先级是：

1. 当前帧 `support_band`
2. 同 cell 可靠历史 `support_height`
3. 没有可靠 `support_ref`

并且：

- 会引入 `support_tracking_window`
- 优先选择与历史 support 更接近的 band
- 但不会重新长出 local / borrowed / explanation-only / leak-eligible 这种旧锚点体系

### 11.4 `ProtrusionCandidate` / `OverheadCandidate`
目标语义已经明确：

- `ProtrusionCandidate`
  - 近地凸起 / 竖直阻挡 / 墙面 / 柱体
- `OverheadCandidate`
  - 低净空 / 顶障 / 悬挑

这两个 candidate 是内部证据入口，不是对外真值。它们后续必须通过地图证据、reasoner 和 publish gate 明确说明是否产生 `/terrain_obstacle_points`。

### 11.5 删除 `AmbiguousCandidate`，简化 `kPartiallyObserved`
当前代码里仍存在：

- `AmbiguousCandidate`
- `ObservabilityState::kPartiallyObserved`

但它们没有形成主链闭环，V2 不建议继续保留。

建议方向：

- 删除 `AmbiguousCandidate`
- 简化扇区状态，不再保留 `kPartiallyObserved`
- 保留“Observed / MissingByDropout 或等价二分状态”的核心语义，具体 enum 命名在代码阶段再冻结
- 不确定性、遮挡和 dropout 下的保守清空策略由 `DropoutAwareMapUpdater` 处理
- 如果仍需要解释“为什么没有激进清空”，用 debug reason 字段表达，而不是新增第三类 candidate

### 11.6 protrusion / overhead evidence 分层维护
后续不再只有一套：

- `overhead_height`
- `obstacle_evidence`

而是会分开维护：

- protrusion evidence
- overhead evidence

这是为了让：

- 近地凸起的确认
- 低净空的确认

各自有独立 hysteresis，而不是继续共用一套模糊层。

### 11.7 future debug contract
未来预计会引入一套更小、更干净的调试合同，例如：

- `support_profile_state`
- `protrusion_stage`
- `overhead_stage`
- `block_reason`
- `obstacle_point_publish_status`

同时逐步替换掉旧的：

- `upper_support_cell`
- `neighbor_upper_support_count`
- `explanation_decision`
- `support_anchor_*`

### 11.8 已明确的约束
这几条是已经谈清楚、不建议回头的边界：

- 不允许重新长出 neighbor confirm 规则树
- 不回到重型 3D voxel
- 不引入学习模型
- 不新增 ROS topic
- `/terrain_obstacle_points` 必须继续作为下游唯一障碍输出合同
- 内部 `block_reason` 不能成为脱离 obstacle point 发布的另一套外部真值

## 12. 建议的落地顺序
### 12.1 Phase 1：先冻结 types / enums / layer contracts / debug contracts
建议先改的文件：

- `include/passable_area/core/types/frame_types.hpp`
- `include/passable_area/core/candidate_types.hpp`
- `include/passable_area/core/mapping/terrain_layers.hpp`
- `include/passable_area/tools/*_analyzer.hpp`

最低验收标准：

- 新旧字段边界明确
- 未来要保留的 debug contract 先定下来
- 不再让实现阶段临时发明字段
- 明确删除或废弃 `AmbiguousCandidate`、`ObservabilityState::kPartiallyObserved`
- 明确 `obstacle_point_publish_status` / 不发布原因这类 obstacle point 发布合同字段

最容易踩的坑：

- 一边改类型一边改行为，最后很难分清 breakage 来源
- 过早删掉 analyzer 还在消费的字段
- 只冻结内部 `block_reason`，却忘记定义它如何影响最终 `/terrain_obstacle_points`

### 12.2 Phase 2：先做 Frontend V2 + MapUpdater 最小链路
建议先改的文件：

- `src/core/polar_frontend.cpp`
- `src/core/mapping/dropout_aware_map_updater.cpp`
- `test/core/test_processor.cpp`

最低验收标准：

- `support_band / upper_band / single_tall_band` 能稳定产生
- `ProtrusionCandidate / OverheadCandidate` 的最小链路跑通
- Ground Protrusion / Low Clearance 都能说明是否产生 obstacle point，以及不产生时的原因
- 合成 case 能覆盖 stairs / wall / low clearance / sparse leak / dropout

最容易踩的坑：

- 为了补 corner case 又把旧 explanation 树塞回来
- 在前端里重新堆历史支撑判断，导致边界再次糊掉
- 把 `AmbiguousCandidate` 换个名字重新引入，造成第三条无主链消费的 candidate

### 12.3 Phase 3：先做 shadow reasoner path，不要立刻全切 solver 主逻辑
建议新增 / 修改的文件：

- 新增 `ObstacleReasoner`
- `src/core/processor.cpp`
- 对应 core tests

最低验收标准：

- reasoner 能在不改最终 passability 的情况下输出 shadow 判定
- 能和当前 solver 结果并行对比

最容易踩的坑：

- 直接切 solver 主判定，导致回归面太大
- 让 reasoner 又去看原始 sample，破坏模块边界

### 12.4 Phase 4：跑 bag 做结构验收，再切 solver 主判定权
建议重点操作：

- `offline_replay --analyze-false-obstacles`
- `offline_replay --analyze-missed-obstacles`
- `offline_replay --inspect-roi`
- `--benchmark-timing`

最低验收标准：

- 目标漏检 bag 的 recall 明显改善
- 没有出现大面积 false obstacle 回潮
- dropout / 缺测 / low-clearance case 没被破坏
- 性能基线已建立并确认没有不可接受回退

最容易踩的坑：

- 只看内部 `block_reason / passability`，却忘记下游实际只看 `obstacle_points`
- 只统计 `obstacle_points` 有无，却不记录为什么发布或为什么被 gate 掉，导致问题不可审计
- 没有先固定 ROI / bag，导致结论不可复现

### 12.5 Phase 5：最后再清理旧兼容字段、测试、docs、analyzer 债务
建议最后改的文件：

- `FrameOutput` / `FrontendOutput` 旧字段
- `test/tools/*`
- `src/tools/*_analyzer.cpp`
- `docs/*.md`

最低验收标准：

- analyzer 与新 contract 完整对齐
- 文档不再残留旧 explanation 语义
- 仓库内不再有“看起来还活着但其实已失效”的旧字段和旧参数

最容易踩的坑：

- 过早删兼容字段，导致定位问题的工具先坏掉
- 只改代码不改文档，导致新窗口继续被旧文档误导

## 13. 新窗口继续工作前的建议检查项
- 先确认当前 `src/core/polar_frontend.cpp` 的真实行为，尤其是它已经不再读取历史 support / explanation 规则。
- 先确认 `test/core/test_processor.cpp` 里哪些 case 是当前 contract，哪些是旧 contract 已删。
- 先确认 false / miss analyzer 现在还在消费哪些兼容字段，不要误删。
- 先确认 `algorithm_scheme.md`、`passable_area_code_walkthrough.md` 里残留的旧语义段落。
- 先确认 acceptance bag / ROI，当前 repo 里没有权威固定列表。
- 先确认下游导航对 `/terrain_obstacle_points` 的精确消费方式和坐标/高度假设。
- 先确认 V2 验收仍以 obstacle points 作为外部检测主口径，同时补充内部原因链审计。
- 先确认当前性能 baseline，repo 里有工具，但没有最新数值被正式写入交接材料。
- 先确认 `debug.publish_grid_map`、`debug.publish_points`、`debug.publish_observability` 是否真的需要继续保留。
- 先确认是否仍需要 `source_cell` 这条 obstacle point 追踪链，当前 analyzer 明确依赖它。
- 先确认外部是否有人在消费 `grid_map` 里的 `support_anchor_used` / `sub_support_leak_count` 这两个历史层；若未知，按“需核实”处理。
- 先确认删除 `AmbiguousCandidate` / `kPartiallyObserved` 前，没有仍然依赖它们的测试、analyzer 或文档合同。

## 14. 附录：建议优先阅读的文件
1. `docs/refactor_handover_status.md`
   - 先建立“当前做到哪、为什么停在这里”的大图景

2. `docs/algorithm_scheme.md`
   - 看整体术语和主链
   - 但遇到 obstacle frontend / map updater 细节时要回到代码核对

3. `src/core/polar_frontend.cpp`
   - 看当前前端到底有多简单
   - 这是理解“为什么下一步要做 V2”的最好入口

4. `src/core/processor.cpp`
   - 看当前主链编排和 obstacle_points 发布逻辑
   - 特别要看 publish gates 和 fallback `support_ref`

5. `src/core/mapping/dropout_aware_map_updater.cpp`
   - 看当前短时记忆、dropout-aware 衰减、support persistence

6. `src/core/traversability_solver.cpp`
   - 看当前 solver 怎么把 clearance / obstacle_evidence / geometry 合起来

7. `test/core/test_processor.cpp`
   - 看当前行为合同
   - 比看旧解释型文档更靠谱

8. `src/tools/false_obstacle_analyzer.cpp` + `src/tools/miss_obstacle_analyzer.cpp`
   - 看当前工具链到底还能解释什么
   - 也能直接看到旧字段债还剩多少

9. `tools/offline_replay.cpp`
   - 看 bag 回放入口和 false / miss / inspect ROI 的数据流

10. `src/interfaces/ros/ros_param_loader.cpp` + `src/interfaces/ros/converters/output_converter.cpp`
    - 看当前参数真值和 map -> base_gravity 输出语义

11. `docs/passable_area_code_walkthrough.md`
    - 适合作为辅助说明
    - 但遇到 map updater 清障和旧 explanation 叙述时要以代码为准

12. 工作区根 `AGENTS.md`
    - 只作为工程结构和开发流程参考
    - 其中关于当前 obstacle frontend 的描述已经落后，不能当主真值

## 15. Phase 1 完成状态（2026-04-21）
本次已按 `docs/implementation_plan.md` 的 Phase 1 完成类型、枚举、层合同和验收 ROI 冻结，范围保持在 V2 预备重构，不引入 Frontend V2 双层摘要或 reasoner 主链。

已完成：

- 删除 `ObservabilityState::kPartiallyObserved`，保留 `kObserved = 0` 与 `kMissingByDropout = 2` 的编码。
- `SectorObservability` 默认状态改为 `kObserved`。
- `FrameObservabilityEstimator` 不再产生 partial sector；覆盖不足仍通过 `frame_partial` 表达，后方连续缺测仍升级为 `kMissingByDropout`。
- `DropoutAwareMapUpdater` 删除 partial 衰减分支，只保留 observed 与 missing-by-dropout 两级观测衰减。
- 删除 `AmbiguousCandidate` 和 `FrontendOutput::ambiguous_candidates`，`PolarFrontend` 不再生成 ambiguous 输出。
- 新增 `include/passable_area/core/types/obstacle_types.hpp`，冻结 `BlockReason` 与 `ObstaclePointPublishStatus`。
- `TerrainLayers` 新增预留层：`protrusion_height`、`protrusion_evidence`、`overhead_evidence`，初始化/平移合同已接入 `LocalTerrainMap`。
- 新增 `config/acceptance_rois.yaml`，冻结 false obstacle bags 来源、排除项、检测 box，以及 missed obstacle Setup A/B ROI。
- 更新 `test/core/test_processor.cpp`，删除 ambiguous 旧 case，并新增 V2 预留 obstacle layers 初始化测试。
- 同步更新 `tools/offline_replay.cpp` 与 `src/tools/false_obstacle_analyzer.cpp` 中的 partial observability 引用。

验证结果：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 80 tests, 0 errors, 0 failures, 0 skipped`。

注意事项：

- `obstacle_evidence` 仍是当前 solver 和 obstacle point publish 的主证据层；`protrusion_evidence` / `overhead_evidence` 目前只是 Phase 1 预留层，尚未参与运行时判定。
- `obstacle_clear_partial_decay_scale` 参数仍存在，但当前二态观测路径不再消费它；是否删除应放到后续参数清理阶段统一处理。
- `FrameOutput` / analyzer / grid_map 中的旧前端兼容字段尚未清理，仍按 Phase 5 处理。

## 16. Phase 2 完成状态（2026-04-21）
本次已按 `docs/implementation_plan.md` 的 Phase 2 完成 Frontend V2 双层摘要和 MapUpdater 三证据链的最小落地。

已完成：

- `PolarFrontend::run()` 接口从 `LocalTerrainMap` 收窄为 `MapGeometry`，前端不再能读取历史 map layers。
- 新增 `profile_split_gap` 参数，已接入 `Config`、`config/passable_area.yaml` 和 `RosParamLoader`。
- 前端按 cell 收集 z profile，按 `profile_split_gap` 分 band，并用 10% / 90% trimmed bounds 作为 band 高度摘要。
- 前端输出从旧 `ObstacleCandidate` 改为：
  - `SupportCandidate`
  - `ProtrusionCandidate`
  - `OverheadCandidate`
- 前端保留旧 stage debug 字段置位，用于 analyzer 过渡期兼容。
- `DropoutAwareMapUpdater` 分别维护：
  - `support_height` / `support_confidence`
  - `protrusion_height` / `protrusion_evidence`
  - `overhead_height` / `overhead_confidence` / `overhead_evidence`
- 过渡期 `obstacle_evidence` 改为 `max(protrusion_evidence, overhead_evidence)`，继续供当前 solver 与 obstacle point publish 使用。
- `FrameOutput` 和 `grid_map` 暴露 `protrusion_height`、`protrusion_evidence`、`overhead_evidence`，便于后续 reasoner/analyzer 对齐。
- 更新 `test/core/test_processor.cpp`，覆盖 ordinary support、protrusion、overhead、small layered step、sparse lower leak、dropout、三证据链独立更新，以及现有 wall / low ceiling / obstacle point publish 行为。

验证结果：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 84 tests, 0 errors, 0 failures, 0 skipped`。

注意事项：

- 当前 `ObstacleReasoner` 仍未引入，solver 仍消费兼容 `obstacle_evidence`。
- Low clearance 仍通过当前 `overhead_height -> clearance -> solver` 路径影响 passability；Phase 3b 才会正式补齐 low-clearance 到 `/terrain_obstacle_points` 的 reasoner bridge。
- false / miss analyzer root cause 集合仍是旧合同，Phase 3a 需要转向 protrusion / overhead / reasoner / publish gate 解释链。

## 17. Phase 3a 完成状态（2026-04-21）
本次已按 `docs/implementation_plan.md` 的 Phase 3a 引入 shadow `ObstacleReasoner`，并同步更新 analyzer 的原因链解释。该阶段只增加诊断和对比输出，不切换 solver 主判定权。

已完成：

- 新增 ROS-free `ObstacleReasoner`：
  - `include/passable_area/core/obstacle_reasoner.hpp`
  - `src/core/obstacle_reasoner.cpp`
- `ObstacleReasoner` 输入 `TerrainLayers`，输出：
  - `block_reason`
  - `protrusion_stage`
  - `overhead_stage`
  - `obstacle_point_publish_status`
- `Processor` 在 `buildOutput()` 中调用 reasoner，并把 shadow 输出写入 `FrameOutput`。
- `grid_map` 新增发布以下诊断层：
  - `block_reason`
  - `protrusion_stage`
  - `overhead_stage`
  - `obstacle_point_publish_status`
- `false_obstacle_analyzer` 增加 protrusion / overhead driven root cause，hotspot 中同步暴露 protrusion / overhead evidence 与 block reason。
- `miss_obstacle_analyzer` 增加 reasoner / publish gate 相关字段，并新增 `ReasonerNotBlocked` root cause。
- 新增 `test/core/test_obstacle_reasoner.cpp`，覆盖 low clearance、protrusion、geometry failure 和 evidence gate。
- 更新 `docs/algorithm_scheme.md`，把 shadow reasoner 和 Phase 3a 边界写入当前算法说明。

验证结果：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 89 tests, 0 errors, 0 failures, 0 skipped`。

注意事项：

- 当前 `ObstacleReasoner` 是 shadow path；`TraversabilitySolver` 仍按现有 `clearance / obstacle_evidence / geometry` 逻辑生成 `passability`。
- `block_reason` 仍是内部诊断信号，不是新的外部障碍真值；下游导航外部合同仍是 `/terrain_obstacle_points`。
- `obstacle_point_publish_status` 当前表达发布意图 / gate 归因，但 `Processor` 的 obstacle point 筛选逻辑尚未改为消费该字段。
- Phase 3b 的重点仍是 Low Clearance 到 `/terrain_obstacle_points` 的桥接，尤其要保持地面点不过滤进 obstacle points 的既有语义。
- Phase 4 前还没有跑冻结 false obstacle bags 的 reasoner / solver 一致率统计；该验收仍待离线 bag 验证。

## 18. Phase 3b 完成状态（2026-04-21）
本次已按 `docs/implementation_plan.md` 的 Phase 3b 完成 Low Clearance 到 `/terrain_obstacle_points` 的最小桥接。

已完成：

- `Processor::buildOutput()` 新增 reasoner-aware obstacle point 发布资格判断：
  - 保留原有 `obstacle_evidence >= obstacle_points_min_evidence` 兼容路径。
  - 新增 `block_reason == LowClearance` 或包含低净空的 `Mixed`，且 `overhead_evidence >= obstacle_points_min_evidence` 的发布路径。
- 低净空 bridge 仍复用既有 sample 高度门控：
  - `sample.point_in_map.z >= support_ref + obstacle_points_min_height`
  - `sample.point_in_base.z <= obstacle_points_max_height_in_base_link`
- 因此 ground samples under low ceiling 仍不会进入 `/terrain_obstacle_points`。
- 新增 `ProcessorTest.LowClearanceReasonerBridgePublishesOverheadObstaclePoints`，覆盖低净空 reasoner cell 能发布 overhead sample。
- 保留既有 `ObstaclePointsExcludeGroundSamplesUnderLowCeiling`，验证地面点不进入 obstacle points。
- 更新 `docs/algorithm_scheme.md`，说明低净空 reasoner bridge 已接入 obstacle point 发布合同。

验证结果：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 90 tests, 0 errors, 0 failures, 0 skipped`。

注意事项：

- Phase 3b 只补齐低净空到外部 obstacle point 的桥，不切换 `TraversabilitySolver` 主判定权。
- 当前 `obstacle_point_publish_status` 仍主要由 reasoner 按 evidence 预判，尚未根据实际 sample 是否通过高度门控回写为 `GatedByHeight` / `BlockedButNoSamples`。
- Phase 4 仍需要跑冻结 false obstacle bags、miss obstacle Setup A/B 和 timing benchmark，再决定是否让 solver 消费 reasoner 输出。

## 19. Phase 4 验收状态（2026-04-21）
本次已按 `docs/implementation_plan.md` 的 Phase 4 跑冻结 false obstacle bags、冻结 miss obstacle ROI 和 timing benchmark，并把结果固化为当前交接基线。

已完成：

- 新增可复现的 miss-obstacle 验收参数文件：
  - `config/passable_area_map_height_max_0p5.yaml`
  - 用途是替代历史 `/tmp/passable_area_map_height_max_0p5.yaml`，避免继续依赖带退休参数的临时文件。
- false obstacle 验收使用：
  - `config/offline_benchmark_bags.yaml` 中全部冻结 bags，排除 `rosbag2_mtbf_x30_upstair_passage`
  - detection box：`x[0.0, 1.4] y[-0.25, 0.25]`
- miss obstacle 验收使用：
  - Setup A：`rosbag2_open_up_down_stairs`
  - Setup B：`rosbag2_open_stair_and_slope`
  - 参数文件：`config/passable_area_map_height_max_0p5.yaml`
- timing 使用：
  - `offline_replay --benchmark-timing`
  - `build/passable_area/passable_area_benchmark`

false obstacle 结果摘要：

- 7 个冻结 false bags 在检测框内 `suspicious_frames = 0`：
  - `rosbag2_mtbf_down_up_slope`：`0 / 219`
  - `rosbag2_mtbf_long_corridor`：`0 / 138`
  - `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
  - `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
  - `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
  - `rosbag2_mtbf_long_passage`：`0 / 146`
  - `rosbag2_mtbf_short_downstair_1`：`0 / 88`
- 2 个冻结 false bags 出现明显 false obstacle 回潮，且根因集中为 `ClearanceDriven`：
  - `rosbag2_b1_upstairs`：`104 / 156`，`suspicious_ratio = 0.667`，`longest_consecutive_run = 47`
  - `rosbag2_open_short_upstairs`：`39 / 120`，`suspicious_ratio = 0.325`，`longest_consecutive_run = 31`
- 两个异常 bag 的 top hotspots 都呈现相同模式：
  - `clearance below threshold`
  - `obstacle_evidence` 已高
  - `support_continuity` 正常
  - `source_overhead_height` 明显存在
  - 当前更像 Phase 3b low-clearance bridge 把原本仅内部阻挡的低净空路径显式投射到了 `/terrain_obstacle_points`

miss obstacle 结果摘要：

- Setup A `rosbag2_open_up_down_stairs`
  - `left_board`：`missed_frames = 4 / 4`
  - `right_board`：`missed_frames = 4 / 4`
  - root cause 全部为 `EvidenceTooLow`
  - 共同模式：
    - ROI 内样本充足
    - `obstacle_suspicious_cells` / `obstacle_candidate_cells` 已存在
    - `max_obstacle_evidence` 仅约 `0.08 ~ 0.32`
    - 没有任何 frame 达到 `obstacle_points_min_evidence = 0.4`
- Setup B `rosbag2_open_stair_and_slope`
  - `left_side_board`：`missed_frames = 2 / 3`
    - root cause：`EvidenceTooLow`
    - `max_obstacle_evidence` 仅约 `0.09 ~ 0.13`
  - `right_side_board`：`missed_frames = 2 / 3`
    - 1 帧为 `EvidenceTooLow`，`max_obstacle_evidence = 0.32`
    - 1 帧为 `NoObstacleSourceSamplesInRoi`
      - analyzer 已能给出：`strong_evidence_cells = 1` 但 `strong_evidence_cells_with_samples = 0`
      - 说明该帧不是“完全没有障碍证据”，而是强 evidence cell 没有对应当前 ROI 样本通过 obstacle point 发布链

timing 结果摘要：

- `offline_replay --benchmark-timing`
  - `rosbag2_mtbf_long_corridor`：`avg = 10.896 ms`，`max = 18.069 ms`，`p95 = 14.036 ms`
  - `rosbag2_open_short_upstairs`：`avg = 9.223 ms`，`max = 13.979 ms`，`p95 = 12.850 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 8.285 ms`，`max = 14.386 ms`，`p95 = 10.843 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 7.428 ms`，`max = 10.650 ms`，`p95 = 8.736 ms`
  - `rosbag2_b1_upstairs`：`avg = 7.121 ms`，`max = 10.690 ms`，`p95 = 9.846 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 15.60 ms`，`p95 = 17.39 ms`，`p99 = 17.45 ms`
  - `160k points`：`avg = 30.65 ms`，`p95 = 34.28 ms`，`p99 = 34.70 ms`

结论：

- **当前不满足 Phase 4 验收标准，不能切换 solver 主判定权。**
- 主要阻塞项有 3 个：
  - 冻结 false obstacle bags 中已有 2 个 bag 出现明显 `ClearanceDriven` 回潮，说明 low-clearance -> obstacle_points bridge 仍需要额外约束。
  - 冻结 miss obstacle Setup A/B 没有达到“ROI 内 obstacle_points > 0 的帧占比明显改善”这一目标；Setup A 仍是 `4 / 4` 全 miss。
  - 仓库文档此前没有正式 timing baseline，且 `offline_replay --benchmark-timing` 当前只输出 `p95`、不输出 `p99`；因此 Phase 4 里的 `p99 <= baseline * 150%` 还没有形成完整可对照账本。

下一步建议：

- 不要切 `TraversabilitySolver` 去直接消费 `block_reason`。
- 先针对 `rosbag2_b1_upstairs` 和 `rosbag2_open_short_upstairs` 分析 low-clearance false obstacle：
  - 重点看是不是需要在 low-clearance bridge 上增加更严格的 publish gate，而不是继续扩大 reasoner 主判定面。
- 先针对 Setup A/B 的 `EvidenceTooLow` 和 `NoObstacleSourceSamplesInRoi` 收敛问题：
  - 前者更像地图证据累计不足
  - 后者更像 source sample / publish gate 与强 evidence cell 没对齐
- 当前 analyzer 输出已经足够定位这两类失败模式，因此本轮没有继续加新的 debug contract；下一轮优先修行为，再按需要补更细粒度诊断。

## 20. Phase 4 修复尝试（2026-04-21）
本轮按 `docs/phase_4_fix_plan.md` 继续做了根因复盘和代码修复，重点处理 Phase 4 中最突出的两类问题：楼梯 bag 的 `ClearanceDriven` false obstacle 回潮，以及 Setup A/B side board 的 `EvidenceTooLow`。

根因复盘：

- false obstacle 部分，离线逐帧检查后确认：此前把根因单纯归到 Phase 3b 的 reasoner bridge 上并不完整。`rosbag2_b1_upstairs` / `rosbag2_open_short_upstairs` 中的热点 cell 不只是被 bridge 放行，`obstacle_evidence` 本身也已经被 overhead path 顶高，导致 legacy obstacle-point publish 路径同样会出点。
- bag 事实里更稳定的 false pattern 是：
  - `block_reason = LowClearance`
  - `support_continuity` 很高
  - `clearance` 落在典型台阶高差量级
  - 当前 cell `raw_sample_count` 很低（2~5 个）
  - `obstacle_local_triggered / obstacle_upper_patch_confirmed` 有时甚至已经回落，但历史 overhead/obstacle evidence 还在
- miss obstacle 部分，`inspect-roi` 结果否定了“候选经常没有生成”这个假设。Setup A/B side board 的主要问题不是 split gap 不稳定，而是 **pure protrusion cell 的 obstacle evidence 累积速度偏慢**：
  - ROI 内样本数充足，`obstacle_candidate_cell` 稳定存在
  - 但 pure protrusion cell 在 3~4 帧窗口内仍爬不过 `obstacle_points_min_evidence = 0.4`

本轮代码修复：

- `src/core/processor.cpp`
  - 在 obstacle point publish gate 里增加了针对 `LowClearance` 的额外抑制：
    - 只在 **low-clearance + sparse step-like projection** 模式下阻断 publish；
    - 条件目前收敛为：`block_reason == LowClearance`、当前 `raw_sample_count <= 5`、`support_continuity >= 0.8`、且 `overhead_height - support_height` 落在台阶步高量级。
  - 该抑制同时作用于 legacy `obstacle_evidence` 路径和 low-clearance bridge 路径，避免“bridge 被关掉但 legacy 兼容路径仍出点”的假修复。
- `src/core/polar_frontend.cpp`
  - 对 **纯 protrusion（无 overhead 共存）** 的 candidate，把 `gain_scale` 提升到 `3.0f`；
  - overhead 共存的 cell 仍保持 `1.0f`，避免把楼梯/低净空混合场景继续往 false obstacle 方向推。
- `test/core/test_processor.cpp`
  - 同步更新 pure protrusion candidate 的 gain-scale 断言。
  - 保留现有 low-ceiling obstacle-point 行为测试；本轮未把 stair suppression 固化成新的 synthetic 单测，因为合成 case 还原不出 bag 中的累积形态，容易给出误导性失败。

回归验证：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 90 tests, 0 errors, 0 failures, 0 skipped`。

修复后的关键离线结果：

- false obstacle
  - `rosbag2_open_short_upstairs`：`39 / 120 -> 0 / 120`
  - `rosbag2_b1_upstairs`：`104 / 156 -> 3 / 156`
    - 剩余 3 帧仍是 `ClearanceDriven`
    - 当前 residual frame 的共同特征是 `raw_sample_count = 5`，且 `obstacle_local_triggered / obstacle_upper_patch_confirmed` 已回落，只剩历史证据拖尾
- miss obstacle
  - Setup A `open_up_down_stairs`
    - `left_board`: `4 / 4 -> 1 / 4`
    - `right_board`: `4 / 4 -> 2 / 4`
      - root cause 从纯 `EvidenceTooLow` 收敛为 `EvidenceTooLow + NoObstacleSourceSamplesInRoi`
  - Setup B `open_stair_and_slope`
    - `left_side_board`: `2 / 3 -> 2 / 3`
      - 其中最接近发布阈值的 frame 已到 `max_obstacle_evidence = 0.396551`
    - `right_side_board`: `1 / 3 -> 0 / 3`

当前结论：

- **仍然不能切 solver 主判定权。**
- 但 Phase 4 的阻塞面已经明显缩小：
  - false obstacle 从“两个 upstairs bag 大面积回潮”收敛为“只剩 `rosbag2_b1_upstairs` 的 3 帧 residual”
  - miss obstacle 从“Setup A/B 四个 ROI 普遍 `EvidenceTooLow`”收敛为“Setup A 左 1 帧、右 2 帧、Setup B 左 2 帧”
- 现阶段更像进入了最后一轮收边：
  - false 侧要继续处理 `raw_sample_count = 5` 的 low-clearance 历史拖尾
  - miss 侧要继续处理 pure protrusion evidence 的最后几帧，以及 `NoObstacleSourceSamplesInRoi`

## 21. Patch 4 评审替代方案验证（2026-04-21 14:24 +0800）
按 `docs/phase_4_fix_plan_2.md`，本轮把上一版 bag-tuned patch 全部替换成评审要求的原则性方案，并重新完成 build / test / frozen bag 验证。

本轮实际替换项：

- 删除 `IsSparseStepLikeLowClearanceProjection()`，不再使用那组 step-like suppress 魔法数字。
- `HasLowClearanceObstaclePointBridge()` 改成只看：
  - `block_reason == LowClearance`
  - `protrusion_evidence <= 0.05`
  - `overhead_evidence >= obstacle_points_min_evidence`
- `DropoutAwareMapUpdater` 在 `support_reobserved && Observed` 的衰减分支里，对已建立的 protrusion evidence 使用 `obstacle_evidence_decay * 0.4`，替代原先的 `3.0x gain` patch。
- `PolarFrontend` 的 pure protrusion gain 恢复到 Phase 2 值：`overhead_triggered ? 1.0f : 1.5f`。
- obstacle point publish 合同同步收口为：
  - protrusion path 看 `protrusion_evidence`
  - low-clearance path 只看 reasoner bridge
  - 不再让 `/terrain_obstacle_points` 直接消费 `obstacle_evidence = max(protrusion_evidence, overhead_evidence)` 这个兼容别名

回归结果：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 91 tests, 0 errors, 0 failures, 0 skipped`。

冻结验收结果：

- false obstacle
  - `rosbag2_b1_upstairs`: `104 / 156`
  - `rosbag2_open_short_upstairs`: `39 / 120`
- miss obstacle
  - Setup A `open_up_down_stairs`
    - `left_board`: `3 / 4 miss`
    - `right_board`: `4 / 4 miss`
  - Setup B `open_stair_and_slope`
    - `left_side_board`: `2 / 3 miss`
    - `right_side_board`: `1 / 3 miss`

和上一轮 bag-tuned patch 对比，这组评审替代方案在 frozen bags 上**没有达到“不劣于 patch 结果”**，其中 false obstacle 甚至直接回到 Phase 4 原始回潮水平。

新的关键根因结论：

- `protrusion_evidence` 共存门控没有命中 upstairs false bags 的主失败模式。
- 从 frozen false bag 结果看，这些 hotspot cell 主要是 **pure low-clearance / overhead-only** 发布，不是 review 假设里的“同时带有非平凡 protrusion evidence 的 mixed cell”：
  - `block_reason` 仍然稳定是 `LowClearance`
  - `ClearanceDriven` 仍然占绝对多数
  - 改成只让 protrusion path 看 `protrusion_evidence` 后，false 结果没有任何改善，说明当前 false publish 主体仍是 low-clearance bridge 自身，而不是 protrusion/legacy 混发
- 衰减侧保护已建立 protrusion evidence 没有带来 frozen miss ROI 的可见改善；Setup A/B 依旧主要卡在 `EvidenceTooLow`，说明当前 ROI 的主瓶颈仍是 candidate 证据累积不足，而不是已建立 evidence 在 observed-ground 帧被过快清掉。

当前结论：

- **按 `phase_4_fix_plan_2.md` 的替代方案，验证不通过。**
- 这不是实现偏差，而是 frozen bag 结果直接否定了该假设对应的根因判断。
- 因此当前仍然不能切 solver 主判定权，也不能把这一组替代方案当成 Phase 4 修复闭环。

下一步建议：

- 先补 analyzer / inspect 输出，显式把 false hotspot 的 `protrusion_evidence`、`overhead_evidence`、bridge 命中状态、publish path 来源打印出来，避免继续基于误判根因做 patch。
- false 侧应重新针对 **pure low-clearance staircase projection** 建模，而不是继续假设它是 protrusion-overhead coexist 问题。
- miss 侧应重新回到 ROI 内 candidate 累积过程本身，确认是单帧 evidence 太低、frame window 太短，还是 publishable source sample 没落在 strong-evidence cell 上。

## 22. Phase 4 fix plan 3 推进状态（2026-04-21 15:25 +0800）
本轮严格按 `docs/phase_4_fix_plan_3.md` 先补 diagnostics，再改行为，并重新跑 build / test / frozen 验收。仍未切换 `TraversabilitySolver` 主判定权。

已完成的 diagnostics：

- `false_obstacle_analyzer` hotspot 增加 source cell 的 `protrusion_evidence`、`overhead_evidence`、`clearance`、`support_continuity`、`block_reason`、`obstacle_point_publish_status`、low-clearance bridge 命中状态和推断 `source_publish_path`。
- `offline_replay --analyze-false-obstacles` 现在打印 center/source protrusion/overhead evidence、bridge hit、publish path，并在 summary 中列出 protrusion / overhead driven root cause。
- `offline_replay --analyze-missed-obstacles` 的 cell 输出补充 protrusion / overhead evidence、`block_reason` 和 `obstacle_point_publish_status`。
- `--inspect-roi` 补充 ROI 内 max protrusion / overhead evidence、protrusion publish cells、low-clearance bridge cells，以及每个 cell 的 publish path / bridge hit。

本轮行为改动：

- low-clearance bridge 从“只看 low-clearance + overhead evidence”收紧为：
  - `block_reason == LowClearance`
  - `clearance > max_step_up`
  - `overhead_evidence >= obstacle_points_min_evidence`
- obstacle point protrusion path 不再消费兼容别名 `obstacle_evidence = max(...)`，只看 `protrusion_evidence`。
- 连续支撑上的纯 `LowClearance` 投影默认不通过普通 protrusion 阈值发布；但 `protrusion_evidence >= 2 * obstacle_points_min_evidence` 的强 protrusion 仍允许发布，避免把真实侧板强证据一并关掉。
- obstacle point 样本高度门控新增基于现有几何阈值的下界：`point_in_base.z >= -max_step_down`，用于过滤明显低于机器人下行台阶范围的地形落差点。
- pure protrusion gain 保持 Phase 2 基线 `overhead_triggered ? 1.0f : 1.5f`，并对“无 overhead、单 tall band、样本数足够且高于 `max_step_up`”的 wall-like protrusion 再乘 `1.5`。
- 回退无效的 protrusion gentle decay，`DropoutAwareMapUpdater` 恢复 Phase 2 的 observed clear decay 行为。

自动化验证：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 92 tests, 0 errors, 0 failures, 0 skipped`。

false obstacle frozen bags（检测框 `x[0.0, 1.4] y[-0.25, 0.25]`，排除 x30 bag）：

- `rosbag2_mtbf_down_up_slope`：`0 / 219`
- `rosbag2_mtbf_long_corridor`：`0 / 138`
- `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
- `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
- `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
- `rosbag2_mtbf_long_passage`：`0 / 146`
- `rosbag2_mtbf_short_downstair_1`：`0 / 88`
- `rosbag2_b1_upstairs`：`0 / 156`
- `rosbag2_open_short_upstairs`：`0 / 120`

miss obstacle frozen ROIs：

- Setup A `rosbag2_open_up_down_stairs`
  - `left_board`：`2 / 4 miss`
    - 剩余 root cause：`EvidenceTooLow`
  - `right_board`：`4 / 4 miss`
    - root cause：`EvidenceTooLow`
- Setup B `rosbag2_open_stair_and_slope`
  - `left_side_board`：`3 / 3 miss`
    - root cause：`EvidenceTooLow` 2 帧，`UnknownOrMixed` 1 帧
  - `right_side_board`：`3 / 3 miss`
    - root cause：`NoObstacleSourceSamplesInRoi` 1 帧，`UnknownOrMixed` 2 帧

timing：

- `offline_replay --benchmark-timing`
  - `rosbag2_open_short_upstairs`：`avg = 9.991 ms`，`max = 15.898 ms`，`p95 = 15.235 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 9.119 ms`，`max = 17.674 ms`，`p95 = 12.338 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 9.644 ms`，`max = 12.625 ms`，`p95 = 11.917 ms`
  - `rosbag2_b1_upstairs`：`avg = 9.199 ms`，`max = 20.900 ms`，`p95 = 12.812 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 14.60 ms`，`p95 = 15.65 ms`，`p99 = 16.66 ms`
  - `160k points`：`avg = 27.20 ms`，`p95 = 33.28 ms`，`p99 = 33.35 ms`

当前结论：

- false obstacle frozen bags 已归零。
- miss obstacle frozen ROIs 未通过，且 Setup B 在当前更严格 publish gate 下退化为全 miss；剩余问题集中在 `EvidenceTooLow`、强 evidence cell 与当前 ROI source sample 对齐不足，以及低净空混合格子的 publish gate。
- timing 未显示明显异常，但 Phase 4 的功能验收仍未完成。
- **仍然不能切换 solver 主判定权，不能进入 Phase 5。**

建议下一步：

- 保留本轮 diagnostics，它已经能区分 `protrusion_evidence`、`overhead_evidence`、bridge hit 和实际 publish path。
- false 侧当前可作为临时通过结果，但后续任何 miss 修复都必须重新跑完整 false frozen bags，避免强 protrusion 例外扩大 false 回潮。
- miss 侧下一轮应聚焦 source sample 对齐和低净空混合格子的发布资格，而不是继续调整 low-clearance bridge 或切 solver。

## 23. Phase 4 fix plan 4 推进状态（2026-04-21 16:55 +0800）

本轮按 `docs/phase_4_fix_plan_4.md` 继续调查 false 侧 protrusion publish false 和 miss 侧 ROI source sample 对齐问题。仍未切换 `TraversabilitySolver` 主判定权，仍未进入 Phase 5。

本轮关键结论：

- `rosbag2_mtbf_upslope_and_downstair` 之前剩余的 1 帧 false 是 `LowClearance` cell 通过 protrusion publish path 被放出，不是 low-clearance bridge 直接命中。
- 直接把 pure protrusion gain 从 `1.5` 提到 `1.6` 会让 `rosbag2_mtbf_upslope_and_downstair` 回潮到 `5 / 154`，`rosbag2_mtbf_short_downstair_1` 回潮到 `1 / 88`，因此已放弃该全局增益方案。
- Setup B right 的最后 1 帧 miss 不是无候选，而是当前 ROI source cell 有 11~12 个真实样本，`protrusion_evidence ~= 0.375`，距离 `obstacle_points_min_evidence = 0.4` 只差一个很小的 evidence quantum。
- A/B 剩余 miss 不全是同一问题：Setup A left 的最高 evidence 约 `0.32`，Setup A right 仍有 strong evidence cell 与当前 ROI source samples 不重合，Setup B left 有稀疏 source samples 和 low-clearance height gate 两类问题。

本轮行为改动：

- `PolarFrontend` 的 protrusion candidate evidence 改为按 `height_above_support / max_step_up` 归一化并 clamp 到 `[0, 1]`，保留 pure protrusion gain `1.5`，移除此前 wall-like 单 tall band 的额外 `1.5x` boost。
- `/terrain_obstacle_points` protrusion publish path 对 `block_reason == LowClearance` 做直接抑制；`LowClearance` 只能通过 `clearance > max_step_up && overhead_evidence >= obstacle_points_min_evidence` 的 low-clearance bridge 发布。
- obstacle point 下界门控从 `base_link.z >= -max_step_down` 调整为最终发布坐标下的 `base_gravity.z >= -max_step_down + obstacle_points_min_height`，并保留 `base_link.z <= obstacle_points_max_height_in_base_link` ceiling。
- 新增 `DenseProtrusionSource` 近阈值发布资格：非 `LowClearance`、已有 frontend obstacle candidate、`protrusion_evidence >= obstacle_points_min_evidence - obstacle_evidence_gain * 0.1`，且 `raw_sample_count >= min_points_per_sector - 1`。该路径用于修复 dense side-board source cell 的单帧证据对齐问题，避免把 2~8 点的稀疏下楼梯投影放出。
- analyzer / replay diagnostics 同步：
  - miss representative cell 输出 `support_continuity`
  - ROI inspect 输出 `sample_base_z`
  - publish path 增加 `DenseProtrusionSource`

自动化验证：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 95 tests, 0 errors, 0 failures, 0 skipped`。

false obstacle frozen bags（检测框 `x[0.0, 1.4] y[-0.25, 0.25]`，排除 x30 bag）：

- `rosbag2_b1_upstairs`：`0 / 156`
- `rosbag2_open_short_upstairs`：`0 / 120`
- `rosbag2_mtbf_down_up_slope`：`0 / 219`
- `rosbag2_mtbf_long_corridor`：`0 / 138`
- `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
- `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
- `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
- `rosbag2_mtbf_long_passage`：`0 / 146`
- `rosbag2_mtbf_short_downstair_1`：`0 / 88`

miss obstacle frozen ROIs：

- Setup A `rosbag2_open_up_down_stairs`
  - `left_board`：`1 / 4 miss`
    - root cause：`EvidenceTooLow`
  - `right_board`：`2 / 4 miss`
    - root cause：`EvidenceTooLow` 1 帧，`NoObstacleSourceSamplesInRoi` 1 帧
- Setup B `rosbag2_open_stair_and_slope`
  - `left_side_board`：`2 / 3 miss`
    - root cause：`EvidenceTooLow` 2 帧
  - `right_side_board`：`0 / 3 miss`

timing：

- `offline_replay --benchmark-timing`
  - `rosbag2_open_short_upstairs`：`avg = 6.755 ms`，`max = 10.245 ms`，`p95 = 9.689 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 7.505 ms`，`max = 11.256 ms`，`p95 = 10.031 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 6.780 ms`，`max = 9.468 ms`，`p95 = 7.845 ms`
  - `rosbag2_b1_upstairs`：`avg = 6.492 ms`，`max = 9.833 ms`，`p95 = 8.926 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 12.59 ms`，`p95 = 12.87 ms`，`p99 = 13.21 ms`
  - `160k points`：`avg = 24.32 ms`，`p95 = 27.45 ms`，`p99 = 28.13 ms`

当前结论：

- false obstacle frozen bags 仍全 0。
- Setup B right 已恢复到 `0 / 3 miss`，不再劣于 v1 patch 期望。
- Setup A left/right 和 Setup B left 仍未通过，Phase 4 功能验收尚未完成。
- timing 未显示异常。
- **仍然不能切换 solver 主判定权，不能进入 Phase 5。**

建议下一步：

- Setup A left 不应再用全局 protrusion gain 解决；当前最高 evidence 只有约 `0.32`，需要重新判断 ROI 窗口首帧是否应接受更低 evidence、是否应引入更明确的 side-board source aggregation，或是否验收标准需要按帧窗口边界区分。
- Setup A right 的 `NoObstacleSourceSamplesInRoi` 需要继续看 strong-evidence cell 和 ROI source sample 的空间错位，避免把无当前 source sample 的历史强证据直接当成外部障碍。
- Setup B left 的两个剩余帧分别是稀疏 2~3 点 source 和 low-clearance height-gated source，不应套用本轮 dense source gate。

## 24. Phase 4.2 推进状态（2026-04-21 17:20 +0800）

本轮按最新 `docs/phase_4_2_plan.md` 做了唯一行为改动：把 `PolarFrontend` 的 overhead candidate evidence 从原始 `min_clearance - clearance_gap` 改为按不可通行净空区间归一化：

`(min_clearance - clearance_gap) / max(min_clearance - max_step_up, 1e-3)`，并 clamp 到 `[0, 1]`。

本轮没有修改 low-clearance bridge 的发布条件，没有切换 `TraversabilitySolver` 主判定权，也没有进入 Phase 5。

同步改动：

- `src/core/polar_frontend.cpp`
  - 新增局部 `ComputeOverheadEvidence()`，集中表达 overhead evidence 归一化公式。
- `test/core/test_processor.cpp`
  - `PolarFrontendFormsOverheadCandidateForLowClearanceBand` 断言归一化后 evidence 为 `1.0`。
- `docs/algorithm_scheme.md`
  - 同步说明 overhead evidence 公式和 low-clearance bridge 仍保留 `clearance > max_step_up` 发布门控。

自动化验证：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 95 tests, 0 errors, 0 failures, 0 skipped`。

false obstacle frozen bags（检测框 `x[0.0, 1.4] y[-0.25, 0.25]`，排除 x30 bag）：

- `rosbag2_b1_upstairs`：`0 / 156`
- `rosbag2_open_short_upstairs`：`0 / 120`
- `rosbag2_mtbf_down_up_slope`：`0 / 219`
- `rosbag2_mtbf_long_corridor`：`0 / 138`
- `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
- `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
- `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
- `rosbag2_mtbf_long_passage`：`0 / 146`
- `rosbag2_mtbf_short_downstair_1`：`0 / 88`

miss obstacle frozen ROIs：

- Setup A `rosbag2_open_up_down_stairs`
  - `left_board`：`1 / 4 miss`
    - root cause：`EvidenceTooLow`
  - `right_board`：`2 / 4 miss`
    - root cause：`EvidenceTooLow` 1 帧，`NoObstacleSourceSamplesInRoi` 1 帧
- Setup B `rosbag2_open_stair_and_slope`
  - `left_side_board`：`2 / 3 miss`
    - root cause：`EvidenceTooLow` 2 帧
  - `right_side_board`：`0 / 3 miss`

timing：

- `offline_replay --benchmark-timing`
  - `rosbag2_open_short_upstairs`：`avg = 6.838 ms`，`max = 15.810 ms`，`p95 = 9.905 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 7.507 ms`，`max = 11.206 ms`，`p95 = 10.076 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 6.752 ms`，`max = 9.461 ms`，`p95 = 7.862 ms`
  - `rosbag2_b1_upstairs`：`avg = 6.426 ms`，`max = 10.202 ms`，`p95 = 8.839 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 12.60 ms`，`p95 = 13.15 ms`，`p99 = 13.91 ms`
  - `160k points`：`avg = 22.38 ms`，`p95 = 24.95 ms`，`p99 = 24.99 ms`

当前结论：

- overhead evidence 归一化未造成 false obstacle 回潮，frozen false bags 仍全 0。
- Miss ROI 结果与 Phase 4 fix plan 4 后基本一致；Setup A right 没有按 `phase_4_2_plan.md` 的预期从 `2 / 4 miss` 改善到 `<= 1 / 4 miss`。
- Setup A left/right 和 Setup B left 仍未通过，Phase 4 功能验收尚未完成。
- timing 未显示异常。
- **仍然不能切换 solver 主判定权，不能进入 Phase 5。**

下一步建议：

- 不要再做全局 protrusion gain 或全局 overhead gain 调整；false frozen bags 对这种扩张很敏感。
- Setup A right 仍需要继续调查 strong-evidence cell 与当前 ROI source samples 的空间错位，尤其是 `NoObstacleSourceSamplesInRoi` 那一帧。
- Setup A left 仍是 dense side-board protrusion evidence 约 `0.32` 的单帧不足问题；如果继续修，应优先建模 source aggregation 或验收窗口边界，而不是放宽低净空 bridge。
- Setup B left 仍是稀疏 2~3 点 source 和 step-range low-clearance height gate 的组合局限，不应直接套 dense source gate。

## 25. Phase 5 起步：solver 主判定权切到 reasoner（2026-04-21 17:35 +0800）

用户确认：剩余 Setup A/B miss 已属于 source sample 对齐 / 架构级问题，不应继续阻塞 solver 主判定权切换。本轮据此推进 Phase 5 起步，但仍不清理旧兼容字段，不改变 `/terrain_obstacle_points` 外部障碍输出合同。

本轮行为改动：

- `ObstacleReasoner` 从 `Processor::buildOutput()` 的 shadow path 前移到 `TerrainFeatureUpdater` 后、`TraversabilitySolver` 前。
- `TraversabilitySolver::update()` 改为接收 `ObstacleReasonerOutput`。
- `TraversabilitySolver` 仍先判 `UNKNOWN`：
  - coverage 不足
  - support 缺失
  - support confidence 不足
  - stale 超时
- 通过 `UNKNOWN` 保护后，solver 使用 `block_reason` 作为主障碍判定：
  - `Protrusion` / `LowClearance` / `Mixed` / `GeometryFailure` -> `IMPASSABLE`
  - `None` 且几何阈值通过 -> `PASSABLE`
- solver 不再直接用旧兼容 `obstacle_evidence + support_continuity` 组合解释障碍阻挡。
- `GeometryFailure` 会影响 `terrain_state` / `terrain_cost` 的通行性，但仍不 claim `/terrain_obstacle_points` 发布资格。
- `Processor::buildOutput()` 复用同一份 reasoner output，避免同帧重复计算和诊断不一致。

新增测试：

- `ProcessorTest.TraversabilitySolverUsesReasonerBlockReason`
  - 验证 `GeometryFailure` block reason 会让可靠支撑 cell 变为 `IMPASSABLE`，cost 为 `100`。
- `ProcessorTest.TraversabilitySolverKeepsUnknownPriorityOverReasonerBlock`
  - 验证无可靠支撑 / unknown cell 即使 reasoner 给出 block reason，也保持 `UNKNOWN`，cost 为 `-1`。

自动化验证：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 93 tests, 0 errors, 0 failures, 0 skipped`。

false obstacle frozen bags（检测框 `x[0.0, 1.4] y[-0.25, 0.25]`，排除 x30 bag）：

- `rosbag2_b1_upstairs`：`0 / 156`
- `rosbag2_open_short_upstairs`：`0 / 120`
- `rosbag2_mtbf_down_up_slope`：`0 / 219`
- `rosbag2_mtbf_long_corridor`：`0 / 138`
- `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
- `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
- `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
- `rosbag2_mtbf_long_passage`：`0 / 146`
- `rosbag2_mtbf_short_downstair_1`：`0 / 88`

miss obstacle frozen ROIs：

- Setup A `rosbag2_open_up_down_stairs`
  - `left_board`：`1 / 4 miss`
    - root cause：`EvidenceTooLow`
  - `right_board`：`2 / 4 miss`
    - root cause：`EvidenceTooLow` 1 帧，`NoObstacleSourceSamplesInRoi` 1 帧
- Setup B `rosbag2_open_stair_and_slope`
  - `left_side_board`：`2 / 3 miss`
    - root cause：`EvidenceTooLow` 2 帧
  - `right_side_board`：`0 / 3 miss`

timing：

- `offline_replay --benchmark-timing`
  - `rosbag2_open_short_upstairs`：`avg = 6.772 ms`，`max = 11.178 ms`，`p95 = 9.822 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 7.459 ms`，`max = 11.932 ms`，`p95 = 10.071 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 6.793 ms`，`max = 9.206 ms`，`p95 = 7.833 ms`
  - `rosbag2_b1_upstairs`：`avg = 6.489 ms`，`max = 12.518 ms`，`p95 = 8.938 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 12.57 ms`，`p95 = 12.79 ms`，`p99 = 12.93 ms`
  - `160k points`：`avg = 21.66 ms`，`p95 = 24.59 ms`，`p99 = 24.67 ms`

当前结论：

- solver 主判定权已切到 reasoner 输出。
- `/terrain_obstacle_points` 外部输出合同未变，false frozen bags 仍全 0。
- Miss ROI 结果与 Phase 4.2 一致；剩余 miss 不再阻塞 solver 切换，但仍应作为后续架构改动输入。
- timing 未显示异常。
- 允许继续 Phase 5 后续清理，但建议分批做：
  - 先清理明显不再使用的 solver 旧障碍解释路径和文档表述。
  - 再逐步清理 `FrameOutput` / analyzer / grid_map 中旧前端兼容字段，避免一次性打断诊断工具。

## 26. DenseProtrusionSource 删除尝试与保留结论（2026-04-21 17:45 +0800）

本轮在正式继续 Phase 5 清理前，专门复核 `Processor::HasDenseNearThresholdProtrusionSource()` 是否可以删除。审查问题是：该路径中的 `0.1f` 不是物理阈值，而是把 `obstacle_points_min_evidence = 0.4` 在 dense source cell 上条件性放宽到默认约 `0.375`。

本轮实际尝试：

- 从 `src/core/processor.cpp` 删除 `HasDenseNearThresholdProtrusionSource()`，只允许：
  - `protrusion_evidence >= obstacle_points_min_evidence`
  - 或 low-clearance bridge
  驱动 `/terrain_obstacle_points`。
- 同步删除 analyzer / offline replay 中的 `DenseProtrusionSource` publish path 推断分支。
- 将对应 processor 测试临时改成 dense near-threshold source 也不发布。
- 同步删除 `docs/algorithm_scheme.md` 当前算法描述里的 DenseProtrusionSource bullet。

删除态验证结果：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 97 tests, 0 errors, 0 failures, 0 skipped`。
- false obstacle frozen bags 仍全 0。
- miss obstacle frozen ROIs 出现退化：
  - Setup A `left_board`：`1 / 4 miss`
  - Setup A `right_board`：`2 / 4 miss`
  - Setup B `left_side_board`：`2 / 3 miss`
  - Setup B `right_side_board`：从 `0 / 3 miss` 退化到 `1 / 3 miss`
- Setup B right 退化帧的代表 source cell 仍是 dense source，`raw_sample_count = 11~12`，`protrusion_evidence ~= 0.38 < 0.4`，与此前 DenseProtrusionSource 修复的目标一致。

因此本轮已恢复 DenseProtrusionSource 路径，并在 `docs/algorithm_scheme.md` 中明确记录 `0.1f` 的依据：

- 它不是物理高度阈值，也不是新障碍真值。
- 它是单帧证据量化容差：`obstacle_evidence_gain * 0.1`。
- 默认 `obstacle_evidence_gain = 0.25` 时，容差为 `0.025`，即默认发布阈值从 `0.4` 临时放宽到 `0.375`。
- 仅在非 `LowClearance`、已有本帧 frontend obstacle candidate、且 `raw_sample_count >= min_points_per_sector - 1` 的 dense current source cell 上生效。
- 若后续 evidence 累积/归一化架构能消除该短窗口量化误差，应优先删除该路径，而不是继续扩大容差。

恢复态验证结果：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 97 tests, 0 errors, 0 failures, 0 skipped`。

false obstacle frozen bags：

- `rosbag2_b1_upstairs`：`0 / 156`
- `rosbag2_open_short_upstairs`：`0 / 120`
- `rosbag2_mtbf_down_up_slope`：`0 / 219`
- `rosbag2_mtbf_long_corridor`：`0 / 138`
- `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
- `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
- `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
- `rosbag2_mtbf_long_passage`：`0 / 146`
- `rosbag2_mtbf_short_downstair_1`：`0 / 88`

miss obstacle frozen ROIs：

- Setup A `left_board`：`1 / 4 miss`，root cause `EvidenceTooLow`
- Setup A `right_board`：`2 / 4 miss`，root cause `EvidenceTooLow` 1 帧，`NoObstacleSourceSamplesInRoi` 1 帧
- Setup B `left_side_board`：`2 / 3 miss`，root cause `EvidenceTooLow` 2 帧
- Setup B `right_side_board`：`0 / 3 miss`

timing：

- `offline_replay --benchmark-timing`
  - `rosbag2_open_short_upstairs`：`avg = 6.765 ms`，`max = 10.446 ms`，`p95 = 9.841 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 7.373 ms`，`max = 11.066 ms`，`p95 = 9.726 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 6.647 ms`，`max = 9.273 ms`，`p95 = 7.645 ms`
  - `rosbag2_b1_upstairs`：`avg = 6.383 ms`，`max = 9.305 ms`，`p95 = 8.691 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 12.71 ms`，`p95 = 15.09 ms`，`p99 = 15.31 ms`
  - `160k points`：`avg = 21.81 ms`，`p95 = 25.38 ms`，`p99 = 25.52 ms`

当前结论：

- DenseProtrusionSource 当前仍需要保留，否则 Setup B right frozen ROI 退化。
- 允许继续 Phase 5 后续清理；但不要在同一轮删除 DenseProtrusionSource，除非同时引入新的 evidence 架构替代短窗口量化容差。

## 27. Phase 5 清理：移除旧前端兼容字段与历史 grid_map/debug 合同（2026-04-21 19:25 +0800）

本轮继续推进 Phase 5，不改 runtime 障碍判定和 `/terrain_obstacle_points` 外部合同，只清理已经失效的旧前端兼容字段、analyzer 解释链和 ROS debug 输出。

本轮代码改动：

- 从 `FrontendOutput` / `FrameOutput` 删除旧前端兼容字段：
  - `support_anchor_used`, `support_anchor_origin`, `support_anchor_authority`
  - `anchor_leak_suppression_enabled`, `sub_support_leak_count`
  - `anchor_below_observation_count`, `stale_anchor_residual_filtered_count`
  - `raw_upper_support_cell`, `explanation_adjusted_upper_support_cell`, `upper_support_cell`
  - `obstacle_local_triggered`, `obstacle_upper_patch_confirmed`, `obstacle_explanation_rejected`
  - `obstacle_rejected_by_neighbor_support`
  - `neighbor_upper_support_count`, `aligned_neighbor_support_count`
  - `explanation_decision`
  - `facade_lower_upper_coexisting`, `facade_upper_edge_aligned_with_supported_neighbors`
- `PolarFrontend` / `Processor::buildOutput()` 不再初始化和透传这些 inert compatibility payload。
- `false_obstacle_analyzer` 与 `miss_obstacle_analyzer` 改成只消费当前 V2 合同：
  - `obstacle_suspicious`
  - `obstacle_candidate_cell`
  - `obstacle_evidence` / `protrusion_evidence` / `overhead_evidence`
  - `block_reason`
  - `obstacle_point_publish_status`
  - raw / filtered sample 统计
- `miss_obstacle_analyzer` root cause 缩减为当前有效集合：
  - `NoSamplesInRoi`
  - `NoFrontendCandidate`
  - `EvidenceTooLow`
  - `PublishHeightGated`
  - `NoObstacleSourceSamplesInRoi`
  - `UnknownOrMixed`
  - `ReasonerNotBlocked`
- `offline_replay` 的 false / miss 打印与 ROI inspect 输出同步删掉旧 explanation / anchor / neighbor-support 字段，只保留当前有效状态。
- `output_converter` 不再向 `grid_map` 发布 `support_anchor_used` / `sub_support_leak_count` 历史层。
- 对应 core/tool/ROS tests 已同步改成守当前合同，不再断言 inert compatibility 默认值。

自动化验证：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 97 tests, 0 errors, 0 failures, 0 skipped`。

false obstacle frozen bags：

- `rosbag2_b1_upstairs`：`0 / 156`
- `rosbag2_open_short_upstairs`：`0 / 120`
- `rosbag2_mtbf_down_up_slope`：`0 / 219`
- `rosbag2_mtbf_long_corridor`：`0 / 138`
- `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
- `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
- `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
- `rosbag2_mtbf_long_passage`：`0 / 146`
- `rosbag2_mtbf_short_downstair_1`：`0 / 88`

miss obstacle frozen ROIs：

- Setup A `rosbag2_open_up_down_stairs`
  - `left_board`：`1 / 4 miss`
    - root cause：`EvidenceTooLow`
  - `right_board`：`2 / 4 miss`
    - root cause：`EvidenceTooLow` 1 帧，`NoObstacleSourceSamplesInRoi` 1 帧
- Setup B `rosbag2_open_stair_and_slope`
  - `left_side_board`：`2 / 3 miss`
    - root cause：`EvidenceTooLow` 2 帧
  - `right_side_board`：`0 / 3 miss`

timing：

- `offline_replay --benchmark-timing --bag ...`
  - `rosbag2_open_short_upstairs`：`avg = 6.693 ms`，`max = 10.209 ms`，`p95 = 9.385 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 7.426 ms`，`max = 10.902 ms`，`p95 = 9.869 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 6.785 ms`，`max = 9.248 ms`，`p95 = 8.357 ms`
  - `rosbag2_b1_upstairs`：`avg = 6.304 ms`，`max = 9.341 ms`，`p95 = 8.658 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 12.68 ms`，`p95 = 13.53 ms`，`p99 = 14.84 ms`
  - `160k points`：`avg = 24.44 ms`，`p95 = 29.08 ms`，`p99 = 31.66 ms`

当前结论：

- solver 主判定权已在 Phase 5 起步阶段切到 reasoner，本轮不回退。
- `/terrain_obstacle_points` 外部障碍输出合同未变，false frozen bags 仍全 0。
- frozen miss ROI 结果未退化。
- timing 无异常。
- 允许继续 Phase 5 后续清理；下一步可以继续收缩 docs 中残余“旧 explanation 语义仍在现役”的表述，并检查外部是否仍有人消费已删除的历史 grid_map 层。

## 28. Phase 5 收口：docs 终清与 solver 去重（2026-04-21 19:55 +0800）

本轮做最后一批 Phase 5 收口动作：

- 继续清理 `docs/algorithm_scheme.md` 中残余的旧 explanation / anchor / neighborhood suppression 表述，确保文档只描述当前 V2 主线。
- `TraversabilitySolver` 去掉 `block_reason == None` 后的几何二次否决：
  - 仍保留 `UNKNOWN` gate
  - `block_reason` 为阻挡时判 `IMPASSABLE`
  - 其余可靠 cell 直接判 `PASSABLE`
  - 几何阈值只继续参与 passable cost 细分，不再作为 solver 的第二套阻挡真值
- 复核 `PolarFrontend.hpp` 与 `LocalTerrainMap.hpp`：
  - 未发现为旧解释器保留的额外成员变量或内部计数器
  - 本轮无须在这两个头文件继续裁剪成员

新增测试：

- `ProcessorTest.TraversabilitySolverDoesNotRedoGeometryFailureChecks`
  - 人工构造“几何值超阈但 reasoner 未给阻挡原因”的不一致输入
  - 验证 solver 现在按 contract 信任 `block_reason == None`，保持 `PASSABLE`

自动化验证：

- `colcon build --packages-select passable_area --symlink-install` 通过。
- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `colcon test-result --verbose`：`Summary: 97 tests, 0 errors, 0 failures, 0 skipped`。

false obstacle frozen bags：

- `rosbag2_b1_upstairs`：`0 / 156`
- `rosbag2_open_short_upstairs`：`0 / 120`
- `rosbag2_mtbf_down_up_slope`：`0 / 219`
- `rosbag2_mtbf_long_corridor`：`0 / 138`
- `rosbag2_mtbf_upstair_and_downslope`：`0 / 129`
- `rosbag2_mtbf_upstair_and_downslope_2`：`0 / 115`
- `rosbag2_mtbf_upslope_and_downstair`：`0 / 154`
- `rosbag2_mtbf_long_passage`：`0 / 146`
- `rosbag2_mtbf_short_downstair_1`：`0 / 88`

miss obstacle frozen ROIs：

- Setup A `rosbag2_open_up_down_stairs`
  - `left_board`：`1 / 4 miss`
  - `right_board`：`2 / 4 miss`
- Setup B `rosbag2_open_stair_and_slope`
  - `left_side_board`：`2 / 3 miss`
  - `right_side_board`：`0 / 3 miss`

timing：

- `offline_replay --benchmark-timing --bag ...`
  - `rosbag2_open_short_upstairs`：`avg = 6.591 ms`，`max = 10.026 ms`，`p95 = 9.407 ms`
  - `rosbag2_open_up_down_stairs`：`avg = 6.422 ms`，`max = 8.735 ms`，`p95 = 7.771 ms`
  - `rosbag2_open_stair_and_slope`：`avg = 6.439 ms`，`max = 8.413 ms`，`p95 = 7.320 ms`
  - `rosbag2_b1_upstairs`：`avg = 5.779 ms`，`max = 8.668 ms`，`p95 = 8.042 ms`
- `build/passable_area/passable_area_benchmark`
  - `80k points`：`avg = 12.46 ms`，`p95 = 12.74 ms`，`p99 = 13.32 ms`
  - `160k points`：`avg = 21.67 ms`，`p95 = 24.91 ms`，`p99 = 26.08 ms`

当前结论：

- `algorithm_scheme.md` 已对齐到当前 V2 主线，不再把旧 explanation / anchor / neighborhood suppression 写成现役逻辑。
- solver 现在只整合 `UNKNOWN` 与 `ObstacleReasoner` 的阻挡结论，不再维护第二套几何阻挡真值。
- frozen false / miss / timing 均未退化。
- 允许把本轮作为 Phase 5 的正式收口状态。

## 29. 后续试验：移除 obstacle point 发布路径的 `base_gravity` 下界过滤（2026-04-21 20:45 +0800）

本轮是一个有意保留在工作树中的试验性行为改动，用于观察 `/terrain_obstacle_points` 在下行/低位样本上的实际输出变化：

- `Processor::PassesObstaclePointPublishHeightGates()` 删除了这一条发布门控：
  - `point_in_base_gravity.z >= -max_step_down + obstacle_points_min_height`
- 当前 obstacle point 发布高度门控只剩：
  - `sample.point_in_map.z >= support_ref + obstacle_points_min_height`
  - `sample.point_in_base.z <= obstacle_points_max_height_in_base_link`
- 这次改动没有触碰：
  - solver 主判定权
  - reasoner block_reason 语义
  - `/terrain_obstacle_points` 作为下游唯一外部障碍输出合同

自动化验证：

- `colcon test --packages-select passable_area --event-handlers console_direct+` 通过。
- `test_processor`、`test_false_obstacle_analyzer`、`test_miss_obstacle_analyzer` 均通过。

当前结论：

- 代码和单测层面无回归，适合继续做 bag 级效果验证。
- 本轮尚未重跑 false frozen bags / miss ROIs / timing benchmark，因此还不能把该行为变更记为已完成离线验收。
