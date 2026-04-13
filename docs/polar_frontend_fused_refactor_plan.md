# PolarFrontend 重构新方案

## 0. 方案定位

这版方案不是简单在“方案一”和“方案二”之间折中，而是明确分工：

- **核心设计采用方案二**
  - 用更清晰的主链重定义 `PolarFrontend` 的语义边界
  - 把前端收敛成一条稳定、可解释、可验证的候选形成链
- **工程推进采用方案一**
  - 用分阶段落地、每阶段设 hard gate 的方式控制回归风险
  - 避免一次性大改导致 false obstacle / miss obstacle 同时漂移

因此，这版方案的目标不是“重写更多逻辑”，而是：

> **先把前端主链语义收干净，再按小步可验证的方式逐步替换现有 patch 风格 gate。**

---

## 1. 现状判断：真正的问题是什么

当前 `PolarFrontend` 的核心问题，不是规则数量多本身，而是：

1. **同一类邻域信号承担了多种角色**
   - 有时用来借 anchor
   - 有时用来确认 obstacle
   - 有时又用来解释成非障碍

2. **同一几何概念被重复表达**
   - upper band
   - upper support
   - upper layer consensus
   - aligned neighbor support
   - ascending trend
   - stale lower anchor mix

   这些概念分散在不同 pass 和不同 helper 里，导致一处改动牵动多处行为。

3. **explanation 链和 candidate 主链缠在一起**
   - `effective_support_ref`、`raw_support_ref`、`support_ref` 同时存在
   - 部分解释逻辑既像“否决理由”，又像“第二套支持基线”
   - 最终 obstacle gate 读了过多中间信号，语义边界不干净

4. **patch 风格的 case-specific helper 已经开始主导主链**
   - `reject_stale_anchor`
   - `reject_wall_only_anchor`
   - `ShouldRejectBelowRobot*`
   - `ShouldUseElevatedEffectiveSupportRef`

   它们各自解决过真实问题，但现在最大的问题是：**入口分散、共享概念不统一、难以预测联动效应。**

---

## 2. 重构目标

这轮重构的目标不是“减少代码行数”，而是把 `PolarFrontend` 收敛成一个职责清晰的前端解释器：

- 输入：当前帧 `odom_samples` 与局部地图 support 信息
- 输出：
  - `support_candidates`
  - `obstacle_candidates`
  - 一组仅供调试 / 解释使用的前端状态层

它应该只负责：

1. 按 cell 聚合当前帧观测
2. 为每个 cell 解析 support 基线
3. 基于当前帧构建局部几何 profile
4. 判定该 cell 是否值得进入 obstacle 检查
5. 用邻域一致性做确认
6. 用 explanation 做单向否决
7. 输出 candidate 与可调试解释层

它**不应**负责：

- 地图长期状态写回
- 输出层 suppress
- 用 evidence 增长速率弥补前端语义混乱
- 把 analyzer / offline replay 变成主链补丁位

---

## 3. 新主链设计

这版方案采用方案二的核心设计，把前端固定为下面这条链：

```text
AnchorResolution
  -> LocalCellProfile
  -> SuspiciousTrigger
  -> AdjacencyContext
  -> ExplanationDecision
  -> CandidateEmission
```

### 3.1 AnchorResolution

职责：

- 为每个有样本的 cell 解析唯一的 `support_ref`
- 允许来源：
  - 当前 cell 的历史 `support_height`
  - 邻域借 anchor（如 3×3 中位数或等价稳健聚合）

输出：

- `support_ref`
- `anchor_source`
- `anchor_validity`

这里要统一现有的：

- `reject_stale_anchor`
- `reject_wall_only_anchor`

改成单一结果：

```text
AnchorValidityDecision
  - Valid
  - InvalidStale
  - InvalidWallOnly
  - InvalidInsufficientSupport
```

要点：

- 统一表达“这个 anchor 当前帧还是否可信”
- 不再把不同 invalid case 直接散落成多个布尔门
- anchor invalid 只影响 support 基线与样本分层，不直接等价为 obstacle 或 non-obstacle

---

### 3.2 LocalCellProfile

职责：

基于统一过滤后的样本，构建当前 cell 的局部几何描述，只回答：

> “这个 cell 自己到底看到了什么高度结构？”

输出建议统一成一个结构，例如：

```text
LocalCellProfile
  - sample_count
  - min_z
  - max_z
  - vertical_span
  - support_band_count
  - upper_band_count
  - sub_support_leak_count
  - min_upper_band_z
  - max_upper_band_z
  - anchor_reobserve_count
  - below_anchor_count
```

要点：

- 所有后续判定都尽量只读取 `LocalCellProfile`
- 不允许后段再自己重复扫样本生成新的“局部事实”
- `stale_lower_anchor_mix` 这类重复表达，应该在这里或这里附近一次性产出标准化字段，而不是在后段重复写表达式

---

### 3.3 SuspiciousTrigger

职责：

只回答：

> “这个 cell 是否值得进入 obstacle 候选检查？”

这里坚持两个原则：

1. **只能由 local evidence 触发**
2. **邻域信号不能直接触发 suspicious**

第一轮建议保留现有低成本触发基线：

- `vertical_span` 为主
- 配合 `upper_band_count > 0` / 等价局部上层存在性条件

例如：

```text
suspicious = local_vertical_separation_sufficient
```

注意：

- `suspicious` 只是“值得检查”
- 不是最终 obstacle
- 也不是 explanation 的反面

---

### 3.4 AdjacencyContext

职责：

只回答：

> “如果这个 cell 已经 suspicious，邻域是否支持它是一个空间连续的上层结构 / 障碍结构？”

输出建议统一成一个结构：

```text
AdjacencyContext
  - neighbor_upper_support_count
  - aligned_neighbor_support_count
  - neighbor_continuity_score
  - adjacency_confirmed
```

严格约束：

- adjacency **只能做确认**
- adjacency **不能替代 local evidence**
- 没有 local suspicious，不允许仅凭邻域升成 obstacle

第一轮建议保留现有 3×3 window，不先改窗口大小，只重写语义边界。

---

### 3.5 ExplanationDecision

职责：

只回答：

> “这个 suspicious cell 是否更应该被解释为 layered ground / stair transition / invalid anchor，而不是 obstacle？”

这一步是这轮重构的核心。

要把现有：

- `ShouldRejectBelowRobotStairMix`
- `ShouldRejectBelowRobotGroundLayerMix`
- `ShouldRejectBelowRobotUpstairGroundMix`
- `ShouldUseElevatedEffectiveSupportRef`

统一收敛成**单一 explanation 分类器**。

建议定义为：

```text
ExplanationDecision
  - None
  - InvalidAnchor
  - LayeredGroundMix
  - StairTransitionMix
  - LowerAnchorDominatedUpperBand
```

它只做一件事：

- **单向否决 obstacle**
- 不允许 explanation 直接造 obstacle
- 不允许 explanation 替代 suspicious
- 不允许 explanation 替代 adjacency confirm

#### 关于 effective_support_ref

这版方案不再把“是否提升 effective_support_ref”作为一条和 candidate gate 并列博弈的副主链。

改法：

- `effective_support_ref` 只允许作为 `ExplanationDecision` 的内部推导结果或辅助解释量
- 它是 **frontend-local explanation artifact**
- 不能再变成到处参与后段 gate 的第二支持基线
- 更不能写回地图

也就是说：

> 不是“是否用 elevated ref 来决定 candidate”，而是“某类 layered / stair mix explanation 在内部可能需要临时 elevated reference 来判断是否成立”。

这样能明显减轻 `support_ref / raw_support_ref / effective_support_ref` 三套语义并存的问题。

---

### 3.6 CandidateEmission

最终候选形成链建议固定成：

```text
obstacle_candidate =
    suspicious
    && local_upper_structure_present
    && adjacency_confirmed
    && explanation == None
```

同理：

- `support_candidate` 仍然由 support 基线与 coverage 逻辑生成
- 但不允许 support_candidate 路径再反向补 obstacle 语义

这一步的关键不是公式本身，而是每个条件都有唯一职责：

- `suspicious`：局部触发
- `local_upper_structure_present`：当前 cell 的上层事实
- `adjacency_confirmed`：空间连续性确认
- `explanation == None`：单向否决通过

---

## 4. 三类证据的正式定义

这是整个方案最重要的“立法层”。

### 4.1 Local evidence

定义：

- 完全来自本 cell 当前帧过滤后的样本分布

包含：

- `vertical_span`
- `upper_band_count`
- `sub_support_leak_count`
- `min/max upper band z`
- `anchor_reobserve_count`
- `below_anchor_count`

用途：

- 决定是否 suspicious
- 决定局部是否存在上层结构
- 为 explanation 提供本地事实

### 4.2 Adjacency signal

定义：

- 来自邻域 cell 的空间一致性支持

包含：

- 邻域 upper-support 计数
- 对齐的邻域 support 数
- 连续性 / 聚簇性信号

用途：

- 只做 suspicious 的确认
- 不能替代 local trigger
- 不能直接造 obstacle
- 不能兼任 explanation

### 4.3 Explanation signal

定义：

- 解释“为什么这个 suspicious cell 更像非障碍结构”的证据

包含：

- invalid anchor
- layered ground
- stair transition
- stale lower anchor dominated upper band
- 需要时的 frontend-local elevated reference 推导

用途：

- 只做 obstacle 否决
- 不能触发 suspicious
- 不能提供 adjacency confirm
- 不能正向造障碍

---

## 5. 需要保留的核心行为

这轮重构不是推倒重来。以下核心行为建议第一轮必须保留：

1. **基于历史 / 邻域的 anchor 借用**
   - 这是 sub-support leak suppression 的基础

2. **sub-support leak filtering**
   - 这是当前 false obstacle 抑制里最接近第一性原理的一部分

3. **vertical_span 触发 suspicious**
   - 第一轮先保留，避免触发基线整体漂移

4. **3×3 邻域确认**
   - 第一轮保留 window，不先改范围

5. **support candidate 输出逻辑**
   - 保持当前对地图更新主链的兼容性

---

## 6. 计划删除 / 合并的逻辑

### 6.1 第一优先级合并

- `reject_stale_anchor`
- `reject_wall_only_anchor`

合并为：

- `AnchorValidityDecision`

### 6.2 第二优先级合并

- `ShouldRejectBelowRobotStairMix`
- `ShouldRejectBelowRobotGroundLayerMix`
- `ShouldRejectBelowRobotUpstairGroundMix`

合并为：

- `ExplanationDecision::{LayeredGroundMix, StairTransitionMix, ...}`

### 6.3 第三优先级收敛

- `ShouldUseElevatedEffectiveSupportRef`

不再单独作为对外 gate helper，而是内收到 explanation classifier 内部。

### 6.4 可延后处理

- `ComputeWallLikeObstacleGainScale`

这轮不建议优先处理。理由：

- 它影响地图 evidence 增速
- 不是 candidate 主链语义的根问题
- 应在主链理顺后再评估是否迁走到 `DropoutAwareMapUpdater`

---

## 7. 工程推进方案（采用方案一的分阶段推进）

这部分明确采用方案一的工程推进方式：**小步、分阶段、每阶段设 hard gate。**

---

## Phase A：结构重组，不改行为

目标：

- 先把 `run()` 的主流程拆清楚
- 不改变现有语义
- 只为后续合并 gate 做准备

实施内容：

1. 将 `PolarFrontend::run()` 重组为显式 phase：
   - `ResolveAnchors(...)`
   - `BuildLocalProfiles(...)`
   - `MarkSuspiciousCells(...)`
   - `BuildAdjacencyContexts(...)`
   - `BuildExplanationInputs(...)`
   - `EmitCandidates(...)`

2. 引入统一的 per-cell workspace，例如：

```text
CellWorkspace
  - raw aggregation
  - anchor context
  - local profile
  - adjacency context
  - explanation scratch
```

3. 消除明显重复表达：
   - `stale_lower_anchor_mix` 只算一次
   - 上层 band / leak / anchor reobserve 的中间量统一收口

验收标准：

- `colcon test` 全绿
- 行为对齐，benchmark 不应出现显著漂移

---

## Phase B：统一 AnchorValidityDecision

目标：

- 把 anchor 可信度判断从多个散落布尔门收敛成统一结果

实施内容：

1. 合并：
   - `reject_stale_anchor`
   - `reject_wall_only_anchor`

2. 明确 anchor validity 输出枚举

3. 下游逻辑只消费统一的 `anchor_validity`

验收标准：

- 现有 stale anchor / wall-only anchor 相关 gtest 全 pass
- false obstacle benchmark 全量 bag 通过
- 如有回归，不允许继续叠 patch，先回到 `AnchorValidityDecision` 设计层分析

---

## Phase C：建立统一的 ExplanationDecision

目标：

- 把 below-robot / layered / stair 类 explanation 收敛成一条主路径

实施内容：

1. 引入 `ExplanationDecision` 枚举
2. 将 3 个 `ShouldRejectBelowRobot*` helper 合并
3. 所有 obstacle 否决只看统一 explanation 结果

实现要求：

- explanation 必须是单向否决
- 不允许 explanation 兼任 trigger 或 confirm
- 必须能映射出 reason enum，方便调试和 benchmark 追因

验收标准：

- 现有 below-robot 相关 gtest 全 pass
- false obstacle benchmark 全量 bag 通过
- 指定 miss hard gate `missed_frames = 0`

---

## Phase D：收敛 effective_support_ref 语义

目标：

- 让 `effective_support_ref` 退出主链博弈位
- 只保留为 explanation 内部的辅助解释量

实施内容：

1. 取消“外部 gate 直接读取 elevated effective support ref”的模式
2. 统一为 explanation classifier 内部推导
3. 清理对外暴露的多套 support 基线依赖
4. 保留 legacy `upper_support_cell = explanation_adjusted_upper_support_cell` 作为兼容语义，
   raw vs adjusted 通过显式调试字段区分

验收标准：

- 所有 `UsesElevatedEffectiveSupportRef*` / 同类守卫测试通过
- false benchmark 与 miss hard gate 双通过
- 如果失败，优先说明“是否这条语义本就不该保留”，而不是立刻补新 patch

---

## Phase E：评估是否迁移 wall_like_gain_scale

目标：

- 在 candidate 主链稳定后，再判断是否把 gain scale 从 frontend 迁到 map updater

实施内容：

1. 梳理当前 `gain_scale` 对 wall 类障碍保持的真实收益
2. 若迁移，必须保证：
   - frontend 只传几何候选特征
   - updater 决定 evidence 增速

验收标准：

- wall 相关 gtest 全通过
- 无新 miss regression

---

## 8. 测试与验证要求

这部分沿用方案一的工程纪律，不放松。

### 8.1 单元测试要求

必须保留当前主干回归测试，不删期望值。

同时建议增加新的“语义层测试”，直接验证三分法：

1. `LocalCellProfile` 正确反映本 cell 分层事实
2. `AdjacencyContext` 只做确认，不造 trigger
3. `ExplanationDecision` 只做否决，不造 obstacle
4. `AnchorValidityDecision` 正确统一 stale / wall-only case

这样比继续堆更多 bag-specific 场景名更健康。

---

### 8.2 False obstacle benchmark

要求：

- 全量基准 bag 全通过
- 每个 bag 的 `suspicious_frames` 维持 hard gate

原则：

- 任一 bag 失败，不允许继续叠 patch“修那个 bag”
- 必须回到：
  - `LocalCellProfile`
  - `AdjacencyContext`
  - `ExplanationDecision`

  这三个设计层复盘

---

### 8.3 Miss obstacle hard gate

必须保留你当前指定的 hard gate：

- `rosbag2_mtbf_upstair_and_downslope_2`
- 目标 `missed_frames = 0`

而且要明确：

- 如果 false 变好了但 miss gate 破了，这版就不能接受
- 解释逻辑再优雅，只要吃掉真实障碍，就必须回退

---

## 9. 风险清单

### 9.1 最大风险

**解释逻辑过强，吃掉真实障碍。**

对应阶段：

- Phase C
- Phase D

表现：

- below-robot 真障碍被归类为 layered/stair explanation
- miss hard gate 直接失败

### 9.2 第二风险

**AnchorValidityDecision 收得过宽，导致 support_ref 漂移。**

表现：

- 前端 candidate 数量变化不大
- 但 support 基线变了，后续 obstacle_points 发布门槛被连带影响

### 9.3 第三风险

**effective_support_ref 虽然被内收，但内部分类标准不稳。**

表现：

- 某些 layered ground bag 行为翻转
- false benchmark 在楼梯/上下坡混合场景复发

---

## 10. 停止条件

必须明确停止条件，避免“越修越 patch”。

任一阶段如果出现以下情况之一，立即停止继续堆规则：

1. `colcon test` 不通过
2. false obstacle benchmark 任一 hard gate 失败
3. miss obstacle hard gate 失败

停止后只允许做两件事：

- 回退到上一个全绿状态
- 回到设计层复盘 `Local / Adjacency / Explanation` 边界

不允许直接在失败状态上继续加 bag-specific 门槛。

---

## 11. 最终建议

这版融合方案的核心思想可以概括成一句话：

> **用方案二重新定义前端主链，用方案一保障它按小步、可验证、可回退的方式落地。**

最终希望得到的 `PolarFrontend`，不是“更多 helper 的集合”，而是一个边界清楚的前端解释器：

- support 基线如何来：清楚
- 局部几何事实是什么：清楚
- 邻域只负责什么：清楚
- explanation 只负责什么：清楚
- obstacle candidate 如何形成：清楚

只有这样，后面无论你再调 false obstacle，还是补 miss obstacle，才是在一条稳定主链上工作，而不是继续在 patch 森林里滚雪球。
