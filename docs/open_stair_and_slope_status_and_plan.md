# `open_stair_and_slope` 当前状态与后续计划

本文档用于在继续修改 `passable_area` 之前，先把当前稳定基线、已知诊断、验证口径和后续拆题方向整理清楚。

目标不是“宣布问题已经解决”，而是把下面这些事情说清楚：

- 当前哪些架构方向已经验证过，后续应继续保留
- `rosbag2_open_stair_and_slope` 的残余漏检到底被拆成了哪些问题桶
- 哪些是已经有诊断支撑的事实
- 哪些仍然只是待验证方向
- 后续应该按什么顺序继续推进，避免再次陷入 patch chase

---

## 1. 背景与当前架构稳定状态

### 1.1 当前需要保留的稳定基线

当前分工和约束里，下面这些方向已经验证过，后续不应该随意打破：

- obstacle point 的发布是 **cell-owned**
- 不允许为发布行为引入 **adjacent-cell evidence borrowing**
- `obstacle_publishable` 只有一个计算 owner：**map-update layer**
- processor / output 层只能消费 publishability 结果，不能再偷偷改发布资格
- `kConfirmedFacade` 已经针对 **below-robot overhead / low ceiling contamination** 做过收紧
- 不能再把放宽 `kConfirmedFacade` 当成捷径
- 不能靠全局 threshold / gain 调参硬压过去
- 不能写 bag-specific、ROI-specific 或时间窗口特化补丁

### 1.2 这些约束用通俗话怎么理解

#### obstacle-point publication 是 cell-owned

意思是：某个 obstacle sample 能不能最终进入 `/terrain_obstacle_points`，要看它自己所在 cell 是否拥有发布权，而不是看附近有没有“更像障碍”的邻居。

这样做的好处是语义清楚：

- 地图层负责回答“这个 cell 当前有没有形成属于自己的 obstacle 结构”
- 输出层只负责把已经拥有发布权的 cell 里的样本发布出来

这避免了“样本明明在 A 格，却借 B 格的证据发出来”的混乱行为。

#### 不允许 adjacent-cell evidence borrowing for publication

这条和上面是配套的。它不是说邻域信息完全不能用，而是说：

- 邻域信息可以用于前端解释或结构理解
- 但不能直接拿邻居的 obstacle evidence 替当前 cell 获得发布资格

否则 publication ownership 会重新变得模糊，回到以前那种“看起来能修复一帧，长期却很难维护”的状态。

#### `obstacle_publishable` 只有 map-update layer 一个 owner

当前合同要求：

- frontend 负责形成 `obstacle_candidate`
- map-update layer 负责累积 `obstacle_evidence`，并据此计算 `obstacle_publishable`
- processor / publication 层不能补算第二套 publishability 逻辑

这个分层很重要，因为它把“结构解释”和“发布 ownership”拆开了。后续如果看到某个 cell 已经 `KeepAsObstacle`，但仍然不 publishable，首先应该理解为：

> frontend 已经允许它作为 obstacle candidate 存在，但 map-update 还没有认为它拥有足够的结构证据和发布 ownership。

这不是 bug 本身，而是当前架构的有意分层。

#### `kConfirmedFacade` 已经收紧，不应再放宽

`kConfirmedFacade` 现在代表的是更窄、更强的一类 facade-like obstacle semantic。它不是“只要看起来像障碍就给强化语义”，而是：

- 当前 cell 本身结构足够像 facade
- 且这个结构适合交给地图层做强化累积
- 同时不能把 below-robot overhead、low ceiling、clearance-only layered structure 混进去

因此后续不能再把“放宽 confirmed-facade 判据”当成通用修复手段。那样虽然可能让某些 bag 过线，但会重新引入 ceiling / overhead / false-obstacle 污染。

#### 不能靠全局阈值、bag 补丁或 ROI 补丁解决

当前问题已经说明：`open_stair_and_slope` 的残余漏检不是一个统一单点故障。如果直接去动：

- `max_step_down`
- 全局 evidence gain
- 全局 publish threshold
- 某个 bag 的时间窗特化
- 某个 ROI 的条件特化

很容易把多个不同来源的问题硬塞进一条规则里，最后得到的是“一个 bag 看起来好了，别的场景开始回归”。

因此当前基线是：

> 优先做 class-based、contract-based 的前端/解释/地图层诊断与修正，而不是做窗口补丁或全局硬调参。

---

## 2. 当前残余问题分解

当前结论已经比较明确：

> `rosbag2_open_stair_and_slope` 在严格 source-of-truth setup 下的残余漏检，并不来自一个单一 bug。

至少已经拆出了三个问题桶，它们所在层次也不一样。

### 2.1 左侧：no-suspicion / weak-span formation

#### 当前表现

有些左侧 frame 在 ROI 内明明有样本，但根本没有形成 frontend suspicious trigger。

严格窗口下，左侧有一帧的主因已经被分类为：

- `NoFrontendObstacleSuspicion`

这类 frame 的典型现象是：

- ROI 内有 sample
- `obstacle_suspicious_cells = 0`
- `obstacle_candidate_cells = 0`
- 很多 cell 的 `max_sample_z_minus_support_ref` 只有 `0.00 ~ 0.17` 左右

也就是说，结构还没进入 explanation 或 map-update 阶段，就已经在前端局部触发之前停住了。

#### dominant layer

- **frontend suspicion**

#### 已知事实

- 问题发生在 explanation 之前
- 不是 `aligned_neighbor_support_count` 不够导致的
- 也不是 publishability / output height gate 的直接问题
- 当前严格窗口下，左侧至少有一类 miss 是“根本没有 local suspicious formation”

#### 仍未知的点

- 为什么这类左侧 side-board 结构在某些 frame 里 vertical separation 形成得偏弱
- 是当前局部统计窗口、support ref 选取、样本分布，还是 upper-band 形成策略导致它没有跨过 suspicious trigger
- 这类 case 是否能通过一条窄而稳定的 frontend-local rule 改善，而不伤到 ground / stair / ceiling guard

### 2.2 右侧：kept-but-unowned default candidates

#### 当前表现

这类 cell 已经进入：

- `obstacle_suspicious = true`
- `obstacle_upper_patch_confirmed = true`
- `explanation_decision = KeepAsObstacle`

但后面仍然没有形成最终 obstacle points。严格窗口下，这类 miss 常被表现成：

- `ObstacleEvidenceTooLow`
- 或 `UnknownOrMixed` 中的 keep-side 子群

典型特征是：

- frontend 已经明确 keep
- semantic 仍是默认 candidate，而不是 `kConfirmedFacade`
- `obstacle_evidence` 还没有跨过 ownership / publishability 需要的门槛
- 或者没有成为当前 publishable structure seed

#### dominant layer

- **map evidence / publishability**

#### 已知事实

- 这不是“frontend 没看见”
- 这也不是“explanation 已经 reject”
- keep-side default candidate 并不自动等于 publishable
- 当前 publishability ownership 由 map-update layer 单独控制，这是有意设计
- 对这些 cell，如果想改善，不能靠 output 层补救，也不能靠重新引入 adjacent-cell publish borrowing

#### 仍未知的点

- 这类 right-side side-board 是否确实属于应该拿到更强 ownership 的 obstacle structure
- 如果属于，应该通过什么更稳妥的 upstream contract 让它在 map-update 中获得合理 ownership
- 是需要 refinement 默认 candidate 到 publishable seed 的桥接条件，还是需要更早地形成更强的 facade-like class
- 这类 kept-but-unowned cell 和真正不该强化的 below-robot layered mix，边界该如何定义

### 2.3 右侧：genuine stair-mix rejects

#### 当前表现

严格窗口下，右侧还有一批 cell 的表现不是“没过 evidence”，而是：

- 已经 suspicious
- 已经过 `upper_patch_confirmed`
- 但在 explanation 中被明确 reject

当前已看到的 reject decision 主要是：

- `BelowRobotStairMix`

这说明它们并不是 keep-side candidate，而是解释系统认为它们更接近真实 stair-mix / mixed-support 结构。

#### dominant layer

- **explanation**

#### 已知事实

- 这类 reject 和 kept-but-unowned 不是一回事
- 它们在当前合同下属于真正的 explanation reject case
- 不能因为同一个 bag 里也存在 keep-side miss，就把所有 right-side reject 都当成“应该放行的 side-board”
- 当前 `BelowRobotStairMix` 与 facade keep override 之间已经有一条较窄的仲裁逻辑，不能随意整体放宽

#### 仍未知的点

- 这批 right-side reject 里，有多少是真正应保留 reject 的 mixed-support case
- 是否存在一小类“表面像 stair-mix，实则应保留为 facade-like obstacle”的边缘结构
- 如果存在，应该通过什么窄 contract 来表达，而不是直接放松整个 stair-mix reject

---

## 3. 已经建立的重要诊断

### 3.1 已数值闭合的左侧 `BelowRobotGroundLayerMix` 诊断

之前有一个很重要的左侧 reject frame 已经被数值闭合：

- bag：`rosbag2_open_stair_and_slope`
- frame_offset_sec：`6.131`
- cell：
  - `base_x = 2.315`
  - `base_y = 0.476`
  - `sample_count = 8`
  - `filtered_min_z = 0.414`
  - `filtered_max_z = 0.785`
  - `support_h = 0.414`
  - `neighbor_upper_support_count = 3`
  - `aligned_neighbor_support_count = 0`
  - `anchor_below_observation_count = 0`
  - `stale_lower_anchor_mix = false`
  - `facade_lower_upper_coexisting = true`
  - `upper_patch_confirmed = true`
  - `suspicious = true`
  - `explanation_decision = BelowRobotGroundLayerMix`

对应阈值：

- `max_step_down = 0.38`
- `upper_min_height_above_support = 0.20`
- `support_anchor_reobserve_tolerance = 0.08`

对应派生量：

- `relative_support_ref = -1.010`
- `relative_upper_z = -0.639`
- `vertical_span = 0.371`

同帧右侧有一个 kept cell，表现为：

- `aligned_neighbor_support_count = 0`
- `neighbor_upper_support_count = 3`
- `facade_lower_upper_coexisting = true`
- `anchor_below_observation_count = 0`
- `stale_lower_anchor_mix = false`
- 但 `vertical_span = 0.426`

### 3.2 为什么 `aligned_neighbor_support_count` 不是那个 frame 的主导 blocker

这个诊断很重要，因为它已经说明：

- 左右 cell 在 `aligned_neighbor_support_count` 上是一样的，都是 `0`
- 左右 cell 在 `neighbor_upper_support_count` 上也一样，都是 `3`
- `facade_lower_upper_coexisting` 也一样成立

因此，那个特定左侧 reject frame 并不是因为“没有 aligned neighbor support”才被挡住。

更准确地说：

> `aligned_neighbor_support_count = 0` 只是背景条件，不是那个具体 frame 左右分化的主导因子。

### 3.3 为什么 `vertical_span <= max_step_down` 才是那个 frame 的真实分界量

那个左侧 reject frame 的关键差别在于：

- 左侧 `vertical_span = 0.371`
- 右侧 kept cell `vertical_span = 0.426`
- `max_step_down = 0.38`

因此，左侧落在：

- `vertical_span <= max_step_down`

而右侧没有落进去。

这就说明，对那个具体的 `BelowRobotGroundLayerMix` reject chain 来说，真正把它压到 reject 侧的量，是“lower / upper split 太浅，仍落在 below-robot ground-layer mix 包络里”。

### 3.4 但这个诊断只是一个子问题，不是整个 bag 的总解

这点必须写得很明确。

虽然上面这个诊断是成立的，但它只能说明：

> “shallow lower/upper split inside the `BelowRobotGroundLayerMix` envelope” 是 Bag B 的一个真实 blocker。

它不能推出：

- 整个 Bag B 只剩这一个问题
- 只要改掉这个 reject 条件，整包就会好
- 只要重新引入 neighbor-consensus 类 patch 就能稳定修复

因为严格 source-of-truth 复查已经表明，Bag B 的残余 miss 还包含：

- 左侧 no-suspicion case
- 右侧 kept-but-unowned default candidates
- 右侧 genuine stair-mix rejects

所以这个已闭合诊断应被理解成：

- 一个重要的、真实存在的子问题
- 但不是整包唯一问题，也不是当前阶段可直接推广成单一总规则的依据

---

## 4. Source-of-Truth 验证口径

后续讨论如果没有明确声明新的 setup，默认应直接使用下面两套 source-of-truth 配置。

### 4.1 Bag A：`rosbag2_open_up_down_stairs`

- bag：
  - `/home/deep/deeprobotics/bags/test_result/rosbag2_open_up_down_stairs`
- miss-obstacle source-of-truth setup：
  - `map_height_max = 0.5`
  - `start_offset_sec = 6.7`
  - `time_window_sec = 0.4`
  - 左 ROI：`x in [0.8, 2.7], y in [0.4, 0.6]`
  - 右 ROI：`x in [0.8, 2.7], y in [-0.6, -0.4]`

同时，这个 bag 也是当前 false-obstacle 侧的重要 source-of-truth gate：

- false-obstacle setup：
  - `map_height_max = 0.2`

### 4.2 Bag B：`rosbag2_open_stair_and_slope`

- bag：
  - `/home/deep/deeprobotics/bags/offline_bags/rosbag2_open_stair_and_slope`
- miss-obstacle source-of-truth setup：
  - `map_height_max = 0.5`
  - `start_offset_sec = 5.7`
  - `time_window_sec = 0.3`
  - 左 ROI：`x in [1.5, 2.5], y in [0.4, 0.6]`
  - 右 ROI：`x in [1.5, 2.5], y in [-0.6, -0.4]`

### 4.3 使用约定

后续报告、讨论和验证结论，默认都应该直接复用上面两套 setup。

如果有人使用了不同窗口、不同 ROI、不同 `map_height_max`，应在报告里显式写清楚：

- 哪个 bag
- 改了什么参数
- 为什么改
- 该结果是否能直接与 source-of-truth 结果比较

否则很容易出现：

- 在一个更有利的窗口里看到改进
- 却被误写成“整包已经修复”

---

## 5. 当前已知结果

### 5.1 Bag A 当前已知结果

在 source-of-truth miss setup 下：

- 左侧干净
- 右侧干净

在 false-obstacle setup 下：

- `map_height_max = 0.2`
- false-obstacle 侧目前仍保持干净

因此 Bag A 当前的作用是：

- side-board / facade-like source-of-truth clean gate
- false-obstacle regression gate

### 5.2 Bag B 当前已知结果

在严格 source-of-truth setup 下：

- 左侧当前 `2/3 miss`
- 右侧当前 `2/3 miss`

但这 `2/3 + 2/3` 并不是一个统一 root cause。

#### 左侧当前分解

已观察到的严格窗口 root cause 包括：

- `NoFrontendObstacleSuspicion`
- `OutputHeightGateNotMet`

具体理解如下：

- 有一类 frame 根本没进入 suspicious trigger，属于前端局部形成过弱
- 还有一类 frame 已经进入 `KeepAsObstacle`，但当前样本没有通过 obstacle-point 输出高度门槛，或者虽然 candidate 成立但还未拥有当前可发布 ownership

这说明左侧也不是单一的 `BelowRobotGroundLayerMix` 问题。

#### 右侧当前分解

已观察到的严格窗口 root cause 包括：

- `ObstacleEvidenceTooLow`
- `UnknownOrMixed`

进一步展开后，可分成两类：

- 一类是 `KeepAsObstacle` 已经成立，但仍然是 kept-but-unowned default candidate
- 一类是 `BelowRobotStairMix` 等 explanation reject 仍然真实存在

也就是说，右侧同时存在：

- keep-side 但 ownership/evidence 不足的 case
- genuine stair-mix reject case

#### 当前最重要的结论

Bag B 目前至少已经拆出了三条不同控制链：

1. 左侧 no-suspicion / weak-span formation
2. 右侧 kept-but-unowned default candidates
3. 右侧 genuine stair-mix rejects

因此目前不能再假设：

> “再找一个统一规则，就能把 `open_stair_and_slope` 整包剩余 miss 一次性清干净。”

---

## 6. TEST 工作流

当前自动测试仍然是回归的第一层门禁。

### 6.1 基本测试命令

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select passable_area --symlink-install
colcon test --packages-select passable_area --event-handlers console_direct+
colcon test-result --verbose
```

如果只想查看当前工作区的测试结果汇总，继续使用：

```bash
colcon test-result --verbose
```

### 6.2 当前应该重点关注的测试类型

后续如果继续改 `open_stair_and_slope` 相关行为，至少要重点盯住下面几类测试：

#### confirmed-facade semantic tests

重点保证：

- 真正 facade-like 的结构仍能进入正确语义
- 不会把过宽的 below-robot layered structure 再次推进 `kConfirmedFacade`

#### below-robot overhead-cover guard tests

重点保证：

- below-robot overhead / low ceiling 不会重新污染 `kConfirmedFacade`
- 低净空 / ceiling-like 结构不会被误当成可强化 facade obstacle

#### stair-mix / facade arbitration tests

重点保证：

- `BelowRobotStairMix`
- `BelowRobotGroundLayerMix`
- `KeepAsObstacle`

之间的仲裁仍然清晰、可解释，不会因为修一类 case 就整体放松 reject 侧语义。

#### publication ownership / publishability tests

重点保证：

- `obstacle_publishable` 仍然是 map-update layer 单一 owner
- obstacle point 发布仍然保持 cell-owned
- 不会重新引入 adjacent-cell publish borrowing

#### weak / sparse / dynamic guard tests

重点保证：

- 弱证据 case 不会被误发成 obstacle
- sparse / partial / dropout 场景下不会因为修某个 bag 而破坏已有 guard
- dynamic / stale / leak 相关保护不被意外削弱

### 6.3 当前文档层面的使用原则

这份文档只记录现状，不替代自动测试。

后续如果某条修复声称“解决了 `open_stair_and_slope`”，至少需要同时回答：

- 自动测试是否仍然全绿
- 哪几类语义守卫被覆盖到
- 是否新增或调整了 synthetic test 来支撑新的 contract

---

## 7. benchmark 与 replay 验证流程

当前针对这类问题的验证链路，主要分成三类：

- miss-obstacle replay
- false-obstacle replay
- benchmark

### 7.1 miss-obstacle replay 流程

基本入口：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
build/passable_area/passable_area_offline_replay --analyze-missed-obstacles ...
```

对 `open_stair_and_slope`，当前默认应使用本文第 4 节记录的 Bag B source-of-truth setup。

如果需要临时把 `map_height_max` 改成 `0.5`，当前常见做法是先生成一个 `/tmp` 下的参数副本，例如：

- 从 `config/passable_area.yaml` 复制一份到 `/tmp`
- 只改 `map_height_max`
- replay 时用 `/tmp` 下这份参数文件

这样做的原因是：

- 不污染仓库默认参数
- 更容易保证“这是一次验证配置”，不是永久配置修改

如果运行 replay 或 inspect 时 ROS 日志目录写入受限，可显式带上：

```bash
ROS_LOG_DIR=/tmp
```

这在受限环境里比较常见，能避免日志写入到默认目录失败。

### 7.2 false-obstacle replay 流程

基本入口：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
build/passable_area/passable_area_offline_replay --analyze-false-obstacles ...
```

当前 false-obstacle 侧的重要 regression gate 包括：

- Bag A：`/home/deep/deeprobotics/bags/test_result/rosbag2_open_up_down_stairs`
  - `map_height_max = 0.2`
- 以及当前常用的几条 MTBF bag

这些 bag 的作用是防止下面这种回归：

- 为了救 `open_stair_and_slope` 的漏检
- 结果把 overhead / low ceiling / layered clutter 又重新放进 obstacle publication

### 7.3 `inspect-roi` 的用途

当 miss-obstacle report 还不够细时，可继续用：

```bash
build/passable_area/passable_area_offline_replay --inspect-roi ...
```

它的用途是把 ROI 内每个 cell 的控制链中间量直接打出来，例如：

- `obstacle_suspicious`
- `obstacle_upper_patch_confirmed`
- `explanation_decision`
- `neighbor_upper_support_count`
- `aligned_neighbor_support_count`
- `obstacle_evidence`
- `support_anchor_*`

当前这一步非常有价值，因为它能帮助区分：

- 前端根本没触发
- explanation reject
- keep-side candidate 但 evidence / ownership 不足
- output height gate 没过

### 7.4 benchmark 流程

当前至少应记录并保留：

```bash
build/passable_area/passable_area_benchmark
```

它主要用于观察 core 处理链吞吐是否发生明显回归。

仓库里也存在：

```bash
build/passable_area/passable_area_e2e_benchmark
```

它更偏向 ROS 节点端到端延迟 smoke benchmark。虽然这份文档当前重点放在 `passable_area_benchmark`，但如果改动涉及更广的运行时行为，也应一并关注它。

### 7.5 当前验证流的实践提醒

- source-of-truth bag、窗口和 ROI 必须写清楚
- 改过的参数文件最好放在 `/tmp`
- 受限环境里如有需要，显式使用 `ROS_LOG_DIR=/tmp`
- 某个 bag 在某个更有利窗口里变好，不等于 source-of-truth 已经修复
- replay 工具是诊断和回归工具，不替代自动测试

---

## 8. 当前明确不建议的方向

这一节专门列出当前不建议继续走的方向，避免后续又回到 patch chase。

- 不要继续 patch chase
- 不要再放宽 `kConfirmedFacade`
- 不要动全局 threshold / gain 企图统一解决问题
- 不要重新引入 adjacent-cell publish borrowing
- 不要把 publication 层拿来兜 frontend 语义错误
- 不要在一个 bag 上找到有效窗口就声称整体修复

再展开一点说：

### 8.1 不要继续 patch chase

当前问题已经被拆成多个问题桶。如果继续围绕单帧、单窗口、单 bag 现象打补丁，只会让规则越来越难解释。

### 8.2 不要再放宽 `kConfirmedFacade`

它已经承担了“窄而强的 facade semantic”角色。再放宽，最容易回来的就是 ceiling / overhead / false-obstacle 污染。

### 8.3 不要动全局 threshold / gain 企图统一解决

`open_stair_and_slope` 现在不是一个统一问题。对全局阈值下手，通常只会把不同来源的 miss 混成更难理解的副作用。

### 8.4 不要重新引入 adjacent-cell publish borrowing

publication ownership 目前已经被清理成单 cell 所有权语义。重新借邻居发点，会直接破坏这层合同。

### 8.5 不要让 publication 层兜 frontend 语义错误

如果某个结构在前端语义上就解释错了，应优先修前端 / explanation contract，而不是在输出层加条件补丁。

### 8.6 不要在有利窗口里“报喜”

source-of-truth 已经明确规定了：

- bag
- `map_height_max`
- `start_offset_sec`
- `time_window_sec`
- ROI

后续如果换了窗口，必须明确写成“非 source-of-truth 观察结果”，不能直接替代主结论。

---

## 9. 已确认事实、当前假设与建议的下一步顺序

这一节故意把“已经确认的事实”和“下一步设想”分开写，避免把 proposal 写成 truth。

### 9.1 已确认事实

- `rosbag2_open_stair_and_slope` 的严格残余 miss 不是一个单一 bug
- 当前至少可拆成三个问题桶：
  - 左侧 no-suspicion / weak-span formation
  - 右侧 kept-but-unowned default candidates
  - 右侧 genuine stair-mix rejects
- 之前已闭合的左侧 `BelowRobotGroundLayerMix` 数值诊断是真实子问题，但不是整包总解
- `aligned_neighbor_support_count` 不是那个已闭合左侧 reject frame 的主导分界量
- 对那个 frame 而言，`vertical_span <= max_step_down` 才是关键分界量
- keep-side default candidate 与 publishable ownership 不是同一层概念
- publication ownership、map-update ownership、`kConfirmedFacade` 收紧这几条基线当前都应继续保留

### 9.2 当前假设 / 待验证方向

下面这些还不是结论，只是当前较合理的待验证方向。

#### 假设 A：左侧 no-suspicion 需要单独做前端局部形成诊断

当前怀疑左侧一部分 miss 发生在 explanation 之前，因此很可能需要单独分析：

- 当前 local trigger 的 span 形成为什么偏弱
- support ref / filtered stats / upper-band 形成是否在 side-board 这类结构上过于保守

这类方向如果成立，应优先是 frontend-local、class-based refinement，而不是全局调阈值。

#### 假设 B：右侧 kept-but-unowned default candidates 需要单独处理 ownership bridge

当前怀疑有一类 right-side cell：

- frontend 已经 `KeepAsObstacle`
- 但由于仍是 `kDefault` 且 ownership seed 不足，始终跨不过 map evidence / publishability 的桥

如果这类结构被确认属于“应该被稳定发布的 side-board / facade-like obstacle”，后续可能需要一个更清晰的 upstream contract，把“应强化 ownership 的 keep-side 结构”和“只是暂时 keep 的 default candidate”再细分开。

这不是当前事实，只是后续值得单独验证的方向。

#### 假设 C：右侧 genuine stair-mix rejects 可能需要独立 contract refinement

当前也不能排除这样一种可能：

- 大多数 right-side reject 确实应该保留 reject
- 但其中有一小类边界 case，现有 stair-mix / facade 仲裁还不够精细

如果后续真要动这部分，应该是：

- 继续沿 explanation contract 做窄修正
- 并明确 synthetic test
- 而不是整体放松 `BelowRobotStairMix`

### 9.3 建议的下一步拆题顺序

当前更建议按下面顺序推进，而不是试图一把梭修完整个 bag。

#### 第一步：先做左侧 no-suspicion / weak-span formation 的专项诊断或修复

原因：

- 它发生在 explanation 之前
- 与 keep / reject / publishability 的混合问题耦合最少
- 先把“根本没触发 suspicious”的问题单独拆掉，能减少后面分析噪音

#### 第二步：再做右侧 kept-but-unowned default candidates 的专项处理

原因：

- 这类 case 已经通过 frontend keep
- 但没有拿到 ownership / evidence / publishability
- 更适合单独检查 map-update layer 与 semantic bridge 的合同

#### 第三步：最后再看右侧 genuine stair-mix rejects 是否需要独立 contract refinement

原因：

- 这类 case 风险最高
- 最容易因为“想救某个 board”而误伤真实 mixed-support reject
- 应该在前两类问题拆干净之后，再决定是否真的有必要动这部分 contract

### 9.4 当前推进原则

后续继续推进时，建议始终保持下面这个顺序：

1. 先确认当前问题桶属于哪个层
2. 再决定应该修 frontend、explanation 还是 map-update
3. 修完后回到 source-of-truth setup 复核
4. 同时检查 false-obstacle 和自动测试 guard

这样做的目的很简单：

> 先把问题拆清楚，再做窄修正；不要再靠“看起来有帮助”的 patch 叠加去碰运气。

