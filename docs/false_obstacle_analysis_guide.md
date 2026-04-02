# 误障碍离线分析使用说明

这份文档是教你怎么看 `offline_replay --analyze-false-obstacles` 的输出。

它不是算法设计文档，也不是源码说明。你可以把它理解成一份“排查指南”：

- 先教你怎么运行
- 再教你先看哪几行
- 最后教你怎么把输出和 RViz 现象对上

如果你不懂算法，也可以直接按这份文档来排查。

## 1. 如何运行

先在工作空间根目录执行：

```bash
source install/setup.bash
build/passable_area/passable_area_offline_replay \
  --analyze-false-obstacles \
  --bag /你的/bag/路径 \
  --params-file /home/deep/deeprobotics/passable_humble_ws/src/passable_area/config/passable_area.yaml \
  --range-x-min 0.0 --range-x-max 1.4 \
  --range-y-min -0.25 --range-y-max 0.25 \
  --top-k 3
```

### 这几个参数是什么意思

| 参数 | 含义 | 怎么理解 |
| --- | --- | --- |
| `--bag` | 要分析的 ros2 bag 路径 | 就是你想排查的录包 |
| `--params-file` | 分析时使用的参数文件 | 建议和真实运行时保持一致 |
| `--range-x-min` / `--range-x-max` | 检测框在前后方向上的范围 | `x` 是前后方向 |
| `--range-y-min` / `--range-y-max` | 检测框在左右方向上的范围 | `y` 是左右方向 |
| `--top-k` | 最多打印多少个最值得排查的 frame | 通常先看前 3 个或前 5 个就够了 |

### 检测框坐标怎么理解

这里的检测框是**机器人相对坐标**，不是地图坐标。

- `x > 0`：机器人前方
- `x < 0`：机器人后方
- `y > 0`：机器人左侧
- `y < 0`：机器人右侧

所以：

- `x[0.0, 1.4]` 表示机器人前方 0 到 1.4 米
- `y[-0.25, 0.25]` 表示机器人左右各 25 厘米以内

如果你在 RViz 里画了一个机器人前方检测框，这里的参数应该尽量和那个范围一致。

## 2. 这个工具是干什么的

这个工具的作用不是“重新跑一遍导航”，而是帮你回答这个问题：

> 在你指定的检测框里，为什么会出现障碍点？

它做的事情很直接：

1. 离线回放 bag
2. 用当前参数重新跑一次处理流程
3. 检查检测框里有没有 `obstacle_points`
4. 如果有，就把最值得优先排查的 frame 和局部热点列出来

要注意两点：

- 它是**离线排查工具**
- 它是帮你**缩小问题范围**

它不是最终真值判定器，也不是“绝对正确”的场景理解器。

## 3. 先看哪几行

第一次看输出时，不要从头到尾一项项抠。建议按这个顺序看：

1. `detection_box`
2. `total_frames` 和 `candidate_frames`
3. `root_causes`
4. `ranked_frames`
5. 排名第一的 `frame`
6. 这个 `frame` 下面的 `hotspot`

简单说就是：

- 先看有没有命中
- 再看大概是哪一类原因
- 最后再看最严重的位置到底在哪里

## 4. summary 怎么看

输出里最上面这几行，主要是总览信息，例如：

```text
false_obstacle_summary bag=/your/bag

detection_box: x[0.00, 1.40] y[-0.25, 0.25]
total_frames: 129
rear_dropout_frames: 0
suspicious_frames: 1
suspicious_ratio: 0.008
longest_consecutive_run: 1
```

### 这些字段是什么意思

| 字段 | 含义 | 怎么理解 |
| --- | --- | --- |
| `total_frames` | 这次一共分析了多少帧 | 总样本数 |
| `rear_dropout_frames` | 有多少帧被判定为后向掉帧 | 用来判断后向观测异常是不是经常发生 |
| `suspicious_frames` | 有多少帧在检测框内出现了障碍点 | 这是最关键的数字 |
| `suspicious_ratio` | 可疑帧占总帧数的比例 | 越高说明问题越频繁 |
| `longest_consecutive_run` | 最长连续命中的帧数 | 越长越像持续问题，越短越像偶发问题 |

### 常见情况怎么理解

- `suspicious_frames = 0`
  - 表示这次分析里，检测框内没有出现 `obstacle_points`
  - 至少按这套规则，没有发现“检测框内障碍点”问题

- `suspicious_frames` 很少，`longest_consecutive_run` 也很小
  - 更像偶发问题

- `suspicious_frames` 很多，而且 `longest_consecutive_run` 很长
  - 更像稳定持续的问题

## 5. root_causes 怎么看

`root_causes` 是把候选 frame 按“更像哪类原因”做了一个统计。

例如：

```text
root_causes:
  ClearanceDriven: 0
  ObstacleEvidenceDriven: 0
  ObstacleEvidencePlusLowContinuity: 0
  ObservabilityInfluenced: 0
  UnknownOrMixed: 1
```

### 每一种是什么意思

| 名称 | 通俗解释 | 使用时要注意什么 |
| --- | --- | --- |
| `ClearanceDriven` | 系统认为这里上方可通过空间不够 | 更像“净空不够”，不一定代表你肉眼看到一个独立障碍物 |
| `ObstacleEvidenceDriven` | 系统认为这里的障碍迹象比较明显 | 更像“确实看到了障碍证据” |
| `ObstacleEvidencePlusLowContinuity` | 既看到了障碍迹象，又觉得地面连续性不好 | 这类通常更复杂，既有障碍信息，也有地面建模不稳的问题 |
| `ObservabilityInfluenced` | 当前观测不够完整，结果受观测质量影响 | 这类要重点结合观测状态和掉帧情况看 |
| `UnknownOrMixed` | 当前证据不够统一，暂时分不清是哪一种原因 | 说明系统没有形成很明确的单一结论 |

### 这里最容易误解的地方

不要把这些类别理解成“场景名字”。

比如：

- 它不是在说“这是楼梯边缘”
- 也不是在说“这是吊挂点”
- 更不是在说“这里一定有某个具体物体”

它只是告诉你：

- 从当前已有证据看
- 这个问题更像是由哪条触发链路引起的

## 6. ranked_frames 是什么意思

`ranked_frames` 是最值得优先排查的 frame 列表。

它的排序规则不是时间顺序，而是：

- 按 `severity` 从高到低排序

也就是说：

- 排在前面的 frame，不一定是最早出现的
- 但通常是最值得你先看的

如果只想快速定位问题，先看 `frame 1` 基本是最省时间的做法。

## 7. 每个 frame 里每一项是什么意思

一个 frame 大概会长这样：

```text
frame 1
  stamp: 1775114321603217125
  class: ClearanceDriven
  severity: 1428.06
  in_box_obstacle_points: 862
  hotspot_count: 3
  frame_partial: true
  rear_dropout: false
  max_obstacle_evidence: 0.47
  min_clearance: 0.18
  min_support_continuity: 1.00
```

### 字段说明

| 字段 | 它表示什么 | 怎么看 |
| --- | --- | --- |
| `stamp` | 这帧的时间戳 | 用来回到 bag 或 RViz 对照 |
| `class` | 这帧最终归到哪一类原因 | 先看它，大方向就清楚了 |
| `severity` | 这帧的排查优先级分数 | 只用于排序，不是物理量 |
| `in_box_obstacle_points` | 检测框内有多少个障碍点 | 越多通常越值得优先排查 |
| `hotspot_count` | 这帧挑出了多少个热点 | 当前最多显示 3 个 |
| `frame_partial` | 这一帧整体观测是否不完整 | 为 `true` 时要更谨慎 |
| `rear_dropout` | 这一帧是否出现后向掉帧现象 | 更多是后方问题排查时有用 |
| `max_obstacle_evidence` | 这帧热点里最大的障碍证据 | 越高说明障碍迹象越明显 |
| `min_clearance` | 这帧热点里最小的净空 | 越小越像“可通过空间不够” |
| `min_support_continuity` | 这帧热点里最小的地面连续性 | 越小越说明地面建模不稳定 |

### 一般怎么读

建议按这个顺序读：

1. 看 `class`
2. 看 `in_box_obstacle_points`
3. 看 `min_clearance`
4. 看 `max_obstacle_evidence`
5. 看 `frame_partial` 和 `rear_dropout`

这样通常能很快判断：

- 更像净空问题
- 更像障碍证据问题
- 还是更像观测状态问题

## 8. 每个 hotspot 里每一项是什么意思

一个 hotspot 大概会长这样：

```text
hotspot 1
  pos: (0.45, -0.19)  obstacle_points: 223  severity: 225.93
  class: ClearanceDriven  observability: Observed  explanation: clearance below threshold
  obstacle_evidence: 0.47  clearance: 0.18  support_continuity: 1.00
```

### hotspot 是什么

你可以把 hotspot 理解成：

- 这一帧检测框里最值得优先看的局部位置

不是整帧平均情况，而是“问题最集中的点位附近”。

### 字段说明

| 字段 | 它表示什么 | 怎么看 |
| --- | --- | --- |
| `pos` | 这个热点的位置 | 是机器人相对坐标，前方 `x` 为正，左侧 `y` 为正 |
| `obstacle_points` | 这个热点附近有多少个障碍点 | 越多说明这里更集中 |
| `severity` | 这个热点的排查优先级分数 | 只用于排序，不是危险值 |
| `class` | 这个热点更像哪一类原因 | 比 frame 的大类更接近局部事实 |
| `observability` | 这个位置的观测状态 | 如果不是 `Observed`，要小心观测不完整带来的影响 |
| `explanation` | 一句简短解释 | 是帮助你快速抓重点的提示 |
| `obstacle_evidence` | 这个位置的障碍证据强度 | 越高说明系统越像“真的看到了障碍” |
| `clearance` | 这个位置的净空 | 越小越说明上方可通过空间不够 |
| `support_continuity` | 这个位置的地面连续性 | 越小越说明地面建模比较断、不稳定 |

### hotspot 最重要的三项

如果你时间不够，先看这三项：

1. `pos`
2. `class`
3. `explanation`

因为它们最直接决定你回 RViz 时先看哪里、先怀疑什么。

## 9. severity 是什么意思

`severity` 可以直接理解成：

- 排查优先级分数

它的作用只有一个：

- 帮你排序，告诉你先看哪一帧、先看哪个热点

### 它不是什么

`severity` 不是：

- 障碍物高度
- 危险程度真值
- 通行性结果本身
- 真实物理量

### hotspot 的 severity

hotspot 的 `severity` 越高，通常意味着：

- 这个局部位置的障碍点更多
- 或者障碍证据更强
- 或者净空更小
- 或者观测状态更差

简单理解就是：

- 越高，越值得先看

### frame 的 severity

frame 的 `severity` 是把整帧里检测框内的情况汇总后的结果。

简单理解就是：

- 这一帧整体有多值得优先排查

所以：

- frame 的 `severity` 用来决定先看哪一帧
- hotspot 的 `severity` 用来决定先看这一帧里的哪个位置

## 10. 常见结果怎么理解

### 情况 1：`suspicious_frames = 0`

这通常表示：

- 这次分析里，检测框内没有出现 `obstacle_points`

这不一定表示“什么问题都没有”，但至少表示：

- 按这套误障碍分析规则
- 没有发现“检测框内障碍点”问题

### 情况 2：只有 1 帧命中

这更像：

- 偶发问题
- 或很短时间的一次异常

这种情况建议重点看：

- 这个 frame 的 `class`
- 排名第一的 hotspot 位置

### 情况 3：`UnknownOrMixed` 很多

这通常表示：

- 当前证据不够统一
- 很难直接归成一种明确原因

这类情况要更多依赖：

- hotspot 的具体位置
- RViz 里对应位置的点云和地图层

### 情况 4：`ClearanceDriven` 很多

这通常表示：

- 系统更像是在说“净空不够”

重点回去看：

- `clearance`
- `grid_map`
- 对应位置是不是被上方结构或高点压低了可通过空间

### 情况 5：`ObservabilityInfluenced` 很多

这通常表示：

- 这不是一个单纯的“障碍物很明显”的问题
- 更像观测本身不完整，或者局部观测质量不好

这时要重点看：

- 是否有掉帧
- 是否有部分区域长期看不全

## 11. 怎么和 RViz 对照

建议按下面这个顺序来：

1. 先看 `ranked_frames` 里的 `frame 1`
2. 再看这个 frame 下的 `hotspot 1`
3. 记下它的 `pos`
4. 回到 RViz 找机器人相对位置相近的区域
5. 重点对照下面几个 topic

### 建议重点看的 topic

- `/terrain_debug/obstacle_points`
- `/terrain_debug/base_gravity_cloud`
- `/terrain_debug/grid_map`

### 对照时怎么做

1. 先看 hotspot 的位置是不是在你关心的检测框里
2. 再看这个位置附近是不是真的有障碍点
3. 再看 `grid_map` 附近的层值是否和输出里的 `class` 一致

例如：

- 如果输出更像 `ClearanceDriven`
  - 重点看净空是不是偏小

- 如果输出更像 `ObstacleEvidenceDriven`
  - 重点看障碍点和障碍证据是不是明显

- 如果输出更像 `ObservabilityInfluenced`
  - 重点看观测状态是不是不完整

## 12. 两个示例

### 示例一：`suspicious_frames = 0`

```text
false_obstacle_summary bag=/your/bag

detection_box: x[0.00, 1.40] y[-0.25, 0.25]
total_frames: 219
rear_dropout_frames: 36
suspicious_frames: 0
suspicious_ratio: 0.000
longest_consecutive_run: 0
root_causes:
  ClearanceDriven: 0
  ObstacleEvidenceDriven: 0
  ObstacleEvidencePlusLowContinuity: 0
  ObservabilityInfluenced: 0
  UnknownOrMixed: 0

ranked_frames:
```

怎么理解：

- 这次分析没有发现检测框内障碍点
- 所以也不会列出具体 frame 和 hotspot
- 如果你在 RViz 里仍然觉得“看起来有问题”，那问题可能不在 `obstacle_points` 这条链路上

### 示例二：有候选 frame

```text
frame 1
  stamp: 1775114321603217125
  class: ClearanceDriven
  severity: 1428.06
  in_box_obstacle_points: 862
  hotspot_count: 3
  frame_partial: true
  rear_dropout: false
  max_obstacle_evidence: 0.47
  min_clearance: 0.18
  min_support_continuity: 1.00
  hotspot 1
    pos: (0.45, -0.19)  obstacle_points: 223  severity: 225.93
    class: ClearanceDriven  observability: Observed  explanation: clearance below threshold
    obstacle_evidence: 0.47  clearance: 0.18  support_continuity: 1.00
```

怎么理解：

1. 先看 `class`
   - 这帧更像净空问题
2. 再看 `min_clearance`
   - 数值比较小，说明系统觉得可通过空间不够
3. 再看 `hotspot 1`
   - 最该优先去 RViz 里看 `(0.45, -0.19)` 附近
4. 再看 `explanation`
   - 这条提示和 `class` 是一致的

---

如果你只是想快速排查，记住一句话就够了：

> 先看 `suspicious_frames`，再看 `root_causes`，然后只盯着 `frame 1` 的 `hotspot 1` 去 RViz 对照。
