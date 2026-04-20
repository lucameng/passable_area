# Passable Area 测试说明

这份文档整理当前 `passable_area` 的测试资产、覆盖范围、运行方式和最近一次本地验证结果。

当前版本有一个重要前提：

- `PolarFrontend` 已经回退到接近 `f92759c` 的简单语义
- 前端只负责 per-cell 原始统计、support candidate、直接 `vertical_span` 障碍形成
- 复杂的 neighbor gate / explanation 链不再是当前主判定路径

因此，测试说明也必须按这个基线理解。

## 1. 自动测试

这些目标会被 `colcon test --packages-select passable_area` 自动执行：

- `test_processor`
- `test_output_converter`
- `test_debug_publishers`
- `test_status_code_manager`
- `test_ros_param_loader`
- `test_passable_area_node_frames`
- `test_false_obstacle_analyzer`
- `test_miss_obstacle_analyzer`

最近一次本地执行：

```bash
colcon test --packages-select passable_area --event-handlers console_direct+
colcon test-result --verbose
```

结果：

- `Summary: 80 tests, 0 errors, 0 failures, 0 skipped`

## 2. 自动测试覆盖矩阵

### 2.1 `test_processor`

文件：

- `test/core/test_processor.cpp`

这是当前最核心的行为回归入口，覆盖：

- 预处理
  - body filter
  - 地图窗口裁剪
  - z 裁剪与 base 相对语义
- observability / dropout
  - rear dropout
  - `MissingByDropout`
  - sparse coverage -> `Unknown`
- 地图更新与通行性
  - support 持续性
  - recenter 后历史层平移
  - dropout 下 obstacle 不被激进清空
  - 动态障碍可被后续地面重观测清除
- 简化后的 `PolarFrontend`
  - support candidate 直接取当前帧 `min_z`
  - `vertical_span` 直接形成 obstacle candidate
  - `PartiallyObserved` 且未触发 obstacle 时生成 ambiguous candidate
  - 历史复杂字段保持兼容但默认不激活
- obstacle point 发布门槛
  - `obstacle_points_min_height`
  - `obstacle_points_max_height_in_base_link`
  - 弱 `obstacle_evidence` 不发布 obstacle points

当前障碍语义回退后，这个文件重点锁的是：

- 前端回到简单基线以后，障碍形成不会再被邻域 gate 或 explanation 分支拦掉
- 但 `Processor` 仍然保留当前 obstacle point 发布门槛

### 2.2 `test_output_converter`

文件：

- `test/interfaces/ros/test_output_converter.cpp`

覆盖：

- `terrain_state / terrain_cost / grid_map` 使用 `base_gravity`
- 非零 yaw 时的重采样方向
- `grid_map` 与 occupancy 输出对齐

### 2.3 `test_debug_publishers`

文件：

- `test/interfaces/ros/test_debug_publishers.cpp`

覆盖：

- `unknown_mask`
- observability debug 输出
- `base_gravity` 上下文是否保持一致

### 2.4 `test_status_code_manager`

文件：

- `test/interfaces/ros/test_status_code_manager.cpp`

覆盖：

- 正常运行状态
- odom timeout
- sync stall
- output stall
- sticky fatal 状态码覆盖

### 2.5 `test_ros_param_loader`

文件：

- `test/interfaces/ros/test_ros_param_loader.cpp`

覆盖：

- frame 参数默认值
- legacy 顶层参数兼容
- 新旧参数并存时优先取新位置

### 2.6 `test_passable_area_node_frames`

文件：

- `test/interfaces/ros/test_passable_area_node_frames.cpp`

覆盖：

- 默认不发布 `map -> base_gravity` TF
- 打开参数后发布指定 TF
- 输入 frame 不匹配时给出 warning

### 2.7 `test_false_obstacle_analyzer`

文件：

- `test/tools/test_false_obstacle_analyzer.cpp`

覆盖：

- detection box 内 obstacle point 命中/漏出
- hotspot 聚类和排序
- `ClearanceDriven`
- `ObstacleEvidencePlusLowContinuity`
- source cell 到离散前端状态查找

说明：

- analyzer 仍会读取一批历史前端字段
- 这些字段的读取契约被保留，但不代表当前前端仍靠这些分支判障碍

### 2.8 `test_miss_obstacle_analyzer`

文件：

- `test/tools/test_miss_obstacle_analyzer.cpp`

覆盖：

- ROI 没有样本
- 有样本但没有前端 obstacle suspicion
- 有 candidate 但 `obstacle_evidence` 不足
- 有 evidence 但没有通过 obstacle point 高度门槛
- evidence cell 在 ROI 内但当前没有对应 source sample
- 历史兼容字段驱动的 reject / leak 类 root cause 仍可被解释

说明：

- 当前简单前端的主路径，重点是：
  - `NoFrontendObstacleSuspicion`
  - `ObstacleEvidenceTooLow`
  - `OutputHeightGateNotMet`
  - `NoObstacleSourceSamplesInRoi`
- `RejectedByNeighborSupport`、`LeakFilteredToNoCandidate` 这类结果现在主要是兼容性守卫，用来保证 analyzer 不被历史字段契约打断

## 3. 手工验证资产

这些目标会被正常构建，但不会进入 `colcon test`：

- `passable_area_benchmark`
- `passable_area_e2e_benchmark`
- `passable_area_offline_replay --analyze-false-obstacles`
- `passable_area_offline_replay --analyze-missed-obstacles`
- `passable_area_offline_replay --inspect-roi`
- `scripts/passable_benchmark.sh`

它们的角色是：

- benchmark
- bag 级离线回归
- 误检 / 漏检诊断

如果你改的是障碍语义，推荐最少补两类验证：

1. `colcon test`
2. 至少一个 `offline_replay --analyze-missed-obstacles` 或 `--analyze-false-obstacles`

## 4. 当前测试策略的重点

这次前端回退后，测试策略也跟着收敛：

- 主门禁由 `test_processor` 锁住简单前端基线
- `Processor` 保留当前 obstacle point 发布门槛，由对应 case 锁住
- tools 层测试继续守住 analyzer 契约，避免离线排查工具失真
- 不再把复杂 neighbor gate / explanation 规则当成当前必须维护的主行为

## 5. 后续补强建议

建议继续补的方向是：

1. 用真实 bag 为 missed obstacle 场景建立固定回放样例
2. 给 false / miss analyzer 增加一组“当前简单前端基线”专用回归样本
3. 如果后续要重新引入障碍增强逻辑，优先加到地图层或证据累计层，不要直接把规则栈塞回 `PolarFrontend`
4. 所有新行为都先补 `test_processor`，再补离线 replay 复现链路

## 6. 常用命令

自动测试：

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select passable_area --event-handlers console_direct+
colcon test-result --verbose
```

离线诊断：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
build/passable_area/passable_area_offline_replay --analyze-false-obstacles ...
build/passable_area/passable_area_offline_replay --analyze-missed-obstacles ...
build/passable_area/passable_area_offline_replay --inspect-roi ...
```

benchmark：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
build/passable_area/passable_area_benchmark
build/passable_area/passable_area_e2e_benchmark
```
