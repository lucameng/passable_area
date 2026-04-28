# Passable Area 测试说明

这份文档整理当前 `passable_area` 的测试资产、覆盖范围、运行方式和最近一次本地验证结果。

当前版本按 V2 双层语义理解：

- `PolarFrontend` 负责 per-cell profile、support/protrusion/overhead candidates 和 dropout-aware observability 输入
- `DropoutAwareMapUpdater` 维护 support、protrusion、overhead、coverage 以及显式 `support_surface_contaminated` layer
- `ObstacleReasoner` 负责 impassable block reason 和 obstacle-point publication status
- runtime 与 offline analyzer 共享 core publication decision/path 语义；诊断输出不应重新推导发布门

## 1. 自动测试

这些目标会被 `colcon test --packages-select passable_area` 自动执行：

- `test_processor`
- `test_obstacle_reasoner`
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

- `Summary: 148 tests, 0 errors, 0 failures, 0 skipped`

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
- `PolarFrontend`
  - support candidate 使用当前帧 lower support band
  - suspicious `vertical_span`、pure protrusion gain、overhead low-clearance candidate
  - sparse lower leak 不污染 support band
  - dropout sector 抑制不可靠 support candidate
- terrain geometry
  - slope 使用实际邻接距离
  - support-plane roughness / slope / step
  - active protrusion support contamination 被排除
  - overhead-only low clearance 仍参与 support geometry
  - support contamination 不跟随 obstacle-point publication threshold 调参
- obstacle point publication contract
  - 需要有限 support reference
  - `height <= max_step_up` 不发布
  - base-link ceiling 生效
  - dense-source、rear dropout bridge、low-clearance bridge 仍经过 Reasoner/publication contract

### 2.2 `test_obstacle_reasoner`

文件：

- `test/core/test_obstacle_reasoner.cpp`

覆盖：

- protrusion / low-clearance / geometry-failure block reason
- protrusion、dense protrusion、geometry failure、overhead bridge publication status
- weak evidence、height gate、no-sample 等 gated status
- `support_surface_contaminated` 不改变 obstacle publication contract

### 2.3 `test_output_converter`

文件：

- `test/interfaces/ros/test_output_converter.cpp`

覆盖：

- `terrain_state / terrain_cost / grid_map` 使用 `base_gravity`
- 非零 yaw 时的重采样方向
- `grid_map` 与 occupancy 输出对齐

### 2.4 `test_debug_publishers`

文件：

- `test/interfaces/ros/test_debug_publishers.cpp`

覆盖：

- `unknown_mask`
- observability debug 输出
- `base_gravity` 上下文是否保持一致

### 2.5 `test_status_code_manager`

文件：

- `test/interfaces/ros/test_status_code_manager.cpp`

覆盖：

- 正常运行状态
- odom timeout
- sync stall
- output stall
- sticky fatal 状态码覆盖

### 2.6 `test_ros_param_loader`

文件：

- `test/interfaces/ros/test_ros_param_loader.cpp`

覆盖：

- frame 参数默认值
- legacy 顶层参数兼容
- 新旧参数并存时优先取新位置

### 2.7 `test_passable_area_node_frames`

文件：

- `test/interfaces/ros/test_passable_area_node_frames.cpp`

覆盖：

- 默认不发布 `map -> base_gravity` TF
- 打开参数后发布指定 TF
- 输入 frame 不匹配时给出 warning

### 2.8 `test_false_obstacle_analyzer`

文件：

- `test/tools/test_false_obstacle_analyzer.cpp`

覆盖：

- detection box 内 obstacle point 命中/漏出
- hotspot 聚类和排序
- `ClearanceDriven`
- `ObstacleEvidencePlusLowContinuity`
- source cell 到离散前端状态查找
- source cell 的 low-clearance bridge 状态来自 core publication status

说明：

- analyzer 面向离线误检归因，但发布路径名称和 bridge 判定必须跟 runtime `obstacle_point_publish_status` 保持一致

### 2.9 `test_miss_obstacle_analyzer`

文件：

- `test/tools/test_miss_obstacle_analyzer.cpp`

覆盖：

- ROI 没有样本
- 有样本但没有前端 obstacle suspicion
- 有 candidate 但 `obstacle_evidence` 不足
- 有 evidence 但没有通过 obstacle point 高度门槛
- evidence cell 在 ROI 内但当前没有对应 source sample
- 高度门要求有限 support reference，且 `height <= max_step_up` 不发布
- sample `min_z` fallback 不能授予发布资格
- runtime sample 坐标必须相关，不能把不同样本的极值组合成发布资格

说明：

- 当前 V2 主路径重点是：
  - `NoFrontendObstacleSuspicion`
  - `ObstacleEvidenceTooLow`
  - `OutputHeightGateNotMet`
  - `NoObstacleSourceSamplesInRoi`
- offline miss 归因必须复用 core publication gate/trace，不在 tool 内维护独立高度门语义

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

当前 V2 后处理后，测试策略收敛到三条主线：

- 主门禁由 `test_processor` 锁住 core pipeline、dropout-aware map、support geometry 和 publication contract
- `test_obstacle_reasoner` 锁住 Reasoner block/status 语义
- tools 层测试继续守住 analyzer 契约，避免离线排查工具失真
- docs/benchmarks 用于补充真实 bag 验收，不替代 core semantic tests

## 5. 后续补强建议

建议继续补的方向是：

1. 用真实 bag 为 missed obstacle 场景建立固定回放样例
2. 给 false / miss analyzer 增加更多 publication trace 边界样本
3. 后续障碍增强逻辑优先落在地图层、证据累计层或 Reasoner，不把 bag-specific 规则塞回 `PolarFrontend`
4. 所有新行为都先补 core semantic tests，再补离线 replay 复现链路

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
