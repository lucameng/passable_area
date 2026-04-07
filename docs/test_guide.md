# Passable Area 测试说明

这份文档整理当前 `passable_area` 的测试资产、覆盖 case、运行方式和最近验证状态。

目标是回答 4 个问题：

1. 哪些测试会被 `colcon test` 自动执行
2. 哪些程序虽然有用，但只是手工 benchmark / harness
3. 现有测试分别覆盖了哪些功能和场景
4. 目前还缺什么，以及后续应怎么补

## 1. 测试资产分类

当前仓库里的测试/验证资产分成两类。

### 1.1 自动测试

这些目标会被 `colcon test --packages-select passable_area` 自动执行：

- `test_processor`
- `test_output_converter`
- `test_debug_publishers`
- `test_false_obstacle_analyzer`
- `test_miss_obstacle_analyzer`

截至本轮整理，自动测试总共有 5 个 target、58 个 gtest case。

### 1.2 手工 benchmark / harness

这些目标会被正常构建，但不会进入 `colcon test`：

- `passable_area_benchmark`
- `passable_area_e2e_benchmark`
- `scripts/passable_benchmark.sh`
- `passable_area_offline_replay --analyze-false-obstacles`
- `passable_area_offline_replay --analyze-missed-obstacles`
- `passable_area_offline_replay --inspect-roi`

它们仍然有效，不属于“过时测试”，但语义上是：

- 性能 benchmark
- 端到端吞吐/延迟 benchmark
- bag 级功能回归或诊断工具

因此这次已将两个 benchmark 源文件移出 `test/` 目录，避免和 gtest 混淆。

## 2. 自动测试覆盖矩阵

### 2.1 `test_processor`

文件：

- `test/core/test_processor.cpp`

覆盖内容：

- 预处理
  - body filter 去除机身内部点
  - 裁剪到局部地图窗口
  - z 裁剪使用机器人相对高度语义
- 通行性基础场景
  - 平地可通行
  - 斜坡保持可通行
  - 楼梯保留可通行带
  - 低净空判为 `Impassable`
- observability / dropout
  - rear dropout 标记
  - 后向 gap 导致 `MissingByDropout`
  - rear 正常时不误伤支撑
  - 不再产生固定 blind sector
  - sparse coverage 导致 `Unknown` 但不是 dropout
- support 持续性与地图更新
  - local hole 不会立刻硬清 support
  - observed support state 不会被同帧 persistent 覆盖
  - recenter 后历史层平移仍正确
  - dropout 下 obstacle 不会被激进清空
  - 地面重观测后动态障碍可以清除
- PolarFrontend obstacle formation
  - suspicious cell 没有邻域支撑时拒绝
  - 有 3x3 upper-support cluster 时确认 obstacle
  - 无历史 support 时使用当前帧 `min_z`
  - 有效历史锚点下的 sub-support leak 抑制
  - finite 但无效历史 support 不参与 leak 过滤
  - 本 cell 锚点失效时使用邻域中位数锚点
  - 锚点过滤后样本为空时不回退原始样本
  - stale lower anchor / stair mix rejection
  - below-robot stair mix rejection
  - below-robot 真实障碍不过滤
  - low anchor 被 upper band 主导时仍 reject
- debug / obstacle point 发布
  - support_points 使用 base_gravity 语义
  - obstacle_points 只发布 obstacle cell 的 upper band
  - low ceiling 下不发布地面点
  - wall base noise 不进入 obstacle_points
  - 弱 evidence 默认不发布 obstacle_points
  - wall with base noise 仍保持 `Impassable`
  - static low ceiling 持续为 `Impassable`

结论：

- 这是当前最核心、覆盖最广的行为测试文件
- 没有过时 case，需要继续作为主力回归入口保留

### 2.2 `test_output_converter`

文件：

- `test/interfaces/ros/test_output_converter.cpp`

覆盖内容：

- `terrain_state` / `terrain_cost` / `grid_map` 使用 `base_gravity`
- 各输出共享一致的机器人中心几何
- 非零 yaw 时重采样方向正确
- `grid_map` 与 occupancy 输出保持对齐

结论：

- 仍然有效
- 用于兜底 ROS 输出层的坐标系与栅格对齐语义

### 2.3 `test_debug_publishers`

文件：

- `test/interfaces/ros/test_debug_publishers.cpp`

覆盖内容：

- `unknown_mask` 与 observability 调试消息是否保持 `base_gravity` 上下文

结论：

- 有效，但覆盖面较窄
- 当前更像“ROS 调试输出语义守卫”

### 2.4 `test_false_obstacle_analyzer`

文件：

- `test/tools/test_false_obstacle_analyzer.cpp`

覆盖内容：

- 检测框内 obstacle point 命中/漏出
- hotspot 聚类与排序
- `ClearanceDriven`
- `ObstacleEvidencePlusLowContinuity`
- source cell 到离散前端状态查找
- invalid source cell 时不误标为 grid-backed

结论：

- 有效
- 已覆盖 false obstacle 分析器最关键的分类和热点选择逻辑
- 但它不是 bag 级回归，更多是局部单元语义测试

### 2.5 `test_miss_obstacle_analyzer`

文件：

- `test/tools/test_miss_obstacle_analyzer.cpp`

覆盖内容：

- ROI 没有样本
- 有样本但没有前端 suspicious
- suspicious 被 neighbor-support gate 拒绝
- 有 candidate 但 `obstacle_evidence` 不足
- 有强 evidence 但没过 obstacle point 高度门槛
- 有强 evidence cell 但当前 ROI 内没有 source sample
- leak filtering 导致没有 candidate
- ROI 内已存在 obstacle points 时，不属于 miss obstacle

结论：

- 这是最新加入的漏检分析测试
- 当前可以作为 miss-obstacle 工具的第一层单元回归

## 3. 手工 benchmark / 离线验证资产

### 3.1 `passable_area_benchmark`

源码：

- `benchmarks/core/benchmark_processor.cpp`

用途：

- 纯 core `Processor` 吞吐 benchmark
- 输入是合成点云
- 输出 `avg_ms / p95_ms / p99_ms`

特点：

- 不验证算法正确性
- 用于观察单机性能回归

### 3.2 `passable_area_e2e_benchmark`

源码：

- `benchmarks/interfaces/ros/e2e_benchmark.cpp`

用途：

- ROS 节点端到端延迟 benchmark
- 统计：
  - `avg_ms`
  - `p95_ms`
  - `p99_ms`
  - `drop_rate_pct`
  - `cpu_pct`

特点：

- 不是 gtest
- 更接近吞吐/延迟 smoke benchmark

### 3.3 `scripts/passable_benchmark.sh`

用途：

- 基于 `config/offline_benchmark_bags.yaml` 跑 bag 级批量验证
- 汇总 obstacle 结果和 timing 结果

特点：

- 这是当前最接近“离线功能回归基线”的入口
- 但它是脚本回归，不属于 `colcon test`

### 3.4 `passable_area_offline_replay`

用途：

- `--analyze-false-obstacles`：误检离线分析
- `--analyze-missed-obstacles`：漏检离线分析
- `--inspect-roi`：低层 ROI 状态转储

特点：

- 它们是诊断工具，不是自动门禁测试
- 但在问题复现和参数回归时非常重要

## 4. 最近验证状态

### 4.1 本轮已实际运行

已跑过：

- `colcon build --packages-select passable_area --symlink-install`
- `colcon test --packages-select passable_area --event-handlers console_direct+`
- `colcon test-result --verbose`

结果：

- 当前 `passable_area` 自动测试 5 个 target 全部通过
- 最近一次该包 gtest 覆盖共 58 个 case

本轮还实跑了：

- `passable_area_offline_replay --analyze-false-obstacles`
- `passable_area_offline_replay --analyze-missed-obstacles`
- `build/passable_area/passable_area_benchmark`
- `build/passable_area/passable_area_e2e_benchmark`

说明：

- 离线分析入口当前是可用的
- false / miss 两条工具链都已经被最近验证过
- core benchmark 已输出吞吐结果
- e2e benchmark 已输出端到端延迟结果，并确认 target 可运行

### 4.2 能用但本轮未重新执行

本轮未重新执行的有效资产：

- `scripts/passable_benchmark.sh`

原因：

- 它不是自动门禁
- 本轮已完成自动测试、离线分析工具和两个本地 benchmark 的可用性确认
- 但没有重新跑整套 bag 批量回归脚本

结论：

- 它们不是过时内容
- 只是当前不属于“每次改动默认会跑”的验证链路

## 5. 过时项与整理结论

### 5.1 本轮判定为过时/误导的内容

过时的不是测试逻辑本身，而是目录归类：

- 旧的
  - `test/core/benchmark_processor.cpp`
  - `test/interfaces/ros/e2e_benchmark.cpp`
- 问题：
  - 放在 `test/` 下容易被误认为属于 `colcon test`
  - 实际它们只是手工 benchmark

本轮处理：

- 将它们迁移到 `benchmarks/`
- 保留可执行目标
- 在 CMake 中明确注释为 manual benchmark

### 5.2 本轮保留的内容

- 所有现有 gtest target
- 所有现有 gtest case
- 离线分析工具
- bag 级 benchmark 脚本

理由：

- 目前没有发现与当前实现语义明显冲突、应立即删除的自动测试
- 它们最近都能构建，自动测试也已实际通过

## 6. 建议补充的信息与后续方向

当前还建议后续补的，不是“删除旧测试”，而是补齐测试治理：

1. 为 `scripts/passable_benchmark.sh` 建立固定的“最近一次运行结果记录”入口  
2. 给 `passable_area_benchmark` / `passable_area_e2e_benchmark` 增加独立说明文档，记录推荐运行环境和基线指标  
3. 后续如果 miss/false analyzer 再扩展分类，优先同步补 tools 层单测，而不是只依赖真实 bag 人工验证  
4. 若将来引入 CI，应明确：
   - 默认门禁只跑 gtest
   - benchmark 和 bag 回归走手工/定时任务

## 7. 常用命令

自动测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select passable_area --symlink-install
colcon test --packages-select passable_area --event-handlers console_direct+
colcon test-result --verbose
```

手工 benchmark：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
build/passable_area/passable_area_benchmark
build/passable_area/passable_area_e2e_benchmark
```

bag 级回归：

```bash
./scripts/passable_benchmark.sh
```

离线诊断：

```bash
build/passable_area/passable_area_offline_replay --analyze-false-obstacles ...
build/passable_area/passable_area_offline_replay --analyze-missed-obstacles ...
build/passable_area/passable_area_offline_replay --inspect-roi ...
```
