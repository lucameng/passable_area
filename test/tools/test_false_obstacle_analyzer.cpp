#include "passable_area/tools/false_obstacle_analyzer.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using passable_area::core::CellDebugPoint;
using passable_area::core::Config;
using passable_area::core::FrameOutput;
using passable_area::core::ObservabilityState;
using passable_area::tools::FalseObstacleAnalyzer;
using passable_area::tools::FalseObstacleAnalyzerConfig;
using passable_area::tools::FalseObstacleDetectionBox;
using passable_area::tools::FalseObstacleRootCause;

Config MakeConfig() {
  Config config;
  config.geometry.min_clearance = 0.35f;
  config.observability.sector_count = 8;
  return config;
}

FalseObstacleAnalyzerConfig MakeAnalyzerConfig() {
  FalseObstacleAnalyzerConfig config;
  config.detection_box = FalseObstacleDetectionBox{0.5f, 0.5f, -0.5f, 0.5f};
  config.hotspot_bin_size = 0.1f;
  return config;
}

FrameOutput MakeOutput() {
  FrameOutput output;
  output.rows = 3;
  output.cols = 3;
  output.resolution = 1.0f;
  output.origin = Eigen::Vector2f(-1.5f, -1.5f);
  output.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  output.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  output.obstacle_evidence.assign(9, 0.0f);
  output.clearance.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.support_continuity.assign(9, 1.0f);
  output.overhead_height.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.observability.sectors.resize(8);
  for (auto &sector : output.observability.sectors) {
    sector.state = ObservabilityState::kObserved;
  }
  return output;
}

int CenterCellIndex(const FrameOutput &output) {
  return 1 * output.cols + 1;
}

} // namespace

TEST(FalseObstacleAnalyzerTest, DetectsObstaclePointsInsideConfigured2DBox) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(CellDebugPoint{{0.1f, 0.0f, 0.0f}});

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->in_box_obstacle_point_count, 1);
}

TEST(FalseObstacleAnalyzerTest, IgnoresObstaclePointsOutsideConfigured2DBox) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(CellDebugPoint{{1.2f, 0.0f, 0.0f}});

  const auto analysis = analyzer.analyzeFrame(output);
  EXPECT_FALSE(analysis.has_value());
}

TEST(FalseObstacleAnalyzerTest, ClassifiesClearanceDriven) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(CellDebugPoint{{0.0f, 0.0f, 0.0f}});
  const int cell = CenterCellIndex(output);
  output.clearance[cell] = 0.2f;
  output.overhead_height[cell] = 0.8f;

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  ASSERT_FALSE(analysis->hotspots.empty());
  EXPECT_EQ(analysis->hotspots.front().classification, FalseObstacleRootCause::kClearanceDriven);
  EXPECT_EQ(analysis->classification, FalseObstacleRootCause::kClearanceDriven);
}

TEST(FalseObstacleAnalyzerTest, ClassifiesObstacleEvidencePlusLowContinuity) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(CellDebugPoint{{0.0f, 0.0f, 0.0f}});
  const int cell = CenterCellIndex(output);
  output.obstacle_evidence[cell] = 0.7f;
  output.support_continuity[cell] = 0.1f;

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  ASSERT_FALSE(analysis->hotspots.empty());
  EXPECT_EQ(analysis->hotspots.front().classification,
            FalseObstacleRootCause::kObstacleEvidencePlusLowContinuity);
}

TEST(FalseObstacleAnalyzerTest, ProducesHotspotRankingForMultipleInBoxClusters) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(CellDebugPoint{{-0.2f, -0.2f, 0.0f}});
  output.obstacle_points.push_back(CellDebugPoint{{-0.18f, -0.18f, 0.0f}});
  output.obstacle_points.push_back(CellDebugPoint{{0.25f, 0.25f, 0.0f}});

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  ASSERT_GE(analysis->hotspots.size(), 2U);
  EXPECT_GE(analysis->hotspots[0].obstacle_point_count, analysis->hotspots[1].obstacle_point_count);
}
