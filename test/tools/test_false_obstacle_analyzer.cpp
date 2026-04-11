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
  config.detection_box = FalseObstacleDetectionBox{-0.5f, 0.5f, -0.5f, 0.5f};
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
  output.support_anchor_used.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.sub_support_leak_count.assign(9, 0U);
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
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPointWithoutSource(
      {0.1f, 0.0f, 0.0f}));

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->in_box_obstacle_point_count, 1);
}

TEST(FalseObstacleAnalyzerTest, IgnoresObstaclePointsOutsideConfigured2DBox) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPointWithoutSource(
      {1.2f, 0.0f, 0.0f}));

  const auto analysis = analyzer.analyzeFrame(output);
  EXPECT_FALSE(analysis.has_value());
}

TEST(FalseObstacleAnalyzerTest, ClassifiesClearanceDriven) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPointWithoutSource(
      {0.0f, 0.0f, 0.0f}));
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
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPointWithoutSource(
      {0.0f, 0.0f, 0.0f}));
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
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPointWithoutSource(
      {-0.2f, -0.2f, 0.1f}));
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPointWithoutSource(
      {-0.18f, -0.18f, 0.3f}));
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPointWithoutSource(
      {0.25f, 0.25f, 0.5f}));

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  ASSERT_GE(analysis->hotspots.size(), 2U);
  EXPECT_GE(analysis->hotspots[0].obstacle_point_count, analysis->hotspots[1].obstacle_point_count);
  EXPECT_FLOAT_EQ(analysis->hotspots[0].min_z, 0.1f);
  EXPECT_FLOAT_EQ(analysis->hotspots[0].max_z, 0.3f);
}

TEST(FalseObstacleAnalyzerTest, UsesSupportingPointSourceCellForDiscreteFrontendStateLookup) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPoint(
      {0.45f, 0.45f, 0.1f}, 0));
  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPoint(
      {0.35f, 0.35f, 0.2f}, 0));
  output.upper_support_cell.assign(9, 0U);
  output.obstacle_suspicious.assign(9, 0U);
  output.obstacle_candidate_cell.assign(9, 0U);
  output.obstacle_rejected_by_neighbor_support.assign(9, 0U);
  output.neighbor_upper_support_count.assign(9, 0);
  output.obstacle_evidence[0] = 0.7f;
  output.overhead_height[0] = 0.9f;
  output.support_anchor_used[0] = 0.25f;
  output.upper_support_cell[0] = 1U;
  output.obstacle_suspicious[0] = 1U;
  output.obstacle_candidate_cell[0] = 1U;
  output.obstacle_rejected_by_neighbor_support[0] = 1U;
  output.neighbor_upper_support_count[0] = 1;
  const int center_cell = CenterCellIndex(output);
  ASSERT_NE(center_cell, 0);
  output.obstacle_evidence[center_cell] = 0.3f;
  output.support_anchor_used[center_cell] = 0.15f;
  output.sub_support_leak_count[center_cell] = 2U;
  output.clearance[center_cell] = 0.2f;
  output.overhead_height[center_cell] = 0.8f;

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  ASSERT_FALSE(analysis->hotspots.empty());
  EXPECT_EQ(analysis->hotspots.front().source_cell, 0);
  EXPECT_EQ(analysis->hotspots.front().center_cell, center_cell);
  EXPECT_FALSE(analysis->hotspots.front().source_matches_center);
  EXPECT_TRUE(analysis->hotspots.front().upper_support_cell);
  EXPECT_TRUE(analysis->hotspots.front().obstacle_suspicious);
  EXPECT_TRUE(analysis->hotspots.front().obstacle_candidate_cell);
  EXPECT_TRUE(analysis->hotspots.front().obstacle_rejected_by_neighbor_support);
  EXPECT_EQ(analysis->hotspots.front().neighbor_upper_support_count, 1);
  EXPECT_FLOAT_EQ(analysis->hotspots.front().source_obstacle_evidence, 0.7f);
  EXPECT_FLOAT_EQ(analysis->hotspots.front().source_overhead_height, 0.9f);
  EXPECT_FLOAT_EQ(analysis->hotspots.front().source_support_anchor_used, 0.25f);
  EXPECT_FLOAT_EQ(analysis->hotspots.front().support_anchor_used, 0.15f);
  EXPECT_EQ(analysis->hotspots.front().sub_support_leak_count, 2U);
  EXPECT_EQ(analysis->hotspots.front().classification, FalseObstacleRootCause::kClearanceDriven);
}

TEST(FalseObstacleAnalyzerTest, InvalidSourceCellDoesNotMarkDiscreteLookupAsGridBacked) {
  FalseObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  FrameOutput output;
  output.rows = 1;
  output.cols = 1;
  output.resolution = 0.1f;
  output.origin = Eigen::Vector2f(-0.05f, -0.05f);
  output.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  output.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  output.obstacle_evidence.assign(1, 0.0f);
  output.clearance.assign(1, std::numeric_limits<float>::quiet_NaN());
  output.support_continuity.assign(1, 1.0f);
  output.overhead_height.assign(1, std::numeric_limits<float>::quiet_NaN());
  output.support_anchor_used.assign(1, std::numeric_limits<float>::quiet_NaN());
  output.sub_support_leak_count.assign(1, 0U);
  output.observability.sectors.resize(8);
  for (auto &sector : output.observability.sectors) {
    sector.state = ObservabilityState::kObserved;
  }

  output.obstacle_points.push_back(passable_area::core::MakeCellDebugPoint({0.45f, 0.45f, 0.1f}, 7));
  output.upper_support_cell.assign(1, 1U);
  output.obstacle_suspicious.assign(1, 1U);
  output.obstacle_rejected_by_neighbor_support.assign(1, 1U);
  output.neighbor_upper_support_count.assign(1, 1);

  const auto analysis = analyzer.analyzeFrame(output);
  ASSERT_TRUE(analysis.has_value());
  ASSERT_FALSE(analysis->hotspots.empty());
  EXPECT_FALSE(analysis->hotspots.front().has_grid_values);
  EXPECT_EQ(analysis->hotspots.front().classification, FalseObstacleRootCause::kUnknownOrMixed);
}
