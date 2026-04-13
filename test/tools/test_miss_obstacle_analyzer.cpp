#include "passable_area/tools/miss_obstacle_analyzer.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using passable_area::core::Config;
using passable_area::core::FrameOutput;
using passable_area::core::MakeCellDebugPointWithoutSource;
using passable_area::core::ObservabilityState;
using passable_area::core::ProcessedFrame;
using passable_area::core::Point3f;
using passable_area::tools::MissObstacleAnalyzer;
using passable_area::tools::MissObstacleAnalyzerConfig;
using passable_area::tools::MissObstacleDetectionBox;
using passable_area::tools::MissObstacleRootCause;

Config MakeConfig() {
  Config config;
  config.geometry.min_clearance = 0.35f;
  config.observability.sector_count = 8;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.2f;
  config.obstacle_points_max_height_in_base_link = 0.2f;
  return config;
}

MissObstacleAnalyzerConfig MakeAnalyzerConfig() {
  MissObstacleAnalyzerConfig config;
  config.detection_box = MissObstacleDetectionBox{-0.5f, 0.5f, -0.5f, 0.5f};
  config.representative_cell_limit = 2;
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
  output.support_height.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.support_confidence.assign(9, 0.0f);
  output.support_anchor_used.assign(9, std::numeric_limits<float>::quiet_NaN());
  output.sub_support_leak_count.assign(9, 0U);
  output.raw_upper_support_cell.assign(9, 0U);
  output.explanation_adjusted_upper_support_cell.assign(9, 0U);
  output.upper_support_cell.assign(9, 0U);
  output.obstacle_suspicious.assign(9, 0U);
  output.obstacle_candidate_cell.assign(9, 0U);
  output.obstacle_rejected_by_neighbor_support.assign(9, 0U);
  output.neighbor_upper_support_count.assign(9, 0);
  output.observability.sectors.resize(8);
  for (auto &sector : output.observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }
  return output;
}

ProcessedFrame MakeProcessedFrame(std::initializer_list<Point3f> points) {
  ProcessedFrame frame;
  frame.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  frame.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  for (const auto &point : points) {
    frame.odom_samples.push_back({point, point});
  }
  return frame;
}

int CenterCellIndex(const FrameOutput &output) {
  return output.cols + 1;
}

} // namespace

TEST(MissObstacleAnalyzerTest, ReturnsNoSamplesRootCauseWhenRoiIsEmpty) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  const auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({});

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kNoSamplesInRoi);
  EXPECT_EQ(analysis->roi_sample_count, 0);
}

TEST(MissObstacleAnalyzerTest, ReturnsNoFrontendSuspicionWhenSamplesStayFlat) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, 0.05f}});
  const int cell = CenterCellIndex(output);
  output.support_height[cell] = 0.05f;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kNoFrontendObstacleSuspicion);
  EXPECT_EQ(analysis->obstacle_suspicious_cell_count, 0);
}

TEST(MissObstacleAnalyzerTest, ReturnsRejectedByNeighborSupportWhenSuspiciousCellsAreRejected) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, 0.3f}});
  const int cell = CenterCellIndex(output);
  output.obstacle_suspicious[cell] = 1U;
  output.obstacle_rejected_by_neighbor_support[cell] = 1U;
  output.neighbor_upper_support_count[cell] = 1;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kRejectedByNeighborSupport);
  EXPECT_EQ(analysis->rejected_suspicious_cell_count, 1);
}

TEST(MissObstacleAnalyzerTest, PreservesRawAdjustedAndLegacyUpperSupportVisibility) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, 0.3f}});
  const int cell = CenterCellIndex(output);
  output.obstacle_suspicious[cell] = 1U;
  output.obstacle_rejected_by_neighbor_support[cell] = 1U;
  output.raw_upper_support_cell[cell] = 1U;
  output.explanation_adjusted_upper_support_cell[cell] = 0U;
  output.upper_support_cell[cell] = 0U;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  ASSERT_FALSE(analysis->representative_cells.empty());
  EXPECT_TRUE(analysis->representative_cells.front().raw_upper_support_cell);
  EXPECT_FALSE(analysis->representative_cells.front().adjusted_upper_support_cell);
  EXPECT_FALSE(analysis->representative_cells.front().upper_support_cell);
}

TEST(MissObstacleAnalyzerTest, ReturnsObstacleEvidenceTooLowWhenCandidateDoesNotAccumulateEnoughEvidence) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, 0.35f}});
  const int cell = CenterCellIndex(output);
  output.obstacle_suspicious[cell] = 1U;
  output.obstacle_candidate_cell[cell] = 1U;
  output.obstacle_evidence[cell] = 0.25f;
  output.support_height[cell] = 0.0f;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kObstacleEvidenceTooLow);
  EXPECT_FLOAT_EQ(analysis->max_obstacle_evidence, 0.25f);
}

TEST(MissObstacleAnalyzerTest, ReturnsOutputHeightGateNotMetWhenStrongEvidenceHasNoHighEnoughSamples) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, 0.10f}});
  const int cell = CenterCellIndex(output);
  output.obstacle_suspicious[cell] = 1U;
  output.obstacle_candidate_cell[cell] = 1U;
  output.obstacle_evidence[cell] = 0.6f;
  output.support_height[cell] = 0.0f;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kOutputHeightGateNotMet);
  ASSERT_FALSE(analysis->representative_cells.empty());
  EXPECT_NEAR(analysis->representative_cells.front().max_sample_z_minus_support_ref, 0.10f, 1e-5f);
}

TEST(MissObstacleAnalyzerTest, ReturnsOutputHeightGateNotMetWhenSamplesExceedBaseLinkHeightCeiling) {
  auto config = MakeConfig();
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 0.2f;
  MissObstacleAnalyzer analyzer(config, MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, 0.25f}});
  const int cell = CenterCellIndex(output);
  output.obstacle_suspicious[cell] = 1U;
  output.obstacle_candidate_cell[cell] = 1U;
  output.obstacle_evidence[cell] = 0.6f;
  output.support_height[cell] = 0.0f;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kOutputHeightGateNotMet);
}

TEST(MissObstacleAnalyzerTest, ReturnsNoObstacleSourceSamplesWhenStrongEvidenceCellsHaveNoCurrentSamples) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  output.resolution = 0.5f;
  output.origin = Eigen::Vector2f(-0.75f, -0.75f);
  const auto frame = MakeProcessedFrame({Point3f{-0.49f, 0.0f, 0.10f}});
  const int cell = CenterCellIndex(output);
  output.obstacle_evidence[cell] = 0.7f;
  output.support_height[cell] = 0.0f;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kNoObstacleSourceSamplesInRoi);
}

TEST(MissObstacleAnalyzerTest, ReturnsLeakFilteredRootCauseWhenOnlyLeakEvidenceRemains) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, -0.30f}});
  const int cell = CenterCellIndex(output);
  output.sub_support_leak_count[cell] = 2U;
  output.support_anchor_used[cell] = 0.0f;

  const auto analysis = analyzer.analyzeFrame(output, frame);
  ASSERT_TRUE(analysis.has_value());
  EXPECT_EQ(analysis->classification, MissObstacleRootCause::kLeakFilteredToNoCandidate);
}

TEST(MissObstacleAnalyzerTest, ReturnsNulloptWhenObstaclePointsAlreadyExistInsideRoi) {
  MissObstacleAnalyzer analyzer(MakeConfig(), MakeAnalyzerConfig());
  auto output = MakeOutput();
  const auto frame = MakeProcessedFrame({Point3f{0.0f, 0.0f, 0.30f}});
  output.obstacle_points.push_back(MakeCellDebugPointWithoutSource({0.0f, 0.0f, 0.30f}));

  const auto analysis = analyzer.analyzeFrame(output, frame);
  EXPECT_FALSE(analysis.has_value());
}
