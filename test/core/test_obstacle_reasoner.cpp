#include "passable_area/core/obstacle_reasoner.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using passable_area::core::BlockReason;
using passable_area::core::Config;
using passable_area::core::ObstacleEvidenceStage;
using passable_area::core::ObstaclePointPublishStatus;
using passable_area::core::ObstacleReasoner;
using passable_area::core::SupportState;
using passable_area::core::TerrainLayers;

TerrainLayers MakeLayers(int cell_count = 3) {
  TerrainLayers layers;
  layers.support_height.assign(cell_count, 0.0f);
  layers.support_confidence.assign(cell_count, 1.0f);
  layers.overhead_height.assign(cell_count,
                                std::numeric_limits<float>::quiet_NaN());
  layers.overhead_confidence.assign(cell_count, 0.0f);
  layers.protrusion_height.assign(cell_count,
                                  std::numeric_limits<float>::quiet_NaN());
  layers.protrusion_evidence.assign(cell_count, 0.0f);
  layers.overhead_evidence.assign(cell_count, 0.0f);
  layers.obstacle_evidence.assign(cell_count, 0.0f);
  layers.coverage_confidence.assign(cell_count, 1.0f);
  layers.slope.assign(cell_count, 0.0f);
  layers.step_up.assign(cell_count, 0.0f);
  layers.step_down.assign(cell_count, 0.0f);
  layers.roughness.assign(cell_count, 0.0f);
  layers.clearance.assign(cell_count, std::numeric_limits<float>::infinity());
  layers.support_continuity.assign(cell_count, 1.0f);
  layers.support_state.assign(cell_count,
                              static_cast<uint8_t>(SupportState::kObserved));
  return layers;
}

Config MakeConfig() {
  Config config;
  config.geometry.min_clearance = 0.35f;
  config.obstacle_points_min_evidence = 0.4f;
  return config;
}

} // namespace

TEST(ObstacleReasonerTest, LowClearanceBlocksAndRequestsOverheadPublish) {
  auto layers = MakeLayers();
  layers.clearance[0] = 0.2f;
  layers.overhead_evidence[0] = 0.5f;

  const auto output = ObstacleReasoner(MakeConfig()).evaluate(layers);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kLowClearance));
  EXPECT_EQ(output.overhead_stage[0],
            static_cast<uint8_t>(ObstacleEvidenceStage::kBlocking));
  EXPECT_EQ(
      output.obstacle_point_publish_status[0],
      static_cast<uint8_t>(ObstaclePointPublishStatus::kPublishedByOverhead));
}

TEST(ObstacleReasonerTest, ProtrusionBlocksWhenEvidenceAndContinuityAgree) {
  auto layers = MakeLayers();
  layers.protrusion_evidence[1] = 0.7f;
  layers.support_continuity[1] = 0.1f;

  const auto output = ObstacleReasoner(MakeConfig()).evaluate(layers);

  EXPECT_EQ(output.block_reason[1],
            static_cast<uint8_t>(BlockReason::kProtrusion));
  EXPECT_EQ(output.protrusion_stage[1],
            static_cast<uint8_t>(ObstacleEvidenceStage::kBlocking));
  EXPECT_EQ(
      output.obstacle_point_publish_status[1],
      static_cast<uint8_t>(ObstaclePointPublishStatus::kPublishedByProtrusion));
}

TEST(ObstacleReasonerTest, GeometryFailureDoesNotClaimObstaclePointPublish) {
  auto layers = MakeLayers();
  layers.slope[2] = 45.0f;

  const auto output = ObstacleReasoner(MakeConfig()).evaluate(layers);

  EXPECT_EQ(output.block_reason[2],
            static_cast<uint8_t>(BlockReason::kGeometryFailure));
  EXPECT_EQ(output.obstacle_point_publish_status[2],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstacleReasonerTest, EvidenceBelowPublishThresholdIsReportedAsGated) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.8f;
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = 0.5f;
  layers.support_continuity[0] = 0.1f;

  const auto output = ObstacleReasoner(config).evaluate(layers);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kProtrusion));
  EXPECT_EQ(output.protrusion_stage[0],
            static_cast<uint8_t>(ObstacleEvidenceStage::kBlocking));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kGatedByEvidence));
}
