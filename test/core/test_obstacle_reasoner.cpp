#include "passable_area/core/obstacle_reasoner.hpp"
#include "passable_area/core/obstacle_publication.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

using passable_area::core::BlockReason;
using passable_area::core::Config;
using passable_area::core::ObstacleEvidenceStage;
using passable_area::core::ObstaclePublicationSample;
using passable_area::core::ObstaclePublicationSampleStats;
using passable_area::core::ObstaclePublicationContext;
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
  layers.clearance[0] = 0.3f;
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

TEST(ObstacleReasonerTest, LowClearanceGatedByStepRangeHeight) {
  auto layers = MakeLayers();
  layers.clearance[0] = 0.15f;
  layers.overhead_evidence[0] = 0.5f;

  const auto output = ObstacleReasoner(MakeConfig()).evaluate(layers);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kLowClearance));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kGatedByHeight));
}

TEST(ObstacleReasonerTest, LowClearanceGeometryWithoutOverheadStateDoesNotBlock) {
  auto layers = MakeLayers();
  layers.clearance[0] = 0.3f;
  layers.overhead_confidence[0] = 0.0f;
  layers.overhead_evidence[0] = 0.0f;

  const auto output = ObstacleReasoner(MakeConfig()).evaluate(layers);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kNone));
  EXPECT_EQ(output.overhead_stage[0],
            static_cast<uint8_t>(ObstacleEvidenceStage::kEvidenceLow));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstacleReasonerTest,
     LowClearanceActiveStateCanBlockBelowPublishEvidence) {
  auto layers = MakeLayers();
  layers.clearance[0] = 0.3f;
  layers.overhead_confidence[0] = 0.3f;
  layers.overhead_evidence[0] = 0.1f;

  const auto output = ObstacleReasoner(MakeConfig()).evaluate(layers);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kLowClearance));
  EXPECT_EQ(output.overhead_stage[0],
            static_cast<uint8_t>(ObstacleEvidenceStage::kBlocking));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kGatedByEvidence));
}

TEST(ObstacleReasonerTest,
     LowClearanceDecayedBelowActiveStateThresholdDoesNotBlock) {
  auto config = MakeConfig();
  auto layers = MakeLayers();
  layers.clearance[0] = 0.3f;
  layers.overhead_confidence[0] =
      config.persistence.obstacle_height_clear_threshold * 0.5f;
  layers.overhead_evidence[0] =
      config.persistence.obstacle_height_clear_threshold * 0.5f;

  const auto output = ObstacleReasoner(config).evaluate(layers);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kNone));
  EXPECT_EQ(output.overhead_stage[0],
            static_cast<uint8_t>(ObstacleEvidenceStage::kEvidenceLow));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstacleReasonerTest, ProtrusionBlocksWhenEvidenceHighAndTallProtrusion) {
  auto config = MakeConfig();
  auto layers = MakeLayers();
  layers.protrusion_evidence[1] = 0.7f;
  layers.support_height[1] = 0.0f;
  layers.protrusion_height[1] = config.geometry.max_step_up + 0.1f;

  const auto output = ObstacleReasoner(config).evaluate(layers);

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

TEST(ObstacleReasonerTest,
     GeometryFailureWithTallNearThresholdCandidateCanPublish) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.4f;
  config.persistence.obstacle_evidence_gain = 0.25f;
  auto layers = MakeLayers();
  layers.slope[0] = config.geometry.max_support_slope_deg + 1.0f;
  layers.protrusion_evidence[0] = 0.38f;
  layers.support_height[0] = 0.0f;
  layers.protrusion_height[0] = config.geometry.max_step_up + 0.1f;
  std::vector<uint16_t> raw_sample_count = {11U, 0U, 0U};
  std::vector<uint8_t> obstacle_candidate_cell = {1U, 0U, 0U};
  std::vector<float> current_protrusion_evidence_gain = {0.38f, 0.0f, 0.0f};
  const ObstaclePublicationContext context{
      &raw_sample_count, &obstacle_candidate_cell,
      &current_protrusion_evidence_gain};

  const auto output = ObstacleReasoner(config).evaluate(layers, context);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kGeometryFailure));
  EXPECT_EQ(
      output.obstacle_point_publish_status[0],
      static_cast<uint8_t>(
          ObstaclePointPublishStatus::kPublishedByGeometryFailure));
}

TEST(ObstacleReasonerTest, ProtrusionBlockingThresholdFollowsPublishEvidence) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.8f;
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = 0.5f;
  layers.support_height[0] = 0.0f;
  layers.protrusion_height[0] = config.geometry.max_step_up + 0.1f;

  const auto output = ObstacleReasoner(config).evaluate(layers);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kNone));
  EXPECT_EQ(output.protrusion_stage[0],
            static_cast<uint8_t>(ObstacleEvidenceStage::kEvidenceLow));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstacleReasonerTest,
     CurrentFrameCandidateHeightCanSatisfyTallPublicationContract) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.4f;
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = 0.7f;
  layers.support_height[0] = 1.5f;
  layers.protrusion_height[0] = std::numeric_limits<float>::quiet_NaN();
  std::vector<uint16_t> raw_sample_count = {6U, 0U, 0U};
  std::vector<uint8_t> obstacle_candidate_cell = {1U, 0U, 0U};
  std::vector<float> current_protrusion_evidence_gain = {0.25f, 0.0f, 0.0f};
  std::vector<float> current_obstacle_candidate_height = {
      1.8f, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()};
  const ObstaclePublicationContext context{
      &raw_sample_count, &obstacle_candidate_cell,
      &current_protrusion_evidence_gain, &current_obstacle_candidate_height};

  const auto output = ObstacleReasoner(config).evaluate(layers, context);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kProtrusion));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(
                ObstaclePointPublishStatus::kPublishedByProtrusion));
}

TEST(ObstacleReasonerTest,
     DenseCurrentFrameSourceIsExplicitPublicationDecision) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.4f;
  config.persistence.obstacle_evidence_gain = 0.25f;
  config.observability.min_points_per_sector = 12;
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = 0.38f;
  layers.support_height[0] = 0.0f;
  layers.protrusion_height[0] = config.geometry.max_step_up + 0.1f;
  std::vector<uint16_t> raw_sample_count = {11U, 0U, 0U};
  std::vector<uint8_t> obstacle_candidate_cell = {1U, 0U, 0U};
  std::vector<float> current_protrusion_evidence_gain = {0.375f, 0.0f, 0.0f};
  const ObstaclePublicationContext context{
      &raw_sample_count, &obstacle_candidate_cell,
      &current_protrusion_evidence_gain};

  const auto output = ObstacleReasoner(config).evaluate(layers, context);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kProtrusion));
  EXPECT_EQ(output.protrusion_stage[0],
            static_cast<uint8_t>(ObstacleEvidenceStage::kBlocking));
  EXPECT_EQ(
      output.obstacle_point_publish_status[0],
      static_cast<uint8_t>(
          ObstaclePointPublishStatus::kPublishedByDenseProtrusion));
}

TEST(ObstacleReasonerTest, DenseNearThresholdStepHeightDoesNotBlock) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.4f;
  config.persistence.obstacle_evidence_gain = 0.25f;
  config.observability.min_points_per_sector = 12;
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = 0.38f;
  layers.support_height[0] = 0.0f;
  layers.protrusion_height[0] = config.geometry.max_step_up;
  std::vector<uint16_t> raw_sample_count = {11U, 0U, 0U};
  std::vector<uint8_t> obstacle_candidate_cell = {1U, 0U, 0U};
  std::vector<float> current_protrusion_evidence_gain = {0.375f, 0.0f, 0.0f};
  const ObstaclePublicationContext context{
      &raw_sample_count, &obstacle_candidate_cell,
      &current_protrusion_evidence_gain};

  const auto output = ObstacleReasoner(config).evaluate(layers, context);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kNone));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstacleReasonerTest, DenseRawSamplesWithoutCurrentCandidateDoNotPublish) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.4f;
  config.observability.min_points_per_sector = 12;
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = 0.38f;
  layers.support_height[0] = 0.0f;
  layers.protrusion_height[0] = config.geometry.max_step_up + 0.1f;
  std::vector<uint16_t> raw_sample_count = {11U, 0U, 0U};
  std::vector<uint8_t> obstacle_candidate_cell = {0U, 0U, 0U};
  std::vector<float> current_protrusion_evidence_gain = {0.375f, 0.0f, 0.0f};
  const ObstaclePublicationContext context{
      &raw_sample_count, &obstacle_candidate_cell,
      &current_protrusion_evidence_gain};

  const auto output = ObstacleReasoner(config).evaluate(layers, context);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kNone));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstacleReasonerTest, SparseCurrentFrameCandidateDoesNotDensePublish) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.4f;
  config.observability.min_points_per_sector = 12;
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = 0.38f;
  layers.support_height[0] = 0.0f;
  layers.protrusion_height[0] = config.geometry.max_step_up + 0.1f;
  std::vector<uint16_t> raw_sample_count = {10U, 0U, 0U};
  std::vector<uint8_t> obstacle_candidate_cell = {1U, 0U, 0U};
  std::vector<float> current_protrusion_evidence_gain = {0.375f, 0.0f, 0.0f};
  const ObstaclePublicationContext context{
      &raw_sample_count, &obstacle_candidate_cell,
      &current_protrusion_evidence_gain};

  const auto output = ObstacleReasoner(config).evaluate(layers, context);

  EXPECT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kNone));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstacleReasonerTest, BlockReasonNoneNeverHasPublishedStatus) {
  auto config = MakeConfig();
  auto layers = MakeLayers();
  layers.protrusion_evidence[0] = config.obstacle_points_min_evidence + 0.1f;
  layers.support_height[0] = 0.0f;
  layers.protrusion_height[0] = config.geometry.max_step_up;

  const auto output = ObstacleReasoner(config).evaluate(layers);

  ASSERT_EQ(output.block_reason[0],
            static_cast<uint8_t>(BlockReason::kNone));
  EXPECT_EQ(output.obstacle_point_publish_status[0],
            static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));
}

TEST(ObstaclePublicationTest, HeightGateRejectsStepBoundaryAndMissingSupport) {
  auto config = MakeConfig();
  ObstaclePublicationSampleStats step_boundary_sample;
  passable_area::core::AccumulateObstaclePublicationSample(
      config,
      static_cast<uint8_t>(ObstaclePointPublishStatus::kPublishedByProtrusion),
      0.0f,
      ObstaclePublicationSample{config.geometry.max_step_up, 0.0f,
                                config.geometry.max_step_up},
      step_boundary_sample);

  const auto step_boundary =
      passable_area::core::EvaluateObstaclePublicationDecisionTrace(
          config,
          static_cast<uint8_t>(
              ObstaclePointPublishStatus::kPublishedByProtrusion),
          0.0f, step_boundary_sample);

  EXPECT_FALSE(step_boundary.height_gate.above_step_height);
  EXPECT_FALSE(step_boundary.height_gate.passes);
  EXPECT_EQ(step_boundary.final_status,
            ObstaclePointPublishStatus::kGatedByHeight);

  ObstaclePublicationSampleStats tall_sample;
  passable_area::core::AccumulateObstaclePublicationSample(
      config,
      static_cast<uint8_t>(ObstaclePointPublishStatus::kPublishedByProtrusion),
      std::numeric_limits<float>::quiet_NaN(),
      ObstaclePublicationSample{config.geometry.max_step_up + 0.1f, 0.0f,
                                config.geometry.max_step_up + 0.1f},
      tall_sample);
  const auto missing_support =
      passable_area::core::EvaluateObstaclePublicationDecisionTrace(
          config,
          static_cast<uint8_t>(
              ObstaclePointPublishStatus::kPublishedByProtrusion),
          std::numeric_limits<float>::quiet_NaN(), tall_sample);

  EXPECT_FALSE(missing_support.height_gate.finite_support_ref);
  EXPECT_FALSE(missing_support.height_gate.passes);
  EXPECT_EQ(missing_support.final_status,
            ObstaclePointPublishStatus::kGatedByHeight);
}

TEST(ObstaclePublicationTest, GeometryFailureRequiresBaseGravityFloor) {
  auto config = MakeConfig();
  ObstaclePublicationSampleStats below_floor_sample;
  passable_area::core::AccumulateObstaclePublicationSample(
      config,
      static_cast<uint8_t>(
          ObstaclePointPublishStatus::kPublishedByGeometryFailure),
      0.0f,
      ObstaclePublicationSample{config.geometry.max_step_up + 0.1f, 0.0f,
                                config.obstacle_points_min_height - 0.01f},
      below_floor_sample);

  const auto trace =
      passable_area::core::EvaluateObstaclePublicationDecisionTrace(
          config,
          static_cast<uint8_t>(
              ObstaclePointPublishStatus::kPublishedByGeometryFailure),
          0.0f, below_floor_sample);

  EXPECT_FALSE(trace.height_gate.above_publish_path_floor);
  EXPECT_FALSE(trace.height_gate.passes);
  EXPECT_EQ(trace.final_status, ObstaclePointPublishStatus::kGatedByHeight);
}

TEST(ObstaclePublicationTest, TraceReportsNoSamplesAndPublishPath) {
  auto config = MakeConfig();
  const ObstaclePublicationSampleStats no_samples{};

  const auto trace =
      passable_area::core::EvaluateObstaclePublicationDecisionTrace(
          config,
          static_cast<uint8_t>(
              ObstaclePointPublishStatus::kPublishedByDenseProtrusion),
          0.0f, no_samples);

  EXPECT_TRUE(trace.reasoner_requests_publication);
  EXPECT_FALSE(trace.has_current_samples);
  EXPECT_EQ(trace.final_status,
            ObstaclePointPublishStatus::kBlockedButNoSamples);
  EXPECT_STREQ(passable_area::core::ObstaclePublicationPathName(
                   static_cast<uint8_t>(ObstaclePointPublishStatus::
                                            kPublishedByDenseProtrusion)),
               "DenseProtrusionSource");
}
