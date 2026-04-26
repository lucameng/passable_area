#include "passable_area/core/frame_preprocessor.hpp"
#include "passable_area/core/mapping/dropout_aware_map_updater.hpp"
#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/polar_frontend.hpp"
#include "passable_area/core/processor.hpp"
#include "passable_area/core/terrain_feature_updater.hpp"
#include "passable_area/core/traversability_solver.hpp"
#include "passable_area/core/types/obstacle_types.hpp"

#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>
#include <utility>

namespace {

using passable_area::core::BlockReason;
using passable_area::core::DropoutAwareMapUpdater;
using passable_area::core::FrameInput;
using passable_area::core::FrameObservability;
using passable_area::core::FramePreprocessor;
using passable_area::core::FrontendOutput;
using passable_area::core::LocalTerrainMap;
using passable_area::core::MapGeometry;
using passable_area::core::ObservabilityState;
using passable_area::core::ObstacleEvidenceStage;
using passable_area::core::ObstacleReasonerOutput;
using passable_area::core::OverheadCandidate;
using passable_area::core::PassabilityState;
using passable_area::core::PolarFrontend;
using passable_area::core::ProcessedFrame;
using passable_area::core::Processor;
using passable_area::core::ProtrusionCandidate;
using passable_area::core::SupportCandidate;
using passable_area::core::SupportState;
using passable_area::core::TerrainFeatureUpdater;
using passable_area::core::TraversabilitySolver;

FrameInput MakeFlatFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (float x = -1.0f; x <= 1.0f; x += 0.2f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.2f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeRampFrame(float slope, int stamp = 1) {
  passable_area::core::FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (float x = -1.0f; x <= 1.0f; x += 0.2f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.2f) {
      input.input_cloud_in_base.push_back({x, y, slope * x});
    }
  }
  return input;
}

FrameInput MakeStairFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (float x = -1.0f; x <= 1.0f; x += 0.08f) {
    const float z = 0.08f * std::floor((x + 1.0f) / 0.4f);
    for (float y = -1.0f; y <= 1.0f; y += 0.08f) {
      input.input_cloud_in_base.push_back({x, y, z});
    }
  }
  return input;
}

FrameInput MakeLowCeilingFrame(float ceiling_height, int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  for (float x = -0.6f; x <= 0.6f; x += 0.12f) {
    for (float y = -0.6f; y <= 0.6f; y += 0.12f) {
      input.input_cloud_in_base.push_back({x, y, ceiling_height});
    }
  }
  return input;
}

FrameInput MakeFrontOnlyFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (float x = 0.0f; x <= 1.2f; x += 0.1f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.1f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeRearOnlyFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (float x = -1.2f; x <= -0.1f; x += 0.1f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.1f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeRearGapFrame(int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  input.input_cloud_in_base.erase(
      std::remove_if(input.input_cloud_in_base.begin(),
                     input.input_cloud_in_base.end(),
                     [](const auto &point) {
                       return point.x < -0.1f && std::abs(point.y) < 1.0f;
                     }),
      input.input_cloud_in_base.end());
  for (float x = 0.2f; x <= 1.2f; x += 0.05f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.05f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeFrontGapFrame(int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  input.input_cloud_in_base.erase(
      std::remove_if(input.input_cloud_in_base.begin(),
                     input.input_cloud_in_base.end(),
                     [](const auto &point) {
                       return point.x > 0.1f && std::abs(point.y) < 1.0f;
                     }),
      input.input_cloud_in_base.end());
  for (float x = -1.2f; x <= -0.2f; x += 0.05f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.05f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeLocalHoleFrame(int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  input.input_cloud_in_base.erase(
      std::remove_if(
          input.input_cloud_in_base.begin(), input.input_cloud_in_base.end(),
          [](const auto &point) {
            return std::abs(point.x) < 0.35f && std::abs(point.y) < 0.35f;
          }),
      input.input_cloud_in_base.end());
  return input;
}

FrameInput MakeSparseFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  int counter = 0;
  for (float x = -1.0f; x <= 1.0f; x += 0.08f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.08f) {
      if ((counter++ % 9) == 0) {
        input.input_cloud_in_base.push_back({x, y, 0.0f});
      }
    }
  }
  return input;
}

FrameInput MakeForwardStripFrame(const Eigen::Quaternionf &orientation,
                                 int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = orientation.normalized();
  for (float x = 0.4f; x <= 1.2f; x += 0.08f) {
    for (float y = -0.12f; y <= 0.12f; y += 0.06f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeObstacleColumnFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},  {0.25f, 0.25f, 0.12f},  {0.25f, 0.25f, 0.30f},
      {-0.25f, 0.25f, 0.0f}, {-0.25f, 0.25f, 0.12f}, {-0.25f, 0.25f, 0.30f},
  };
  return input;
}

FrameInput MakeSingleCellColumnFrame(int upper_sample_count, int stamp = 1,
                                     float upper_height = 0.45f) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (int i = 0; i < 3; ++i) {
    input.input_cloud_in_base.push_back({0.25f, 0.25f, 0.0f});
  }
  for (int i = 0; i < upper_sample_count; ++i) {
    input.input_cloud_in_base.push_back({0.25f, 0.25f, upper_height});
  }
  return input;
}

FrameInput MakeSingleCellZCountsFrame(
    std::initializer_list<std::pair<float, int>> z_counts, int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (const auto &[z, count] : z_counts) {
    for (int i = 0; i < count; ++i) {
      input.input_cloud_in_base.push_back({0.25f, 0.25f, z});
    }
  }
  return input;
}

FrameInput MakeWallWithBaseNoiseFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},   {0.25f, 0.25f, 0.05f},  {0.25f, 0.25f, 0.25f},
      {0.25f, 0.25f, 0.45f},  {-0.25f, 0.25f, 0.0f},  {-0.25f, 0.25f, 0.05f},
      {-0.25f, 0.25f, 0.25f}, {-0.25f, 0.25f, 0.45f},
  };
  return input;
}

FrameInput MakeUnsupportedWallFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.25f},  {0.25f, 0.25f, 0.45f},  {0.25f, 0.25f, 0.65f},
      {-0.25f, 0.25f, 0.25f}, {-0.25f, 0.25f, 0.45f}, {-0.25f, 0.25f, 0.65f},
  };
  return input;
}

FrameInput MakePitchedObstacleColumnFrame(const Eigen::Quaternionf &orientation,
                                          float obstacle_height_in_base,
                                          int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = orientation.normalized();
  input.input_cloud_in_base = {
      {0.55f, -0.25f, 0.0f},
      {0.55f, -0.25f, obstacle_height_in_base},
      {0.55f, 0.25f, 0.0f},
      {0.55f, 0.25f, obstacle_height_in_base},
  };
  return input;
}

FrameInput MakePitchedObstacleStackFrame(const Eigen::Quaternionf &orientation,
                                         float lower_obstacle_height_in_base,
                                         float upper_obstacle_height_in_base,
                                         int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = orientation.normalized();
  input.input_cloud_in_base = {
      {0.55f, -0.25f, 0.0f},
      {0.55f, -0.25f, lower_obstacle_height_in_base},
      {0.55f, -0.25f, upper_obstacle_height_in_base},
      {0.55f, 0.25f, 0.0f},
      {0.55f, 0.25f, lower_obstacle_height_in_base},
      {0.55f, 0.25f, upper_obstacle_height_in_base},
  };
  return input;
}

ProcessedFrame MakeProcessedFrame(
    const std::vector<passable_area::core::MapPointSample> &samples) {
  ProcessedFrame frame;
  frame.base_pose_in_map.position = Eigen::Vector3f::Zero();
  frame.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  frame.map_samples = samples;
  frame.cloud_in_base.reserve(samples.size());
  frame.cloud_in_map.reserve(samples.size());
  for (const auto &sample : samples) {
    frame.cloud_in_base.push_back(sample.point_in_base);
    frame.cloud_in_map.push_back(sample.point_in_map);
  }
  return frame;
}

FrameInput MakeDynamicObstacleCellFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},  {0.25f, 0.25f, 0.18f},  {0.25f, 0.25f, 0.45f},
      {-0.25f, 0.25f, 0.0f}, {-0.25f, 0.25f, 0.18f}, {-0.25f, 0.25f, 0.45f},
  };
  return input;
}

FrameInput MakeGroundOnlyCellFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},  {0.28f, 0.22f, 0.0f},  {0.22f, 0.28f, 0.0f},
      {-0.25f, 0.25f, 0.0f}, {-0.22f, 0.22f, 0.0f}, {-0.28f, 0.28f, 0.0f},
  };
  return input;
}

FrameInput MakeRearObstacleCellFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {-0.8f, 0.8f, 0.0f}, {-0.8f, 0.8f, 0.18f}, {-0.8f, 0.8f, 0.45f},
      {-0.6f, 0.8f, 0.0f}, {-0.6f, 0.8f, 0.18f}, {-0.6f, 0.8f, 0.45f},
  };
  return input;
}

FrameInput MakeTranslatedFrontRaisedColumnFrame(float base_x, float support_z,
                                                float obstacle_z,
                                                int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_map.position = Eigen::Vector3f(base_x, 0.0f, 0.0f);
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  for (float x = 0.0f; x <= 1.2f; x += 0.05f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.05f) {
      input.input_cloud_in_base.push_back({x, y, support_z});
    }
  }
  input.input_cloud_in_base.push_back({0.0f, 0.8f, obstacle_z});
  input.input_cloud_in_base.push_back({0.02f, 0.8f, obstacle_z});
  input.input_cloud_in_base.push_back({0.0f, 0.82f, obstacle_z});
  return input;
}

int CellIndex(const passable_area::core::FrameOutput &output, float x,
              float y) {
  const int col =
      static_cast<int>(std::floor((x - output.origin.x()) / output.resolution));
  const int row =
      static_cast<int>(std::floor((y - output.origin.y()) / output.resolution));
  return row * output.cols + col;
}

int CountRearObstaclePoints(const passable_area::core::FrameOutput &output) {
  return static_cast<int>(std::count_if(
      output.obstacle_points.begin(), output.obstacle_points.end(),
      [](const auto &point) { return point.point.x < 0.0f; }));
}

bool IsPublishedObstaclePointStatus(uint8_t status) {
  switch (
      static_cast<passable_area::core::ObstaclePointPublishStatus>(status)) {
  case passable_area::core::ObstaclePointPublishStatus::kPublishedByProtrusion:
  case passable_area::core::ObstaclePointPublishStatus::
      kPublishedByDenseProtrusion:
  case passable_area::core::ObstaclePointPublishStatus::
      kPublishedByGeometryFailure:
  case passable_area::core::ObstaclePointPublishStatus::kPublishedByOverhead:
    return true;
  case passable_area::core::ObstaclePointPublishStatus::kNotApplicable:
  case passable_area::core::ObstaclePointPublishStatus::kGatedByEvidence:
  case passable_area::core::ObstaclePointPublishStatus::kGatedByHeight:
  case passable_area::core::ObstaclePointPublishStatus::kBlockedButNoSamples:
    return false;
  }
  return false;
}

bool HasObstaclePointNearZ(const passable_area::core::FrameOutput &output,
                           float z, float tolerance = 1e-4f) {
  return std::any_of(output.obstacle_points.begin(),
                     output.obstacle_points.end(), [&](const auto &point) {
                       return std::abs(point.point.z - z) <= tolerance;
                     });
}

passable_area::core::Config MakeConfig() {
  passable_area::core::Config config;
  config.map.length = 4.0f;
  config.map.width = 4.0f;
  config.map.resolution = 0.2f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 36;
  config.observability.min_points_per_sector = 4;
  config.observability.dropout_sector_gap_threshold = 4;
  config.observability.min_support_confidence = 0.15f;
  config.persistence.support_persistence_frames = 6;
  config.geometry.max_support_slope_deg = 30.0f;
  config.geometry.max_step_up = 0.22f;
  config.geometry.max_step_down = 0.30f;
  config.geometry.max_support_roughness = 0.10f;
  return config;
}

FrameObservability
MakeUniformObservability(const passable_area::core::Config &config,
                         ObservabilityState state,
                         float coverage_confidence = 1.0f) {
  FrameObservability observability;
  observability.sectors.resize(
      static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = state;
    sector.coverage_confidence = coverage_confidence;
  }
  return observability;
}

MapGeometry MakeMapGeometry(const LocalTerrainMap &map) {
  return MapGeometry{map.rows(), map.cols(), map.size(), map.resolution(),
                     map.origin()};
}

ObstacleReasonerOutput MakeReasonerOutput(int cell_count) {
  ObstacleReasonerOutput output;
  output.block_reason.assign(cell_count,
                             static_cast<uint8_t>(BlockReason::kNone));
  output.protrusion_stage.assign(
      cell_count, static_cast<uint8_t>(ObstacleEvidenceStage::kNone));
  output.overhead_stage.assign(
      cell_count, static_cast<uint8_t>(ObstacleEvidenceStage::kNone));
  output.obstacle_point_publish_status.assign(
      cell_count,
      static_cast<uint8_t>(
          passable_area::core::ObstaclePointPublishStatus::kNotApplicable));
  return output;
}

void SeedReliableSupportCell(LocalTerrainMap &map, int cell) {
  auto &layers = map.layers();
  layers.coverage_confidence[cell] = 1.0f;
  layers.support_height[cell] = 0.0f;
  layers.support_confidence[cell] = 1.0f;
  layers.support_state[cell] = static_cast<uint8_t>(SupportState::kObserved);
  layers.last_reliable_age[cell] = 0U;
  layers.clearance[cell] = std::numeric_limits<float>::infinity();
  layers.support_continuity[cell] = 1.0f;
}

} // namespace

TEST(ProcessorTest, FlatGroundProducesPassableCells) {
  Processor processor(MakeConfig());

  const auto output = processor.update(MakeFlatFrame());
  ASSERT_TRUE(output.valid);

  int passable_count = 0;
  for (const auto value : output.passability) {
    if (value == static_cast<int8_t>(PassabilityState::kPassable)) {
      ++passable_count;
    }
  }
  EXPECT_GT(passable_count, 0);
}

TEST(ProcessorTest, TraversabilitySolverUsesReasonerBlockReason) {
  const auto config = MakeConfig();
  LocalTerrainMap map(config);
  TraversabilitySolver solver(config);
  const int cell = 0;
  SeedReliableSupportCell(map, cell);

  auto reasoner_output = MakeReasonerOutput(map.size());
  reasoner_output.block_reason[cell] =
      static_cast<uint8_t>(BlockReason::kGeometryFailure);

  solver.update(map, reasoner_output);

  EXPECT_EQ(map.layers().passability_state[cell],
            static_cast<int8_t>(PassabilityState::kImpassable));
  EXPECT_EQ(map.layers().traversal_cost[cell], 100);
}

TEST(ProcessorTest, TraversabilitySolverKeepsUnknownPriorityOverReasonerBlock) {
  const auto config = MakeConfig();
  LocalTerrainMap map(config);
  TraversabilitySolver solver(config);
  const int cell = 0;

  auto reasoner_output = MakeReasonerOutput(map.size());
  reasoner_output.block_reason[cell] =
      static_cast<uint8_t>(BlockReason::kLowClearance);

  solver.update(map, reasoner_output);

  EXPECT_EQ(map.layers().passability_state[cell],
            static_cast<int8_t>(PassabilityState::kUnknown));
  EXPECT_EQ(map.layers().traversal_cost[cell], -1);
}

TEST(ProcessorTest, TraversabilitySolverDoesNotRedoGeometryFailureChecks) {
  const auto config = MakeConfig();
  LocalTerrainMap map(config);
  TraversabilitySolver solver(config);
  const int cell = 0;
  SeedReliableSupportCell(map, cell);
  map.layers().slope[cell] = config.geometry.max_support_slope_deg + 10.0f;
  map.layers().step_up[cell] = config.geometry.max_step_up + 0.1f;
  map.layers().step_down[cell] = config.geometry.max_step_down + 0.1f;
  map.layers().roughness[cell] = config.geometry.max_support_roughness + 0.1f;
  map.layers().clearance[cell] = 0.0f;

  const auto reasoner_output = MakeReasonerOutput(map.size());
  solver.update(map, reasoner_output);

  EXPECT_EQ(map.layers().passability_state[cell],
            static_cast<int8_t>(PassabilityState::kPassable));
  EXPECT_GE(map.layers().traversal_cost[cell], 1);
  EXPECT_LE(map.layers().traversal_cost[cell], 99);
}

TEST(PreprocessorTest, BodyFilterRemovesPointsInsideConfiguredBaseLinkBox) {
  auto config = MakeConfig();
  config.preprocess.body_filter.enable = true;
  config.preprocess.body_filter.x_min = -0.4f;
  config.preprocess.body_filter.x_max = 0.4f;
  config.preprocess.body_filter.y_min = -0.2f;
  config.preprocess.body_filter.y_max = 0.2f;
  config.preprocess.body_filter.z_min = -0.5f;
  config.preprocess.body_filter.z_max = 0.5f;
  FramePreprocessor preprocessor(config);

  FrameInput input;
  input.stamp = 1;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {{0.0f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}};

  ProcessedFrame output;
  ASSERT_TRUE(preprocessor.process(input, output));
  ASSERT_EQ(output.cloud_in_base.size(), 1U);
  ASSERT_EQ(output.cloud_in_map.size(), 1U);
  EXPECT_FLOAT_EQ(output.cloud_in_base.front().x, 0.6f);
}

TEST(PreprocessorTest, CropToMapRemovesPointsOutsideLocalMapWindow) {
  auto config = MakeConfig();
  config.preprocess.crop_to_map.enable = true;
  config.preprocess.crop_to_map.xy_margin = 0.0f;
  FramePreprocessor preprocessor(config);

  FrameInput input;
  input.stamp = 1;
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {{0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}};

  ProcessedFrame output;
  ASSERT_TRUE(preprocessor.process(input, output));
  ASSERT_EQ(output.cloud_in_base.size(), 1U);
  ASSERT_EQ(output.cloud_in_map.size(), 1U);
  EXPECT_FLOAT_EQ(output.cloud_in_base.front().x, 0.0f);
}

TEST(PreprocessorTest, CropToMapUsesRobotRelativeHeightForZWindow) {
  auto config = MakeConfig();
  config.map.height_min = -0.5f;
  config.map.height_max = 0.5f;
  config.preprocess.crop_to_map.enable = true;
  config.preprocess.crop_to_map.xy_margin = 0.0f;
  FramePreprocessor preprocessor(config);

  FrameInput input;
  input.stamp = 1;
  input.base_pose_in_map.position = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {{0.6f, 0.0f, 0.2f}, {0.8f, 0.0f, 0.7f}};

  ProcessedFrame output;
  ASSERT_TRUE(preprocessor.process(input, output));
  ASSERT_EQ(output.cloud_in_base.size(), 1U);
  ASSERT_EQ(output.cloud_in_map.size(), 1U);
  EXPECT_NEAR(output.cloud_in_map.front().z, 1.2f, 1e-5f);
}

TEST(ProcessorTest, MissingCoverageLeavesUnknownCells) {
  auto config = MakeConfig();
  config.observability.min_points_per_sector = 50;
  Processor processor(config);

  const auto output = processor.update(MakeFlatFrame());
  ASSERT_TRUE(output.valid);

  int unknown_count = 0;
  for (const auto value : output.passability) {
    if (value == static_cast<int8_t>(PassabilityState::kUnknown)) {
      ++unknown_count;
    }
  }
  EXPECT_GT(unknown_count, 0);
}

TEST(ProcessorTest, RampRemainsPassable) {
  Processor processor(MakeConfig());
  const auto output = processor.update(MakeRampFrame(0.12f));
  ASSERT_TRUE(output.valid);

  int passable_count = 0;
  int impassable_count = 0;
  for (const auto value : output.passability) {
    if (value == static_cast<int8_t>(PassabilityState::kPassable)) {
      ++passable_count;
    }
    if (value == static_cast<int8_t>(PassabilityState::kImpassable)) {
      ++impassable_count;
    }
  }
  EXPECT_GT(passable_count, impassable_count);
}

TEST(ProcessorTest, StairSceneKeepsTraversableBand) {
  Processor processor(MakeConfig());
  const auto output = processor.update(MakeStairFrame());
  ASSERT_TRUE(output.valid);

  int passable_count = 0;
  for (const auto value : output.passability) {
    if (value == static_cast<int8_t>(PassabilityState::kPassable)) {
      ++passable_count;
    }
  }
  EXPECT_GT(passable_count, 10);
}

TEST(ProcessorTest, LowCeilingCreatesImpassableCells) {
  Processor processor(MakeConfig());
  const auto output = processor.update(MakeLowCeilingFrame(0.2f));
  ASSERT_TRUE(output.valid);

  int impassable_count = 0;
  for (const auto value : output.passability) {
    if (value == static_cast<int8_t>(PassabilityState::kImpassable)) {
      ++impassable_count;
    }
  }
  EXPECT_GT(impassable_count, 0);
}

TEST(ProcessorTest, RearDropoutIsFlaggedAndSupportPersists) {
  Processor processor(MakeConfig());
  ASSERT_TRUE(processor.update(MakeFlatFrame(1)).valid);
  const auto output = processor.update(MakeFrontOnlyFrame(100000001));
  ASSERT_TRUE(output.valid);
  EXPECT_FALSE(output.observability.front_dropout);
  EXPECT_TRUE(output.observability.rear_dropout);

  float max_rear_support_conf = 0.0f;
  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const int idx = row * output.cols + col;
      const float x = output.origin.x() +
                      (static_cast<float>(col) + 0.5f) * output.resolution;
      if (x < -0.4f) {
        max_rear_support_conf =
            std::max(max_rear_support_conf, output.support_confidence[idx]);
      }
    }
  }
  EXPECT_GE(max_rear_support_conf, 0.05f);
}

TEST(ProcessorTest, FrontDropoutIsFlaggedAndSupportPersists) {
  Processor processor(MakeConfig());
  ASSERT_TRUE(processor.update(MakeFlatFrame(1)).valid);
  const auto output = processor.update(MakeRearOnlyFrame(100000001));
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.observability.front_dropout);
  EXPECT_FALSE(output.observability.rear_dropout);

  float max_front_support_conf = 0.0f;
  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const int idx = row * output.cols + col;
      const float x = output.origin.x() +
                      (static_cast<float>(col) + 0.5f) * output.resolution;
      if (x > 0.4f) {
        max_front_support_conf =
            std::max(max_front_support_conf, output.support_confidence[idx]);
      }
    }
  }
  EXPECT_GE(max_front_support_conf, 0.05f);
}

TEST(ProcessorTest, RearDropoutBridgesPreviousRearObstaclePointsForOneFrame) {
  auto config = MakeConfig();
  config.map.resolution = 0.3f;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_GT(CountRearObstaclePoints(source_output), 0);

  const auto dropout_output = processor.update(MakeFrontOnlyFrame(300000001));
  ASSERT_TRUE(dropout_output.valid);
  EXPECT_TRUE(dropout_output.observability.rear_dropout);
  EXPECT_GT(CountRearObstaclePoints(dropout_output), 0);
  EXPECT_LE(CountRearObstaclePoints(dropout_output),
            CountRearObstaclePoints(source_output));
}

TEST(ProcessorTest, RearDropoutBridgeReprojectsAfterTranslation) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_GT(CountRearObstaclePoints(source_output), 0);

  auto dropout_frame = MakeFrontOnlyFrame(300000001);
  dropout_frame.base_pose_in_map.position.x() = 0.1f;
  const auto dropout_output = processor.update(dropout_frame);
  ASSERT_TRUE(dropout_output.valid);
  ASSERT_TRUE(dropout_output.observability.rear_dropout);
  ASSERT_GT(CountRearObstaclePoints(dropout_output), 0);
  ASSERT_LE(CountRearObstaclePoints(dropout_output),
            CountRearObstaclePoints(source_output));
  for (const auto &bridged_point : dropout_output.obstacle_points) {
    if (bridged_point.point.x >= 0.0f) {
      continue;
    }
    const bool matches_reprojected_source = std::any_of(
        source_output.obstacle_points.begin(),
        source_output.obstacle_points.end(), [&](const auto &source_point) {
          return source_point.point.x < 0.0f &&
                 std::abs((bridged_point.point.x + 0.1f) -
                          source_point.point.x) < 1e-4f;
        });
    EXPECT_TRUE(matches_reprojected_source);
    const bool still_at_old_base_gravity_x = std::any_of(
        source_output.obstacle_points.begin(),
        source_output.obstacle_points.end(), [&](const auto &source_point) {
          return source_point.point.x < 0.0f &&
                 std::abs(bridged_point.point.x - source_point.point.x) < 1e-4f;
        });
    EXPECT_FALSE(still_at_old_base_gravity_x);
  }
}

TEST(ProcessorTest, RearDropoutBridgeReprojectsAfterYawRotation) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_GT(CountRearObstaclePoints(source_output), 0);

  constexpr float kYaw = 0.2f;
  auto dropout_frame = MakeFrontOnlyFrame(300000001);
  dropout_frame.base_pose_in_map.orientation =
      Eigen::AngleAxisf(kYaw, Eigen::Vector3f::UnitZ());
  const auto dropout_output = processor.update(dropout_frame);
  ASSERT_TRUE(dropout_output.valid);
  ASSERT_TRUE(dropout_output.observability.rear_dropout);
  ASSERT_GT(CountRearObstaclePoints(dropout_output), 0);
  ASSERT_LE(CountRearObstaclePoints(dropout_output),
            CountRearObstaclePoints(source_output));

  const float cos_yaw = std::cos(kYaw);
  const float sin_yaw = std::sin(kYaw);
  for (const auto &bridged_point : dropout_output.obstacle_points) {
    if (bridged_point.point.x >= 0.0f) {
      continue;
    }
    const bool matches_reprojected_source = std::any_of(
        source_output.obstacle_points.begin(),
        source_output.obstacle_points.end(), [&](const auto &source_point) {
          if (source_point.point.x >= 0.0f) {
            return false;
          }
          const float expected_x =
              cos_yaw * source_point.point.x + sin_yaw * source_point.point.y;
          const float expected_y =
              -sin_yaw * source_point.point.x + cos_yaw * source_point.point.y;
          return std::abs(bridged_point.point.x - expected_x) < 1e-4f &&
                 std::abs(bridged_point.point.y - expected_y) < 1e-4f;
        });
    EXPECT_TRUE(matches_reprojected_source);
    const bool still_at_old_base_gravity_xy = std::any_of(
        source_output.obstacle_points.begin(),
        source_output.obstacle_points.end(), [&](const auto &source_point) {
          return source_point.point.x < 0.0f &&
                 std::abs(bridged_point.point.x - source_point.point.x) <
                     1e-4f &&
                 std::abs(bridged_point.point.y - source_point.point.y) < 1e-4f;
        });
    EXPECT_FALSE(still_at_old_base_gravity_xy);
  }
}

TEST(ProcessorTest,
     RearDropoutBridgeDropsCachedPointBelowCurrentSupportRelativeGate) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_TRUE(HasObstaclePointNearZ(source_output, 0.45f));

  const auto dropout_output = processor.update(
      MakeTranslatedFrontRaisedColumnFrame(-0.8f, 0.4f, 0.8f, 300000001));
  ASSERT_TRUE(dropout_output.valid);
  ASSERT_TRUE(dropout_output.observability.rear_dropout);
  EXPECT_FALSE(HasObstaclePointNearZ(dropout_output, 0.45f));
  for (const auto &point : dropout_output.obstacle_points) {
    ASSERT_GE(point.source_cell, 0);
    ASSERT_LT(point.source_cell, dropout_output.rows * dropout_output.cols);
    ASSERT_TRUE(
        std::isfinite(dropout_output.support_height[point.source_cell]));
    EXPECT_GT(point.point.z, dropout_output.support_height[point.source_cell] +
                                 config.geometry.max_step_up);
    EXPECT_GE(point.point.z, dropout_output.support_height[point.source_cell] +
                                 config.obstacle_points_min_height);
  }
}

TEST(ProcessorTest,
     RearDropoutBridgePublishesCachedPointAboveCurrentSupportRelativeGate) {
  auto config = MakeConfig();
  config.map.resolution = 0.3f;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_TRUE(HasObstaclePointNearZ(source_output, 0.45f));

  const auto dropout_output = processor.update(
      MakeTranslatedFrontRaisedColumnFrame(-0.8f, 0.2f, 0.8f, 300000001));
  ASSERT_TRUE(dropout_output.valid);
  ASSERT_TRUE(dropout_output.observability.rear_dropout);
  EXPECT_TRUE(HasObstaclePointNearZ(dropout_output, 0.45f));
}

TEST(ProcessorTest, RearDropoutBridgeDoesNotPublishStepRangeCachedPoint) {
  auto config = MakeConfig();
  config.map.resolution = 0.3f;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_TRUE(HasObstaclePointNearZ(source_output, 0.45f));

  const auto dropout_output = processor.update(
      MakeTranslatedFrontRaisedColumnFrame(-0.8f, 0.25f, 0.8f, 300000001));
  ASSERT_TRUE(dropout_output.valid);
  ASSERT_TRUE(dropout_output.observability.rear_dropout);
  EXPECT_FALSE(HasObstaclePointNearZ(dropout_output, 0.45f));
}

TEST(ProcessorTest, RearDropoutBridgeDropsAfterLargeMotion) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_GT(CountRearObstaclePoints(source_output), 0);

  auto dropout_frame = MakeFrontOnlyFrame(300000001);
  dropout_frame.base_pose_in_map.position.x() = 1.0f;
  const auto dropout_output = processor.update(dropout_frame);
  ASSERT_TRUE(dropout_output.valid);
  ASSERT_TRUE(dropout_output.observability.rear_dropout);
  EXPECT_EQ(CountRearObstaclePoints(dropout_output), 0);
}

TEST(ProcessorTest, RearDropoutBridgeExpiresAfterOneFrame) {
  auto config = MakeConfig();
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto source_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(source_output.valid);
  ASSERT_GT(CountRearObstaclePoints(source_output), 0);

  const auto first_dropout = processor.update(MakeFrontOnlyFrame(300000001));
  ASSERT_TRUE(first_dropout.valid);
  EXPECT_TRUE(first_dropout.observability.rear_dropout);
  EXPECT_GT(CountRearObstaclePoints(first_dropout), 0);
  EXPECT_LE(CountRearObstaclePoints(first_dropout),
            CountRearObstaclePoints(source_output));

  const auto second_dropout = processor.update(MakeFrontOnlyFrame(400000001));
  ASSERT_TRUE(second_dropout.valid);
  EXPECT_TRUE(second_dropout.observability.rear_dropout);
  EXPECT_EQ(CountRearObstaclePoints(second_dropout), 0);
}

TEST(ProcessorTest, RearGapTriggersMissingByDropoutSectors) {
  Processor processor(MakeConfig());
  const auto output = processor.update(MakeRearGapFrame());
  ASSERT_TRUE(output.valid);
  EXPECT_FALSE(output.observability.front_dropout);
  EXPECT_TRUE(output.observability.rear_dropout);

  int missing_count = 0;
  for (const auto &sector : output.observability.sectors) {
    if (sector.state == ObservabilityState::kMissingByDropout) {
      ++missing_count;
    }
  }
  EXPECT_GT(missing_count, 0);
}

TEST(ProcessorTest, FrontGapTriggersMissingByDropoutSectors) {
  Processor processor(MakeConfig());
  const auto output = processor.update(MakeFrontGapFrame());
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.observability.front_dropout);
  EXPECT_FALSE(output.observability.rear_dropout);

  int front_missing_count = 0;
  int rear_missing_count = 0;
  const float angle_per_sector =
      2.0f * static_cast<float>(M_PI) /
      static_cast<float>(output.observability.sectors.size());
  for (size_t sector = 0; sector < output.observability.sectors.size();
       ++sector) {
    if (output.observability.sectors[sector].state !=
        ObservabilityState::kMissingByDropout) {
      continue;
    }
    const float angle =
        -static_cast<float>(M_PI) +
        (static_cast<float>(sector) + 0.5f) * angle_per_sector;
    if (std::abs(angle) <= static_cast<float>(M_PI_2)) {
      ++front_missing_count;
    } else {
      ++rear_missing_count;
    }
  }
  EXPECT_GT(front_missing_count, 0);
  EXPECT_EQ(rear_missing_count, 0);
}

TEST(ProcessorTest, RearSupportPointsAreAllowedWhenRearCoverageIsNormal) {
  Processor processor(MakeConfig());
  const auto output = processor.update(MakeFlatFrame());
  ASSERT_TRUE(output.valid);
  EXPECT_FALSE(output.observability.rear_dropout);

  int rear_support_points = 0;
  for (const auto &point : output.support_points) {
    if (point.point.x < -0.4f) {
      ++rear_support_points;
    }
  }
  EXPECT_GT(rear_support_points, 0);
}

TEST(ProcessorTest, NoBlindSectorStatesAreProduced) {
  Processor processor(MakeConfig());

  const auto normal_output = processor.update(MakeFlatFrame(1));
  ASSERT_TRUE(normal_output.valid);
  for (const auto &sector : normal_output.observability.sectors) {
    EXPECT_EQ(sector.state, ObservabilityState::kObserved);
  }

  const auto dropout_output = processor.update(MakeFrontOnlyFrame(100000001));
  ASSERT_TRUE(dropout_output.valid);
  EXPECT_FALSE(dropout_output.observability.front_dropout);
  EXPECT_TRUE(dropout_output.observability.rear_dropout);
  for (const auto &sector : dropout_output.observability.sectors) {
    EXPECT_TRUE(sector.state == ObservabilityState::kObserved ||
                sector.state == ObservabilityState::kMissingByDropout);
  }
}

TEST(ProcessorTest, LocalHoleDoesNotHardClearSupportImmediately) {
  Processor processor(MakeConfig());
  ASSERT_TRUE(processor.update(MakeFlatFrame(1)).valid);
  const auto output = processor.update(MakeLocalHoleFrame(100000001));
  ASSERT_TRUE(output.valid);

  const int center_cell = CellIndex(output, 0.0f, 0.0f);
  EXPECT_GE(output.support_confidence[center_cell], 0.05f);
  EXPECT_NE(output.passability[center_cell],
            static_cast<int8_t>(PassabilityState::kImpassable));
}

TEST(ProcessorTest, SparseCoverageCreatesUnknownButNotDropout) {
  auto config = MakeConfig();
  config.observability.min_points_per_sector = 20;
  Processor processor(config);
  const auto output = processor.update(MakeSparseFrame());
  ASSERT_TRUE(output.valid);
  EXPECT_FALSE(output.observability.front_dropout);
  EXPECT_FALSE(output.observability.rear_dropout);
  EXPECT_TRUE(output.observability.frame_partial);

  int observed_count = 0;
  for (const auto &sector : output.observability.sectors) {
    if (sector.state == ObservabilityState::kObserved) {
      ++observed_count;
    }
    EXPECT_NE(sector.state, ObservabilityState::kMissingByDropout);
  }
  EXPECT_GT(observed_count, 0);
}

TEST(ProcessorTest, PolarFrontendCellSectorIsStableUnderSampleOrderChanges) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(4);
  observability.sectors[0].state = ObservabilityState::kObserved;
  observability.sectors[1].state = ObservabilityState::kObserved;
  observability.sectors[2].state = ObservabilityState::kObserved;
  observability.sectors[3].state = ObservabilityState::kMissingByDropout;
  for (auto &sector : observability.sectors) {
    sector.coverage_confidence = 1.0f;
  }

  ProcessedFrame forward;
  forward.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  forward.base_pose_in_map.position = Eigen::Vector3f::Zero();
  forward.cloud_in_base = {{1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f, 0.0f}};
  forward.cloud_in_map = {{0.25f, 0.25f, 0.0f},
                          {0.25f, 0.25f, 0.2f},
                          {-0.25f, 0.25f, 0.0f},
                          {-0.25f, 0.25f, 0.2f}};
  forward.map_samples.push_back(passable_area::core::MapPointSample{
      {1.0f, 1.0f, 0.0f}, {0.25f, 0.25f, 0.0f}});
  forward.map_samples.push_back(passable_area::core::MapPointSample{
      {-1.0f, 1.0f, 0.0f}, {0.25f, 0.25f, 0.2f}});
  forward.map_samples.push_back(passable_area::core::MapPointSample{
      {1.0f, 0.0f, 0.0f}, {-0.25f, 0.25f, 0.0f}});
  forward.map_samples.push_back(passable_area::core::MapPointSample{
      {1.0f, 0.0f, 0.2f}, {-0.25f, 0.25f, 0.2f}});
  ProcessedFrame reversed = forward;
  std::reverse(reversed.map_samples.begin(), reversed.map_samples.end());

  const auto forward_output =
      frontend.run(forward, observability, MakeMapGeometry(map));
  const auto reversed_output =
      frontend.run(reversed, observability, MakeMapGeometry(map));

  ASSERT_EQ(forward_output.support_candidates.size(), 1U);
  ASSERT_EQ(reversed_output.support_candidates.size(), 1U);
  EXPECT_EQ(forward_output.support_candidates[0].cell,
            reversed_output.support_candidates[0].cell);
  EXPECT_FLOAT_EQ(forward_output.support_candidates[0].confidence,
                  reversed_output.support_candidates[0].confidence);
  ASSERT_EQ(forward_output.protrusion_candidates.size(), 2U);
  ASSERT_EQ(reversed_output.protrusion_candidates.size(), 2U);
  auto sorted_forward_obstacles = forward_output.protrusion_candidates;
  auto sorted_reversed_obstacles = reversed_output.protrusion_candidates;
  std::sort(
      sorted_forward_obstacles.begin(), sorted_forward_obstacles.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.cell < rhs.cell; });
  std::sort(
      sorted_reversed_obstacles.begin(), sorted_reversed_obstacles.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.cell < rhs.cell; });
  for (size_t i = 0; i < sorted_forward_obstacles.size(); ++i) {
    EXPECT_EQ(sorted_forward_obstacles[i].cell,
              sorted_reversed_obstacles[i].cell);
    EXPECT_FLOAT_EQ(sorted_forward_obstacles[i].evidence,
                    sorted_reversed_obstacles[i].evidence);
  }
}

// Historical complex-front-end regressions were removed after rolling obstacle
// formation back to the simpler per-cell baseline.

TEST(
    ProcessorTest,
    PolarFrontendFormsProtrusionCandidateFromVerticalSpanWithoutNeighborGates) {
  auto config = MakeConfig();
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
      {{0.25f, 0.25f, 0.45f}, {0.25f, 0.25f, 0.45f}},
  });

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  int cell = -1;
  ASSERT_TRUE(map.mapToIndex(0.25f, 0.25f, cell));
  ASSERT_EQ(output.protrusion_candidates.size(), 1U);
  EXPECT_EQ(output.protrusion_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.protrusion_candidates.front().z, 0.45f);
  EXPECT_FLOAT_EQ(output.protrusion_candidates.front().evidence, 1.0f);
  EXPECT_FLOAT_EQ(output.protrusion_candidates.front().gain_scale, 1.5f);
  ASSERT_EQ(output.support_candidates.size(), 1U);
  EXPECT_TRUE(output.support_candidates.front().obstacle_overlap);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.obstacle_candidate_cell[static_cast<size_t>(cell)], 1U);
}

TEST(ProcessorTest, PolarFrontendKeepsSingleTallBandAtPureProtrusionGain) {
  auto config = MakeConfig();
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
      {{0.25f, 0.25f, 0.08f}, {0.25f, 0.25f, 0.08f}},
      {{0.25f, 0.25f, 0.16f}, {0.25f, 0.25f, 0.16f}},
      {{0.25f, 0.25f, 0.24f}, {0.25f, 0.25f, 0.24f}},
      {{0.25f, 0.25f, 0.32f}, {0.25f, 0.25f, 0.32f}},
  });

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  int cell = -1;
  ASSERT_TRUE(map.mapToIndex(0.25f, 0.25f, cell));
  ASSERT_EQ(output.protrusion_candidates.size(), 1U);
  EXPECT_EQ(output.protrusion_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.protrusion_candidates.front().gain_scale, 1.5f);
  EXPECT_TRUE(output.overhead_candidates.empty());
}

TEST(ProcessorTest, PolarFrontendFormsOverheadCandidateForLowClearanceBand) {
  auto config = MakeConfig();
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
      {{0.25f, 0.25f, 0.20f}, {0.25f, 0.25f, 0.20f}},
  });

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  int cell = -1;
  ASSERT_TRUE(map.mapToIndex(0.25f, 0.25f, cell));
  ASSERT_EQ(output.overhead_candidates.size(), 1U);
  EXPECT_EQ(output.overhead_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.overhead_candidates.front().z, 0.20f);
  EXPECT_FLOAT_EQ(output.overhead_candidates.front().evidence, 1.0f);
}

TEST(ProcessorTest, PolarFrontendDoesNotCreateProtrusionForSmallLayeredStep) {
  auto config = MakeConfig();
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
      {{0.25f, 0.25f, 0.14f}, {0.25f, 0.25f, 0.14f}},
  });

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  EXPECT_TRUE(output.protrusion_candidates.empty());
  ASSERT_EQ(output.support_candidates.size(), 1U);
  EXPECT_FLOAT_EQ(output.support_candidates.front().z, 0.0f);
  EXPECT_TRUE(output.support_candidates.front().obstacle_overlap);
  EXPECT_EQ(output.support_candidates.front().support_sample_count, 1);
  EXPECT_EQ(output.support_candidates.front().obstacle_sample_count, 1);
}

TEST(ProcessorTest, PolarFrontendFiltersSparseLowerLeakFromSupportBand) {
  auto config = MakeConfig();
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.45f}, {0.25f, 0.25f, -0.45f}},
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
      {{0.25f, 0.25f, 0.01f}, {0.25f, 0.25f, 0.01f}},
      {{0.25f, 0.25f, 0.02f}, {0.25f, 0.25f, 0.02f}},
      {{0.25f, 0.25f, 0.35f}, {0.25f, 0.25f, 0.35f}},
  });

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  ASSERT_EQ(output.support_candidates.size(), 1U);
  EXPECT_FLOAT_EQ(output.support_candidates.front().z, 0.0f);
  EXPECT_TRUE(output.support_candidates.front().obstacle_overlap);
  ASSERT_EQ(output.protrusion_candidates.size(), 1U);
  EXPECT_FLOAT_EQ(output.protrusion_candidates.front().z, 0.35f);
}

TEST(ProcessorTest,
     PolarFrontendIgnoresHistoricalSupportAndUsesFrameMinZForSupportCandidate) {
  auto config = MakeConfig();
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);

  int cell = -1;
  ASSERT_TRUE(map.mapToIndex(0.25f, 0.25f, cell));
  map.layers().support_height[static_cast<size_t>(cell)] = 0.60f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.8f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.05f}, {0.25f, 0.25f, 0.05f}},
      {{0.25f, 0.25f, 0.35f}, {0.25f, 0.25f, 0.35f}},
  });

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  ASSERT_EQ(output.support_candidates.size(), 1U);
  EXPECT_EQ(output.support_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.support_candidates.front().z, 0.05f);
  ASSERT_EQ(output.protrusion_candidates.size(), 1U);
  EXPECT_EQ(output.protrusion_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.protrusion_candidates.front().z, 0.35f);
}

TEST(ProcessorTest,
     PolarFrontendSuppressesSupportCandidateInMissingByDropoutSector) {
  auto config = MakeConfig();
  config.observability.sector_count = 4;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  observability.sectors[2].state = ObservabilityState::kMissingByDropout;
  observability.sectors[2].coverage_confidence = 0.0f;

  ProcessedFrame frame;
  frame.base_pose_in_map.position = Eigen::Vector3f::Zero();
  frame.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
  frame.map_samples.push_back(passable_area::core::MapPointSample{
      {1.0f, 1.0f, 0.0f}, {0.25f, 0.25f, 0.0f}});
  frame.map_samples.push_back(passable_area::core::MapPointSample{
      {-1.0f, 1.0f, 0.4f}, {0.25f, 0.25f, 0.4f}});
  frame.cloud_in_base = {{1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f, 0.4f}};
  frame.cloud_in_map = {{0.25f, 0.25f, 0.0f}, {0.25f, 0.25f, 0.4f}};

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  int cell = -1;
  ASSERT_TRUE(map.mapToIndex(0.25f, 0.25f, cell));
  EXPECT_TRUE(output.support_candidates.empty());
  ASSERT_EQ(output.protrusion_candidates.size(), 1U);
  EXPECT_EQ(output.protrusion_candidates.front().cell, cell);
}

TEST(ProcessorTest,
     PolarFrontendCopiesRawStatsIntoFilteredStatsWithoutFrontendFiltering) {
  auto config = MakeConfig();
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.10f}, {0.25f, 0.25f, -0.10f}},
      {{0.25f, 0.25f, 0.15f}, {0.25f, 0.25f, 0.15f}},
      {{0.25f, 0.25f, 0.35f}, {0.25f, 0.25f, 0.35f}},
  });

  const auto output = frontend.run(frame, observability, MakeMapGeometry(map));

  int cell = -1;
  ASSERT_TRUE(map.mapToIndex(0.25f, 0.25f, cell));
  EXPECT_FLOAT_EQ(output.raw_sample_min_z[static_cast<size_t>(cell)], -0.10f);
  EXPECT_FLOAT_EQ(output.filtered_sample_min_z[static_cast<size_t>(cell)],
                  -0.10f);
  EXPECT_FLOAT_EQ(output.raw_sample_max_z[static_cast<size_t>(cell)], 0.35f);
  EXPECT_FLOAT_EQ(output.filtered_sample_max_z[static_cast<size_t>(cell)],
                  0.35f);
  EXPECT_EQ(output.raw_sample_count[static_cast<size_t>(cell)], 3U);
  EXPECT_EQ(output.filtered_sample_count[static_cast<size_t>(cell)], 3U);
}

TEST(ProcessorTest, LocalTerrainMapRecenterShiftsHistoricalLayers) {
  auto config = MakeConfig();
  config.map.length = 4.0f;
  config.map.width = 4.0f;
  config.map.resolution = 1.0f;
  LocalTerrainMap map(config);

  int original_index = -1;
  ASSERT_TRUE(map.mapToIndex(0.5f, 0.5f, original_index));
  map.layers().support_confidence[original_index] = 0.75f;
  map.layers().support_state[original_index] =
      static_cast<uint8_t>(SupportState::kPersistent);

  map.recenter(Eigen::Vector2f(1.0f, 0.0f));

  int shifted_index = -1;
  ASSERT_TRUE(map.mapToIndex(0.5f, 0.5f, shifted_index));
  EXPECT_NE(original_index, shifted_index);
  EXPECT_FLOAT_EQ(map.layers().support_confidence[shifted_index], 0.75f);
  EXPECT_EQ(map.layers().support_state[shifted_index],
            static_cast<uint8_t>(SupportState::kPersistent));
}

TEST(ProcessorTest, LocalTerrainMapInitializesReservedObstacleV2Layers) {
  LocalTerrainMap map(MakeConfig());
  ASSERT_GT(map.size(), 0);
  ASSERT_EQ(map.layers().protrusion_height.size(),
            static_cast<size_t>(map.size()));
  ASSERT_EQ(map.layers().protrusion_evidence.size(),
            static_cast<size_t>(map.size()));
  ASSERT_EQ(map.layers().overhead_evidence.size(),
            static_cast<size_t>(map.size()));

  for (int cell = 0; cell < map.size(); ++cell) {
    EXPECT_TRUE(
        std::isnan(map.layers().protrusion_height[static_cast<size_t>(cell)]));
    EXPECT_FLOAT_EQ(map.layers().protrusion_evidence[static_cast<size_t>(cell)],
                    0.0f);
    EXPECT_FLOAT_EQ(map.layers().overhead_evidence[static_cast<size_t>(cell)],
                    0.0f);
  }
}

TEST(ProcessorTest, TerrainFeatureUpdaterUsesActualDiagonalDistanceForSlope) {
  auto config = MakeConfig();
  config.map.length = 3.0f;
  config.map.width = 3.0f;
  config.map.resolution = 1.0f;

  LocalTerrainMap map(config);
  auto &layers = map.layers();
  const int center = 1 * map.cols() + 1;
  const int diagonal = 2 * map.cols() + 2;
  layers.support_height[center] = 0.0f;
  layers.support_height[diagonal] = 0.20f;
  layers.support_confidence[center] = 1.0f;

  TerrainFeatureUpdater updater(config);
  updater.update({center}, map);

  const float expected_slope =
      std::atan(0.20f / std::sqrt(2.0f)) * 180.0f /
      static_cast<float>(M_PI);
  EXPECT_NEAR(layers.slope[center], expected_slope, 1e-4f);
  EXPECT_NEAR(layers.step_up[center], 0.20f, 1e-5f);
  EXPECT_NEAR(layers.roughness[center], 0.20f, 1e-5f);
}

TEST(ProcessorTest,
     ObservedSupportStateIsNotOverwrittenAsPersistentInSameUpdate) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  DropoutAwareMapUpdater updater(config);
  FrameObservability observability;
  observability.sectors.resize(4);
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  FrontendOutput frontend_output;
  frontend_output.support_candidates.push_back(SupportCandidate{3, 0.0f, 1.0f});
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();
  const auto dirty =
      updater.update(frontend_output, observability, base_pose, map);

  ASSERT_FALSE(dirty.empty());
  EXPECT_EQ(map.layers().support_state[3],
            static_cast<uint8_t>(SupportState::kObserved));
}

TEST(ProcessorTest,
     ObstacleOverlappedSupportDoesNotRaiseTrustedReachableSupport) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  map.layers().support_height[3] = 0.0f;
  map.layers().support_confidence[3] = 1.0f;
  map.layers().support_state[3] =
      static_cast<uint8_t>(SupportState::kPersistent);

  DropoutAwareMapUpdater updater(config);
  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();

  FrontendOutput frontend_output;
  frontend_output.support_candidates.push_back(
      SupportCandidate{3, 0.06f, 1.0f, true});
  frontend_output.protrusion_candidates.push_back(
      ProtrusionCandidate{3, 0.37f, 1.0f, 1.5f});

  const auto dirty =
      updater.update(frontend_output, observability, base_pose, map);

  ASSERT_FALSE(dirty.empty());
  EXPECT_FLOAT_EQ(map.layers().support_height[3], 0.0f);
  EXPECT_GE(map.layers().support_confidence[3],
            config.observability.min_support_confidence);
  EXPECT_GT(map.layers().protrusion_evidence[3], 0.0f);
}

TEST(ProcessorTest, RejectedObstacleOverlapRefreshesRetainedTrustedSupport) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;
  config.persistence.support_confidence_decay = 0.4f;
  config.persistence.support_persistence_frames = 2;

  LocalTerrainMap map(config);
  map.layers().support_height[3] = 0.0f;
  map.layers().support_confidence[3] = 0.3f;
  map.layers().support_state[3] =
      static_cast<uint8_t>(SupportState::kPersistent);

  DropoutAwareMapUpdater updater(config);
  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();

  FrontendOutput frontend_output;
  frontend_output.support_candidates.push_back(
      SupportCandidate{3, 0.06f, 1.0f, true});
  frontend_output.protrusion_candidates.push_back(
      ProtrusionCandidate{3, 0.37f, 1.0f, 1.5f});

  for (int i = 0; i < 8; ++i) {
    updater.update(frontend_output, observability, base_pose, map);
  }

  EXPECT_FLOAT_EQ(map.layers().support_height[3], 0.0f);
  EXPECT_GE(map.layers().support_confidence[3],
            config.observability.min_support_confidence);
  EXPECT_EQ(map.layers().support_state[3],
            static_cast<uint8_t>(SupportState::kPersistent));
  EXPECT_EQ(map.layers().last_reliable_age[3], 0U);
  EXPECT_GT(map.layers().protrusion_evidence[3], 0.0f);
}

TEST(ProcessorTest, NonOverlappedSupportCanRaiseReachableSupport) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  map.layers().support_height[3] = 0.0f;
  map.layers().support_confidence[3] = 1.0f;
  map.layers().support_state[3] =
      static_cast<uint8_t>(SupportState::kPersistent);

  DropoutAwareMapUpdater updater(config);
  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();

  FrontendOutput frontend_output;
  frontend_output.support_candidates.push_back(
      SupportCandidate{3, 0.08f, 1.0f, false});

  updater.update(frontend_output, observability, base_pose, map);

  EXPECT_FLOAT_EQ(map.layers().support_height[3], 0.08f);
  EXPECT_EQ(map.layers().support_state[3],
            static_cast<uint8_t>(SupportState::kObserved));
}

TEST(ProcessorTest,
     ObstacleOverlappedSupportCanInitializeWhenNoTrustedSupportExists) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  map.layers().support_height[3] = 0.0f;
  map.layers().support_confidence[3] =
      config.observability.min_support_confidence * 0.5f;

  DropoutAwareMapUpdater updater(config);
  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();

  FrontendOutput frontend_output;
  frontend_output.support_candidates.push_back(
      SupportCandidate{3, 0.06f, 1.0f, true});

  updater.update(frontend_output, observability, base_pose, map);

  EXPECT_FLOAT_EQ(map.layers().support_height[3], 0.06f);
  EXPECT_GE(map.layers().support_confidence[3],
            config.observability.min_support_confidence);
}

TEST(ProcessorTest,
     ObstacleOverlappedSupportCanInitializeWhenSupportIsUnknown) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  map.layers().support_height[3] = std::numeric_limits<float>::quiet_NaN();
  map.layers().support_confidence[3] = 0.0f;

  DropoutAwareMapUpdater updater(config);
  const auto observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();

  FrontendOutput frontend_output;
  frontend_output.support_candidates.push_back(
      SupportCandidate{3, 0.06f, 1.0f, true});

  updater.update(frontend_output, observability, base_pose, map);

  EXPECT_FLOAT_EQ(map.layers().support_height[3], 0.06f);
  EXPECT_EQ(map.layers().support_state[3],
            static_cast<uint8_t>(SupportState::kObserved));
}

TEST(ProcessorTest, DropoutMapUpdaterAppliesBaseYawToSectorStates) {
  auto config = MakeConfig();
  config.map.length = 3.0f;
  config.map.width = 3.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  DropoutAwareMapUpdater updater(config);

  FrameObservability observability =
      MakeUniformObservability(config, ObservabilityState::kObserved);
  observability.sectors[2].state = ObservabilityState::kMissingByDropout;
  observability.sectors[2].coverage_confidence = 0.0f;

  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation =
      Eigen::AngleAxisf(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitZ());

  FrontendOutput frontend_output;
  updater.update(frontend_output, observability, base_pose, map);

  int robot_front_in_map_cell = -1;
  int map_x_cell = -1;
  ASSERT_TRUE(map.mapToIndex(0.0f, 1.0f, robot_front_in_map_cell));
  ASSERT_TRUE(map.mapToIndex(1.0f, 0.0f, map_x_cell));

  EXPECT_FLOAT_EQ(map.layers().coverage_confidence[robot_front_in_map_cell],
                  0.0f);
  EXPECT_NEAR(map.layers().coverage_confidence[map_x_cell], 0.85f, 1e-5f);
}

TEST(ProcessorTest, ProtrusionCandidateGainScaleBoostsEvidenceAccumulation) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  DropoutAwareMapUpdater updater(config);
  FrameObservability observability;
  observability.sectors.resize(4);
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  FrontendOutput frontend_output;
  frontend_output.protrusion_candidates.push_back(
      ProtrusionCandidate{3, 0.5f, 0.9f, 1.8f});
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();
  const auto dirty =
      updater.update(frontend_output, observability, base_pose, map);

  ASSERT_FALSE(dirty.empty());
  EXPECT_NEAR(map.layers().protrusion_evidence[3],
              config.persistence.obstacle_evidence_gain * 1.8f * 0.9f, 1e-5f);
  EXPECT_NEAR(map.layers().obstacle_evidence[3],
              config.persistence.obstacle_evidence_gain * 1.8f * 0.9f, 1e-5f);
  EXPECT_FLOAT_EQ(map.layers().overhead_evidence[3], 0.0f);
}

TEST(ProcessorTest, OverheadCandidateMaintainsIndependentEvidenceChain) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.observability.sector_count = 4;

  LocalTerrainMap map(config);
  DropoutAwareMapUpdater updater(config);
  FrameObservability observability;
  observability.sectors.resize(4);
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  FrontendOutput frontend_output;
  frontend_output.support_candidates.push_back(SupportCandidate{3, 0.0f, 1.0f});
  frontend_output.overhead_candidates.push_back(
      OverheadCandidate{3, 0.25f, 0.7f, 1.4f});
  passable_area::core::Pose3D base_pose;
  base_pose.position = Eigen::Vector3f::Zero();
  base_pose.orientation = Eigen::Quaternionf::Identity();
  const auto dirty =
      updater.update(frontend_output, observability, base_pose, map);

  ASSERT_FALSE(dirty.empty());
  EXPECT_FLOAT_EQ(map.layers().protrusion_evidence[3], 0.0f);
  EXPECT_NEAR(map.layers().overhead_evidence[3],
              config.persistence.obstacle_evidence_gain * 1.4f * 0.7f, 1e-5f);
  EXPECT_NEAR(map.layers().obstacle_evidence[3],
              map.layers().overhead_evidence[3], 1e-5f);
  EXPECT_NEAR(map.layers().overhead_confidence[3],
              config.persistence.obstacle_evidence_gain * 1.4f, 1e-5f);
}

TEST(ProcessorTest, DebugSupportPointsRespectRobotCentricGravityFrame) {
  auto config = MakeConfig();
  config.map.length = 4.0f;
  config.map.width = 4.0f;
  config.map.resolution = 0.1f;
  config.preprocess.enable_downsample = false;
  Processor processor(config);

  const Eigen::Quaternionf orientation =
      Eigen::AngleAxisf(0.18f, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(-0.12f, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(static_cast<float>(M_PI_2), Eigen::Vector3f::UnitZ());
  const auto output = processor.update(MakeForwardStripFrame(orientation));
  ASSERT_TRUE(output.valid);
  ASSERT_FALSE(output.support_points.empty());

  float mean_x = 0.0f;
  float mean_y = 0.0f;
  for (const auto &point : output.support_points) {
    mean_x += point.point.x;
    mean_y += point.point.y;
  }
  mean_x /= static_cast<float>(output.support_points.size());
  mean_y /= static_cast<float>(output.support_points.size());

  EXPECT_GT(mean_x, 0.2f);
  EXPECT_LT(std::abs(mean_y), 0.2f);
}

TEST(ProcessorTest, ObstaclePointsPublishUpperBandSamplesFromObstacleCells) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeObstacleColumnFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeObstacleColumnFrame(100000001)).valid);
  ASSERT_TRUE(processor.update(MakeObstacleColumnFrame(200000001)).valid);
  const auto output = processor.update(MakeObstacleColumnFrame(300000001));
  ASSERT_TRUE(output.valid);

  ASSERT_EQ(output.obstacle_points.size(), 2U);
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  for (const auto &point : output.obstacle_points) {
    min_z = std::min(min_z, point.point.z);
    max_z = std::max(max_z, point.point.z);
  }
  EXPECT_NEAR(min_z, 0.30f, 1e-5f);
  EXPECT_NEAR(max_z, 0.30f, 1e-5f);
}

TEST(ProcessorTest, ObstaclePointPublishStatusMatchesActualNativePoints) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeObstacleColumnFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeObstacleColumnFrame(100000001)).valid);
  ASSERT_TRUE(processor.update(MakeObstacleColumnFrame(200000001)).valid);
  const auto output = processor.update(MakeObstacleColumnFrame(300000001));
  ASSERT_TRUE(output.valid);
  ASSERT_FALSE(output.obstacle_points.empty());

  std::vector<uint8_t> cells_with_native_obstacle_points(
      static_cast<size_t>(output.rows * output.cols), 0U);
  for (const auto &point : output.obstacle_points) {
    ASSERT_GE(point.source_cell, 0);
    ASSERT_LT(point.source_cell, output.rows * output.cols);
    cells_with_native_obstacle_points[static_cast<size_t>(point.source_cell)] =
        1U;
    EXPECT_NE(output.block_reason[point.source_cell],
              static_cast<uint8_t>(BlockReason::kNone));
    EXPECT_TRUE(IsPublishedObstaclePointStatus(
        output.obstacle_point_publish_status[point.source_cell]));
    EXPECT_EQ(output.passability[point.source_cell],
              static_cast<int8_t>(PassabilityState::kImpassable));
    ASSERT_TRUE(std::isfinite(output.support_height[point.source_cell]));
    EXPECT_GT(point.point.z, output.support_height[point.source_cell] +
                                 config.geometry.max_step_up);
    EXPECT_GE(point.point.z, output.support_height[point.source_cell] +
                                 config.obstacle_points_min_height);
  }

  for (int cell = 0; cell < output.rows * output.cols; ++cell) {
    if (IsPublishedObstaclePointStatus(
            output.obstacle_point_publish_status[cell])) {
      EXPECT_NE(output.block_reason[cell],
                static_cast<uint8_t>(BlockReason::kNone));
      EXPECT_NE(cells_with_native_obstacle_points[static_cast<size_t>(cell)],
                0U);
    } else {
      EXPECT_EQ(cells_with_native_obstacle_points[static_cast<size_t>(cell)],
                0U);
    }
  }
}

TEST(ProcessorTest, DenseCurrentFrameProtrusionSourcePublishesObstaclePoints) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 12;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  const auto output = processor.update(MakeSingleCellColumnFrame(8));
  ASSERT_TRUE(output.valid);

  const int col = static_cast<int>(
      std::floor((0.25f - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(
      std::floor((0.25f - output.origin.y()) / output.resolution));
  ASSERT_GE(row, 0);
  ASSERT_LT(row, output.rows);
  ASSERT_GE(col, 0);
  ASSERT_LT(col, output.cols);
  const int cell = row * output.cols + col;
  ASSERT_LT(output.protrusion_evidence[cell],
            config.obstacle_points_min_evidence);
  ASSERT_GE(output.raw_sample_count[cell],
            config.observability.min_points_per_sector - 1);
  EXPECT_EQ(output.block_reason[cell],
            static_cast<uint8_t>(BlockReason::kProtrusion));
  EXPECT_EQ(
      output.obstacle_point_publish_status[cell],
      static_cast<uint8_t>(passable_area::core::ObstaclePointPublishStatus::
                               kPublishedByDenseProtrusion));
  ASSERT_FALSE(output.obstacle_points.empty());
}

TEST(ProcessorTest, DenseNearThresholdStepRangeDoesNotPublishObstaclePoints) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 12;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  const auto output = processor.update(
      MakeSingleCellColumnFrame(8, 1, config.geometry.max_step_up));
  ASSERT_TRUE(output.valid);

  const int col = static_cast<int>(
      std::floor((0.25f - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(
      std::floor((0.25f - output.origin.y()) / output.resolution));
  ASSERT_GE(row, 0);
  ASSERT_LT(row, output.rows);
  ASSERT_GE(col, 0);
  ASSERT_LT(col, output.cols);
  const int cell = row * output.cols + col;
  ASSERT_LT(output.protrusion_evidence[cell],
            config.obstacle_points_min_evidence);
  ASSERT_GE(output.raw_sample_count[cell],
            config.observability.min_points_per_sector - 1);
  EXPECT_EQ(output.block_reason[cell],
            static_cast<uint8_t>(BlockReason::kLowClearance));
  EXPECT_EQ(
      output.obstacle_point_publish_status[cell],
      static_cast<uint8_t>(
          passable_area::core::ObstaclePointPublishStatus::kGatedByHeight));
  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest,
     StepUpBoundaryWithHighEvidenceDoesNotPublishObstaclePoints) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  passable_area::core::FrameOutput output;
  for (int i = 0; i < 4; ++i) {
    output = processor.update(MakeSingleCellColumnFrame(
        12, 100000000 * i + 1, config.geometry.max_step_up));
    ASSERT_TRUE(output.valid);
  }

  const int cell = CellIndex(output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  ASSERT_GE(output.protrusion_evidence[cell],
            config.obstacle_points_min_evidence);
  EXPECT_FALSE(IsPublishedObstaclePointStatus(
      output.obstacle_point_publish_status[cell]));
  EXPECT_EQ(
      output.obstacle_point_publish_status[cell],
      static_cast<uint8_t>(
          passable_area::core::ObstaclePointPublishStatus::kGatedByHeight));
  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest,
     ObstacleOverlappedSupportDoesNotHideTallCurrentObstaclePoint) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.geometry.max_step_up = 0.32f;
  config.geometry.min_clearance = 0.5f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 4;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(
      processor.update(MakeSingleCellZCountsFrame({{0.0f, 6}}, 1)).valid);
  const auto output = processor.update(
      MakeSingleCellZCountsFrame({{0.06f, 3}, {0.37f, 4}}, 100000001));
  ASSERT_TRUE(output.valid);

  const int cell = CellIndex(output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_NEAR(output.support_height[cell], 0.0f, 1e-5f);
  EXPECT_TRUE(IsPublishedObstaclePointStatus(
      output.obstacle_point_publish_status[cell]));
  EXPECT_TRUE(HasObstaclePointNearZ(output, 0.37f));
}

TEST(ProcessorTest,
     PersistentObstacleOverlapDoesNotDecaySupportAndHideObstacle) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.geometry.max_step_up = 0.32f;
  config.geometry.min_clearance = 0.5f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 4;
  config.persistence.support_confidence_decay = 0.4f;
  config.persistence.support_persistence_frames = 2;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(
      processor.update(MakeSingleCellZCountsFrame({{0.0f, 6}}, 1)).valid);
  passable_area::core::FrameOutput output;
  for (int i = 0; i < 8; ++i) {
    output = processor.update(MakeSingleCellZCountsFrame(
        {{0.06f, 3}, {0.37f, 4}}, 100000001 + i * 100000000));
    ASSERT_TRUE(output.valid);
  }

  const int cell = CellIndex(output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_NEAR(output.support_height[cell], 0.0f, 1e-5f);
  EXPECT_GE(output.support_confidence[cell],
            config.observability.min_support_confidence);
  EXPECT_TRUE(IsPublishedObstaclePointStatus(
      output.obstacle_point_publish_status[cell]));
  EXPECT_TRUE(HasObstaclePointNearZ(output, 0.37f));
}

TEST(ProcessorTest,
     ObstacleOverlappedSupportCanInitializeWithoutPublishingStepRangePoint) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.geometry.max_step_up = 0.32f;
  config.geometry.min_clearance = 0.5f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 4;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  const auto output =
      processor.update(MakeSingleCellZCountsFrame({{0.06f, 3}, {0.37f, 4}}));
  ASSERT_TRUE(output.valid);

  const int cell = CellIndex(output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_NEAR(output.support_height[cell], 0.06f, 1e-5f);
  EXPECT_FALSE(IsPublishedObstaclePointStatus(
      output.obstacle_point_publish_status[cell]));
  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest, NonOverlappedSmallStepRaisesCurrentSupportWithoutPublish) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.geometry.max_step_up = 0.32f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 4;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(
      processor.update(MakeSingleCellZCountsFrame({{0.0f, 6}}, 1)).valid);
  const auto output =
      processor.update(MakeSingleCellZCountsFrame({{0.08f, 6}}, 100000001));
  ASSERT_TRUE(output.valid);

  const int cell = CellIndex(output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_NEAR(output.support_height[cell], 0.08f, 1e-5f);
  EXPECT_FALSE(IsPublishedObstaclePointStatus(
      output.obstacle_point_publish_status[cell]));
  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest, SparseNearThresholdProtrusionSourceDoesNotPublish) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 12;
  config.obstacle_points_min_evidence = 0.4f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  const auto output = processor.update(MakeSingleCellColumnFrame(5));
  ASSERT_TRUE(output.valid);

  const int col = static_cast<int>(
      std::floor((0.25f - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(
      std::floor((0.25f - output.origin.y()) / output.resolution));
  ASSERT_GE(row, 0);
  ASSERT_LT(row, output.rows);
  ASSERT_GE(col, 0);
  ASSERT_LT(col, output.cols);
  const int cell = row * output.cols + col;
  ASSERT_LT(output.protrusion_evidence[cell],
            config.obstacle_points_min_evidence);
  ASSERT_LT(output.raw_sample_count[cell],
            config.observability.min_points_per_sector - 1);
  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest, ObstaclePointsExcludeGroundSamplesUnderLowCeiling) {
  auto config = MakeConfig();
  config.preprocess.enable_downsample = false;
  config.geometry.min_clearance = 0.5f;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.15f;
  config.obstacle_points_max_height_in_base_link = 0.35f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.28f, 1)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.28f, 100000001)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.28f, 200000001)).valid);
  const auto output = processor.update(MakeLowCeilingFrame(0.28f, 300000001));
  ASSERT_TRUE(output.valid);
  ASSERT_FALSE(output.obstacle_points.empty());

  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  for (const auto &point : output.obstacle_points) {
    min_z = std::min(min_z, point.point.z);
    max_z = std::max(max_z, point.point.z);
  }
  EXPECT_NEAR(min_z, 0.28f, 1e-5f);
  EXPECT_NEAR(max_z, 0.28f, 1e-5f);
}

TEST(ProcessorTest, LowClearanceReasonerBridgePublishesOverheadObstaclePoints) {
  auto config = MakeConfig();
  config.preprocess.enable_downsample = false;
  config.geometry.min_clearance = 0.5f;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.08f;
  config.obstacle_points_max_height_in_base_link = 0.35f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.28f, 1)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.28f, 100000001)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.28f, 200000001)).valid);
  const auto output = processor.update(MakeLowCeilingFrame(0.28f, 300000001));
  ASSERT_TRUE(output.valid);

  bool found_overhead_reasoner_cell = false;
  for (int cell = 0; cell < output.rows * output.cols; ++cell) {
    const bool low_clearance_blocked =
        output.block_reason[cell] ==
            static_cast<uint8_t>(BlockReason::kLowClearance) ||
        output.block_reason[cell] == static_cast<uint8_t>(BlockReason::kMixed);
    if (low_clearance_blocked &&
        output.overhead_stage[cell] ==
            static_cast<uint8_t>(ObstacleEvidenceStage::kBlocking) &&
        output.overhead_evidence[cell] >= config.obstacle_points_min_evidence) {
      found_overhead_reasoner_cell = true;
      break;
    }
  }
  ASSERT_TRUE(found_overhead_reasoner_cell);
  ASSERT_FALSE(output.obstacle_points.empty());

  for (const auto &point : output.obstacle_points) {
    ASSERT_GE(point.source_cell, 0);
    ASSERT_LT(point.source_cell, output.rows * output.cols);
    EXPECT_EQ(output.overhead_stage[point.source_cell],
              static_cast<uint8_t>(ObstacleEvidenceStage::kBlocking));
    EXPECT_NEAR(point.point.z, 0.28f, 1e-5f);
  }
}

TEST(ProcessorTest, LowClearanceBridgeSuppressesStepRangeProjection) {
  auto config = MakeConfig();
  config.preprocess.enable_downsample = false;
  config.geometry.min_clearance = 0.5f;
  config.obstacle_points_min_evidence = 0.15f;
  config.obstacle_points_min_height = 0.08f;
  config.obstacle_points_max_height_in_base_link = 0.25f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.18f, 1)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.18f, 100000001)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.18f, 200000001)).valid);
  const auto output = processor.update(MakeLowCeilingFrame(0.18f, 300000001));
  ASSERT_TRUE(output.valid);

  // Verify that step-range low-clearance cells exist but do NOT have
  // obstacle_point_publish_status set to kPublishedByOverhead — the bridge
  // should be suppressed because clearance <= max_step_up.
  bool found_step_range_low_clearance_cell = false;
  bool found_step_range_low_clearance_cell_with_protrusion_evidence = false;
  for (int cell = 0; cell < output.rows * output.cols; ++cell) {
    if (output.block_reason[cell] ==
            static_cast<uint8_t>(BlockReason::kLowClearance) &&
        output.overhead_evidence[cell] >= config.obstacle_points_min_evidence &&
        std::isfinite(output.clearance[cell]) &&
        output.clearance[cell] <= config.geometry.max_step_up) {
      found_step_range_low_clearance_cell = true;
      if (output.protrusion_evidence[cell] >=
          config.obstacle_points_min_evidence) {
        found_step_range_low_clearance_cell_with_protrusion_evidence = true;
      }
      // Bridge should NOT mark this cell as published-by-overhead.
      EXPECT_NE(
          output.obstacle_point_publish_status[cell],
          static_cast<uint8_t>(passable_area::core::ObstaclePointPublishStatus::
                                   kPublishedByOverhead));
    }
  }
  ASSERT_TRUE(found_step_range_low_clearance_cell);
  ASSERT_TRUE(found_step_range_low_clearance_cell_with_protrusion_evidence);
  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest, ObstaclePointsExcludeSamplesAboveBaseLinkHeightCeiling) {
  auto config = MakeConfig();
  config.map.length = 3.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 0.2f;
  Processor processor(config);

  const Eigen::Quaternionf pitched_orientation(Eigen::AngleAxisf(
      -static_cast<float>(M_PI) / 6.0f, Eigen::Vector3f::UnitY()));
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(pitched_orientation,
                                                        0.25f, 0.35f, 1))
                  .valid);
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(
                      pitched_orientation, 0.25f, 0.35f, 100000001))
                  .valid);
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(
                      pitched_orientation, 0.25f, 0.35f, 200000001))
                  .valid);
  const auto output = processor.update(MakePitchedObstacleStackFrame(
      pitched_orientation, 0.25f, 0.35f, 300000001));
  ASSERT_TRUE(output.valid);

  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest, ObstaclePointsKeepSamplesAtOrBelowBaseLinkHeightCeiling) {
  auto config = MakeConfig();
  config.map.length = 3.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 0.30f;
  Processor processor(config);

  const Eigen::Quaternionf pitched_orientation(Eigen::AngleAxisf(
      -static_cast<float>(M_PI) / 6.0f, Eigen::Vector3f::UnitY()));
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(pitched_orientation,
                                                        0.29f, 0.35f, 1))
                  .valid);
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(
                      pitched_orientation, 0.29f, 0.35f, 100000001))
                  .valid);
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(
                      pitched_orientation, 0.29f, 0.35f, 200000001))
                  .valid);
  const auto output = processor.update(MakePitchedObstacleStackFrame(
      pitched_orientation, 0.29f, 0.35f, 300000001));
  ASSERT_TRUE(output.valid);

  ASSERT_FALSE(output.obstacle_points.empty());
  for (const auto &point : output.obstacle_points) {
    EXPECT_GT(point.point.z, config.geometry.max_step_up);
  }
}

TEST(ProcessorTest,
     ObstaclePointPublishUsesBaseLinkCeilingButPublishesInBaseGravity) {
  auto config = MakeConfig();
  config.map.length = 3.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.1f;
  config.obstacle_points_max_height_in_base_link = 0.30f;
  Processor processor(config);
  FramePreprocessor preprocessor(config);

  const Eigen::Quaternionf pitched_orientation(Eigen::AngleAxisf(
      -static_cast<float>(M_PI) / 6.0f, Eigen::Vector3f::UnitY()));
  const FrameInput input =
      MakePitchedObstacleStackFrame(pitched_orientation, 0.29f, 0.35f, 1);
  ProcessedFrame preprocessed;
  ASSERT_TRUE(preprocessor.process(input, preprocessed));

  bool found_publishable_source_sample = false;
  for (const auto &sample : preprocessed.map_samples) {
    if (std::abs(sample.point_in_base.x - 0.55f) < 1e-5f &&
        std::abs(sample.point_in_base.y - 0.25f) < 1e-5f &&
        std::abs(sample.point_in_base.z - 0.29f) < 1e-5f) {
      found_publishable_source_sample = true;
      EXPECT_LE(sample.point_in_base.z,
                config.obstacle_points_max_height_in_base_link);
    }
  }
  EXPECT_TRUE(found_publishable_source_sample);

  ASSERT_TRUE(processor.update(input).valid);
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(
                      pitched_orientation, 0.29f, 0.35f, 100000001))
                  .valid);
  ASSERT_TRUE(processor
                  .update(MakePitchedObstacleStackFrame(
                      pitched_orientation, 0.29f, 0.35f, 200000001))
                  .valid);
  const auto output = processor.update(MakePitchedObstacleStackFrame(
      pitched_orientation, 0.29f, 0.35f, 300000001));
  ASSERT_TRUE(output.valid);
  ASSERT_FALSE(output.obstacle_points.empty());

  bool found_published_point_above_base_link_ceiling = false;
  for (const auto &point : output.obstacle_points) {
    if (point.point.z > config.obstacle_points_max_height_in_base_link) {
      found_published_point_above_base_link_ceiling = true;
    }
  }
  EXPECT_TRUE(found_published_point_above_base_link_ceiling);
}

TEST(ProcessorTest, ObstaclePointsExcludeWallBaseNoise) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.2f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeWallWithBaseNoiseFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeWallWithBaseNoiseFrame(100000001)).valid);
  ASSERT_TRUE(processor.update(MakeWallWithBaseNoiseFrame(200000001)).valid);
  const auto output = processor.update(MakeWallWithBaseNoiseFrame(300000001));
  ASSERT_TRUE(output.valid);

  ASSERT_EQ(output.obstacle_points.size(), 4U);
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  for (const auto &point : output.obstacle_points) {
    min_z = std::min(min_z, point.point.z);
    max_z = std::max(max_z, point.point.z);
  }
  EXPECT_NEAR(min_z, 0.25f, 1e-5f);
  EXPECT_NEAR(max_z, 0.45f, 1e-5f);
}

TEST(ProcessorTest, WallWithBaseNoiseStillProducesImpassableCells) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.8f;
  Processor processor(config);

  passable_area::core::FrameOutput output;
  for (int i = 0; i < 4; ++i) {
    output = processor.update(MakeWallWithBaseNoiseFrame(100000000 * i + 1));
    ASSERT_TRUE(output.valid);
  }

  const int right_wall_cell = CellIndex(output, 0.25f, 0.25f);
  const int left_wall_cell = CellIndex(output, -0.25f, 0.25f);
  ASSERT_GE(right_wall_cell, 0);
  ASSERT_GE(left_wall_cell, 0);
  EXPECT_TRUE(std::isfinite(output.overhead_height[right_wall_cell]));
  EXPECT_TRUE(std::isfinite(output.overhead_height[left_wall_cell]));
  EXPECT_EQ(output.passability[right_wall_cell],
            static_cast<int8_t>(PassabilityState::kImpassable));
  EXPECT_EQ(output.passability[left_wall_cell],
            static_cast<int8_t>(PassabilityState::kImpassable));
}

TEST(ProcessorTest, DebugObstaclePointsIgnoreWeakObstacleEvidenceByDefault) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.8f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(100000001)).valid);
  const auto output = processor.update(MakeDynamicObstacleCellFrame(200000001));
  ASSERT_TRUE(output.valid);

  const int cell = CellIndex(output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_GT(output.obstacle_evidence[cell], 0.25f);
  EXPECT_LT(output.obstacle_evidence[cell],
            config.obstacle_points_min_evidence);
  EXPECT_TRUE(output.obstacle_points.empty());
}

TEST(ProcessorTest, WallWithoutGroundSupportStillPublishesObstaclePoints) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.15f;
  config.obstacle_points_max_height_in_base_link = 1.0f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeUnsupportedWallFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeUnsupportedWallFrame(100000001)).valid);
  ASSERT_TRUE(processor.update(MakeUnsupportedWallFrame(200000001)).valid);
  const auto output = processor.update(MakeUnsupportedWallFrame(300000001));
  ASSERT_TRUE(output.valid);

  ASSERT_FALSE(output.obstacle_points.empty());
  for (const auto &point : output.obstacle_points) {
    EXPECT_NEAR(std::abs(point.point.x), 0.25f, 0.15f);
    EXPECT_NEAR(point.point.y, 0.25f, 0.15f);
    EXPECT_GT(point.point.z, 0.35f);
  }
}

TEST(ProcessorTest, DynamicObstacleClearsAfterObservedGroundReturns) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  config.obstacle_points_min_evidence = 0.8f;
  config.persistence.obstacle_clear_observed_decay = 0.20f;
  config.persistence.obstacle_clear_partial_decay_scale = 0.35f;
  config.persistence.obstacle_height_clear_threshold = 0.25f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(100000001)).valid);
  const auto obstacle_output =
      processor.update(MakeDynamicObstacleCellFrame(200000001));
  ASSERT_TRUE(obstacle_output.valid);

  const int cell = CellIndex(obstacle_output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_GT(obstacle_output.obstacle_evidence[cell], 0.25f);
  ASSERT_TRUE(std::isfinite(obstacle_output.overhead_height[cell]));

  ASSERT_TRUE(processor.update(MakeGroundOnlyCellFrame(300000001)).valid);
  ASSERT_TRUE(processor.update(MakeGroundOnlyCellFrame(400000001)).valid);
  const auto cleared_output =
      processor.update(MakeGroundOnlyCellFrame(500000001));
  ASSERT_TRUE(cleared_output.valid);
  EXPECT_LE(cleared_output.obstacle_evidence[cell],
            config.persistence.obstacle_height_clear_threshold);
  EXPECT_FALSE(std::isfinite(cleared_output.overhead_height[cell]));
  EXPECT_TRUE(std::isinf(cleared_output.clearance[cell]));
  EXPECT_NE(cleared_output.passability[cell],
            static_cast<int8_t>(PassabilityState::kImpassable));
}

TEST(ProcessorTest, ObstacleNotClearedAggressivelyDuringDropout) {
  auto config = MakeConfig();
  config.map.length = 4.0f;
  config.map.width = 4.0f;
  config.map.resolution = 0.2f;
  config.preprocess.enable_downsample = false;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeRearObstacleCellFrame(100000001)).valid);
  const auto obstacle_output =
      processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(obstacle_output.valid);
  const int cell = CellIndex(obstacle_output, -0.8f, 0.8f);
  ASSERT_GE(cell, 0);
  ASSERT_TRUE(std::isfinite(obstacle_output.overhead_height[cell]));
  const float evidence_before = obstacle_output.obstacle_evidence[cell];

  const auto dropout_output = processor.update(MakeFrontOnlyFrame(300000001));
  ASSERT_TRUE(dropout_output.valid);
  EXPECT_TRUE(dropout_output.observability.rear_dropout);
  EXPECT_GT(dropout_output.obstacle_evidence[cell],
            config.persistence.obstacle_height_clear_threshold);
  EXPECT_GT(dropout_output.obstacle_evidence[cell],
            evidence_before - config.persistence.obstacle_clear_observed_decay);
  EXPECT_TRUE(std::isfinite(dropout_output.overhead_height[cell]));
}

TEST(ProcessorTest, StaticLowCeilingStillRemainsImpassable) {
  auto config = MakeConfig();
  config.map.length = 4.0f;
  config.map.width = 4.0f;
  config.map.resolution = 0.2f;
  config.preprocess.enable_downsample = false;
  config.persistence.obstacle_clear_observed_decay = 0.20f;
  config.persistence.obstacle_clear_partial_decay_scale = 0.35f;
  config.persistence.obstacle_height_clear_threshold = 0.25f;
  Processor processor(config);

  passable_area::core::FrameOutput output;
  for (int i = 0; i < 5; ++i) {
    output = processor.update(MakeLowCeilingFrame(0.2f, 100000000 * i + 1));
    ASSERT_TRUE(output.valid);
  }

  bool found_static_low_ceiling_cell = false;
  for (int cell = 0; cell < output.rows * output.cols; ++cell) {
    if (std::isfinite(output.overhead_height[cell]) &&
        output.clearance[cell] < config.geometry.min_clearance &&
        output.passability[cell] ==
            static_cast<int8_t>(PassabilityState::kImpassable)) {
      found_static_low_ceiling_cell = true;
      break;
    }
  }
  EXPECT_TRUE(found_static_low_ceiling_cell);
}
