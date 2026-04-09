#include "passable_area/core/frontend/polar_frontend.hpp"
#include "passable_area/core/mapping/dropout_aware_map_updater.hpp"
#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/pipeline/processor.hpp"
#include "passable_area/core/preprocess/frame_preprocessor.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

using passable_area::core::FrameInput;
using passable_area::core::FrameObservability;
using passable_area::core::FrontendOutput;
using passable_area::core::LocalTerrainMap;
using passable_area::core::ObservabilityState;
using passable_area::core::PassabilityState;
using passable_area::core::PolarFrontend;
using passable_area::core::ProcessedFrame;
using passable_area::core::Processor;
using passable_area::core::DropoutAwareMapUpdater;
using passable_area::core::FramePreprocessor;
using passable_area::core::SupportCandidate;
using passable_area::core::SupportState;

FrameInput MakeFlatFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
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
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
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
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
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
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  for (float x = 0.0f; x <= 1.2f; x += 0.1f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.1f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeRearGapFrame(int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  input.input_cloud_in_base.erase(
      std::remove_if(input.input_cloud_in_base.begin(), input.input_cloud_in_base.end(),
                     [](const auto &point) { return point.x < -0.1f && std::abs(point.y) < 1.0f; }),
      input.input_cloud_in_base.end());
  for (float x = 0.2f; x <= 1.2f; x += 0.05f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.05f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeLocalHoleFrame(int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  input.input_cloud_in_base.erase(
      std::remove_if(input.input_cloud_in_base.begin(), input.input_cloud_in_base.end(),
                     [](const auto &point) {
        return std::abs(point.x) < 0.35f && std::abs(point.y) < 0.35f;
      }),
      input.input_cloud_in_base.end());
  return input;
}

FrameInput MakeSparseFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
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

FrameInput MakeForwardStripFrame(const Eigen::Quaternionf &orientation, int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = orientation.normalized();
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
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},
      {0.25f, 0.25f, 0.12f},
      {0.25f, 0.25f, 0.30f},
      {-0.25f, 0.25f, 0.0f},
      {-0.25f, 0.25f, 0.12f},
      {-0.25f, 0.25f, 0.30f},
  };
  return input;
}

FrameInput MakeWallWithBaseNoiseFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},
      {0.25f, 0.25f, 0.05f},
      {0.25f, 0.25f, 0.25f},
      {0.25f, 0.25f, 0.45f},
      {-0.25f, 0.25f, 0.0f},
      {-0.25f, 0.25f, 0.05f},
      {-0.25f, 0.25f, 0.25f},
      {-0.25f, 0.25f, 0.45f},
  };
  return input;
}

FrameInput MakeUnsupportedWallFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.25f},
      {0.25f, 0.25f, 0.45f},
      {0.25f, 0.25f, 0.65f},
      {-0.25f, 0.25f, 0.25f},
      {-0.25f, 0.25f, 0.45f},
      {-0.25f, 0.25f, 0.65f},
  };
  return input;
}

FrameInput MakePitchedObstacleColumnFrame(const Eigen::Quaternionf &orientation,
                                          float obstacle_height_in_base,
                                          int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = orientation.normalized();
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
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = orientation.normalized();
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

ProcessedFrame MakeProcessedFrame(const std::vector<passable_area::core::OdomPointSample> &samples) {
  ProcessedFrame frame;
  frame.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  frame.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  frame.odom_samples = samples;
  frame.cloud_in_base.reserve(samples.size());
  frame.cloud_in_odom.reserve(samples.size());
  for (const auto &sample : samples) {
    frame.cloud_in_base.push_back(sample.point_in_base);
    frame.cloud_in_odom.push_back(sample.point_in_odom);
  }
  return frame;
}

FrameInput MakeDynamicObstacleCellFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},
      {0.25f, 0.25f, 0.18f},
      {0.25f, 0.25f, 0.45f},
      {-0.25f, 0.25f, 0.0f},
      {-0.25f, 0.25f, 0.18f},
      {-0.25f, 0.25f, 0.45f},
  };
  return input;
}

FrameInput MakeGroundOnlyCellFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {0.25f, 0.25f, 0.0f},
      {0.28f, 0.22f, 0.0f},
      {0.22f, 0.28f, 0.0f},
      {-0.25f, 0.25f, 0.0f},
      {-0.22f, 0.22f, 0.0f},
      {-0.28f, 0.28f, 0.0f},
  };
  return input;
}

FrameInput MakeRearObstacleCellFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {
      {-0.8f, 0.8f, 0.0f},
      {-0.8f, 0.8f, 0.18f},
      {-0.8f, 0.8f, 0.45f},
      {-0.6f, 0.8f, 0.0f},
      {-0.6f, 0.8f, 0.18f},
      {-0.6f, 0.8f, 0.45f},
  };
  return input;
}

int CellIndex(const passable_area::core::FrameOutput &output, float x, float y) {
  const int col = static_cast<int>(std::floor((x - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(std::floor((y - output.origin.y()) / output.resolution));
  return row * output.cols + col;
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
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {{0.0f, 0.0f, 0.0f}, {0.6f, 0.0f, 0.0f}};

  ProcessedFrame output;
  ASSERT_TRUE(preprocessor.process(input, output));
  ASSERT_EQ(output.cloud_in_base.size(), 1U);
  ASSERT_EQ(output.cloud_in_odom.size(), 1U);
  EXPECT_FLOAT_EQ(output.cloud_in_base.front().x, 0.6f);
}

TEST(PreprocessorTest, CropToMapRemovesPointsOutsideLocalMapWindow) {
  auto config = MakeConfig();
  config.preprocess.crop_to_map.enable = true;
  config.preprocess.crop_to_map.xy_margin = 0.0f;
  FramePreprocessor preprocessor(config);

  FrameInput input;
  input.stamp = 1;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {{0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}};

  ProcessedFrame output;
  ASSERT_TRUE(preprocessor.process(input, output));
  ASSERT_EQ(output.cloud_in_base.size(), 1U);
  ASSERT_EQ(output.cloud_in_odom.size(), 1U);
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
  input.base_pose_in_odom.position = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base = {{0.6f, 0.0f, 0.2f}, {0.8f, 0.0f, 0.7f}};

  ProcessedFrame output;
  ASSERT_TRUE(preprocessor.process(input, output));
  ASSERT_EQ(output.cloud_in_base.size(), 1U);
  ASSERT_EQ(output.cloud_in_odom.size(), 1U);
  EXPECT_NEAR(output.cloud_in_odom.front().z, 1.2f, 1e-5f);
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
  EXPECT_TRUE(output.observability.rear_dropout);

  float max_rear_support_conf = 0.0f;
  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const int idx = row * output.cols + col;
      const float x = output.origin.x() + (static_cast<float>(col) + 0.5f) * output.resolution;
      if (x < -0.4f) {
        max_rear_support_conf = std::max(max_rear_support_conf, output.support_confidence[idx]);
      }
    }
  }
  EXPECT_GE(max_rear_support_conf, 0.05f);
}

TEST(ProcessorTest, RearGapTriggersMissingByDropoutSectors) {
  Processor processor(MakeConfig());
  const auto output = processor.update(MakeRearGapFrame());
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.observability.rear_dropout);

  int missing_count = 0;
  for (const auto &sector : output.observability.sectors) {
    if (sector.state == ObservabilityState::kMissingByDropout) {
      ++missing_count;
    }
  }
  EXPECT_GT(missing_count, 0);
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
    EXPECT_TRUE(sector.state == ObservabilityState::kObserved ||
                sector.state == ObservabilityState::kPartiallyObserved);
  }

  const auto dropout_output = processor.update(MakeFrontOnlyFrame(100000001));
  ASSERT_TRUE(dropout_output.valid);
  EXPECT_TRUE(dropout_output.observability.rear_dropout);
  for (const auto &sector : dropout_output.observability.sectors) {
    EXPECT_TRUE(sector.state == ObservabilityState::kObserved ||
                sector.state == ObservabilityState::kPartiallyObserved ||
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
  EXPECT_NE(output.passability[center_cell], static_cast<int8_t>(PassabilityState::kImpassable));
}

TEST(ProcessorTest, SparseCoverageCreatesUnknownButNotDropout) {
  auto config = MakeConfig();
  config.observability.min_points_per_sector = 20;
  Processor processor(config);
  const auto output = processor.update(MakeSparseFrame());
  ASSERT_TRUE(output.valid);
  EXPECT_FALSE(output.observability.rear_dropout);

  int partial_count = 0;
  for (const auto &sector : output.observability.sectors) {
    if (sector.state == ObservabilityState::kPartiallyObserved) {
      ++partial_count;
    }
  }
  EXPECT_GT(partial_count, 0);
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
  forward.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  forward.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  forward.cloud_in_base = {{1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f, 0.0f}};
  forward.cloud_in_odom = {{0.25f, 0.25f, 0.0f}, {0.25f, 0.25f, 0.2f}, {-0.25f, 0.25f, 0.0f},
                           {-0.25f, 0.25f, 0.2f}};
  forward.odom_samples.push_back(
      passable_area::core::OdomPointSample{{1.0f, 1.0f, 0.0f}, {0.25f, 0.25f, 0.0f}});
  forward.odom_samples.push_back(
      passable_area::core::OdomPointSample{{-1.0f, 1.0f, 0.0f}, {0.25f, 0.25f, 0.2f}});
  forward.odom_samples.push_back(
      passable_area::core::OdomPointSample{{1.0f, 0.0f, 0.0f}, {-0.25f, 0.25f, 0.0f}});
  forward.odom_samples.push_back(
      passable_area::core::OdomPointSample{{1.0f, 0.0f, 0.2f}, {-0.25f, 0.25f, 0.2f}});
  ProcessedFrame reversed = forward;
  std::reverse(reversed.odom_samples.begin(), reversed.odom_samples.end());

  const auto forward_output = frontend.run(forward, observability, map);
  const auto reversed_output = frontend.run(reversed, observability, map);

  ASSERT_EQ(forward_output.support_candidates.size(), 1U);
  ASSERT_EQ(reversed_output.support_candidates.size(), 1U);
  EXPECT_EQ(forward_output.support_candidates[0].cell, reversed_output.support_candidates[0].cell);
  EXPECT_FLOAT_EQ(forward_output.support_candidates[0].confidence,
                  reversed_output.support_candidates[0].confidence);
  ASSERT_EQ(forward_output.obstacle_candidates.size(), 2U);
  ASSERT_EQ(reversed_output.obstacle_candidates.size(), 2U);
  auto sorted_forward_obstacles = forward_output.obstacle_candidates;
  auto sorted_reversed_obstacles = reversed_output.obstacle_candidates;
  std::sort(sorted_forward_obstacles.begin(), sorted_forward_obstacles.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.cell < rhs.cell; });
  std::sort(sorted_reversed_obstacles.begin(), sorted_reversed_obstacles.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.cell < rhs.cell; });
  for (size_t i = 0; i < sorted_forward_obstacles.size(); ++i) {
    EXPECT_EQ(sorted_forward_obstacles[i].cell, sorted_reversed_obstacles[i].cell);
    EXPECT_FLOAT_EQ(sorted_forward_obstacles[i].evidence, sorted_reversed_obstacles[i].evidence);
  }
}

TEST(ProcessorTest, PolarFrontendRejectsIsolatedSuspiciousObstacleWithoutNeighborSupport) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.0f}, {0.25f, 0.25f, 0.0f}},
      {{0.25f, 0.25f, 0.45f}, {0.25f, 0.25f, 0.45f}},
  });

  const auto output = frontend.run(frame, observability, map);

  int cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  EXPECT_TRUE(output.obstacle_candidates.empty());
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(cell)], 1);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)], 1U);
}

TEST(ProcessorTest, PolarFrontendConfirmsSuspiciousObstacleWithNeighborSupportCluster) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.0f}, {0.25f, 0.25f, 0.0f}},
      {{0.25f, 0.25f, 0.45f}, {0.25f, 0.25f, 0.45f}},
      {{0.45f, 0.25f, 0.0f}, {0.45f, 0.25f, 0.0f}},
      {{0.45f, 0.25f, 0.38f}, {0.45f, 0.25f, 0.38f}},
  });

  const auto output = frontend.run(frame, observability, map);

  int primary_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_EQ(output.obstacle_candidates.size(), 2U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 2);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
}

TEST(ProcessorTest, PolarFrontendUsesFrameMinZWhenHistoricalSupportIsMissing) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int supported_neighbor = -1;
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, supported_neighbor));
  map.layers().support_height[static_cast<size_t>(supported_neighbor)] = 0.0f;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, 0.05f}, {0.25f, 0.25f, 0.05f}},
      {{0.25f, 0.25f, 0.32f}, {0.25f, 0.25f, 0.32f}},
      {{0.45f, 0.25f, 0.0f}, {0.45f, 0.25f, 0.0f}},
      {{0.45f, 0.25f, 0.33f}, {0.45f, 0.25f, 0.33f}},
  });

  const auto output = frontend.run(frame, observability, map);

  int fallback_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, fallback_cell));
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(fallback_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(fallback_cell)], 2);
  ASSERT_EQ(output.obstacle_candidates.size(), 2U);
}

TEST(ProcessorTest, PolarFrontendFiltersSubSupportLeakUsingValidHistoricalAnchor) {
  auto config = MakeConfig();
  config.geometry.sub_support_leak_tolerance = 0.18f;
  config.geometry.min_neighbor_upper_support_cells = 1;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  map.layers().support_height[static_cast<size_t>(cell)] = 0.0f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.5f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.30f}, {0.25f, 0.25f, -0.30f}},
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
  });

  const auto output = frontend.run(frame, observability, map);

  ASSERT_EQ(output.support_candidates.size(), 1U);
  EXPECT_EQ(output.support_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.support_candidates.front().z, 0.0f);
  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(cell)], 0.0f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(cell)], 0U);
  EXPECT_TRUE(output.obstacle_candidates.empty());
}

TEST(ProcessorTest, PolarFrontendIgnoresFiniteButInvalidHistoricalSupportForLeakFiltering) {
  auto config = MakeConfig();
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  map.layers().support_height[static_cast<size_t>(cell)] = 0.0f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.05f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.30f}, {0.25f, 0.25f, -0.30f}},
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
  });

  const auto output = frontend.run(frame, observability, map);

  ASSERT_EQ(output.support_candidates.size(), 1U);
  EXPECT_EQ(output.support_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.support_candidates.front().z, -0.30f);
  EXPECT_TRUE(std::isnan(output.support_anchor_used[static_cast<size_t>(cell)]));
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(cell)], 0U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 1U);
}

TEST(ProcessorTest, PolarFrontendUsesNeighborMedianAnchorWhenLocalAnchorIsUnavailable) {
  auto config = MakeConfig();
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int cell = -1;
  int neighbor = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, neighbor));
  map.layers().support_height[static_cast<size_t>(cell)] = 0.0f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.05f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().support_height[static_cast<size_t>(neighbor)] = 0.0f;
  map.layers().support_confidence[static_cast<size_t>(neighbor)] = 0.5f;
  map.layers().support_state[static_cast<size_t>(neighbor)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(neighbor)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.30f}, {0.25f, 0.25f, -0.30f}},
      {{0.25f, 0.25f, 0.00f}, {0.25f, 0.25f, 0.00f}},
      {{0.45f, 0.25f, 0.00f}, {0.45f, 0.25f, 0.00f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(cell)], 0.0f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 0U);
}

TEST(ProcessorTest, PolarFrontendDoesNotFallbackToRawSamplesWhenAnchorFilteredCellBecomesEmpty) {
  auto config = MakeConfig();
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  map.layers().support_height[static_cast<size_t>(cell)] = 0.0f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.5f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.30f}, {0.25f, 0.25f, -0.30f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_TRUE(output.support_candidates.empty());
  EXPECT_TRUE(output.ambiguous_candidates.empty());
  EXPECT_TRUE(output.obstacle_candidates.empty());
  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(cell)], 0.0f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(cell)], 0U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 0U);
}

TEST(ProcessorTest, PolarFrontendRejectsStaleLowerAnchorWhenHigherBandDominates) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.support_anchor_reobserve_tolerance = 0.08f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  map.layers().support_height[static_cast<size_t>(cell)] = -1.0f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.5f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.62f}, {0.25f, 0.25f, -0.62f}},
      {{0.25f, 0.25f, -0.55f}, {0.25f, 0.25f, -0.55f}},
  });

  const auto output = frontend.run(frame, observability, map);

  ASSERT_EQ(output.support_candidates.size(), 1U);
  EXPECT_EQ(output.support_candidates.front().cell, cell);
  EXPECT_FLOAT_EQ(output.support_candidates.front().z, -0.62f);
  EXPECT_TRUE(std::isnan(output.support_anchor_used[static_cast<size_t>(cell)]));
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(cell)], 0U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 0U);
  EXPECT_TRUE(output.obstacle_candidates.empty());
}

TEST(ProcessorTest, PolarFrontendIgnoresAnchorForWallOnlyCellsWithoutSupportBand) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 1;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  config.geometry.support_anchor_reobserve_tolerance = 0.08f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  map.layers().support_height[static_cast<size_t>(cell)] = 0.05f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -1.00f}, {0.25f, 0.25f, -1.00f}},
      {{0.25f, 0.25f, -0.82f}, {0.25f, 0.25f, -0.82f}},
      {{0.25f, 0.25f, -0.58f}, {0.25f, 0.25f, -0.58f}},
      {{0.25f, 0.25f, -0.34f}, {0.25f, 0.25f, -0.34f}},
      {{0.25f, 0.25f, 0.05f}, {0.25f, 0.25f, 0.05f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_TRUE(std::isnan(output.support_anchor_used[static_cast<size_t>(cell)]));
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(cell)], 1U);
}

TEST(ProcessorTest, PolarFrontendIgnoresAnchorForWallOnlyCellsWithShallowUpperBand) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 1;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  config.geometry.support_anchor_reobserve_tolerance = 0.08f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, cell));
  map.layers().support_height[static_cast<size_t>(cell)] = 0.05f;
  map.layers().support_confidence[static_cast<size_t>(cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -1.00f}, {0.25f, 0.25f, -1.00f}},
      {{0.25f, 0.25f, -0.82f}, {0.25f, 0.25f, -0.82f}},
      {{0.25f, 0.25f, -0.58f}, {0.25f, 0.25f, -0.58f}},
      {{0.25f, 0.25f, -0.34f}, {0.25f, 0.25f, -0.34f}},
      {{0.25f, 0.25f, 0.27f}, {0.25f, 0.25f, 0.27f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_TRUE(std::isnan(output.support_anchor_used[static_cast<size_t>(cell)]));
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(cell)], 1U);
}

TEST(ProcessorTest, PolarFrontendRejectsBelowRobotStairMixDespiteNeighborUpperSupport) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int neighbor_cell = -1;
  int diagonal_neighbor_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, neighbor_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, diagonal_neighbor_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.90f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(neighbor_cell)] = -0.55f;
  map.layers().support_confidence[static_cast<size_t>(neighbor_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(neighbor_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(neighbor_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -1.12f}, {0.25f, 0.25f, -1.12f}},
      {{0.25f, 0.25f, -0.90f}, {0.25f, 0.25f, -0.90f}},
      {{0.25f, 0.25f, -0.52f}, {0.25f, 0.25f, -0.52f}},
      {{0.45f, 0.25f, -0.55f}, {0.45f, 0.25f, -0.55f}},
      {{0.45f, 0.25f, -0.20f}, {0.45f, 0.25f, -0.20f}},
      {{0.45f, 0.45f, -0.56f}, {0.45f, 0.45f, -0.56f}},
      {{0.45f, 0.45f, -0.18f}, {0.45f, 0.45f, -0.18f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(neighbor_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(diagonal_neighbor_cell)], 1U);
  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.90f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 3);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendDoesNotRejectBelowRobotObstacleWithoutLeakEvidence) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int neighbor_cell = -1;
  int diagonal_neighbor_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, neighbor_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, diagonal_neighbor_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.90f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(neighbor_cell)] = -0.55f;
  map.layers().support_confidence[static_cast<size_t>(neighbor_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(neighbor_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(neighbor_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.90f}, {0.25f, 0.25f, -0.90f}},
      {{0.25f, 0.25f, -0.52f}, {0.25f, 0.25f, -0.52f}},
      {{0.45f, 0.25f, -0.55f}, {0.45f, 0.25f, -0.55f}},
      {{0.45f, 0.25f, -0.20f}, {0.45f, 0.25f, -0.20f}},
      {{0.45f, 0.45f, -0.56f}, {0.45f, 0.45f, -0.56f}},
      {{0.45f, 0.45f, -0.18f}, {0.45f, 0.45f, -0.18f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 3);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 1);
}

TEST(ProcessorTest, PolarFrontendRejectsBelowRobotGroundLayerMixWithOnlyMinimalNeighborSupport) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int neighbor_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.45f, neighbor_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.68f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.68f}, {0.25f, 0.25f, -0.68f}},
      {{0.25f, 0.25f, -0.65f}, {0.25f, 0.25f, -0.65f}},
      {{0.25f, 0.25f, -0.46f}, {0.25f, 0.25f, -0.46f}},
      {{0.25f, 0.45f, -0.67f}, {0.25f, 0.45f, -0.67f}},
      {{0.25f, 0.45f, -0.45f}, {0.25f, 0.45f, -0.45f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.68f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(neighbor_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 2);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendKeepsBelowRobotGroundLayerMixWhenUpperSupportStructureIsStronger) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int upper_support_neighbor_cell = -1;
  int aligned_neighbor_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, upper_support_neighbor_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, aligned_neighbor_cell));
  ASSERT_NE(primary_cell, upper_support_neighbor_cell);
  ASSERT_NE(primary_cell, aligned_neighbor_cell);
  ASSERT_NE(upper_support_neighbor_cell, aligned_neighbor_cell);
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.68f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(upper_support_neighbor_cell)] = -0.67f;
  map.layers().support_confidence[static_cast<size_t>(upper_support_neighbor_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(upper_support_neighbor_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(upper_support_neighbor_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(aligned_neighbor_cell)] = -0.46f;
  map.layers().support_confidence[static_cast<size_t>(aligned_neighbor_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(aligned_neighbor_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(aligned_neighbor_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.68f}, {0.25f, 0.25f, -0.68f}},
      {{0.25f, 0.25f, -0.65f}, {0.25f, 0.25f, -0.65f}},
      {{0.25f, 0.25f, -0.46f}, {0.25f, 0.25f, -0.46f}},
      {{0.45f, 0.25f, -0.67f}, {0.45f, 0.25f, -0.67f}},
      {{0.45f, 0.25f, -0.45f}, {0.45f, 0.25f, -0.45f}},
      {{0.45f, 0.45f, -0.46f}, {0.45f, 0.45f, -0.46f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.68f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(upper_support_neighbor_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 2);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 1);
}

TEST(ProcessorTest, PolarFrontendKeepsBelowRobotGroundLayerMixWithoutUpstairTrendWhenLocalStructureIsStrong) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int upper_support_neighbor_cell = -1;
  int aligned_neighbor_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, upper_support_neighbor_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, aligned_neighbor_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.68f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(upper_support_neighbor_cell)] = -0.67f;
  map.layers().support_confidence[static_cast<size_t>(upper_support_neighbor_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(upper_support_neighbor_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(upper_support_neighbor_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(aligned_neighbor_cell)] = -0.46f;
  map.layers().support_confidence[static_cast<size_t>(aligned_neighbor_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(aligned_neighbor_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(aligned_neighbor_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.68f}, {0.25f, 0.25f, -0.68f}},
      {{0.25f, 0.25f, -0.65f}, {0.25f, 0.25f, -0.65f}},
      {{0.25f, 0.25f, -0.46f}, {0.25f, 0.25f, -0.46f}},
      {{0.45f, 0.25f, -0.67f}, {0.45f, 0.25f, -0.67f}},
      {{0.45f, 0.25f, -0.45f}, {0.45f, 0.25f, -0.45f}},
      {{0.45f, 0.45f, -0.46f}, {0.45f, 0.45f, -0.46f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.68f);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(upper_support_neighbor_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 2);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 1);
}

TEST(ProcessorTest, PolarFrontendRejectsBelowRobotGroundLayerMixWhenUpperSupportSurplusStaysOneSided) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int front_left_cell = -1;
  int front_center_cell = -1;
  int front_right_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.05f, 0.45f, front_left_cell));
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.45f, front_center_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, front_right_cell));
  ASSERT_NE(front_left_cell, front_center_cell);
  ASSERT_NE(front_left_cell, front_right_cell);
  ASSERT_NE(front_center_cell, front_right_cell);
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.68f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.68f}, {0.25f, 0.25f, -0.68f}},
      {{0.25f, 0.25f, -0.65f}, {0.25f, 0.25f, -0.65f}},
      {{0.25f, 0.25f, -0.46f}, {0.25f, 0.25f, -0.46f}},
      {{0.05f, 0.45f, -0.67f}, {0.05f, 0.45f, -0.67f}},
      {{0.05f, 0.45f, -0.45f}, {0.05f, 0.45f, -0.45f}},
      {{0.25f, 0.45f, -0.67f}, {0.25f, 0.45f, -0.67f}},
      {{0.25f, 0.45f, -0.45f}, {0.25f, 0.45f, -0.45f}},
      {{0.45f, 0.45f, -0.66f}, {0.45f, 0.45f, -0.66f}},
      {{0.45f, 0.45f, -0.44f}, {0.45f, 0.45f, -0.44f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.68f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(front_left_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(front_center_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(front_right_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 4);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendRejectsBelowRobotGroundLayerMixWhenUpperSupportLacksDiagonalPatch) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int left_cell = -1;
  int right_cell = -1;
  int front_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.05f, 0.25f, left_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, right_cell));
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.45f, front_cell));
  ASSERT_NE(left_cell, right_cell);
  ASSERT_NE(left_cell, front_cell);
  ASSERT_NE(right_cell, front_cell);
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.68f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.68f}, {0.25f, 0.25f, -0.68f}},
      {{0.25f, 0.25f, -0.65f}, {0.25f, 0.25f, -0.65f}},
      {{0.25f, 0.25f, -0.46f}, {0.25f, 0.25f, -0.46f}},
      {{0.05f, 0.25f, -0.67f}, {0.05f, 0.25f, -0.67f}},
      {{0.05f, 0.25f, -0.45f}, {0.05f, 0.25f, -0.45f}},
      {{0.45f, 0.25f, -0.66f}, {0.45f, 0.25f, -0.66f}},
      {{0.45f, 0.25f, -0.44f}, {0.45f, 0.25f, -0.44f}},
      {{0.25f, 0.45f, -0.67f}, {0.25f, 0.45f, -0.67f}},
      {{0.25f, 0.45f, -0.45f}, {0.25f, 0.45f, -0.45f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.68f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(left_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(right_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(front_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 4);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendRejectsBelowRobotUpstairGroundMixWithoutSupportAnchor) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.68f}, {0.25f, 0.25f, -0.68f}},
      {{0.25f, 0.25f, -0.66f}, {0.25f, 0.25f, -0.66f}},
      {{0.25f, 0.25f, -0.37f}, {0.25f, 0.25f, -0.37f}},
      {{0.15f, 0.45f, -0.50f}, {0.15f, 0.45f, -0.50f}},
      {{0.15f, 0.45f, -0.35f}, {0.15f, 0.45f, -0.35f}},
      {{0.35f, 0.45f, -0.49f}, {0.35f, 0.45f, -0.49f}},
      {{0.35f, 0.45f, -0.34f}, {0.35f, 0.45f, -0.34f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_TRUE(std::isnan(output.support_anchor_used[static_cast<size_t>(primary_cell)]));
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 1);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendRejectsBelowRobotUpstairGroundMixWithBelowRobotSupportAnchor) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.64f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.68f}, {0.25f, 0.25f, -0.68f}},
      {{0.25f, 0.25f, -0.66f}, {0.25f, 0.25f, -0.66f}},
      {{0.25f, 0.25f, -0.37f}, {0.25f, 0.25f, -0.37f}},
      {{0.15f, 0.45f, -0.50f}, {0.15f, 0.45f, -0.50f}},
      {{0.15f, 0.45f, -0.35f}, {0.15f, 0.45f, -0.35f}},
      {{0.35f, 0.45f, -0.49f}, {0.35f, 0.45f, -0.49f}},
      {{0.35f, 0.45f, -0.34f}, {0.35f, 0.45f, -0.34f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.64f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendKeepsBelowRobotObstacleOnUpstairSupportTrend) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int left_up_cell = -1;
  int right_up_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.15f, 0.45f, left_up_cell));
  ASSERT_TRUE(map.odomToIndex(0.35f, 0.45f, right_up_cell));

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.72f}, {0.25f, 0.25f, -0.72f}},
      {{0.25f, 0.25f, -0.69f}, {0.25f, 0.25f, -0.69f}},
      {{0.25f, 0.25f, -0.06f}, {0.25f, 0.25f, -0.06f}},
      {{0.15f, 0.45f, -0.52f}, {0.15f, 0.45f, -0.52f}},
      {{0.15f, 0.45f, -0.31f}, {0.15f, 0.45f, -0.31f}},
      {{0.35f, 0.45f, -0.50f}, {0.35f, 0.45f, -0.50f}},
      {{0.35f, 0.45f, -0.29f}, {0.35f, 0.45f, -0.29f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_TRUE(std::isnan(output.support_anchor_used[static_cast<size_t>(primary_cell)]));
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(left_up_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(right_up_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 3);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 1);
}

TEST(ProcessorTest, PolarFrontendKeepsAnchoredBelowRobotObstacleOnUpstairSupportTrend) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int left_up_cell = -1;
  int right_up_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.15f, 0.45f, left_up_cell));
  ASSERT_TRUE(map.odomToIndex(0.35f, 0.45f, right_up_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.64f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.72f}, {0.25f, 0.25f, -0.72f}},
      {{0.25f, 0.25f, -0.69f}, {0.25f, 0.25f, -0.69f}},
      {{0.25f, 0.25f, 0.02f}, {0.25f, 0.25f, 0.02f}},
      {{0.15f, 0.45f, -0.52f}, {0.15f, 0.45f, -0.52f}},
      {{0.15f, 0.45f, -0.31f}, {0.15f, 0.45f, -0.31f}},
      {{0.35f, 0.45f, -0.50f}, {0.35f, 0.45f, -0.50f}},
      {{0.35f, 0.45f, -0.29f}, {0.35f, 0.45f, -0.29f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.64f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(left_up_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(right_up_cell)], 1U);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 1);
}

TEST(ProcessorTest, PolarFrontendUsesElevatedEffectiveSupportRefForLayeredGroundWithWeakUpperStructure) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int left_neighbor = -1;
  int right_neighbor = -1;
  int diagonal_neighbor = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.05f, 0.25f, left_neighbor));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, right_neighbor));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, diagonal_neighbor));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.82f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(left_neighbor)] = -0.74f;
  map.layers().support_confidence[static_cast<size_t>(left_neighbor)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(left_neighbor)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(left_neighbor)] = 0U;
  map.layers().support_height[static_cast<size_t>(right_neighbor)] = -0.73f;
  map.layers().support_confidence[static_cast<size_t>(right_neighbor)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(right_neighbor)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(right_neighbor)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.82f}, {0.25f, 0.25f, -0.82f}},
      {{0.25f, 0.25f, -0.80f}, {0.25f, 0.25f, -0.80f}},
      {{0.25f, 0.25f, -0.74f}, {0.25f, 0.25f, -0.74f}},
      {{0.25f, 0.25f, -0.73f}, {0.25f, 0.25f, -0.73f}},
      {{0.25f, 0.25f, -0.55f}, {0.25f, 0.25f, -0.55f}},
      {{0.05f, 0.25f, -0.74f}, {0.05f, 0.25f, -0.74f}},
      {{0.05f, 0.25f, -0.72f}, {0.05f, 0.25f, -0.72f}},
      {{0.45f, 0.25f, -0.73f}, {0.45f, 0.25f, -0.73f}},
      {{0.45f, 0.25f, -0.71f}, {0.45f, 0.25f, -0.71f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.82f);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendUsesNeighborUpperLayerConsensusForLayeredGroundExplanation) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int left_neighbor = -1;
  int right_neighbor = -1;
  int diagonal_neighbor = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.05f, 0.25f, left_neighbor));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, right_neighbor));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, diagonal_neighbor));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.82f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(left_neighbor)] = -0.82f;
  map.layers().support_confidence[static_cast<size_t>(left_neighbor)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(left_neighbor)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(left_neighbor)] = 0U;
  map.layers().support_height[static_cast<size_t>(right_neighbor)] = -0.81f;
  map.layers().support_confidence[static_cast<size_t>(right_neighbor)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(right_neighbor)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(right_neighbor)] = 0U;
  map.layers().support_height[static_cast<size_t>(diagonal_neighbor)] = -0.80f;
  map.layers().support_confidence[static_cast<size_t>(diagonal_neighbor)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(diagonal_neighbor)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(diagonal_neighbor)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.82f}, {0.25f, 0.25f, -0.82f}},
      {{0.25f, 0.25f, -0.80f}, {0.25f, 0.25f, -0.80f}},
      {{0.25f, 0.25f, -0.81f}, {0.25f, 0.25f, -0.81f}},
      {{0.25f, 0.25f, -0.79f}, {0.25f, 0.25f, -0.79f}},
      {{0.25f, 0.25f, -0.61f}, {0.25f, 0.25f, -0.61f}},
      {{0.25f, 0.25f, -0.60f}, {0.25f, 0.25f, -0.60f}},
      {{0.25f, 0.25f, -0.43f}, {0.25f, 0.25f, -0.43f}},
      {{0.05f, 0.25f, -0.82f}, {0.05f, 0.25f, -0.82f}},
      {{0.05f, 0.25f, -0.60f}, {0.05f, 0.25f, -0.60f}},
      {{0.45f, 0.25f, -0.81f}, {0.45f, 0.25f, -0.81f}},
      {{0.45f, 0.25f, -0.60f}, {0.45f, 0.25f, -0.60f}},
      {{0.45f, 0.45f, -0.80f}, {0.45f, 0.45f, -0.80f}},
      {{0.45f, 0.45f, -0.59f}, {0.45f, 0.45f, -0.59f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.82f);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 1);
}

TEST(ProcessorTest, PolarFrontendKeepsLowObstacleWhenUpperLayerLacksNeighborSupportConsistency) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int left_neighbor = -1;
  int right_neighbor = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.05f, 0.25f, left_neighbor));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, right_neighbor));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.82f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.82f}, {0.25f, 0.25f, -0.82f}},
      {{0.25f, 0.25f, -0.80f}, {0.25f, 0.25f, -0.80f}},
      {{0.25f, 0.25f, -0.74f}, {0.25f, 0.25f, -0.74f}},
      {{0.25f, 0.25f, -0.73f}, {0.25f, 0.25f, -0.73f}},
      {{0.25f, 0.25f, -0.45f}, {0.25f, 0.25f, -0.45f}},
      {{0.05f, 0.25f, -0.82f}, {0.05f, 0.25f, -0.82f}},
      {{0.05f, 0.25f, -0.56f}, {0.05f, 0.25f, -0.56f}},
      {{0.45f, 0.25f, -0.81f}, {0.45f, 0.25f, -0.81f}},
      {{0.45f, 0.25f, -0.55f}, {0.45f, 0.25f, -0.55f}},
      {{0.45f, 0.45f, -0.80f}, {0.45f, 0.45f, -0.80f}},
      {{0.45f, 0.45f, -0.54f}, {0.45f, 0.45f, -0.54f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.82f);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 4);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 0U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 1);
}

TEST(ProcessorTest, PolarFrontendDoesNotUseEffectiveSupportRefForAscendingTrend) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.max_step_up = 0.28f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int left_up_cell = -1;
  int right_up_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.15f, 0.45f, left_up_cell));
  ASSERT_TRUE(map.odomToIndex(0.35f, 0.45f, right_up_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.64f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(left_up_cell)] = -0.38f;
  map.layers().support_confidence[static_cast<size_t>(left_up_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(left_up_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(left_up_cell)] = 0U;
  map.layers().support_height[static_cast<size_t>(right_up_cell)] = -0.37f;
  map.layers().support_confidence[static_cast<size_t>(right_up_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(right_up_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(right_up_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.64f}, {0.25f, 0.25f, -0.64f}},
      {{0.25f, 0.25f, -0.62f}, {0.25f, 0.25f, -0.62f}},
      {{0.25f, 0.25f, -0.44f}, {0.25f, 0.25f, -0.44f}},
      {{0.25f, 0.25f, -0.41f}, {0.25f, 0.25f, -0.41f}},
      {{0.15f, 0.45f, -0.38f}, {0.15f, 0.45f, -0.38f}},
      {{0.15f, 0.45f, -0.14f}, {0.15f, 0.45f, -0.14f}},
      {{0.35f, 0.45f, -0.37f}, {0.35f, 0.45f, -0.37f}},
      {{0.35f, 0.45f, -0.13f}, {0.35f, 0.45f, -0.13f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.64f);
  EXPECT_EQ(output.obstacle_suspicious[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, PolarFrontendRejectsBelowRobotMixWhenLowAnchorUpperBandDominates) {
  auto config = MakeConfig();
  config.geometry.upper_min_height_above_support = 0.2f;
  config.geometry.min_neighbor_upper_support_cells = 2;
  config.geometry.sub_support_leak_tolerance = 0.18f;
  PolarFrontend frontend(config);
  LocalTerrainMap map(config);
  map.recenter(Eigen::Vector2f::Zero());

  FrameObservability observability;
  observability.sectors.resize(static_cast<size_t>(config.observability.sector_count));
  for (auto &sector : observability.sectors) {
    sector.state = ObservabilityState::kObserved;
    sector.coverage_confidence = 1.0f;
  }

  int primary_cell = -1;
  int neighbor_cell = -1;
  int diagonal_neighbor_cell = -1;
  ASSERT_TRUE(map.odomToIndex(0.25f, 0.25f, primary_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.25f, neighbor_cell));
  ASSERT_TRUE(map.odomToIndex(0.45f, 0.45f, diagonal_neighbor_cell));
  map.layers().support_height[static_cast<size_t>(primary_cell)] = -0.90f;
  map.layers().support_confidence[static_cast<size_t>(primary_cell)] = 0.6f;
  map.layers().support_state[static_cast<size_t>(primary_cell)] =
      static_cast<uint8_t>(SupportState::kPersistent);
  map.layers().last_reliable_age[static_cast<size_t>(primary_cell)] = 0U;

  const auto frame = MakeProcessedFrame({
      {{0.25f, 0.25f, -0.90f}, {0.25f, 0.25f, -0.90f}},
      {{0.25f, 0.25f, -0.87f}, {0.25f, 0.25f, -0.87f}},
      {{0.25f, 0.25f, -0.62f}, {0.25f, 0.25f, -0.62f}},
      {{0.25f, 0.25f, -0.55f}, {0.25f, 0.25f, -0.55f}},
      {{0.45f, 0.25f, -0.58f}, {0.45f, 0.25f, -0.58f}},
      {{0.45f, 0.25f, -0.22f}, {0.45f, 0.25f, -0.22f}},
      {{0.45f, 0.45f, -0.60f}, {0.45f, 0.45f, -0.60f}},
      {{0.45f, 0.45f, -0.20f}, {0.45f, 0.45f, -0.20f}},
  });

  const auto output = frontend.run(frame, observability, map);

  EXPECT_FLOAT_EQ(output.support_anchor_used[static_cast<size_t>(primary_cell)], -0.90f);
  EXPECT_EQ(output.sub_support_leak_count[static_cast<size_t>(primary_cell)], 0U);
  EXPECT_EQ(output.upper_support_cell[static_cast<size_t>(primary_cell)], 1U);
  EXPECT_EQ(output.neighbor_upper_support_count[static_cast<size_t>(primary_cell)], 3);
  EXPECT_EQ(output.obstacle_rejected_by_neighbor_support[static_cast<size_t>(primary_cell)], 1U);
  const auto primary_cell_candidate_count =
      std::count_if(output.obstacle_candidates.begin(), output.obstacle_candidates.end(),
                    [primary_cell](const auto &candidate) {
                      return candidate.cell == primary_cell;
                    });
  EXPECT_EQ(primary_cell_candidate_count, 0);
}

TEST(ProcessorTest, LocalTerrainMapRecenterShiftsHistoricalLayers) {
  auto config = MakeConfig();
  config.map.length = 4.0f;
  config.map.width = 4.0f;
  config.map.resolution = 1.0f;
  LocalTerrainMap map(config);

  int original_index = -1;
  ASSERT_TRUE(map.odomToIndex(0.5f, 0.5f, original_index));
  map.layers().support_confidence[original_index] = 0.75f;
  map.layers().support_state[original_index] = static_cast<uint8_t>(SupportState::kPersistent);

  map.recenter(Eigen::Vector2f(1.0f, 0.0f));

  int shifted_index = -1;
  ASSERT_TRUE(map.odomToIndex(0.5f, 0.5f, shifted_index));
  EXPECT_NE(original_index, shifted_index);
  EXPECT_FLOAT_EQ(map.layers().support_confidence[shifted_index], 0.75f);
  EXPECT_EQ(map.layers().support_state[shifted_index],
            static_cast<uint8_t>(SupportState::kPersistent));
}

TEST(ProcessorTest, ObservedSupportStateIsNotOverwrittenAsPersistentInSameUpdate) {
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
  const auto dirty = updater.update(frontend_output, observability, map);

  ASSERT_FALSE(dirty.empty());
  EXPECT_EQ(map.layers().support_state[3], static_cast<uint8_t>(SupportState::kObserved));
}

TEST(ProcessorTest, ObstacleCandidateGainScaleBoostsEvidenceAccumulation) {
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
  frontend_output.obstacle_candidates.push_back(
      passable_area::core::ObstacleCandidate{3, 0.5f, 0.9f, 1.8f});
  const auto dirty = updater.update(frontend_output, observability, map);

  ASSERT_FALSE(dirty.empty());
  EXPECT_NEAR(map.layers().obstacle_evidence[3],
              config.persistence.obstacle_evidence_gain * 1.8f * 0.9f,
              1e-5f);
  EXPECT_NEAR(map.layers().overhead_confidence[3],
              config.persistence.obstacle_evidence_gain * 1.8f,
              1e-5f);
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

  ASSERT_EQ(output.obstacle_points.size(), 4U);
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  for (const auto &point : output.obstacle_points) {
    min_z = std::min(min_z, point.point.z);
    max_z = std::max(max_z, point.point.z);
  }
  EXPECT_NEAR(min_z, 0.12f, 1e-5f);
  EXPECT_NEAR(max_z, 0.30f, 1e-5f);
}

TEST(ProcessorTest, ObstaclePointsExcludeGroundSamplesUnderLowCeiling) {
  auto config = MakeConfig();
  config.preprocess.enable_downsample = false;
  config.obstacle_points_min_evidence = 0.2f;
  config.obstacle_points_min_height = 0.15f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.2f, 1)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.2f, 100000001)).valid);
  ASSERT_TRUE(processor.update(MakeLowCeilingFrame(0.2f, 200000001)).valid);
  const auto output = processor.update(MakeLowCeilingFrame(0.2f, 300000001));
  ASSERT_TRUE(output.valid);
  ASSERT_FALSE(output.obstacle_points.empty());

  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  for (const auto &point : output.obstacle_points) {
    min_z = std::min(min_z, point.point.z);
    max_z = std::max(max_z, point.point.z);
  }
  EXPECT_NEAR(min_z, 0.2f, 1e-5f);
  EXPECT_NEAR(max_z, 0.2f, 1e-5f);
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
  ASSERT_TRUE(processor.update(
      MakePitchedObstacleStackFrame(pitched_orientation, 0.25f, 0.35f, 1)).valid);
  ASSERT_TRUE(processor.update(
      MakePitchedObstacleStackFrame(pitched_orientation, 0.25f, 0.35f, 100000001)).valid);
  ASSERT_TRUE(processor.update(
      MakePitchedObstacleStackFrame(pitched_orientation, 0.25f, 0.35f, 200000001)).valid);
  const auto output =
      processor.update(MakePitchedObstacleStackFrame(pitched_orientation, 0.25f, 0.35f, 300000001));
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
  config.obstacle_points_max_height_in_base_link = 0.2f;
  Processor processor(config);

  const Eigen::Quaternionf pitched_orientation(Eigen::AngleAxisf(
      -static_cast<float>(M_PI) / 6.0f, Eigen::Vector3f::UnitY()));
  ASSERT_TRUE(processor.update(
      MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 1)).valid);
  ASSERT_TRUE(processor.update(
      MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 100000001)).valid);
  ASSERT_TRUE(processor.update(
      MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 200000001)).valid);
  const auto output =
      processor.update(MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 300000001));
  ASSERT_TRUE(output.valid);

  ASSERT_FALSE(output.obstacle_points.empty());
  for (const auto &point : output.obstacle_points) {
    EXPECT_GT(point.point.z, 0.2f);
  }
}

TEST(ProcessorTest, ObstaclePointPublishUsesBaseLinkCeilingButPublishesInBaseGravity) {
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
  FramePreprocessor preprocessor(config);

  const Eigen::Quaternionf pitched_orientation(Eigen::AngleAxisf(
      -static_cast<float>(M_PI) / 6.0f, Eigen::Vector3f::UnitY()));
  const FrameInput input = MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 1);
  ProcessedFrame preprocessed;
  ASSERT_TRUE(preprocessor.process(input, preprocessed));

  bool found_publishable_source_sample = false;
  for (const auto &sample : preprocessed.odom_samples) {
    if (std::abs(sample.point_in_base.x - 0.55f) < 1e-5f &&
        std::abs(sample.point_in_base.y - 0.25f) < 1e-5f &&
        std::abs(sample.point_in_base.z - 0.18f) < 1e-5f) {
      found_publishable_source_sample = true;
      EXPECT_LE(sample.point_in_base.z, config.obstacle_points_max_height_in_base_link);
    }
  }
  EXPECT_TRUE(found_publishable_source_sample);

  ASSERT_TRUE(processor.update(input).valid);
  ASSERT_TRUE(
      processor.update(MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 100000001))
          .valid);
  ASSERT_TRUE(
      processor.update(MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 200000001))
          .valid);
  const auto output =
      processor.update(MakePitchedObstacleStackFrame(pitched_orientation, 0.18f, 0.35f, 300000001));
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
  EXPECT_EQ(output.passability[right_wall_cell], static_cast<int8_t>(PassabilityState::kImpassable));
  EXPECT_EQ(output.passability[left_wall_cell], static_cast<int8_t>(PassabilityState::kImpassable));
}

TEST(ProcessorTest, DebugObstaclePointsIgnoreWeakObstacleEvidenceByDefault) {
  auto config = MakeConfig();
  config.map.length = 2.0f;
  config.map.width = 2.0f;
  config.map.resolution = 1.0f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 8;
  config.observability.min_points_per_sector = 1;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(100000001)).valid);
  const auto output = processor.update(MakeDynamicObstacleCellFrame(200000001));
  ASSERT_TRUE(output.valid);

  const int cell = CellIndex(output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_GT(output.obstacle_evidence[cell], 0.25f);
  EXPECT_LT(output.obstacle_evidence[cell], config.obstacle_points_min_evidence);
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
  config.persistence.obstacle_clear_observed_decay = 0.20f;
  config.persistence.obstacle_clear_partial_decay_scale = 0.35f;
  config.persistence.obstacle_height_clear_threshold = 0.25f;
  Processor processor(config);

  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(1)).valid);
  ASSERT_TRUE(processor.update(MakeDynamicObstacleCellFrame(100000001)).valid);
  const auto obstacle_output = processor.update(MakeDynamicObstacleCellFrame(200000001));
  ASSERT_TRUE(obstacle_output.valid);

  const int cell = CellIndex(obstacle_output, 0.25f, 0.25f);
  ASSERT_GE(cell, 0);
  EXPECT_GT(obstacle_output.obstacle_evidence[cell], 0.25f);
  ASSERT_TRUE(std::isfinite(obstacle_output.overhead_height[cell]));

  ASSERT_TRUE(processor.update(MakeGroundOnlyCellFrame(300000001)).valid);
  const auto cleared_output = processor.update(MakeGroundOnlyCellFrame(400000001));
  ASSERT_TRUE(cleared_output.valid);
  EXPECT_LE(cleared_output.obstacle_evidence[cell], config.persistence.obstacle_height_clear_threshold);
  EXPECT_FALSE(std::isfinite(cleared_output.overhead_height[cell]));
  EXPECT_TRUE(std::isinf(cleared_output.clearance[cell]));
  EXPECT_NE(cleared_output.passability[cell], static_cast<int8_t>(PassabilityState::kImpassable));
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
  const auto obstacle_output = processor.update(MakeRearObstacleCellFrame(200000001));
  ASSERT_TRUE(obstacle_output.valid);
  const int cell = CellIndex(obstacle_output, -0.8f, 0.8f);
  ASSERT_GE(cell, 0);
  ASSERT_TRUE(std::isfinite(obstacle_output.overhead_height[cell]));
  const float evidence_before = obstacle_output.obstacle_evidence[cell];

  const auto dropout_output = processor.update(MakeFrontOnlyFrame(300000001));
  ASSERT_TRUE(dropout_output.valid);
  EXPECT_TRUE(dropout_output.observability.rear_dropout);
  EXPECT_GT(dropout_output.obstacle_evidence[cell], config.persistence.obstacle_height_clear_threshold);
  EXPECT_GT(dropout_output.obstacle_evidence[cell], evidence_before - config.persistence.obstacle_clear_observed_decay);
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

  const int center_cell = CellIndex(output, 0.0f, 0.0f);
  ASSERT_GE(center_cell, 0);
  EXPECT_TRUE(std::isfinite(output.overhead_height[center_cell]));
  EXPECT_LT(output.clearance[center_cell], config.geometry.min_clearance);
  EXPECT_EQ(output.passability[center_cell], static_cast<int8_t>(PassabilityState::kImpassable));
}
