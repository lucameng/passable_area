#include "passable_area/core/pipeline/processor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

using passable_area::core::FrameInput;
using passable_area::core::ObservabilityState;
using passable_area::core::PassabilityState;
using passable_area::core::Processor;

FrameInput MakeFlatFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_local.position = Eigen::Vector3f::Zero();
  input.base_pose_in_local.orientation = Eigen::Quaternionf::Identity();
  for (float x = -1.0f; x <= 1.0f; x += 0.2f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.2f) {
      input.merged_cloud.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeRampFrame(float slope, int stamp = 1) {
  passable_area::core::FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_local.position = Eigen::Vector3f::Zero();
  input.base_pose_in_local.orientation = Eigen::Quaternionf::Identity();
  for (float x = -1.0f; x <= 1.0f; x += 0.2f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.2f) {
      input.merged_cloud.push_back({x, y, slope * x});
    }
  }
  return input;
}

FrameInput MakeStairFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_local.position = Eigen::Vector3f::Zero();
  input.base_pose_in_local.orientation = Eigen::Quaternionf::Identity();
  for (float x = -1.0f; x <= 1.0f; x += 0.08f) {
    const float z = 0.08f * std::floor((x + 1.0f) / 0.4f);
    for (float y = -1.0f; y <= 1.0f; y += 0.08f) {
      input.merged_cloud.push_back({x, y, z});
    }
  }
  return input;
}

FrameInput MakeLowCeilingFrame(float ceiling_height, int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  for (float x = -0.6f; x <= 0.6f; x += 0.12f) {
    for (float y = -0.6f; y <= 0.6f; y += 0.12f) {
      input.merged_cloud.push_back({x, y, ceiling_height});
    }
  }
  return input;
}

FrameInput MakeFrontOnlyFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_local.position = Eigen::Vector3f::Zero();
  input.base_pose_in_local.orientation = Eigen::Quaternionf::Identity();
  for (float x = 0.0f; x <= 1.2f; x += 0.1f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.1f) {
      input.merged_cloud.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeRearGapFrame(int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  input.merged_cloud.erase(
      std::remove_if(input.merged_cloud.begin(), input.merged_cloud.end(),
                     [](const auto &point) { return point.x < -0.1f && std::abs(point.y) < 1.0f; }),
      input.merged_cloud.end());
  for (float x = 0.2f; x <= 1.2f; x += 0.05f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.05f) {
      input.merged_cloud.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeLocalHoleFrame(int stamp = 1) {
  auto input = MakeFlatFrame(stamp);
  input.merged_cloud.erase(
      std::remove_if(input.merged_cloud.begin(), input.merged_cloud.end(), [](const auto &point) {
        return std::abs(point.x) < 0.35f && std::abs(point.y) < 0.35f;
      }),
      input.merged_cloud.end());
  return input;
}

FrameInput MakeSparseFrame(int stamp = 1) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_local.position = Eigen::Vector3f::Zero();
  input.base_pose_in_local.orientation = Eigen::Quaternionf::Identity();
  int counter = 0;
  for (float x = -1.0f; x <= 1.0f; x += 0.08f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.08f) {
      if ((counter++ % 9) == 0) {
        input.merged_cloud.push_back({x, y, 0.0f});
      }
    }
  }
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
