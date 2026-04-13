#include "passable_area/core/processor.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {

passable_area::core::Config MakeConfig() {
  passable_area::core::Config config;
  config.map.length = 8.0f;
  config.map.width = 8.0f;
  config.map.resolution = 0.1f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 72;
  config.observability.min_points_per_sector = 12;
  config.observability.dropout_sector_gap_threshold = 8;
  config.observability.min_support_confidence = 0.2f;
  config.persistence.support_persistence_frames = 6;
  return config;
}

passable_area::core::FrameInput MakeBenchmarkFrame(int point_count, int64_t stamp) {
  passable_area::core::FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  input.input_cloud_in_base.reserve(point_count);

  std::mt19937 rng(42 + point_count);
  std::uniform_real_distribution<float> xy(-3.8f, 3.8f);
  std::uniform_real_distribution<float> noise(-0.015f, 0.015f);
  std::uniform_real_distribution<float> ceiling_coin(0.0f, 1.0f);
  for (int i = 0; i < point_count; ++i) {
    const float x = xy(rng);
    const float y = xy(rng);
    float z = 0.04f * x + noise(rng);
    if (std::abs(x) > 1.2f && std::abs(y) < 0.8f) {
      z += 0.06f;
    }
    input.input_cloud_in_base.push_back({x, y, z});
    if (ceiling_coin(rng) > 0.985f) {
      input.input_cloud_in_base.push_back({x, y, 0.28f + noise(rng)});
    }
  }
  return input;
}

void PrintStats(const std::vector<double> &samples, int point_count) {
  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const double avg =
      std::accumulate(sorted.begin(), sorted.end(), 0.0) / std::max<size_t>(sorted.size(), 1U);
  const auto percentile = [&](double p) {
    const size_t idx = std::min(sorted.size() - 1,
                                static_cast<size_t>(std::floor(p * (sorted.size() - 1))));
    return sorted[idx];
  };
  std::cout << "points=" << point_count << " avg_ms=" << std::fixed << std::setprecision(2) << avg
            << " p95_ms=" << percentile(0.95) << " p99_ms=" << percentile(0.99) << '\n';
}

} // namespace

int main() {
  const auto config = MakeConfig();
  passable_area::core::Processor processor(config);
  for (const int point_count : {80000, 160000}) {
    std::vector<double> samples;
    samples.reserve(30);
    for (int frame = 0; frame < 30; ++frame) {
      auto input = MakeBenchmarkFrame(point_count, 100000000L * frame + 1L);
      const auto start = std::chrono::steady_clock::now();
      const auto output = processor.update(input);
      const double ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
      if (!output.valid) {
        std::cerr << "processor returned invalid output for benchmark frame\n";
        return 1;
      }
      samples.push_back(ms);
    }
    PrintStats(samples, point_count);
  }
  return 0;
}
