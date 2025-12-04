#include "elevation_map.hpp"
#include "layer_processing.hpp"
#include "maths.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

ElevationMap::ElevationMap(float map_length, float map_width, float min_height,
                           float max_height, float grid_s,
                           const std::string &frame_id,
                           const rclcpp::Logger &logger)
    : grid_map::GridMap({"elevation", "ground_height", "ceiling_height",
                         "clearance", "float_mask", "passability",
                         "coverability", "padding", "point_count", "slope",
                         "roughness", "step_height", "traversal_cost"}),
      map_length_(std::max(map_length, grid_s)),
      map_width_(std::max(map_width, grid_s)),
      min_height_(std::min(min_height, max_height)),
      max_height_(std::max(min_height, max_height)), grid_size_(grid_s),
      max_inpaint_pixels_(200), center_padding_enabled_(true),
      center_padding_radius_(0.8f), current_T_g2b_(Eigen::Affine3f::Identity()),
      frame_(frame_id), logger_(logger) {
  if ((max_height_ - min_height_) < grid_size_) {
    max_height_ = min_height_ + grid_size_;
  }
  setFrameId(frame_);
  setGeometry(grid_map::Length(map_length_, map_width_), grid_size_);
  map_cells_ = getSize();
  setPosition(grid_map::Position(0.0, 0.0));
  get("passability").setConstant(toFloat(Passability::Unknown));
  get("coverability").setConstant(toFloat(CoverageStatus::Covered));
  get("padding").setConstant(toFloat(Padding::Unpadded));
  get("point_count").setZero();
  get("ground_height").setConstant(std::numeric_limits<float>::quiet_NaN());
  get("ceiling_height").setConstant(std::numeric_limits<float>::quiet_NaN());
  get("clearance").setConstant(std::numeric_limits<float>::infinity());
  get("float_mask").setZero();
  get("slope").setConstant(std::numeric_limits<float>::quiet_NaN());
  get("roughness").setConstant(std::numeric_limits<float>::quiet_NaN());
  get("step_height").setConstant(std::numeric_limits<float>::quiet_NaN());
  get("traversal_cost").setConstant(std::numeric_limits<float>::quiet_NaN());

  solver_params_.min_points = 1;
  solver_params_.ceiling_window_bins = 2;
  solver_params_.ceiling_min_points = 1;
  solver_params_.gap_empty_bins = 1;
  solver_params_.gap_empty_count_threshold = 0;
  solver_params_.ground_min_count = 1;
  solver_params_.float_ratio_threshold = 0.6f;
  solver_params_.neighbor_min_support = 0;
  solver_params_.neighbor_height_tolerance = 0.25f;
}

void ElevationMap::setElevationSolverParams(
    const ElevationSolverParams &params) noexcept {
  solver_params_.use_histogram_solver = params.use_histogram_solver;
  solver_params_.histogram_bins = std::max(1, params.histogram_bins);
  solver_params_.region = params.region;
  auto &region = solver_params_.region;
  if (region.min_x > region.max_x)
    std::swap(region.min_x, region.max_x);
  if (region.min_y > region.max_y)
    std::swap(region.min_y, region.max_y);
}

void ElevationMap::setMaxInpaintPixels(int max_pixels) noexcept {
  max_inpaint_pixels_ = std::max(1, max_pixels);
}

void ElevationMap::setCenterPaddingParams(bool enabled, float radius) noexcept {
  center_padding_enabled_ = enabled;
  center_padding_radius_ = std::max(0.0f, radius);
}

void ElevationMap::setTraversalCostParams(
    const TraversalCostParams &params) noexcept {
  traversal_params_ = params;
  traversal_params_.terrain_sample_window =
      std::max(1, traversal_params_.terrain_sample_window);
  traversal_params_.max_cost =
      std::clamp(traversal_params_.max_cost, 0.0f, 254.0f);
  traversal_params_.hard_cost =
      std::clamp(traversal_params_.hard_cost, 0.0f, traversal_params_.max_cost);
  traversal_params_.easy_cost = std::clamp(traversal_params_.easy_cost, 0.0f,
                                           traversal_params_.hard_cost);
  const float weight_sum = traversal_params_.slope_weight +
                           traversal_params_.roughness_weight +
                           traversal_params_.step_weight;
  if (weight_sum <= 0.0f) {
    traversal_params_.slope_weight = 1.0f;
    traversal_params_.roughness_weight = 0.0f;
    traversal_params_.step_weight = 0.0f;
  }
  if (traversal_params_.curve_power <= 0.0f) {
    traversal_params_.curve_power = 1.0f;
  }
  traversal_params_.hard_cost =
      std::min(traversal_params_.hard_cost, traversal_params_.max_cost);
  traversal_params_.easy_cost =
      std::min(traversal_params_.easy_cost, traversal_params_.hard_cost);
  if (traversal_params_.slope_block_deg <= traversal_params_.slope_free_deg) {
    traversal_params_.slope_block_deg = traversal_params_.slope_free_deg + 1.0f;
  }
  if (traversal_params_.rough_block <= traversal_params_.rough_free) {
    traversal_params_.rough_block = traversal_params_.rough_free + 0.01f;
  }
  if (traversal_params_.step_block <= traversal_params_.step_free) {
    traversal_params_.step_block = traversal_params_.step_free + 0.01f;
  }
}

void ElevationMap::setRaycastParams(const RaycastParams &params) noexcept {
  raycast_params_ = params;
  raycast_params_.max_ray_distance =
      std::max(0.1f, raycast_params_.max_ray_distance);
  raycast_params_.max_nan_gap = std::max(0.0f, raycast_params_.max_nan_gap);
}

void ElevationMap::processPointCloud(
    const sensor_msgs::msg::PointCloud2 &ros_cloud,
    const PassabilityParams &pass_params, const Eigen::Affine3f &T_g2b) {
  current_T_g2b_ = T_g2b;
  setInputCloud(ros_cloud);
  cloud2Elevation();
  inpaint("elevation", "padding", Inpaint::MinLimit);
  denoise("elevation", Denoise::Median, 3);
  const int rough_kernel =
      std::clamp(2 * traversal_params_.terrain_sample_window + 1, 3, 7);
  judgePassability(pass_params.roughness_threshold, pass_params.drop_threshold,
                   pass_params.max_slope_deg, rough_kernel,
                   pass_params.treat_nan_as_stiff);
}

void ElevationMap::setInputCloud(
    const sensor_msgs::msg::PointCloud2 &ros_cloud) {
  try {
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromROSMsg(ros_cloud, temp_cloud);
    working_cloud_.swap(temp_cloud);
  } catch (const std::exception &e) {
    working_cloud_.clear();
  }
}

void ElevationMap::cloud2Elevation() {
  auto &elevation_layer = get("elevation");
  auto &ground_layer = get("ground_height");
  auto &ceiling_layer = get("ceiling_height");
  auto &clearance_layer = get("clearance");
  auto &float_mask_layer = get("float_mask");
  auto &point_count_layer = get("point_count");

  elevation_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
  ground_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
  ceiling_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
  clearance_layer.setConstant(std::numeric_limits<float>::infinity());
  float_mask_layer.setZero();
  point_count_layer.setZero();

  const auto size = getSize();
  const int rows = size.x();
  const int cols = size.y();
  const int bins = std::max(1, solver_params_.histogram_bins);

  std::vector<uint16_t> histogram(rows * cols * bins, 0);
  std::vector<float> hist_max(rows * cols * bins,
                              -std::numeric_limits<float>::infinity());

  const float half_length = map_length_ * 0.5f;
  const float half_width = map_width_ * 0.5f;
  const float bin_width =
      (max_height_ - min_height_) / static_cast<float>(bins);

  if (bin_width <= 0.0f) {
    RCLCPP_WARN(logger_, "Invalid bin width %.3f for vertical structure",
                bin_width);
    return;
  }

  for (const auto &p : working_cloud_) {
    if (p.x < -half_length || p.x > half_length || p.y < -half_width ||
        p.y > half_width || p.z < min_height_ || p.z > max_height_)
      continue;

    Eigen::Vector2d pos(p.x, p.y);
    grid_map::Index idx;
    if (!getIndex(pos, idx))
      continue;

    const int linear = idx.x() * cols + idx.y();
    const int offset = linear * bins;

    point_count_layer(idx.x(), idx.y()) += 1.0f;

    int bin = static_cast<int>((p.z - min_height_) / bin_width);
    bin = std::clamp(bin, 0, bins - 1);

    const int slot = offset + bin;
    auto &bin_count = histogram[slot];
    if (bin_count < std::numeric_limits<uint16_t>::max()) {
      bin_count += 1;
    }
    auto &bin_peak = hist_max[slot];
    if (p.z > bin_peak) {
      bin_peak = p.z;
    }
  }

  ElevationSolverContext ctx_solver;
  ctx_solver.rows = rows;
  ctx_solver.cols = cols;
  ctx_solver.bins = bins;
  ctx_solver.min_height = min_height_;
  ctx_solver.bin_width = bin_width;
  ctx_solver.counts = &histogram;
  ctx_solver.peaks = &hist_max;
  std::vector<uint8_t> region_mask;
  const auto &region = solver_params_.region;
  if (region.enabled) {
    region_mask.resize(static_cast<std::size_t>(rows) *
                           static_cast<std::size_t>(cols),
                       static_cast<uint8_t>(0));
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        grid_map::Position pos;
        getPosition({r, c}, pos);
        if (pos.x() >= region.min_x && pos.x() <= region.max_x &&
            pos.y() >= region.min_y && pos.y() <= region.max_y) {
          const std::size_t idx =
              static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
              static_cast<std::size_t>(c);
          region_mask[idx] = static_cast<uint8_t>(1);
        }
      }
    }
    ctx_solver.region_mask = &region_mask;
  }

  auto solver_params = solver_params_;
  auto solver_result =
      ElevationSolver::solve(ctx_solver, solver_params, logger_);

  const std::size_t expected_cells =
      static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  if (solver_result.ground.size() != expected_cells ||
      solver_result.ceiling.size() != expected_cells ||
      solver_result.clearance.size() != expected_cells ||
      solver_result.float_mask.size() != expected_cells) {
    RCLCPP_WARN(logger_,
                "ElevationSolver returned mismatched result size. "
                "ground=%zu ceiling=%zu clearance=%zu mask=%zu expected=%zu",
                solver_result.ground.size(), solver_result.ceiling.size(),
                solver_result.clearance.size(), solver_result.float_mask.size(),
                expected_cells);
    return;
  }

  const auto &ground_values = solver_result.ground;
  const auto &ceiling_values = solver_result.ceiling;
  const auto &clearance_values = solver_result.clearance;
  const auto &float_mask = solver_result.float_mask;

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const std::size_t idx =
          static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
          static_cast<std::size_t>(c);
      const float ground_z = ground_values[idx];
      const float ceiling_z = ceiling_values[idx];
      const float clearance = clearance_values[idx];
      const uint8_t mask = float_mask[idx];

      if (std::isfinite(ground_z)) {
        ground_layer(r, c) = ground_z;
      }

      if (std::isfinite(ceiling_z)) {
        ceiling_layer(r, c) = ceiling_z;
      } else {
        ceiling_layer(r, c) = std::numeric_limits<float>::quiet_NaN();
      }

      if (mask == 0 && std::isfinite(ceiling_z)) {
        elevation_layer(r, c) = ceiling_z;
      } else if (std::isfinite(ground_z)) {
        elevation_layer(r, c) = ground_z;
      }

      clearance_layer(r, c) = clearance;
      float_mask_layer(r, c) = static_cast<float>(mask);
    }
  }
}

float ElevationMap::getMinheight() const noexcept {
  return dr::minCoeffOfFinites(get("elevation"));
}

float ElevationMap::getMaxheight() const noexcept {
  return dr::maxCoeffOfFinites(get("elevation"));
}

void ElevationMap::setAltitude(const Eigen::Array2i &idx, float height) {
  if (!isIndexValid(idx))
    return;
  at("elevation", idx) = height;
}

void ElevationMap::setAltitude(const Eigen::Vector2d &pos, float height) {
  if (!isPositionInside(pos))
    return;
  atPosition("elevation", pos) = height;
}

void ElevationMap::setGroundHeight(const Eigen::Array2i &idx, float height) {
  if (!isIndexValid(idx))
    return;
  at("ground_height", idx) = height;
}

void ElevationMap::setGroundHeight(const Eigen::Vector2d &pos, float height) {
  if (!isPositionInside(pos))
    return;
  atPosition("ground_height", pos) = height;
}

float ElevationMap::getAltitude(const Eigen::Array2i &idx) const {
  if (!isIndexValid(idx))
    return NAN;
  return at("elevation", grid_map::Index(idx.x(), idx.y()));
}

float ElevationMap::getAltitude(const Eigen::Vector2d &pos) const {
  if (!isPositionInside(pos))
    return NAN;
  return atPosition("elevation", pos);
}

float ElevationMap::getGroundHeight(const Eigen::Array2i &idx) const {
  if (!isIndexValid(idx))
    return NAN;
  return at("ground_height", grid_map::Index(idx.x(), idx.y()));
}

float ElevationMap::getGroundHeight(const Eigen::Vector2d &pos) const {
  if (!isPositionInside(pos))
    return NAN;
  return atPosition("ground_height", pos);
}

float ElevationMap::getBaselineGround(float radius) const {
  const auto size = getSize();
  const int rows = size.x();
  const int cols = size.y();
  const int center_r = rows / 2;
  const int center_c = cols / 2;

  const int max_radius = static_cast<int>(std::floor(radius / grid_size_));
  float sum = 0.0f;
  int count = 0;

  auto accumulate_ring = [&](int r_min, int r_max) {
    for (int dr = -r_max; dr <= r_max; ++dr) {
      for (int dc = -r_max; dc <= r_max; ++dc) {
        if (std::max(std::abs(dr), std::abs(dc)) > r_max ||
            std::max(std::abs(dr), std::abs(dc)) < r_min)
          continue;

        const int rr = center_r + dr;
        const int cc = center_c + dc;
        if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
          continue;

        const float ground = at("ground_height", grid_map::Index(rr, cc));
        if (!std::isnan(ground)) {
          sum += ground;
          ++count;
        }
      }
    }
  };

  for (int r = 0; r <= max_radius; ++r) {
    accumulate_ring(r == 0 ? 0 : r, r);
    if (count > 0)
      break;
  }

  if (count == 0)
    return std::numeric_limits<float>::quiet_NaN();

  return sum / static_cast<float>(count);
}

void ElevationMap::setPassability(const Eigen::Array2i &idx,
                                  Passability passability) {
  if (!isIndexValid(idx))
    return;
  at("passability", grid_map::Index(idx.x(), idx.y())) = toFloat(passability);
}

void ElevationMap::setPassability(const Eigen::Vector2d &pos,
                                  Passability passability) {
  if (!isPositionInside(pos))
    return;
  atPosition("passability", pos) = toFloat(passability);
}

Passability ElevationMap::getPassability(const Eigen::Array2i &idx) const {
  if (!isIndexValid(idx))
    return Passability::Unknown;
  return toPassability(at("passability", grid_map::Index(idx.x(), idx.y())));
}

Passability ElevationMap::getPassability(const Eigen::Vector2d &pos) const {
  if (!isPositionInside(pos))
    return Passability::Unknown;
  return toPassability(atPosition("passability", pos));
}

CoverageStatus ElevationMap::getCoverability(const Eigen::Array2i &idx) const {
  if (!isIndexValid(idx))
    return CoverageStatus::Uncovered;
  return toCoverageStatus(
      at("coverability", grid_map::Index(idx.x(), idx.y())));
}

CoverageStatus ElevationMap::getCoverability(const Eigen::Vector2d &pos) const {
  if (!isPositionInside(pos))
    return CoverageStatus::Uncovered;
  return toCoverageStatus(atPosition("coverability", pos));
}

int ElevationMap::getPointCount(const Eigen::Array2i &idx) const {
  if (!isIndexValid(idx))
    return 0;
  return static_cast<int>(at("point_count", grid_map::Index(idx.x(), idx.y())));
}

int ElevationMap::getPointCount(const Eigen::Vector2d &pos) const {
  if (!isPositionInside(pos))
    return 0;
  return static_cast<int>(atPosition("point_count", pos));
}

float ElevationMap::extractVariance(const Eigen::Vector3f &mean,
                                    const Eigen::Matrix3f &square) const {
  const Eigen::Matrix3f covari_matrix = square - mean * mean.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
  solver.computeDirect(covari_matrix,
                       Eigen::DecompositionOptions::ComputeEigenvectors);

  if (solver.eigenvalues()(0) > 1e-8) {
    float square_error =
        (solver.eigenvalues()(0) > 0.0) ? solver.eigenvalues()(0) : 0.0;
    return square_error;
  } else {
    return 0.0;
  }
}

float ElevationMap::computeRoughness(int x, int y, int kernel_size,
                                     Eigen::Vector3f &mean,
                                     Eigen::Matrix3f &square) const {
  int N = 0;
  int bias = (kernel_size - 1) / 2;
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  Eigen::Matrix3f sum_square = Eigen::Matrix3f::Zero();

  for (int i = x - bias; i <= x + bias; ++i) {
    for (int j = y - bias; j <= y + bias; ++j) {
      if (i < 0 || i >= getSize().x() || j < 0 || j >= getSize().y())
        continue;
      float height = getAltitude(grid_map::Index(i, j));
      if (!std::isfinite(height))
        continue;

      Eigen::Vector3f p(i * grid_size_, j * grid_size_, height);
      sum += p;
      sum_square.noalias() += p * p.transpose();
      ++N;
    }
  }

  if (N < 3)
    return 0.0f;
  else {
    mean = sum / N;
    square = sum_square / N;
    return extractVariance(mean, square);
  }
}

void ElevationMap::inpaint(const std::string &layer_height,
                           const std::string &layer_filled, Inpaint method) {
  switch (method) {
  case Inpaint::MeanOnce:
    dr::fillMeanValuesOnce(*this, layer_height, layer_filled);
    break;
  case Inpaint::Min:
    dr::fillMinValues(*this, layer_height, layer_filled);
    break;
  case Inpaint::MinLimit:
    dr::fillMinValuesLimited(*this, layer_height, layer_filled,
                             max_inpaint_pixels_, center_padding_enabled_,
                             center_padding_radius_);
    break;
  case Inpaint::Max:
    dr::fillMaxValues(*this, layer_height, layer_filled);
    break;
  case Inpaint::Mean:
    dr::fillMeanValues(*this, layer_height, layer_filled);
    break;
  default:
    break;
  }
}

void ElevationMap::denoise(const std::string &layer_height, Denoise method,
                           int kernel_size) {
  kernel_size = std::clamp(kernel_size, 1, 5);

  if (method == Denoise::Median) {
    dr::applyMedianFilter(*this, layer_height, kernel_size, -0.1f);
  } else if (method == Denoise::Gauss) {
    dr::applyGaussianFilter(*this, layer_height, kernel_size);
  }
}

bool ElevationMap::isPassable(float roughness_value,
                              float roughness_thres) const {
  return roughness_value < roughness_thres;
}

void ElevationMap::judgePassability(float rough_thres, float drop_thres,
                                    float max_slope_deg, int kernel_size,
                                    bool treat_nan_as_stiff) {
  kernel_size = std::clamp(kernel_size, 3, 5);
  const auto size = getSize();
  const int size_x = size.x();
  const int size_y = size.y();
  get("passability").setConstant(toFloat(Passability::Unknown));
  const auto &elevation_layer = get("elevation");
  auto &slope_layer = get("slope");
  auto &rough_layer = get("roughness");
  auto &step_layer = get("step_height");
  rough_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
  step_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
  slope_layer.setConstant(std::numeric_limits<float>::quiet_NaN());

  const auto linear_index = [size_x](int x, int y) { return x + y * size_x; };
  auto assign_roughness = [&](int cell_x, int cell_y) {
    if (cell_x < 0 || cell_x >= size_x || cell_y < 0 || cell_y >= size_y)
      return;
    float &slot = rough_layer(cell_x, cell_y);
    if (std::isfinite(slot))
      return;
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    Eigen::Matrix3f square = Eigen::Matrix3f::Zero();
    float variance =
        computeRoughness(cell_x, cell_y, kernel_size, mean, square);
    slot = std::sqrt(std::max(0.0f, variance));
  };
  auto assign_slope = [&](int cell_x, int cell_y) {
    if (cell_x < 0 || cell_x >= size_x || cell_y < 0 || cell_y >= size_y)
      return;
    float &slot = slope_layer(cell_x, cell_y);
    if (std::isfinite(slot))
      return;
    slot = computeSlopeRad(cell_x, cell_y, elevation_layer);
  };
  auto assign_step = [&](int cell_x, int cell_y,
                         float diff = std::numeric_limits<float>::quiet_NaN()) {
    if (cell_x < 0 || cell_x >= size_x || cell_y < 0 || cell_y >= size_y)
      return;
    float &slot = step_layer(cell_x, cell_y);
    if (std::isfinite(diff)) {
      if (!std::isfinite(slot) || diff > slot)
        slot = diff;
      return;
    }
    if (std::isfinite(slot))
      return;
    slot = computeStepHeight(cell_x, cell_y, elevation_layer);
  };

  std::vector<bool> visited(size_x * size_y, false);
  std::queue<std::pair<int, int>> que;
  auto enqueue_seed_index= [&](int cell_x, int cell_y) {
    if (cell_x < 0 || cell_x >= size_x || cell_y < 0 || cell_y >= size_y)
      return;
    if (visited[linear_index(cell_x, cell_y)])
      return;
    que.emplace(cell_x, cell_y);
    visited[linear_index(cell_x, cell_y)] = true;
    setPassability(Eigen::Array2i(cell_x, cell_y), Passability::Passable);
    assign_roughness(cell_x, cell_y);
    assign_slope(cell_x, cell_y);
    assign_step(cell_x, cell_y);
  };
  auto enqueue_seed_position = [&](float pos_x, float pos_y) {
    grid_map::Position pos(pos_x, pos_y);
    grid_map::Index idx;
    if (!getIndex(pos, idx))
      return;
    enqueue_seed_index(idx.x(), idx.y());
  };

  // the starting point of flood-filling (BFS)
  const int center_x = size_x / 2;
  const int center_y = size_y / 2;
  enqueue_seed_index(center_x, center_y);
  // manually add a point located at the front
  int front_x = std::min(static_cast<int>(center_x * 0.3f), size_x - 1);
  int front_y = std::min(static_cast<int>(center_y * 0.8f), size_y - 1);
  enqueue_seed_index(front_x, front_y);
  // manually add a point located at the back
  int back_x = std::min(static_cast<int>(center_x * 1.7f), size_x - 1);
  int back_y = std::min(static_cast<int>(center_y * 1.2f), size_y - 1);
  enqueue_seed_index(back_x, back_y);
  // manually add points at metric positions relative to map center
  enqueue_seed_position(0.4f, 0.0f);
  enqueue_seed_position(-0.4f, 0.0f);
  enqueue_seed_position(0.0f, 1.0f);
  enqueue_seed_position(0.0f, -1.0f);

  const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy[8] = {0, -1, -1, -1, 0, 1, 1, 1};
  const int n_dir = 8;

  while (!que.empty()) {
    auto [x, y] = que.front();
    que.pop();
    auto curr_idx = grid_map::Index(x, y);
    float curr_height = getAltitude(curr_idx);
    if (!std::isfinite(curr_height)) {
      curr_height = GROUD_HEIGHT;
      setAltitude(curr_idx, curr_height);
    }

    bool curr_is_cliff = false;
    for (int i = 0; i < n_dir && !curr_is_cliff; ++i) {
      int nx = x + dx[i], ny = y + dy[i];
      if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y)
        continue;
      auto nbr_idx = grid_map::Index(nx, ny);

      int idx = nx + ny * size_x;
      auto nbr_height = getAltitude(nbr_idx);

      if (std::isnan(nbr_height)) {
        visited[idx] = true;
        if (treat_nan_as_stiff) {
          if (raycast_params_.enable) {
            if (hasCliffDropOnRay(nbr_idx, elevation_layer, drop_thres)) {
              setPassability(curr_idx, Passability::Impassable);
            }
          } else {
            setPassability(curr_idx, Passability::Impassable);
          }
          curr_is_cliff = true;
        }
        continue;
      }
      if (visited[idx])
        continue;
      visited[idx] = true;

      float height_diff = nbr_height - curr_height;
      height_diff =
          std::fabs(projectToBodyZ(height_diff, current_T_g2b_.linear()));
      assign_step(x, y, height_diff);
      assign_step(nx, ny, height_diff);
      assign_roughness(nx, ny);
      assign_slope(nx, ny);

      if (height_diff > drop_thres) {
        setPassability(nbr_idx, Passability::Impassable);
        // for negative obstacles, adopt stricter strategies
        if (nbr_height < curr_height) {
          setPassability(curr_idx, Passability::Impassable);
        }
        continue;
      }

      float roughness = rough_layer(nx, ny);
      if (!isPassable(roughness, rough_thres)) {
        setPassability(nbr_idx, Passability::Impassable);
        continue;
      }

      float slope_rad = slope_layer(nx, ny);
      float max_slope_rad = static_cast<float>(
          dr::degreeToRadian(static_cast<double>(max_slope_deg)));
      if (slope_rad > max_slope_rad) {
        setPassability(nbr_idx, Passability::Impassable);
        continue;
      }

      setPassability(nbr_idx, Passability::Passable);
      que.emplace(nx, ny);
    }
  }
}

std::optional<std::reference_wrapper<const LidarParams>>
ElevationMap::selectRaycastLidar(const Eigen::Vector3f &target_body) const {
  if (raycast_lidars_.empty())
    return std::nullopt;

  const LidarParams *front = nullptr;
  const LidarParams *rear = nullptr;

  for (const auto &lidar : raycast_lidars_) {
    if (lidar.name.find("front") != std::string::npos) {
      front = &lidar;
    } else if (lidar.name.find("rear") != std::string::npos) {
      rear = &lidar;
    }
  }

  if (target_body.x() >= 0.0f && front)
    return std::cref(*front);
  if (target_body.x() < 0.0f && rear)
    return std::cref(*rear);
  return std::cref(raycast_lidars_.front());
}

bool ElevationMap::hasCliffDropOnRay(const grid_map::Index &target_idx,
                                     const grid_map::Matrix &elevation,
                                     float drop_thres) const {
  if (raycast_lidars_.empty())
    return false;

  grid_map::Position target_pos;
  getPosition(target_idx, target_pos);

  Eigen::Vector3f target_grav(target_pos.x(), target_pos.y(), 0.0f);
  Eigen::Vector3f target_body = current_T_g2b_ * target_grav;

  auto lidar_opt = selectRaycastLidar(target_body);
  if (!lidar_opt.has_value())
    return false;
  const LidarParams &lidar = lidar_opt->get();

  Eigen::Vector3f sensor_grav = current_T_g2b_.inverse() * lidar.pos_body;
  Eigen::Vector2f sensor_xy(sensor_grav.x(), sensor_grav.y());
  Eigen::Vector2f target_xy(target_pos.x(), target_pos.y());

  Eigen::Vector2f dir = target_xy - sensor_xy;
  float dist_to_target = dir.norm();
  if (dist_to_target < 1e-3f)
    return false;
  dir /= dist_to_target;

  const float step_len = grid_size_;
  const float max_ray = std::min(raycast_params_.max_ray_distance,
                                 0.5f * std::min(map_length_, map_width_));
  const int max_steps =
      std::max(1, static_cast<int>(std::ceil(max_ray / step_len)));
  const int target_step =
      std::max(1, static_cast<int>(std::lround(dist_to_target / step_len)));
  const int step_tolerance = 1; // allow +/-1 step around target

  float near_height = std::numeric_limits<float>::quiet_NaN();
  float far_height = std::numeric_limits<float>::quiet_NaN();
  int gap_steps = 0;
  bool gap_started = false;
  int gap_start_step = -1;
  int gap_end_step = -1;

  grid_map::Index sample_idx;

  for (int s = 1; s <= max_steps; ++s) {
    Eigen::Vector2f sample_xy = sensor_xy + dir * (s * step_len);
    grid_map::Position sample_pos(sample_xy.x(), sample_xy.y());
    if (!getIndex(sample_pos, sample_idx))
      break;

    float h = elevation(sample_idx.x(), sample_idx.y());

    if (!std::isfinite(h)) {
      gap_steps = gap_started ? gap_steps + 1 : 1;
      gap_started = true;
      if (gap_start_step < 0)
        gap_start_step = s;
      gap_end_step = s;
      if (gap_steps * step_len > raycast_params_.max_nan_gap) {
        return false;
      }
      // if we have passed the target window and are not in a covering gap,
      // early exit
      if (s > target_step + step_tolerance &&
          gap_end_step < target_step - step_tolerance)
        return false;
      continue;
    }

    // finite height
    if (!gap_started) {
      near_height = h;
      // if we already stepped past the target without hitting a gap, stop
      if (s > target_step + step_tolerance)
        break;
      continue;
    }

    // gap has ended at step s-1
    const bool gap_covers_target =
        (gap_start_step <= (target_step + step_tolerance)) &&
        (gap_end_step >= (target_step - step_tolerance));
    if (!gap_covers_target) {
      // reset and keep using this finite as the latest near
      gap_started = false;
      gap_steps = 0;
      gap_start_step = -1;
      gap_end_step = -1;
      near_height = h;
      // if we've passed target window, stop searching
      if (s > target_step + step_tolerance)
        break;
      continue;
    }

    // gap covers target, current finite is far side
    if (!std::isfinite(near_height)) {
      near_height = GROUD_HEIGHT; // assume ground if ray starts in NaN
    }
    far_height = h;
    break;
  }

  if (!std::isfinite(near_height) || !std::isfinite(far_height) ||
      gap_steps == 0) {
    return false;
  }

  float drop_body =
      projectToBodyZ(near_height - far_height, current_T_g2b_.linear());
  return drop_body > drop_thres;
}

float ElevationMap::computeSlopeRad(int row, int col,
                                    const grid_map::Matrix &elevation) const {
  const auto size = getSize();
  const int rows = size.x();
  const int cols = size.y();
  const int kernel =
      std::clamp(2 * traversal_params_.terrain_sample_window + 1, 3, 7);
  const int radius = kernel / 2;

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  int count = 0;
  for (int dr = -radius; dr <= radius; ++dr) {
    for (int dc = -radius; dc <= radius; ++dc) {
      const int rr = row + dr;
      const int cc = col + dc;
      if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
        continue;
      const float z = elevation(rr, cc);
      if (!std::isfinite(z))
        continue;
      Eigen::Vector3f p(static_cast<float>(rr) * grid_size_,
                        static_cast<float>(cc) * grid_size_, z);
      mean += p;
      ++count;
    }
  }

  if (count < 3)
    return 0.0f;

  mean /= static_cast<float>(count);

  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (int dr = -radius; dr <= radius; ++dr) {
    for (int dc = -radius; dc <= radius; ++dc) {
      const int rr = row + dr;
      const int cc = col + dc;
      if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
        continue;
      const float z = elevation(rr, cc);
      if (!std::isfinite(z))
        continue;
      Eigen::Vector3f p(static_cast<float>(rr) * grid_size_,
                        static_cast<float>(cc) * grid_size_, z);
      Eigen::Vector3f centered = p - mean;
      cov.noalias() += centered * centered.transpose();
    }
  }

  cov /= static_cast<float>(std::max(1, count - 1));

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
  solver.compute(cov);
  Eigen::Vector3f normal = solver.eigenvectors().col(0);
  if (!normal.allFinite() || normal.norm() < 1e-6f)
    return 0.0f;
  normal.normalize();

  float cos_theta = std::abs(normal.dot(Eigen::Vector3f::UnitZ()));
  cos_theta = std::clamp(cos_theta, 0.0f, 1.0f);
  float slope_rad = std::acos(cos_theta);
  return slope_rad;
}

float ElevationMap::normalizeMetric(float value, float free_threshold,
                                    float block_threshold) const noexcept {
  if (block_threshold <= free_threshold) {
    return value > free_threshold ? 1.0f : 0.0f;
  }
  const float normalized =
      (value - free_threshold) / (block_threshold - free_threshold);
  return std::clamp(normalized, 0.0f, 1.0f);
}

float ElevationMap::computeStepHeight(int row, int col,
                                      const grid_map::Matrix &elevation) const {
  const auto size = getSize();
  const int rows = size.x();
  const int cols = size.y();
  const float center = elevation(row, col);
  if (!std::isfinite(center))
    return std::numeric_limits<float>::quiet_NaN();

  float max_diff = 0.0f;
  bool has_neighbor = false;
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      if (dr == 0 && dc == 0)
        continue;
      const int rr = row + dr;
      const int cc = col + dc;
      if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
        continue;
      const float nh = elevation(rr, cc);
      if (!std::isfinite(nh))
        continue;
      has_neighbor = true;
      const float diff =
          std::fabs(projectToBodyZ(center - nh, current_T_g2b_.linear()));
      if (diff > max_diff)
        max_diff = diff;
    }
  }

  if (!has_neighbor)
    return std::numeric_limits<float>::quiet_NaN();
  return max_diff;
}
