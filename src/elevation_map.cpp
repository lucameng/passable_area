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
                         "coverability", "padding", "point_count",
                         "dummy_height", "slope", "roughness", "step_height",
                         "traversal_cost"}),
      map_length_(std::max(map_length, grid_s)),
      map_width_(std::max(map_width, grid_s)),
      min_height_(std::min(min_height, max_height)),
      max_height_(std::max(min_height, max_height)), grid_size_(grid_s),
      max_inpaint_pixels_(200),
      center_padding_enabled_(true), center_padding_radius_(0.8f),
      frame_(frame_id), logger_(logger), max_slope_deg_(40.0f),
      current_T_g2b_(Eigen::Affine3f::Identity()) {
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

void ElevationMap::setSolverParams(
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

void ElevationMap::setMaxSlopeDeg(float deg) noexcept {
  max_slope_deg_ = std::clamp(deg, 0.0f, 89.0f);
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

void ElevationMap::processPointCloud(
    const sensor_msgs::msg::PointCloud2 &ros_cloud, float rough_thres,
    float drop_thres, const Eigen::Affine3f &T_g2b, bool fill_blind) {
  current_T_g2b_ = T_g2b;
  setInputCloud(ros_cloud);
  cloud2Elevation();
  inpaint("elevation", "padding", Inpaint::MinLimit);
  denoise("elevation", Denoise::Median, 3);
  if (fill_blind) {
    fillElevationHoles("elevation", "padding", 1.2f, 10);
    // fillPointCloudFromLayer("elevation", "padding");
  }
  const int rough_kernel =
      std::clamp(2 * traversal_params_.terrain_sample_window + 1, 3, 7);
  judgePassability(rough_thres, drop_thres, rough_kernel);
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

bool ElevationMap::isPassable(float variance_error,
                              float roughness_thres) const {
  float square_thres = roughness_thres * roughness_thres;
  return (variance_error < square_thres);
}

void ElevationMap::judgePassability(float rough_thres, float drop_thres,
                                    int kernel_size) {
  kernel_size = std::clamp(kernel_size, 3, 5);
  const auto size = getSize();
  const int size_x = size.x();
  const int size_y = size.y();
  const auto &elevation_layer = get("elevation");
  const auto linear_index = [size_x](int x, int y) { return x + y * size_x; };
  const int center_x = size_x / 2;
  const int center_y = size_y / 2;
  const int nan_radius_cells =
      std::max(1, static_cast<int>(std::ceil(0.4f / grid_size_)));
  const int nan_min_cells =
      std::max(4, static_cast<int>(0.6f * nan_radius_cells * nan_radius_cells));
  const float drop_buffer = 0.02f;
  const float far_distance = 6.0f;
  std::vector<CliffState> cliff_cache(size_x * size_y, CliffState::Unknown);
  get("passability").setConstant(toFloat(Passability::Unknown));
  auto &rough_layer = get("roughness");
  auto &step_layer = get("step_height");
  rough_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
  step_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
  std::vector<bool> visited(size_x * size_y, false);
  std::queue<std::pair<int, int>> que;

  que.emplace(center_x, center_y);
  visited[linear_index(center_x, center_y)] = true;
  setPassability(Eigen::Array2i(center_x, center_y), Passability::Passable);

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

  auto update_step = [&](const grid_map::Index &idx, float diff) {
    if (idx.x() < 0 || idx.x() >= size_x || idx.y() < 0 || idx.y() >= size_y)
      return;
    float &slot = step_layer(idx.x(), idx.y());
    if (!std::isfinite(slot) || diff > slot)
      slot = diff;
  };

  assign_roughness(center_x, center_y);

  int back_x = std::min(static_cast<int>(center_x * 1.8f), size_x - 1);
  int back_y = center_y;
  auto back_idx = grid_map::Index(back_x, back_y);
  que.emplace(back_x, back_y);
  visited[linear_index(back_x, back_y)] = true;
  setPassability(Eigen::Array2i(back_x, back_y), Passability::Passable);
  assign_roughness(back_x, back_y);

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
        setPassability(curr_idx, Passability::Impassable);
        // if (isCliffCandidate(x, y, T_g2b, drop_thres, nan_radius_cells,
        //                      nan_min_cells, drop_buffer, far_distance,
        //                      cliff_cache)) {
        //   setPassability(curr_idx, Passability::Impassable);
        //   curr_is_cliff = true;
        // }
        continue;
      }
      if (visited[idx])
        continue;
      visited[idx] = true;

      float height_diff = nbr_height - curr_height;
      height_diff =
          std::fabs(projectToBodyZ(height_diff, current_T_g2b_.linear()));
      update_step(curr_idx, height_diff);
      update_step(nbr_idx, height_diff);
      if (height_diff > drop_thres) {
        setPassability(nbr_idx, Passability::Impassable);
        continue;
      }

      Eigen::Vector3f mean = Eigen::Vector3f::Zero();
      Eigen::Matrix3f square = Eigen::Matrix3f::Zero();
      auto e = computeRoughness(nx, ny, kernel_size, mean, square);
      rough_layer(nx, ny) = std::sqrt(std::max(0.0f, e));
      if (!isPassable(e, rough_thres) && curr_height > mean.z()) {
        setPassability(nbr_idx, Passability::Impassable);
        continue;
      }

      float slope_rad = computeSlopeRad(nx, ny, elevation_layer);
      float max_slope_rad = static_cast<float>(
          dr::degreeToRadian(static_cast<double>(max_slope_deg_)));
      if (slope_rad > max_slope_rad) {
        setPassability(nbr_idx, Passability::Impassable);
        continue;
      }

      setPassability(nbr_idx, Passability::Passable);
      assign_roughness(nx, ny);
      que.emplace(nx, ny);
    }
  }
}

bool ElevationMap::isCliffCandidate(
    int cx, int cy, const Eigen::Affine3f &T_g2b, float drop_thres,
    int nan_radius_cells, int nan_min_cells, float drop_buffer,
    float far_distance, std::vector<CliffState> &cliff_cache) const {
  const auto size = getSize();
  const int size_x = size.x();
  const int size_y = size.y();
  const int idx = cx + cy * size_x;
  if (cliff_cache[idx] != CliffState::Unknown) {
    return cliff_cache[idx] == CliffState::Cliff;
  }

  const auto &height_layer = get("elevation");
  const grid_map::Index center_idx(cx, cy);
  float center_height = height_layer(center_idx.x(), center_idx.y());
  if (!std::isfinite(center_height)) {
    cliff_cache[idx] = CliffState::Cliff;
    return true;
  }

  int nan_count = 0;
  int sample_count = 0;
  bool drop_detected = false;
  float min_neighbor_height = center_height;

  for (int dx = -nan_radius_cells; dx <= nan_radius_cells; ++dx) {
    for (int dy = -nan_radius_cells; dy <= nan_radius_cells; ++dy) {
      const int rx = cx + dx;
      const int ry = cy + dy;
      if (rx < 0 || rx >= size_x || ry < 0 || ry >= size_y)
        continue;
      ++sample_count;
      const float h = height_layer(rx, ry);
      if (std::isnan(h)) {
        ++nan_count;
        continue;
      }

      float diff = center_height - h;
      diff = std::fabs(projectToBodyZ(diff, current_T_g2b_.linear()));
      if (diff > drop_thres + drop_buffer) {
        drop_detected = true;
      }
      if (h < min_neighbor_height) {
        min_neighbor_height = h;
      }
    }
  }

  float nan_ratio =
      sample_count > 0 ? static_cast<float>(nan_count) / sample_count : 0.0f;

  grid_map::Position pos;
  getPosition(center_idx, pos);
  const float dist = std::hypot(pos.x(), pos.y());
  const float ratio_threshold = dist > far_distance ? 0.85f : 0.6f;

  const bool sufficient_nan =
      nan_count >= nan_min_cells && nan_ratio >= ratio_threshold;

  const bool drop_sufficient =
      drop_detected ||
      (center_height - min_neighbor_height) > (drop_thres + drop_buffer);

  const bool is_cliff = sufficient_nan && drop_sufficient;
  cliff_cache[idx] = is_cliff ? CliffState::Cliff : CliffState::NotCliff;
  return is_cliff;
}

void ElevationMap::fillElevationHoles(const std::string &layer_height,
                                      const std::string &layer_filled,
                                      float search_radius, int min_neighbors,
                                      const Eigen::Vector2f &bound_min,
                                      const Eigen::Vector2f &bound_max) {
  if (!exists(layer_height) || !exists(layer_filled))
    return;

  const grid_map::Matrix &H_in = get(layer_height);
  grid_map::Matrix H_out = H_in; // copy

  const int radius_cells =
      static_cast<int>(std::ceil(search_radius / grid_size_));

  const auto size = getSize();
  const int rows = size.x();
  const int cols = size.y();

#pragma omp parallel for collapse(2)
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (!std::isnan(H_in(r, c)))
        continue;

      grid_map::Position pos;
      getPosition({r, c}, pos);

      if (pos.x() < bound_min.x() || pos.x() > bound_max.x() ||
          pos.y() < bound_min.y() || pos.y() > bound_max.y()) {
        continue;
      }

      float sum = 0.0;
      float weight_sum = 0.0;
      int count = 0;

      for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
        for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
          int nr = r + dr;
          int nc = c + dc;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
            continue;

          float h = H_in(nr, nc);
          if (std::isnan(h))
            continue;

          float dist = std::sqrt(dr * dr + dc * dc) * grid_size_;
          if (dist > search_radius || dist < 1e-6)
            continue;

          float w = 1.0 / (dist * dist + 1e-6);
          sum += h * w;
          weight_sum += w;
          count++;
        }
      }

      if (count >= min_neighbors && weight_sum > 0.0) {
        H_out(r, c) = sum / weight_sum;
        grid_map::Index idx = {r, c};
        // at(layer_filled, idx) = toFloat(Padding::Padded);
      }
    }
  }
  add("dummy_height", H_out);
}

void ElevationMap::fillPointCloudFromLayer(const std::string &layer_height,
                                           const std::string &layer_filled) {
  if (!exists(layer_height) || !exists(layer_filled))
    return;

  auto &cloud = getWorkingCloud();

  for (grid_map::GridMapIterator it(*this); !it.isPastEnd(); ++it) {
    grid_map::Index idx(*it);
    if (at(layer_filled, idx) != toFloat(Padding::Padded))
      continue;
    grid_map::Position pos;
    getPosition(idx, pos);

    float z = at(layer_height, idx);

    pcl::PointXYZ p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = z;
    cloud.points.push_back(p);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
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
