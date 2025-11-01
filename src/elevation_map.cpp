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
                           const std::string &frame_id)
    : grid_map::GridMap({"elevation", "ground_height", "passability",
                         "coverability", "padding", "point_count",
                         "dummy_height"}),
      map_length_(std::max(map_length, grid_s)),
      map_width_(std::max(map_width, grid_s)),
      min_height_(std::min(min_height, max_height)),
      max_height_(std::max(min_height, max_height)), grid_size_(grid_s),
      frame_(frame_id) {
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
}

void ElevationMap::processPointCloud(
    const sensor_msgs::msg::PointCloud2 &ros_cloud, float rough_thres,
    float drop_thres, const Eigen::Affine3f &T_g2b, bool fill_blind) {
  setInputCloud(ros_cloud);
  cloud2Elevation();
  inpaint("elevation", "padding", Inpaint::MinLimit);
  denoise("elevation", Denoise::Median, 3);
  if (fill_blind) {
    fillElevationHoles("elevation", "padding", 1.2f, 10);
    // fillPointCloudFromLayer("elevation", "padding");
  }
  judgePassability(rough_thres, drop_thres, 3, T_g2b);
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
  clear("elevation");
  clear("ground_height");
  clear("point_count");
  get("point_count").setZero();

  const float half_length = map_length_ * 0.5f;
  const float half_width = map_width_ * 0.5f;

#pragma omp parallel for
  for (const auto &p : working_cloud_) {
    if (p.x < -half_length || p.x > half_length || p.y < -half_width ||
        p.y > half_width || p.z < min_height_ || p.z > max_height_)
      continue;

    Eigen::Vector2d pos(p.x, p.y);
    float height = getAltitude(pos);
    if (std::isnan(height) || p.z > height) {
      height = p.z;
      setAltitude(pos, height);
    }

    float ground = getGroundHeight(pos);
    if (std::isnan(ground) || p.z < ground) {
      setGroundHeight(pos, p.z);
    }

    grid_map::Index idx;
    if (getIndex(pos, idx)) {
      get("point_count")(idx.x(), idx.y()) += 1.0f;
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

float ElevationMap::errorFromCovariance(const Eigen::Vector3f &mean,
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

float ElevationMap::computeError(int x, int y, int kernel_size,
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
    return errorFromCovariance(mean, square);
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
    dr::fillMinValuesLimited(*this, layer_height, layer_filled, 20, true, 0.8f);
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
                                    int kernel_size,
                                    const Eigen::Affine3f &T_g2b) {
  kernel_size = std::clamp(kernel_size, 3, 5);
  const auto size = getSize();
  const int size_x = size.x();
  const int size_y = size.y();
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
  std::vector<bool> visited(size_x * size_y, false);
  std::queue<std::pair<int, int>> que;

  que.emplace(center_x, center_y);
  visited[linear_index(center_x, center_y)] = true;
  setPassability(Eigen::Array2i(center_x, center_y), Passability::Passable);

  int back_x = std::min(static_cast<int>(center_x * 1.8f), size_x - 1);
  int back_y = center_y;
  auto back_idx = grid_map::Index(back_x, back_y);
  que.emplace(back_x, back_y);
  visited[linear_index(back_x, back_y)] = true;
  setPassability(Eigen::Array2i(back_x, back_y), Passability::Passable);

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
        if (isCliffCandidate(x, y, T_g2b, drop_thres, nan_radius_cells,
                             nan_min_cells, drop_buffer, far_distance,
                             cliff_cache)) {
          setPassability(curr_idx, Passability::Impassable);
          curr_is_cliff = true;
        }
        continue;
      }
      if (visited[idx])
        continue;
      visited[idx] = true;

      float height_diff = nbr_height - curr_height;
      height_diff = std::fabs(projectToBodyZ(height_diff, T_g2b.linear()));
      if (height_diff > drop_thres) {
        setPassability(nbr_idx, Passability::Impassable);
        continue;
      }

      Eigen::Vector3f mean = Eigen::Vector3f::Zero();
      Eigen::Matrix3f square = Eigen::Matrix3f::Zero();
      auto e = computeError(nx, ny, kernel_size, mean, square);
      if (!isPassable(e, rough_thres) && curr_height > mean.z()) {
        setPassability(nbr_idx, Passability::Impassable);
        continue;
      }

      setPassability(nbr_idx, Passability::Passable);
      que.emplace(nx, ny);
    }
  }
}

bool ElevationMap::isCliffCandidate(int cx, int cy,
                                    const Eigen::Affine3f &T_g2b,
                                    float drop_thres, int nan_radius_cells,
                                    int nan_min_cells, float drop_buffer,
                                    float far_distance,
                                    std::vector<CliffState> &cliff_cache) const {
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
      diff = std::fabs(projectToBodyZ(diff, T_g2b.linear()));
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
