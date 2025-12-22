#include "layer_processing.hpp"

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <grid_map_cv/GridMapCvConverter.hpp>

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

namespace dr {

float minCoeffOfFinites(const Eigen::MatrixXf &mat) noexcept {
  return mat.array()
      .isFinite()
      .select(mat, std::numeric_limits<float>::max())
      .minCoeff();
}

float maxCoeffOfFinites(const Eigen::MatrixXf &mat) noexcept {
  return mat.array()
      .isFinite()
      .select(mat, std::numeric_limits<float>::lowest())
      .maxCoeff();
}

void fillMinValues(grid_map::GridMap &map, const std::string &layer_height,
                   const std::string &layer_filled) {
  grid_map::Matrix &H_ele = map.get(layer_height);
  grid_map::Matrix &H_pad = map.get(layer_filled);
  H_pad.setConstant(toFloat(Padding::Unpadded));

  const int num_cols = H_ele.cols();
  const int max_col_id = num_cols - 1;
  const int num_rows = H_ele.rows();
  const int max_row_id = num_rows - 1;

  auto compare_and_store_min = [](float new_value, float &current_min,
                                  bool &is_changed) {
    if (!std::isnan(new_value)) {
      if (new_value < current_min || std::isnan(current_min)) {
        current_min = new_value;
        is_changed = true;
      }
    }
  };

  bool has_at_least_one_value = true;
  bool is_changed = true;

  while (is_changed && has_at_least_one_value) {
    has_at_least_one_value = false;
    is_changed = false;

    for (int col_id = 0; col_id < num_cols; ++col_id) {
      for (int row_id = 0; row_id < num_rows; ++row_id) {
        if (std::isnan(H_ele(row_id, col_id))) {
          auto &middle_value = H_ele(row_id, col_id);
          auto &padding_sign = H_pad(row_id, col_id);

          if (col_id > 0)
            compare_and_store_min(H_ele(row_id, col_id - 1), middle_value,
                                  is_changed);
          if (col_id < max_col_id)
            compare_and_store_min(H_ele(row_id, col_id + 1), middle_value,
                                  is_changed);
          if (row_id > 0)
            compare_and_store_min(H_ele(row_id - 1, col_id), middle_value,
                                  is_changed);
          if (row_id < max_row_id)
            compare_and_store_min(H_ele(row_id + 1, col_id), middle_value,
                                  is_changed);

          if (!std::isnan(middle_value) && is_changed)
            padding_sign = toFloat(Padding::Padded);
        } else {
          has_at_least_one_value = true;
        }
      }
    }
  }
}

void fillMinValuesLimited(grid_map::GridMap &map,
                          const std::string &layer_height,
                          const std::string &layer_filled, int max_hole_pixels,
                          bool enable_center_padding,
                          float center_dist_thresh) {
  grid_map::Matrix &H_ele = map.get(layer_height);
  grid_map::Matrix &H_pad = map.get(layer_filled);
  H_pad.setConstant(toFloat(Padding::Unpadded));

  const int rows = H_ele.rows(), cols = H_ele.cols();
  std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

  auto inBounds = [&](int r, int c) {
    return r >= 0 && c >= 0 && r < rows && c < cols;
  };

  const int dr[4] = {-1, 1, 0, 0};
  const int dc[4] = {0, 0, -1, 1};

  const float r_center = rows / 2.0f;
  const float c_center = cols / 2.0f;

  struct Pixel {
    int r;
    int c;
  };

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (!std::isnan(H_ele(r, c)) || visited[r][c])
        continue;

      std::vector<Pixel> comp;
      std::queue<Pixel> que;
      que.push({r, c});
      visited[r][c] = true;
      float r_sum = 0.0f, c_sum = 0.0f;

      while (!que.empty()) {
        Pixel p = que.front();
        que.pop();
        comp.push_back(p);
        r_sum += static_cast<float>(p.r);
        c_sum += static_cast<float>(p.c);

        for (int k = 0; k < 4; ++k) {
          int nr = p.r + dr[k], nc = p.c + dc[k];
          if (inBounds(nr, nc) && !visited[nr][nc] &&
              std::isnan(H_ele(nr, nc))) {
            visited[nr][nc] = true;
            que.push({nr, nc});
          }
        }
      }

      const bool hole_too_big =
          comp.size() > static_cast<size_t>(max_hole_pixels);
      const bool allow_center_override =
          enable_center_padding && center_dist_thresh > 0.0f;

      auto cell_distance = [&](float r_idx, float c_idx) {
        const float dr = r_idx - r_center;
        const float dc = c_idx - c_center;
        return map.getResolution() * std::hypot(dr, dc);
      };

      auto canFill = [&](const Pixel &px) {
        if (!hole_too_big)
          return true;
        if (!allow_center_override)
          return false;
        return cell_distance(static_cast<float>(px.r),
                             static_cast<float>(px.c)) < center_dist_thresh;
      };

      if (hole_too_big && !allow_center_override)
        continue;

      bool has_fillable_cell = false;
      for (const auto &px : comp) {
        if (canFill(px)) {
          has_fillable_cell = true;
          break;
        }
      }

      if (!has_fillable_cell)
        continue;

      bool changed = true;
      for (int iter = 0; iter < 3 && changed; ++iter) {
        changed = false;
        for (auto &px : comp) {
          if (!canFill(px))
            continue;
          int i = px.r, j = px.c;
          float &center = H_ele(i, j);
          for (int k = 0; k < 4; ++k) {
            int ni = i + dr[k], nj = j + dc[k];
            if (!inBounds(ni, nj))
              continue;
            float nb = H_ele(ni, nj);
            if (!std::isnan(nb) && (std::isnan(center) || nb < center)) {
              center = nb;
              H_pad(i, j) = toFloat(Padding::Padded);
              changed = true;
            }
          }
        }
      }
    }
  }
}

void fillMaxValues(grid_map::GridMap &map, const std::string &layer_height,
                   const std::string &layer_filled) {
  grid_map::Matrix &H_ele = map.get(layer_height);
  grid_map::Matrix &H_pad = map.get(layer_filled);

  H_pad.setConstant(toFloat(Padding::Unpadded));

  const int num_cols = H_ele.cols();
  const int max_col_id = num_cols - 1;
  const int num_rows = H_ele.rows();
  const int max_row_id = num_rows - 1;

  auto compare_and_store_max = [](float new_value, float &current_max,
                                  bool &is_changed) {
    if (!std::isnan(new_value)) {
      if (new_value > current_max || std::isnan(current_max)) {
        current_max = new_value;
        is_changed = true;
      }
    }
  };

  bool has_at_least_one_value = true;
  bool is_changed = true;

  while (is_changed && has_at_least_one_value) {
    has_at_least_one_value = false;
    is_changed = false;

    for (int col_id = 0; col_id < num_cols; ++col_id) {
      for (int row_id = 0; row_id < num_rows; ++row_id) {
        if (std::isnan(H_ele(row_id, col_id))) {
          auto &middle_value = H_ele(row_id, col_id);

          if (col_id > 0)
            compare_and_store_max(H_ele(row_id, col_id - 1), middle_value,
                                  is_changed);
          if (col_id < max_col_id)
            compare_and_store_max(H_ele(row_id, col_id + 1), middle_value,
                                  is_changed);
          if (row_id > 0)
            compare_and_store_max(H_ele(row_id - 1, col_id), middle_value,
                                  is_changed);
          if (row_id < max_row_id)
            compare_and_store_max(H_ele(row_id + 1, col_id), middle_value,
                                  is_changed);
          if (!std::isnan(middle_value) && is_changed)
            H_pad(row_id, col_id) = toFloat(Padding::Padded);
        } else {
          has_at_least_one_value = true;
        }
      }
    }
  }
}

void fillMeanValues(grid_map::GridMap &map, const std::string &layer_height,
                    const std::string &layer_filled) {
  grid_map::Matrix &H_ele = map.get(layer_height);
  grid_map::Matrix &H_pad = map.get(layer_filled);

  H_pad.setConstant(toFloat(Padding::Unpadded));

  const int num_rows = H_ele.rows();
  const int num_cols = H_ele.cols();
  const int max_row_id = num_rows - 1;
  const int max_col_id = num_cols - 1;

  bool has_at_least_one_value = true;
  bool is_changed = true;

  while (is_changed && has_at_least_one_value) {
    has_at_least_one_value = false;
    is_changed = false;

    for (int col_id = 0; col_id < num_cols; ++col_id) {
      for (int row_id = 0; row_id < num_rows; ++row_id) {
        if (std::isnan(H_ele(row_id, col_id))) {
          auto &mean_value = H_ele(row_id, col_id);
          float sum = 0.0f;
          int count = 0;

          if (col_id > 0 && !std::isnan(H_ele(row_id, col_id - 1))) {
            sum += H_ele(row_id, col_id - 1);
            ++count;
          }
          if (col_id < max_col_id && !std::isnan(H_ele(row_id, col_id + 1))) {
            sum += H_ele(row_id, col_id + 1);
            ++count;
          }
          if (row_id > 0 && !std::isnan(H_ele(row_id - 1, col_id))) {
            sum += H_ele(row_id - 1, col_id);
            ++count;
          }
          if (row_id < max_row_id && !std::isnan(H_ele(row_id + 1, col_id))) {
            sum += H_ele(row_id + 1, col_id);
            ++count;
          }

          if (count > 0) {
            float new_value = sum / static_cast<float>(count);
            if (new_value != mean_value || std::isnan(mean_value)) {
              mean_value = new_value;
              H_pad(row_id, col_id) = toFloat(Padding::Padded);
              is_changed = true;
            }
          }
        } else {
          has_at_least_one_value = true;
        }
      }
    }
  }
}

void fillMeanValuesOnce(grid_map::GridMap &map, const std::string &layer_height,
                        const std::string &layer_filled) {
  grid_map::Matrix &H_ele = map.get(layer_height);
  grid_map::Matrix &H_pad = map.get(layer_filled);
  H_pad.setConstant(toFloat(Padding::Unpadded));

  const int num_rows = H_ele.rows();
  const int num_cols = H_ele.cols();
  const int center_row = num_rows / 2;
  const int center_col = num_cols / 2;

  std::vector<std::vector<bool>> visited(num_rows,
                                         std::vector<bool>(num_cols, false));
  std::queue<std::pair<int, int>> queue;
  queue.push({center_row, center_col});
  visited[center_row][center_col] = true;

  const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

  while (!queue.empty()) {
    auto [row, col] = queue.front();
    queue.pop();

    if (std::isnan(H_ele(row, col))) {
      auto &mean_value = H_ele(row, col);
      float sum = 0.0f;
      int count = 0;

      for (int i = 0; i < 8; ++i) {
        int new_row = row + dx[i];
        int new_col = col + dy[i];
        if (new_row >= 0 && new_row < num_rows && new_col >= 0 &&
            new_col < num_cols && visited[new_row][new_col]) {
          float value = H_ele(new_row, new_col);
          if (!std::isnan(value)) {
            sum += value;
            ++count;
          }
        }
      }

      if (count > 0) {
        float new_value = sum / static_cast<float>(count);
        if (std::fabs(new_value - mean_value) > 1e-3f ||
            std::isnan(mean_value)) {
          mean_value = new_value;
          H_pad(row, col) = toFloat(Padding::Padded);
        }
      }
    }

    for (int i = 0; i < 8; ++i) {
      int new_row = row + dx[i];
      int new_col = col + dy[i];
      if (new_row >= 0 && new_row < num_rows && new_col >= 0 &&
          new_col < num_cols && !visited[new_row][new_col]) {
        queue.push({new_row, new_col});
        visited[new_row][new_col] = true;
      }
    }
  }
}

void applyMedianFilter(grid_map::GridMap &map, const std::string &layer_height,
                       int kernel_size, float threshold) {
  cv::Mat elevation_image;
  cv::eigen2cv(map.get(layer_height), elevation_image);

  cv::Mat mask;
  if (threshold != -std::numeric_limits<float>::infinity()) {
    mask = elevation_image > threshold;
  }

  cv::Mat filtered_image;

  if (kernel_size <= 5) {
    cv::medianBlur(elevation_image, filtered_image, kernel_size);

    // Fix the issue where OpenCV generates NaN on ARM
    cv::Mat nan_mask = (filtered_image != filtered_image); // NaN != NaN
    elevation_image.copyTo(filtered_image, nan_mask);

    if (!mask.empty()) {
      elevation_image.copyTo(filtered_image, mask);
    }
  } else {
    const float min_value = minCoeffOfFinites(map.get(layer_height));
    const float max_value = maxCoeffOfFinites(map.get(layer_height));

    cv::Mat elevation_image_u8;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
        map, layer_height, CV_8UC1, min_value, max_value, elevation_image_u8);

    cv::Mat filtered_image_u8;
    cv::medianBlur(elevation_image_u8, filtered_image_u8, kernel_size);

    cv::Mat filtered_image_float;
    constexpr float max_uchar_value = 255.F;
    filtered_image_u8.convertTo(filtered_image_float, CV_32F,
                                (max_value - min_value) / max_uchar_value,
                                min_value);

    if (!mask.empty()) {
      elevation_image.copyTo(filtered_image_float, mask);
    }
    filtered_image = filtered_image_float;
  }

  cv::cv2eigen(filtered_image, map.get(layer_height));
}

void applyGaussianFilter(grid_map::GridMap &map,
                         const std::string &layer_height, int kernel_size) {
  cv::Mat layer_cv;
  cv::eigen2cv(map.get(layer_height), layer_cv);
  cv::GaussianBlur(layer_cv, layer_cv, cv::Size(kernel_size, kernel_size), 0.0);
  cv::cv2eigen(layer_cv, map.get(layer_height));
}

} // namespace dr
