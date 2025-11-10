#ifndef PASSABLE_AREA_LAYER_PROCESSING_HPP
#define PASSABLE_AREA_LAYER_PROCESSING_HPP

#include "common.hpp"

#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>

#include <limits>
#include <string>

namespace dr {

float minCoeffOfFinites(const Eigen::MatrixXf &mat) noexcept;
float maxCoeffOfFinites(const Eigen::MatrixXf &mat) noexcept;

void fillMinValues(grid_map::GridMap &map, const std::string &layer_height,
                   const std::string &layer_filled);
void fillMinValuesLimited(grid_map::GridMap &map,
                          const std::string &layer_height,
                          const std::string &layer_filled, int max_hole_pixels,
                          bool enable_center_padding, float center_dist_thresh);
void fillMaxValues(grid_map::GridMap &map, const std::string &layer_height,
                   const std::string &layer_filled);
void fillMeanValues(grid_map::GridMap &map, const std::string &layer_height,
                    const std::string &layer_filled);
void fillMeanValuesOnce(grid_map::GridMap &map, const std::string &layer_height,
                        const std::string &layer_filled);

void applyMedianFilter(
    grid_map::GridMap &map, const std::string &layer_height, int kernel_size,
    float threshold = -std::numeric_limits<float>::infinity());
void applyGaussianFilter(grid_map::GridMap &map,
                         const std::string &layer_height, int kernel_size);

} // namespace dr

#endif // PASSABLE_AREA_LAYER_PROCESSING_HPP
