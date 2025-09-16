#ifndef ELEVATION_MAP_HPP
#define ELEVATION_MAP_HPP

#include <string>
#include <vector>
#include <deque>
#include <queue>
#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv2/core/eigen.hpp>

enum Passability : uint8_t
{
    PASSABLE = 0,
    IMPASSABLE = 1,
    UNKNOWN = 2
};

enum
{
    MEANONCE,
    MIN,
    MAX,
    CONDITIONAL,
    MEAN,
    MINLIMIT
};

enum
{
    MEDIAN,
    GAUSS
};

class elevationMap : public grid_map::GridMap
{
public:
    elevationMap() = default;
    elevationMap(float map_s, float grid_s, const std::string &frame_id);

    float getMinheight() const noexcept;
    float getMaxheight() const noexcept;

    void setAltitude(const Eigen::Array2i &id, float height);
    void setAltitude(const Eigen::Vector2d &pos, float height);

    float getAltitude(const Eigen::Array2i &id) const;
    float getAltitude(const Eigen::Vector2d &pos) const;

    void setPassability(const Eigen::Array2i &id, uint8_t passability);
    void setPassability(const Eigen::Vector2d &pos, uint8_t passability);

    uint8_t getPassability(const Eigen::Array2i &id) const;
    uint8_t getPassability(const Eigen::Vector2d &pos) const;

    void inPainting(const std::string &layer_in, int method);
    void deNoise(const std::string &layer_in, int method, int kernel_size);
    bool isPassable(float variance_error, float roughness_thres) const;
    void judgePassability(float roughness_thres, float drop_thres, int kernel_size);

    float getGridSize() const noexcept { return grid_size_; }
    int getMapSizeGrid() const noexcept { return map_size_grid_; }

private:
    float minCoeffOfFinites(const Eigen::MatrixXf &mat) const noexcept
    {
        return mat.array().isFinite().select(mat, std::numeric_limits<float>::max()).minCoeff();
    }

    float maxCoeffOfFinites(const Eigen::MatrixXf &mat) const noexcept
    {
        return mat.array().isFinite().select(mat, std::numeric_limits<float>::lowest()).maxCoeff();
    }

    bool isIndexValid(const Eigen::Array2i &id) const noexcept
    {
        return (id.x() >= 0 && id.y() >= 0 && id.x() < getSize().x() && id.y() < getSize().y());
    }

    bool isPositionInside(const Eigen::Vector2d &pos) const noexcept { return isInside(pos); }

    void minValues(const std::string &layer_in, const std::string &layer_out);
    void minValuesLimited(const std::string &layer_in, const std::string &layer_out,
                          int max_hole_pixels = 200);
    void maxValues(const std::string &layer_in, const std::string &layer_out);
    void meanValues(const std::string &layer_in, const std::string &layer_out);
    void meanValuesOnce(const std::string &layer_in, const std::string &layer_out);
    void medianFilter(const std::string &layer_in, const std::string &layer_out, int kernel_size,
                      float threshold = -std::numeric_limits<float>::infinity());
    void gaussianFilter(const std::string &layer_in, const std::string &layer_out, int kernel_size);
    float errorFromCovariance(const Eigen::Vector3f &mean, const Eigen::Matrix3f &square) const;
    float computeError(int x, int y, int kernel_size, Eigen::Vector3f &mean,
                       Eigen::Matrix3f &square) const;

private:
    float map_size_;
    float grid_size_;
    int map_size_grid_;
    std::string frame_;
    struct Pixel { int r, c; };
};

#endif // ELEVATION_MAP_HPP
