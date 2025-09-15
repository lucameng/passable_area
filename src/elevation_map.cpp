#include "elevation_map.hpp"

#include <ros/ros.h>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

elevationMap::elevationMap(float map_s, float grid_s, const std::string &frame_id)
    : grid_map::GridMap({"elevation", "passability"}),
      map_size_(map_s),
      grid_size_(grid_s),
      map_size_grid_(map_size_ / grid_size_),
      frame_(frame_id)
{
    setFrameId(frame_);
    setGeometry(grid_map::Length(map_size_, map_size_), grid_size_);
    setPosition(grid_map::Position(0.0, 0.0));
    get("passability").setConstant(UNKNOWN);
}

float elevationMap::getMinheight() const noexcept 
{ 
    return minCoeffOfFinites(get("elevation")); 
}

float elevationMap::getMaxheight() const noexcept 
{ 
    return maxCoeffOfFinites(get("elevation")); 
}

void elevationMap::setAltitude(const Eigen::Array2i &id, float height)
{
    if (!isIndexValid(id)) return;
    at("elevation", id) = height;
}

void elevationMap::setAltitude(const Eigen::Vector2d &pos, float height)
{
    if (!isPositionInside(pos)) return;
    atPosition("elevation", pos) = height;
}

float elevationMap::getAltitude(const Eigen::Array2i &id) const
{
    if (!isIndexValid(id)) return NAN;
    return at("elevation", grid_map::Index(id.x(), id.y()));
}

float elevationMap::getAltitude(const Eigen::Vector2d &pos) const
{
    if (!isPositionInside(pos)) return NAN;
    return atPosition("elevation", pos);
}

void elevationMap::setPassability(const Eigen::Array2i &id, uint8_t passability)
{
    if (!isIndexValid(id)) return;
    at("passability", grid_map::Index(id.x(), id.y())) = static_cast<float>(passability);
}

void elevationMap::setPassability(const Eigen::Vector2d &pos, uint8_t passability)
{
    if (!isPositionInside(pos)) return;
    atPosition("passability", pos) = static_cast<float>(passability);
}

uint8_t elevationMap::getPassability(const Eigen::Array2i &id) const
{
    if (!isIndexValid(id)) return UNKNOWN;
    return static_cast<uint8_t>(at("passability", grid_map::Index(id.x(), id.y())));
}

uint8_t elevationMap::getPassability(const Eigen::Vector2d &pos) const
{
    if (!isPositionInside(pos)) return UNKNOWN;
    return static_cast<uint8_t>(atPosition("passability", pos));
}

void elevationMap::minValues(const std::string &layer_in, const std::string &layer_out)
{
    if (!exists(layer_out))
    {
        add(layer_out, get(layer_in));
    }

    const grid_map::Matrix &H_in = get(layer_in);
    grid_map::Matrix &H_out = get(layer_out);
    H_out = H_in;

    const int num_cols = H_in.cols();
    const int max_col_id = num_cols - 1;
    const int num_rows = H_in.rows();
    const int max_row_id = num_rows - 1;

    auto compare_and_store_min = [](float new_value, float &current_min, bool &changed_value) {
        if (!std::isnan(new_value))
        {
            if (new_value < current_min || std::isnan(current_min))
            {
                current_min = new_value;
                changed_value = true;
            }
        }
    };

    bool has_at_least_one_value = true;
    bool changed_value = true;

    while (changed_value && has_at_least_one_value)
    {
        has_at_least_one_value = false;
        changed_value = false;

        for (int col_id = 0; col_id < num_cols; ++col_id)
        {
            for (int row_id = 0; row_id < num_rows; ++row_id)
            {
                if (std::isnan(H_in(row_id, col_id)))
                {
                    auto &middle_value = H_out(row_id, col_id);

                    if (col_id > 0)
                        compare_and_store_min(H_out(row_id, col_id - 1), middle_value,
                                              changed_value);
                    if (col_id < max_col_id)
                        compare_and_store_min(H_out(row_id, col_id + 1), middle_value,
                                              changed_value);
                    if (row_id > 0)
                        compare_and_store_min(H_out(row_id - 1, col_id), middle_value,
                                              changed_value);
                    if (row_id < max_row_id)
                        compare_and_store_min(H_out(row_id + 1, col_id), middle_value,
                                              changed_value);
                }
                else
                {
                    has_at_least_one_value = true;
                }
            }
        }
    }
}

void elevationMap::minValuesLimited(const std::string &layer_in, const std::string &layer_out,
                                    int max_hole_pixels)
{
    if (!exists(layer_out))
    {
        add(layer_out, get(layer_in));
    }

    const grid_map::Matrix &H_in = get(layer_in);
    grid_map::Matrix &H_out = get(layer_out);
    H_out = H_in;

    const int rows = H_in.rows(), cols = H_in.cols();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

    auto inBounds = [&](int r, int c) { return r >= 0 && c >= 0 && r < rows && c < cols; };

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            if (!std::isnan(H_in(r, c)) || visited[r][c]) continue;

            std::vector<Pixel> comp;
            std::queue<Pixel> q;
            q.push({r, c});
            visited[r][c] = true;
            while (!q.empty())
            {
                Pixel p = q.front();
                q.pop();
                comp.push_back(p);
                for (int k = 0; k < 4; ++k)
                {
                    int nr = p.r + dr[k], nc = p.c + dc[k];
                    if (inBounds(nr, nc) && !visited[nr][nc] && std::isnan(H_in(nr, nc)))
                    {
                        visited[nr][nc] = true;
                        q.push({nr, nc});
                    }
                }
            }

            if (comp.size() > static_cast<size_t>(max_hole_pixels)) continue;

            bool changed = true;
            for (int iter = 0; iter < 3 && changed; ++iter)
            {
                changed = false;
                for (auto &px : comp)
                {
                    int i = px.r, j = px.c;
                    float &center = H_out(i, j);
                    for (int k = 0; k < 4; ++k)
                    {
                        int ni = i + dr[k], nj = j + dc[k];
                        if (!inBounds(ni, nj)) continue;
                        float nb = H_out(ni, nj);
                        if (!std::isnan(nb) && (std::isnan(center) || nb < center))
                        {
                            center = nb;
                            changed = true;
                        }
                    }
                }
            }
        }
    }
}

void elevationMap::maxValues(const std::string &layer_in, const std::string &layer_out)
{
    if (!exists(layer_out))
    {
        add(layer_out, get(layer_in));
    }

    const grid_map::Matrix &H_in = get(layer_in);
    grid_map::Matrix &H_out = get(layer_out);
    H_out = H_in;

    const int num_cols = H_in.cols();
    const int max_col_id = num_cols - 1;
    const int num_rows = H_in.rows();
    const int max_row_id = num_rows - 1;

    auto compare_and_store_max = [](float new_value, float &current_max, bool &changed_value) {
        if (!std::isnan(new_value))
        {
            if (new_value > current_max || std::isnan(current_max))
            {
                current_max = new_value;
                changed_value = true;
            }
        }
    };

    bool has_at_least_one_value = true;
    bool changed_value = true;

    while (changed_value && has_at_least_one_value)
    {
        has_at_least_one_value = false;
        changed_value = false;

        for (int col_id = 0; col_id < num_cols; ++col_id)
        {
            for (int row_id = 0; row_id < num_rows; ++row_id)
            {
                if (std::isnan(H_in(row_id, col_id)))
                {
                    auto &middle_value = H_out(row_id, col_id);

                    if (col_id > 0)
                        compare_and_store_max(H_out(row_id, col_id - 1), middle_value,
                                              changed_value);
                    if (col_id < max_col_id)
                        compare_and_store_max(H_out(row_id, col_id + 1), middle_value,
                                              changed_value);
                    if (row_id > 0)
                        compare_and_store_max(H_out(row_id - 1, col_id), middle_value,
                                              changed_value);
                    if (row_id < max_row_id)
                        compare_and_store_max(H_out(row_id + 1, col_id), middle_value,
                                              changed_value);
                }
                else
                {
                    has_at_least_one_value = true;
                }
            }
        }
    }
}

void elevationMap::meanValues(const std::string &layer_in, const std::string &layer_out)
{
    if (!exists(layer_out)) add(layer_out, get(layer_in));

    const grid_map::Matrix &H_in = get(layer_in);
    grid_map::Matrix &H_out = get(layer_out);
    H_out = H_in;

    const int num_rows = H_in.rows();
    const int num_cols = H_in.cols();
    const int max_row_id = num_rows - 1;
    const int max_col_id = num_cols - 1;

    bool has_at_least_one_value = true;
    bool changed_value = true;

    while (changed_value && has_at_least_one_value)
    {
        has_at_least_one_value = false;
        changed_value = false;

        for (int col_id = 0; col_id < num_cols; ++col_id)
        {
            for (int row_id = 0; row_id < num_rows; ++row_id)
            {
                if (std::isnan(H_in(row_id, col_id)))
                {
                    auto &mean_value = H_out(row_id, col_id);
                    float sum = 0.0f;
                    int count = 0;

                    if (col_id > 0 && !std::isnan(H_out(row_id, col_id - 1)))
                    {
                        sum += H_out(row_id, col_id - 1);
                        ++count;
                    }
                    if (col_id < max_col_id && !std::isnan(H_out(row_id, col_id + 1)))
                    {
                        sum += H_out(row_id, col_id + 1);
                        ++count;
                    }
                    if (row_id > 0 && !std::isnan(H_out(row_id - 1, col_id)))
                    {
                        sum += H_out(row_id - 1, col_id);
                        ++count;
                    }
                    if (row_id < max_row_id && !std::isnan(H_out(row_id + 1, col_id)))
                    {
                        sum += H_out(row_id + 1, col_id);
                        ++count;
                    }

                    if (count > 0)
                    {
                        float new_value = sum / count;
                        if (new_value != mean_value || std::isnan(mean_value))
                        {
                            mean_value = new_value;
                            changed_value = true;
                        }
                    }
                }
                else
                {
                    has_at_least_one_value = true;
                }
            }
        }
    }
}

void elevationMap::meanValuesOnce(const std::string &layer_in, const std::string &layer_out)
{
    if (!exists(layer_out)) add(layer_out, get(layer_in));

    const grid_map::Matrix &H_in = get(layer_in);
    grid_map::Matrix &H_out = get(layer_out);
    H_out = H_in;

    const int num_rows = H_in.rows();
    const int num_cols = H_in.cols();
    const int center_row = num_rows / 2;
    const int center_col = num_cols / 2;

    std::vector<std::vector<bool>> visited(num_rows, std::vector<bool>(num_cols, false));
    std::queue<std::pair<int, int>> queue;
    queue.push({center_row, center_col});
    visited[center_row][center_col] = true;

    const int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    while (!queue.empty())
    {
        auto [row, col] = queue.front();
        queue.pop();

        if (std::isnan(H_in(row, col)))
        {
            auto &mean_value = H_out(row, col);
            float sum = 0.0f;
            int count = 0;

            for (int i = 0; i < 8; ++i)
            {
                int new_row = row + dx[i];
                int new_col = col + dy[i];
                if (new_row >= 0 && new_row < num_rows && new_col >= 0 && new_col < num_cols &&
                    visited[new_row][new_col])
                {
                    float value = H_out(new_row, new_col);
                    if (!std::isnan(value))
                    {
                        sum += value;
                        ++count;
                    }
                }
            }

            if (count > 0)
            {
                float new_value = sum / count;
                if (fabs(new_value - mean_value) > 1e-3 || std::isnan(mean_value))
                {
                    mean_value = new_value;
                }
            }
        }

        for (int i = 0; i < 8; ++i)
        {
            int new_row = row + dx[i];
            int new_col = col + dy[i];
            if (new_row >= 0 && new_row < num_rows && new_col >= 0 && new_col < num_cols &&
                !visited[new_row][new_col])
            {
                queue.push({new_row, new_col});
                visited[new_row][new_col] = true;
            }
        }
    }
}

void elevationMap::medianFilter(const std::string &layer_in, const std::string &layer_out,
                                int kernel_size, float threshold)
{
    if (!exists(layer_out)) add(layer_out);

    cv::Mat elevation_image;
    cv::eigen2cv(get(layer_in), elevation_image);

    cv::Mat mask;
    if (threshold != -std::numeric_limits<float>::infinity())
    {
        mask = elevation_image > threshold;
    }

    if (kernel_size <= 5)
    {
        cv::Mat filtered_image;
        cv::medianBlur(elevation_image, filtered_image, kernel_size);

        if (!mask.empty())
        {
            elevation_image.copyTo(filtered_image, mask);
        }
        cv::cv2eigen(filtered_image, get(layer_out));
    }
    else
    {
        const float min_value = minCoeffOfFinites(get(layer_in));
        const float max_value = maxCoeffOfFinites(get(layer_in));

        cv::Mat elevation_image_u8;
        grid_map::GridMapCvConverter::toImage<unsigned char, 1>(*this, layer_in, CV_8UC1, min_value,
                                                                max_value, elevation_image_u8);

        cv::Mat filtered_image_u8;
        cv::medianBlur(elevation_image_u8, filtered_image_u8, kernel_size);

        cv::Mat filtered_image_float;
        constexpr float max_uchar_value = 255.F;
        filtered_image_u8.convertTo(filtered_image_float, CV_32F,
                                    (max_value - min_value) / max_uchar_value, min_value);

        if (!mask.empty())
        {
            elevation_image.copyTo(filtered_image_float, mask);
        }
        cv::cv2eigen(filtered_image_float, get(layer_out));
    }
}

void elevationMap::gaussianFilter(const std::string &layer_in, const std::string &layer_out,
                                  int kernel_size)
{
    if (!exists(layer_out)) add(layer_out);

    cv::Mat layer_cv;
    cv::eigen2cv(get(layer_in), layer_cv);
    cv::GaussianBlur(layer_cv, layer_cv, cv::Size(kernel_size, kernel_size), 0.0);
    cv::cv2eigen(layer_cv, get(layer_out));
}

float elevationMap::errorFromCovariance(const Eigen::Vector3f &mean,
                                        const Eigen::Matrix3f &square) const
{
    const Eigen::Matrix3f covari_matrix = square - mean * mean.transpose();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
    solver.computeDirect(covari_matrix, Eigen::DecompositionOptions::ComputeEigenvectors);

    if (solver.eigenvalues()(0) > 1e-8)
    {
        float square_error = (solver.eigenvalues()(0) > 0.0) ? solver.eigenvalues()(0) : 0.0;
        return square_error;
    }
    else
    {
        return 0.0;
    }
}

float elevationMap::computeError(int x, int y, int kernel_size, Eigen::Vector3f &mean,
                                 Eigen::Matrix3f &square) const
{
    int N = 0;
    int bias = (kernel_size - 1) / 2;
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_square = Eigen::Matrix3f::Zero();

    for (int i = x - bias; i <= x + bias; ++i)
    {
        for (int j = y - bias; j <= y + bias; ++j)
        {
            if (i < 0 || i >= getSize().x() || j < 0 || j >= getSize().y()) continue;
            float height = getAltitude(grid_map::Index(i, j));
            if (!std::isfinite(height)) continue;

            Eigen::Vector3f p(i * grid_size_, j * grid_size_, height);
            sum += p;
            sum_square.noalias() += p * p.transpose();
            ++N;
        }
    }

    if (N < 3)
        return 0.0f;
    else
    {
        mean = sum / N;
        square = sum_square / N;
        return errorFromCovariance(mean, square);
    }
}

void elevationMap::inPainting(const std::string &layer_in, int method)
{
    const std::string& layer_out = "tmp";

    switch (method)
    {
        case MEANONCE: 
            meanValuesOnce(layer_in, layer_out); 
            break;
        case MIN: 
            minValues(layer_in, layer_out); 
            break;
        case MINLIMIT: 
            minValuesLimited(layer_in, layer_out, 200); 
            break;
        case MAX: 
            maxValues(layer_in, layer_out); 
            break;
        case MEAN: 
            meanValues(layer_in, layer_out); 
            break;
        default: 
            break;
    }

    get(layer_in) = std::move(get(layer_out));
    erase(layer_out);
}

void elevationMap::deNoise(const std::string &layer_in, int method, int kernel_size)
{
    kernel_size = std::clamp(kernel_size, 1, 5);
    const std::string layer_out = "tmp";

    if (method == MEDIAN)
    {
        medianFilter(layer_in, layer_out, kernel_size, -0.1f);
    }
    else if (method == GAUSS)
    {
        gaussianFilter(layer_in, layer_out, kernel_size);
    }

    get(layer_in) = std::move(get(layer_out));
    erase(layer_out);
}

bool elevationMap::isPassable(float variance_error, float roughness_thres) const
{
    float square_thres = roughness_thres * roughness_thres;
    return (variance_error < square_thres);
}

void elevationMap::judgePassability(float roughness_thres, float drop_thres, int kernel_size)
{
    kernel_size = std::clamp(kernel_size, 3, 5);
    int center = map_size_grid_ / 2;

    get("passability").setConstant(UNKNOWN);
    std::vector<bool> visited(map_size_grid_ * map_size_grid_, false);
    std::deque<std::pair<int, int>> q;
    q.emplace_back(center, center);
    visited[center + center * map_size_grid_] = true;
    setPassability(Eigen::Array2i(center, center), PASSABLE);

    const int dx[8] = {1,  1,  0, -1, -1, -1, 0, 1};
    const int dy[8] = {0, -1, -1, -1,  0,  1, 1, 1};
    const int n_dir = 8;

    while (!q.empty())
    {
        auto [x, y] = q.front();
        q.pop_front();
        float cur_height = getAltitude(grid_map::Index(x, y));
        if (!std::isfinite(cur_height))
        {
            cur_height = -0.5f;
            setAltitude(grid_map::Index(x, y), cur_height);
        }

        for (int i = 0; i < n_dir; ++i)
        {
            int nx = x + dx[i], ny = y + dy[i];
            if (nx < 0 || nx >= map_size_grid_ || ny < 0 || ny >= map_size_grid_) continue;

            int idx = nx + ny * map_size_grid_;
            if (visited[idx]) continue;
            visited[idx] = true;

            float nbr_height = getAltitude(grid_map::Index(nx, ny));
            if (!std::isfinite(nbr_height))
            {
                nbr_height = cur_height;
                setAltitude(grid_map::Index(nx, ny), nbr_height);
            }

            if (std::fabs(nbr_height - cur_height) > drop_thres)
            {
                setPassability(grid_map::Index(nx, ny), IMPASSABLE);
                continue;
            }

            Eigen::Vector3f mean = Eigen::Vector3f::Zero();
            Eigen::Matrix3f square = Eigen::Matrix3f::Zero();
            auto e = computeError(nx, ny, kernel_size, mean, square);
            if (!isPassable(e, roughness_thres) && cur_height > mean.z())
            {
                setPassability(grid_map::Index(nx, ny), IMPASSABLE);
                continue;
            }
    
            setPassability(grid_map::Index(nx, ny), PASSABLE);
            q.emplace_back(nx, ny);
        }
    }
}
