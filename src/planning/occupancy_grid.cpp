module;

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

module mininav.planning.occupancy_grid;

import mininav.planning.grid_types;

namespace mininav::planning
{
    OccupancyGrid::OccupancyGrid(const int width, const int height, const double resolution,
                                 Eigen::Vector2d origin, std::vector<std::int8_t> data)
        : width_{width}, height_{height}, resolution_{resolution},
          origin_{std::move(origin)}, data_{std::move(data)}
    {
        if (width_ <= 0 || height_ <= 0)
        {
            throw std::invalid_argument(
                "OccupancyGrid: width/height must be positive, got " +
                std::to_string(width_) + "x" + std::to_string(height_));
        }
        if (resolution_ <= 0.0)
        {
            throw std::invalid_argument(
                "OccupancyGrid: resolution must be positive, got " +
                std::to_string(resolution_));
        }
        const std::size_t expected =
            static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
        if (data_.size() != expected)
        {
            throw std::invalid_argument(
                "OccupancyGrid: data size " + std::to_string(data_.size()) +
                " does not match width*height " + std::to_string(expected));
        }
    }

    std::size_t OccupancyGrid::index(const GridCoord c) const noexcept
    {
        return static_cast<std::size_t>(c.y) * static_cast<std::size_t>(width_) +
               static_cast<std::size_t>(c.x);
    }

    bool OccupancyGrid::in_bounds(const GridCoord c) const noexcept
    {
        return c.x >= 0 && c.x < width_ && c.y >= 0 && c.y < height_;
    }

    std::int8_t OccupancyGrid::at(const GridCoord c) const noexcept
    {
        if (!in_bounds(c))
        {
            return kOccupied;
        }
        return data_[index(c)];
    }

    bool OccupancyGrid::is_free(const GridCoord c) const noexcept
    {
        return at(c) == kFree;
    }

    GridCoord OccupancyGrid::world_to_grid(const Eigen::Vector2d& p) const noexcept
    {
        const double gx = std::floor((p.x() - origin_.x()) / resolution_);
        const double gy = std::floor((p.y() - origin_.y()) / resolution_);
        return GridCoord{static_cast<int>(gx), static_cast<int>(gy)};
    }

    Eigen::Vector2d OccupancyGrid::grid_to_world(const GridCoord c) const noexcept
    {
        const double wx = origin_.x() + (static_cast<double>(c.x) + 0.5) * resolution_;
        const double wy = origin_.y() + (static_cast<double>(c.y) + 0.5) * resolution_;
        return Eigen::Vector2d{wx, wy};
    }
}
