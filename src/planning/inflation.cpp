module;

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

module mininav.planning.inflation;

import mininav.planning.occupancy_grid;
import mininav.planning.grid_types;

namespace mininav::planning
{
    namespace
    {
        // 8 邻居偏移(传播用)。
        constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    } // namespace

    std::vector<double> obstacle_distance_cells(const OccupancyGrid& grid)
    {
        const int w = grid.width();
        const int h = grid.height();
        const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

        constexpr double kInf = std::numeric_limits<double>::infinity();
        std::vector<double> dist(n, kInf);
        std::vector<int> src_x(n, -1);
        std::vector<int> src_y(n, -1);

        // 最小堆:(距离, flat index)。
        using QItem = std::pair<double, int>;
        std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;

        // 播种:所有占据 cell 距离 0、源为自身。
        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                if (grid.at(GridCoord{x, y}) == kOccupied)
                {
                    const int idx = y * w + x;
                    const std::size_t uidx = static_cast<std::size_t>(idx);
                    dist[uidx] = 0.0;
                    src_x[uidx] = x;
                    src_y[uidx] = y;
                    pq.push({0.0, idx});
                }
            }
        }

        while (!pq.empty())
        {
            const auto [d, idx] = pq.top();
            pq.pop();
            const std::size_t uidx = static_cast<std::size_t>(idx);
            if (d > dist[uidx])
            {
                continue; // 过期条目
            }
            const int cx = idx % w;
            const int cy = idx / w;
            const int sx = src_x[uidx];
            const int sy = src_y[uidx];

            for (int k = 0; k < 8; ++k)
            {
                const int nx = cx + kDx[k];
                const int ny = cy + kDy[k];
                if (nx < 0 || nx >= w || ny < 0 || ny >= h)
                {
                    continue;
                }
                // 候选距离 = 邻居到"当前 cell 记忆源"的直线距离。
                const double cand = std::hypot(static_cast<double>(nx - sx),
                                               static_cast<double>(ny - sy));
                const int nidx = ny * w + nx;
                const std::size_t unidx = static_cast<std::size_t>(nidx);
                if (cand < dist[unidx])
                {
                    dist[unidx] = cand;
                    src_x[unidx] = sx;
                    src_y[unidx] = sy;
                    pq.push({cand, nidx});
                }
            }
        }

        return dist;
    }

    OccupancyGrid inflate(const OccupancyGrid& grid, const double radius_m)
    {
        const int w = grid.width();
        const int h = grid.height();
        const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

        std::vector<std::int8_t> out(n);

        // radius ≤ 0:直接复制原图(costmap == 原图)。
        const bool do_inflate = radius_m > 0.0;
        const std::vector<double> dist =
            do_inflate ? obstacle_distance_cells(grid) : std::vector<double>{};
        const double radius_cells = radius_m / grid.resolution();

        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                const std::size_t idx = static_cast<std::size_t>(y * w + x);
                std::int8_t value = grid.at(GridCoord{x, y});
                if (do_inflate && value != kOccupied && dist[idx] <= radius_cells)
                {
                    value = kOccupied;
                }
                out[idx] = value;
            }
        }

        return OccupancyGrid{w, h, grid.resolution(), grid.origin(), std::move(out)};
    }
}
