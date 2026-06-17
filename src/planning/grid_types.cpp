module;

#include <cmath>
#include <cstddef>

module mininav.planning.grid_types;

import mininav.core.types;

namespace mininav::planning
{
    double Path::length() const noexcept
    {
        double total = 0.0;
        for (std::size_t i = 1; i < poses.size(); ++i)
        {
            const double dx = poses[i].x() - poses[i - 1].x();
            const double dy = poses[i].y() - poses[i - 1].y();
            total += std::hypot(dx, dy);
        }
        return total;
    }
}
