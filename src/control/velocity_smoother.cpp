module;

#include <algorithm>
#include <cmath>
#include <stdexcept>

module mininav.control.velocity_smoother;

import mininav.core.types;

namespace mininav::control
{
    void validate(const VelocitySmootherConfig& cfg)
    {
        if (!(cfg.max_accel > 0.0) || !(cfg.max_angular_accel > 0.0))
        {
            throw std::invalid_argument{"velocity_smoother: accelerations must be positive"};
        }
    }

    VelocitySmoother::VelocitySmoother(const VelocitySmootherConfig& cfg) : cfg_{cfg}
    {
        validate(cfg_);
    }

    Twist2D VelocitySmoother::smooth(const Twist2D& target, const double dt) noexcept
    {
        const double dv = target.v() - last_.v();
        const double dw = target.w() - last_.w();
        const double max_dv = cfg_.max_accel * dt;
        const double max_dw = cfg_.max_angular_accel * dt;

        double scale = 1.0;
        if (std::abs(dv) > max_dv)
        {
            scale = std::min(scale, max_dv / std::abs(dv));
        }
        if (std::abs(dw) > max_dw)
        {
            scale = std::min(scale, max_dw / std::abs(dw));
        }

        last_ = Twist2D{last_.v() + scale * dv, last_.w() + scale * dw};
        return last_;
    }
}
