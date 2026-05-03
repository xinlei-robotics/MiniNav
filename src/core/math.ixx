module;

#include <cmath>
#include <numbers>

export module mininav.core.math;

export namespace mininav
{
    inline constexpr double kPi = std::numbers::pi_v<double>;
    inline constexpr double kTwoPi = 2.0 * kPi;

    [[nodiscard]] constexpr double wrap_angle(double angle) noexcept
    {
        angle = std::fmod(angle + kPi, kTwoPi);
        if (angle <= 0.0)
        {
            angle += kTwoPi;
        }
        return angle - kPi;
    }
}
