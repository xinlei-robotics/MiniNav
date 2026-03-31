module;

#include <cmath>
#include <numbers>

export module mininav.core.math;

export namespace mininav {

constexpr double kPi = std::numbers::pi_v<double>;

[[nodiscard]] constexpr double wrap_angle(double angle) noexcept {
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

} // namespace mininav