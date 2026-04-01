module mininav.core.command_source;

import mininav.core.types;
import mininav.core.math;

namespace mininav {
Twist2D StagedCommandSource::command_at(double t) const noexcept {
  if (t < 0.0) {
    return Twist2D{0.0, 0.0};
  } else if (t < 5.0) {
    return Twist2D{1.0, 0.1 * kPi};
  } else if (t < 10.0) {
    return Twist2D{1.0, 0.3 * kPi};
  } else if (t < 15.0) {
    return Twist2D{1.0, 0.0};
  } else {
    return Twist2D{0.5, -0.4 * kPi};
  }
}
} // namespace mininav