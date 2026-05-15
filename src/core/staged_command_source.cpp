module mininav.core.command_source;

import mininav.core.types;
import mininav.core.math;

namespace mininav
{
    Twist2D StagedCommandSource::command_at(const double t) const noexcept
    {
        if (t < 0.0)
        {
            return Twist2D{0.0, 0.0};
        }
        if (t < 5.0)
        {
            return Twist2D{2.0,  0.0 * kPi};
        }
        if (t < 10.0)
        {
            return Twist2D{4.0, 0.2 * kPi};
        }
        if (t < 15.0)
        {
            return Twist2D{2.0, 0.0 * kPi};
        }
        return Twist2D{4.0, 0.2 * kPi};
    }
}
