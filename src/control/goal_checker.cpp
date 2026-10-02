module;

#include <cmath>
#include <stdexcept>

module mininav.control.goal_checker;

import mininav.core.types;
import mininav.core.math;
import mininav.control.controller;

namespace mininav::control
{
    void validate(const GoalCheckerConfig& cfg)
    {
        if (!(cfg.xy_tolerance > 0.0) || !(cfg.yaw_tolerance > 0.0))
        {
            throw std::invalid_argument{"goal_checker: tolerances must be positive"};
        }
    }

    SimpleGoalChecker::SimpleGoalChecker(const GoalCheckerConfig& cfg, const bool check_yaw)
        : cfg_{cfg}, check_yaw_{check_yaw}
    {
        validate(cfg_);
    }

    bool SimpleGoalChecker::is_goal_reached(const Pose2D& pose, const Pose2D& goal,
                                            const Twist2D& /*velocity*/)
    {
        if (!position_reached_)
        {
            if ((goal.position() - pose.position()).norm() > cfg_.xy_tolerance)
            {
                return false;
            }
            position_reached_ = true;
        }
        return !check_yaw_ || std::abs(wrap_angle(goal.yaw() - pose.yaw())) <= cfg_.yaw_tolerance;
    }

    GoalTolerance SimpleGoalChecker::tolerances() const
    {
        return GoalTolerance{.xy = cfg_.xy_tolerance, .yaw = cfg_.yaw_tolerance, .check_yaw = check_yaw_};
    }
}
