module;

#include <stdexcept>

module mininav.control.progress_checker;

import mininav.core.types;
import mininav.control.controller;

namespace mininav::control
{
    void validate(const ProgressCheckerConfig& cfg)
    {
        if (!(cfg.required_movement > 0.0) || !(cfg.time_allowance > 0.0))
        {
            throw std::invalid_argument{"progress_checker: required_movement and time_allowance "
                                        "must be positive"};
        }
    }

    SimpleProgressChecker::SimpleProgressChecker(const ProgressCheckerConfig& cfg) : cfg_{cfg}
    {
        validate(cfg_);
    }

    bool SimpleProgressChecker::check(const Pose2D& pose, const double t)
    {
        if (!has_baseline_ ||
            (pose.position() - baseline_pose_.position()).norm() > cfg_.required_movement)
        {
            has_baseline_ = true;
            baseline_pose_ = pose;
            baseline_time_ = t;
            return true;
        }
        return t - baseline_time_ <= cfg_.time_allowance;
    }
}
