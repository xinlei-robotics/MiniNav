module;

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>

module mininav.control.pure_pursuit;

import mininav.core.types;
import mininav.core.math;
import mininav.core.path;
import mininav.core.robot_description;
import mininav.control.controller;

namespace mininav::control
{
    namespace
    {
        // G 与车重合(已在路径终点)时,转向几何没有定义。
        constexpr double kMinCarrotDistSq = 1e-12;

        void require(const bool ok, const std::string& message)
        {
            if (!ok)
            {
                throw std::invalid_argument{"pure_pursuit: " + message};
            }
        }

        // world 点 → 车体系(x 朝前,y 朝左)。
        [[nodiscard]] Eigen::Vector2d to_robot_frame(const Pose2D& pose, const Eigen::Vector2d& world) noexcept
        {
            const double dx = world.x() - pose.x();
            const double dy = world.y() - pose.y();
            const double c = std::cos(pose.yaw());
            const double s = std::sin(pose.yaw());
            return Eigen::Vector2d{c * dx + s * dy, -s * dx + c * dy};
        }
    }

    void validate(const PurePursuitConfig& cfg)
    {
        require(cfg.desired_linear_vel > 0.0, "desired_linear_vel must be positive");
        require(cfg.lookahead_time > 0.0, "lookahead_time must be positive");
        require(cfg.min_lookahead > 0.0, "min_lookahead must be positive");
        require(cfg.max_lookahead >= cfg.min_lookahead, "max_lookahead must be >= min_lookahead");
        require(cfg.lookahead_dist > 0.0, "lookahead_dist must be positive");
        require(cfg.curvature_lookahead_dist > 0.0, "curvature_lookahead_dist must be positive");
        require(cfg.regulated_min_radius > 0.0, "regulated_min_radius must be positive");
        require(cfg.approach_dist > 0.0, "approach_dist must be positive");
        require(cfg.min_approach_vel >= 0.0 && cfg.min_approach_vel <= cfg.desired_linear_vel,
                "min_approach_vel must be in [0, desired_linear_vel]");
        require(cfg.rotate_to_heading_angle > 0.0 && cfg.rotate_to_heading_angle <= kPi,
                "rotate_to_heading_angle must be in (0, pi]");
        require(cfg.rotate_vel > 0.0, "rotate_vel must be positive");
        require(cfg.max_projection_search_dist > 0.0, "max_projection_search_dist must be positive");
    }

    std::string_view to_string(const PursuitRegime regime) noexcept
    {
        switch (regime)
        {
        case PursuitRegime::Stopped: return "stopped";
        case PursuitRegime::RotateToPath: return "rotate_to_path";
        case PursuitRegime::Track: return "track";
        case PursuitRegime::Approach: return "approach";
        case PursuitRegime::RotateToGoal: return "rotate_to_goal";
        }
        return "stopped";
    }

    PurePursuitController::PurePursuitController(const PurePursuitConfig& cfg, const RobotLimits& limits)
        : cfg_{cfg}, limits_{limits}, speed_cap_{cfg.desired_linear_vel}
    {
        validate(cfg_);
        require(limits_.max_linear_vel > 0.0 && limits_.max_angular_vel > 0.0,
                "robot limits must be positive");
        require(cfg_.desired_linear_vel <= limits_.max_linear_vel,
                "desired_linear_vel exceeds the robot's max_linear_vel");
        require(cfg_.rotate_vel <= limits_.max_angular_vel,
                "rotate_vel exceeds the robot's max_angular_vel");
    }

    void PurePursuitController::set_plan(const Path& path)
    {
        plan_ = path;
        segment_starts_.clear();
        plan_length_ = 0.0;
        for (std::size_t i = 1; i < plan_.size(); ++i)
        {
            segment_starts_.push_back(plan_length_);
            plan_length_ += (plan_.poses[i].position() - plan_.poses[i - 1].position()).norm();
        }
        reset();
    }

    void PurePursuitController::set_speed_limit(const double limit, const bool percentage)
    {
        // 限速只会降低期望速度,不会抬高它。
        if (limit <= 0.0)
        {
            speed_cap_ = cfg_.desired_linear_vel;
        }
        else if (percentage)
        {
            speed_cap_ = std::min(cfg_.desired_linear_vel, cfg_.desired_linear_vel * limit / 100.0);
        }
        else
        {
            speed_cap_ = std::min(cfg_.desired_linear_vel, limit);
        }
    }

    void PurePursuitController::reset()
    {
        segment_ = 0;
        progress_ = 0.0;
        debug_ = PursuitDebug{};
    }

    double PurePursuitController::lookahead_distance(const double speed) const noexcept
    {
        if (!cfg_.use_velocity_scaled_lookahead)
        {
            return cfg_.lookahead_dist;
        }
        return std::clamp(cfg_.lookahead_time * std::abs(speed), cfg_.min_lookahead, cfg_.max_lookahead);
    }

    // 从当前段起,覆盖起点弧长不超过"当前进度 + max_projection_search_dist"的线段。
    // 从进度点而不是当前段终点起算:当前段很长时(如 U 形弯的去程),回程段不会被
    // 提前纳入搜索。
    std::size_t PurePursuitController::search_window() const noexcept
    {
        const double horizon = progress_ + cfg_.max_projection_search_dist;
        std::size_t window = 1;
        for (std::size_t i = segment_ + 1; i < segment_starts_.size() && segment_starts_[i] <= horizon; ++i)
        {
            ++window;
        }
        return window;
    }

    double PurePursuitController::rotate_command(const double angle_error) const noexcept
    {
        if (angle_error > 0.0)
        {
            return cfg_.rotate_vel;
        }
        return angle_error < 0.0 ? -cfg_.rotate_vel : 0.0;
    }

    double PurePursuitController::regulated_velocity(const Pose2D& pose, const PathProjection& proj,
                                                     const double kappa)
    {
        double v = speed_cap_;

        // 曲率限速:固定距离 L_κ 处的调节曲率 κ_reg,R = 1/|κ_reg|。
        if (cfg_.use_curvature_regulation)
        {
            const Eigen::Vector2d g = to_robot_frame(
                pose, lookahead_point(plan_, proj, pose.position(), cfg_.curvature_lookahead_dist));
            const double d2 = g.squaredNorm();
            if (d2 > kMinCarrotDistSq && g.y() != 0.0)
            {
                const double radius = d2 / (2.0 * std::abs(g.y()));
                if (radius < cfg_.regulated_min_radius)
                {
                    v *= radius / cfg_.regulated_min_radius;
                }
            }
        }

        // 接近目标:按剩余路程线性减速,保底 min_approach_vel(停车交给 GoalChecker)。
        debug_.regime = PursuitRegime::Track;
        const double remaining = std::max(plan_length_ - proj.arclength, 0.0);
        if (remaining < cfg_.approach_dist)
        {
            debug_.regime = PursuitRegime::Approach;
            v = std::min(v, std::max(cfg_.min_approach_vel, speed_cap_ * remaining / cfg_.approach_dist));
        }

        // 角速度约束:|v·κ| 不超过执行器上限,否则按比例降线速度(保持曲率)。
        if (std::abs(v * kappa) > limits_.max_angular_vel)
        {
            v = limits_.max_angular_vel / std::abs(kappa);
        }
        return v;
    }

    Twist2D PurePursuitController::compute_velocity_commands(const Pose2D& pose, const Twist2D& velocity,
                                                             const GoalChecker* goal_checker)
    {
        debug_ = PursuitDebug{};
        if (plan_.empty())
        {
            return Twist2D{0.0, 0.0};
        }

        // 1. 进度:在当前段附近求最近投影,段号单调不减。
        const Eigen::Vector2d p = pose.position();
        const PathProjection proj = project_onto(plan_, p, segment_, search_window());
        segment_ = proj.segment;
        progress_ = proj.arclength;
        debug_.cross_track_error = proj.distance;
        debug_.arclength = proj.arclength;

        // 2. 位置已在目标容差内且要求朝向:原地转到目标朝向(路径末点的 yaw)。
        const Pose2D& goal = plan_.poses.back();
        if (cfg_.use_rotate_to_heading && goal_checker != nullptr)
        {
            const GoalTolerance tol = goal_checker->tolerances();
            if (tol.check_yaw && (goal.position() - p).norm() <= tol.xy)
            {
                debug_.regime = PursuitRegime::RotateToGoal;
                return Twist2D{0.0, rotate_command(wrap_angle(goal.yaw() - pose.yaw()))};
            }
        }

        // 3. look-ahead 点(车体系)。
        const double L = lookahead_distance(velocity.v());
        const Eigen::Vector2d G = lookahead_point(plan_, proj, p, L);
        debug_.lookahead_dist = L;
        debug_.lookahead_point = G;
        const Eigen::Vector2d g = to_robot_frame(pose, G);
        const double d2 = g.squaredNorm();
        if (d2 < kMinCarrotDistSq)
        {
            return Twist2D{0.0, 0.0}; // 已在路径终点:没有可追的点
        }

        // 4. 夹角过大:先原地转向,再跟踪。
        const double alpha = std::atan2(g.y(), g.x());
        if (cfg_.use_rotate_to_heading && std::abs(alpha) > cfg_.rotate_to_heading_angle)
        {
            debug_.regime = PursuitRegime::RotateToPath;
            return Twist2D{0.0, rotate_command(alpha)};
        }

        // 5. 曲率与线速度。
        const double kappa = 2.0 * g.y() / d2;
        debug_.curvature = kappa;
        const double v = regulated_velocity(pose, proj, kappa);
        return Twist2D{v, v * kappa};
    }
}
