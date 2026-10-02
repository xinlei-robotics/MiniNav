module;

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

module mininav.viz.nav_log;

import mininav.core.types;
import mininav.viz.sink;
import mininav.viz.plan_log;
import mininav.viz.sim_state_log;

namespace mininav
{
    namespace
    {
        // 配色(RGB):原始路径=灰、look-ahead 与追踪弧=橙、协方差椭圆=蓝紫。
        constexpr std::array<std::uint8_t, 3> kRawPathColor{150, 150, 150};
        constexpr std::array<std::uint8_t, 3> kPursuitColor{245, 150, 30};
        constexpr std::array<std::uint8_t, 3> kCovarianceColor{120, 110, 230};

        constexpr int kArcSegments = 24;
        constexpr int kEllipseSegments = 48;
        constexpr double kEllipseSigma = 3.0;

        // "/world/robot" → "/world";没有上级时返回空串。
        [[nodiscard]] std::string parent_of(const std::string_view path)
        {
            const auto slash = path.rfind('/');
            return slash == std::string_view::npos ? std::string{} : std::string{path.substr(0, slash)};
        }
    }

    std::vector<Eigen::Vector2d> covariance_ellipse(const Eigen::Vector2d& center, const double a,
                                                    const double b, const double c, const double k,
                                                    const int segments)
    {
        // 2×2 对称阵的特征分解(闭式):λ = (a+c)/2 ± √(((a−c)/2)² + b²),主轴角 ½·atan2(2b, a−c)。
        const double mean = 0.5 * (a + c);
        const double radius = std::hypot(0.5 * (a - c), b);
        const double major = k * std::sqrt(std::max(mean + radius, 0.0));
        const double minor = k * std::sqrt(std::max(mean - radius, 0.0));
        const double phi = 0.5 * std::atan2(2.0 * b, a - c);
        const double cp = std::cos(phi);
        const double sp = std::sin(phi);

        std::vector<Eigen::Vector2d> points;
        points.reserve(static_cast<std::size_t>(segments) + 1);
        for (int i = 0; i <= segments; ++i)
        {
            const double t = 2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(segments);
            const double u = major * std::cos(t);
            const double v = minor * std::sin(t);
            points.emplace_back(center.x() + cp * u - sp * v, center.y() + sp * u + cp * v);
        }
        return points;
    }

    std::vector<Eigen::Vector2d> pursuit_arc(const Pose2D& pose, const double curvature,
                                             const Eigen::Vector2d& target, const int segments)
    {
        const double c = std::cos(pose.yaw());
        const double s = std::sin(pose.yaw());
        const double dx = target.x() - pose.x();
        const double dy = target.y() - pose.y();
        const double gx = c * dx + s * dy; // 车体系
        if (gx <= 0.0)
        {
            return {};
        }

        // 与车头相切、曲率 κ 的圆上,到 target 的弧长:弦长 d = 2·sin(κs/2)/κ。
        const double chord = std::hypot(dx, dy);
        const double k = curvature;
        const double arc_len = std::abs(k) < 1e-9
                                   ? chord
                                   : 2.0 * std::asin(std::min(1.0, std::abs(k) * chord / 2.0)) / std::abs(k);

        std::vector<Eigen::Vector2d> points;
        points.reserve(static_cast<std::size_t>(segments) + 1);
        for (int i = 0; i <= segments; ++i)
        {
            const double arc_s = arc_len * static_cast<double>(i) / static_cast<double>(segments);
            // 车体系下的圆弧:(sin(κs)/κ, (1 − cos(κs))/κ);κ → 0 退化为直线。
            const double lx = std::abs(k) < 1e-9 ? arc_s : std::sin(k * arc_s) / k;
            const double ly = std::abs(k) < 1e-9 ? 0.0 : (1.0 - std::cos(k * arc_s)) / k;
            points.emplace_back(pose.x() + c * lx - s * ly, pose.y() + s * lx + c * ly);
        }
        return points;
    }

    void log_nav_scene(VizSink& sink, const NavScene& scene, const std::string_view world_root)
    {
        log_plan(sink, scene.plan, world_root);
        sink.log_line_strip_static(std::string{world_root} + "/plan/raw", scene.raw_path, kRawPathColor);
    }

    void log_to_rerun(VizSink& sink, const NavStep& step, const std::string_view robot_root)
    {
        log_to_rerun(sink, step.sim, robot_root);

        const NavDiagnostics& nav = step.nav;
        const std::string robot{robot_root};
        const std::string world = parent_of(robot_root);

        sink.log_twist(robot + "/actuator", nav.actuator);

        sink.log_points(world + "/control/lookahead", {nav.lookahead}, kPursuitColor, 0.03F);
        sink.log_line_strip(world + "/control/arc",
                            pursuit_arc(nav.control_pose, nav.curvature, nav.lookahead, kArcSegments),
                            kPursuitColor);

        const Eigen::Vector2d ekf_xy{step.sim.ekf_mean(0), step.sim.ekf_mean(1)};
        sink.log_line_strip(world + "/estimate/ekf_cov",
                            covariance_ellipse(ekf_xy, step.sim.ekf_cov(0, 0), step.sim.ekf_cov(0, 1),
                                               step.sim.ekf_cov(1, 1), kEllipseSigma, kEllipseSegments),
                            kCovarianceColor);

        sink.log_scalar("/plots/error/ctrl", nav.e_ctrl);
        sink.log_scalar("/plots/error/est", nav.e_est);
        sink.log_scalar("/plots/error/true", nav.e_true);
        sink.log_scalar("/plots/clearance", nav.clearance);
        sink.log_scalar("/plots/regime", static_cast<double>(nav.regime_code));
    }
}
