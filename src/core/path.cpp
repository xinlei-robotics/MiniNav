module;

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>

module mininav.core.path;

import mininav.core.types;

namespace mininav
{
    namespace
    {
        [[nodiscard]] std::size_t segment_count(const Path& path) noexcept
        {
            return path.size() < 2 ? 0 : path.size() - 1;
        }

        [[nodiscard]] Eigen::Vector2d point_at(const Path& path, const std::size_t i) noexcept
        {
            return path.poses[i].position();
        }
    }

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

    PathProjection project_onto(const Path& path, const Eigen::Vector2d& p,
                                const std::size_t from_segment, const std::size_t window)
    {
        const std::size_t n_seg = segment_count(path);
        if (n_seg == 0)
        {
            const Eigen::Vector2d only = point_at(path, 0);
            return PathProjection{
                .segment = 0, .t = 0.0, .arclength = 0.0, .point = only, .distance = (p - only).norm(),
            };
        }

        const std::size_t first = std::min(from_segment, n_seg - 1);
        const std::size_t last = std::min(n_seg, first + std::max<std::size_t>(window, 1));

        // 窗口之前的弧长:投影的 arclength 从路径起点算起。
        double arclength_before = 0.0;
        for (std::size_t i = 0; i < first; ++i)
        {
            arclength_before += (point_at(path, i + 1) - point_at(path, i)).norm();
        }

        PathProjection best{};
        double best_distance_sq = 0.0;
        bool found = false;
        for (std::size_t i = first; i < last; ++i)
        {
            const Eigen::Vector2d a = point_at(path, i);
            const Eigen::Vector2d ab = point_at(path, i + 1) - a;
            const double len_sq = ab.squaredNorm();
            // 零长度段(重复 waypoint)退化为点:t = 0,不做除法。
            const double t = len_sq > 0.0 ? std::clamp((p - a).dot(ab) / len_sq, 0.0, 1.0) : 0.0;
            const Eigen::Vector2d q = a + ab * t; // 向量 * 标量:clang 18 模块里 Eigen 的"标量 * 向量"友元运算符链接不到
            const double d_sq = (p - q).squaredNorm();
            if (!found || d_sq < best_distance_sq)
            {
                found = true;
                best_distance_sq = d_sq;
                best.segment = i;
                best.t = t;
                best.arclength = arclength_before + t * std::sqrt(len_sq);
                best.point = q;
            }
            arclength_before += std::sqrt(len_sq);
        }
        best.distance = std::sqrt(best_distance_sq);
        return best;
    }

    Eigen::Vector2d lookahead_point(const Path& path, const PathProjection& from,
                                    const Eigen::Vector2d& p, const double L)
    {
        const std::size_t n_seg = segment_count(path);
        if (n_seg == 0 || from.distance >= L)
        {
            return from.point;
        }

        const double L_sq = L * L;
        for (std::size_t i = std::min(from.segment, n_seg - 1); i < n_seg; ++i)
        {
            // 本段的起点:第一段从投影点开始,之后从段首开始。进入本循环时起点必在圆内
            // (上一段没有出圆),所以出圆点就是二次方程的较大根。
            const Eigen::Vector2d a = i == from.segment ? from.point : point_at(path, i);
            const Eigen::Vector2d u = point_at(path, i + 1) - a;
            const double qa = u.squaredNorm();
            if (qa == 0.0)
            {
                continue;
            }
            const Eigen::Vector2d w = a - p;
            const double qb = 2.0 * u.dot(w);
            const double qc = w.squaredNorm() - L_sq;
            // 起点在圆内 ⇒ qc ≤ 0 ⇒ 判别式 ≥ 0;max 只防舍入误差。
            const double disc = std::max(qb * qb - 4.0 * qa * qc, 0.0);
            const double s = (-qb + std::sqrt(disc)) / (2.0 * qa);
            if (s <= 1.0)
            {
                return a + u * std::max(s, 0.0);
            }
        }
        return point_at(path, n_seg);
    }
}
