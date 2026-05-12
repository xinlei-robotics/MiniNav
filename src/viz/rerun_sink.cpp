module;

#include <array>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

#include <rerun.hpp>

module mininav.viz.rerun_sink;

import mininav.core.types;

namespace mininav
{
    namespace
    {
        // ---------------------------------------------------------------------
        // ensure_viewer_in_path: 把项目 venv 的 bin 目录前置到当前进程 PATH.
        //
        // 仅 Spawn 模式需要(它要在 PATH 里找到 rerun viewer 可执行文件).
        // 注意 ::setenv 修改的是进程全局环境, 不是线程安全的;
        // ---------------------------------------------------------------------
        void ensure_viewer_in_path() noexcept
        {
            constexpr std::string_view kViewerBinDir = MININAV_RERUN_VIEWER_BIN_DIR;
            if constexpr (kViewerBinDir.empty())
            {
                return;
            }

            const char* current = std::getenv("PATH");
            std::string new_path{kViewerBinDir};
            if (current != nullptr && current[0] != '\0')
            {
                new_path += ':';
                new_path += current;
            }

            ::setenv("PATH", new_path.c_str(), 1);
        }

        // ---------------------------------------------------------------------
        // Axes 几何: 三条沿 X/Y/Z 的单位向量, 红/绿/蓝染色.
        // ---------------------------------------------------------------------
        rerun::Arrows3D make_axes(const float length)
        {
            const std::array origins{
                rerun::Position3D{0.0F, 0.0F, 0.0F},
                rerun::Position3D{0.0F, 0.0F, 0.0F},
                rerun::Position3D{0.0F, 0.0F, 0.0F},
            };
            const std::array vectors{
                rerun::Vec3D{length, 0.0F, 0.0F},
                rerun::Vec3D{0.0F, length, 0.0F},
                rerun::Vec3D{0.0F, 0.0F, length},
            };
            const std::array colors{
                rerun::Color{255, 0, 0}, // X = red
                rerun::Color{0, 255, 0}, // Y = green
                rerun::Color{0, 0, 255}, // Z = blue
            };
            return rerun::Arrows3D::from_vectors(vectors)
                   .with_origins(origins)
                   .with_colors(colors);
        }
    }

    // ---------------------------------------------------------------------------
    // RerunSink::Impl
    // ---------------------------------------------------------------------------
    struct RerunSink::Impl
    {
        rerun::RecordingStream stream;

        static constexpr auto kTimelineName = "sim_time";

        explicit Impl(std::string_view application_id)
            : stream{std::string{application_id}}
        {
        }
    };

    // ----- 构造 / 析构 / 移动 ----------------------------------------------------
    RerunSink::RerunSink(const std::string_view application_id)
        : impl_{std::make_unique<Impl>(application_id)}
    {
        ensure_viewer_in_path();

        if (const auto result = impl_->stream.spawn(); result.is_err())
        {
            throw std::runtime_error{
                "RerunSink: failed to spawn Rerun Viewer: " + result.description
            };
        }
    }

    RerunSink::RerunSink(const std::string_view application_id, const std::filesystem::path& rrd_path)
        : impl_{std::make_unique<Impl>(application_id)}
    {
        if (rrd_path.has_parent_path())
        {
            std::filesystem::create_directories(rrd_path.parent_path());
        }

        if (const auto result = impl_->stream.save(rrd_path.string()); result.is_err())
        {
            throw std::runtime_error{
                "RerunSink: failed to open rrd output '" + rrd_path.string()
                + "': " + result.description
            };
        }
    }

    RerunSink::~RerunSink() = default;
    RerunSink::RerunSink(RerunSink&&) noexcept = default;
    RerunSink& RerunSink::operator=(RerunSink&&) noexcept = default;

    // ----- 时间轴 ---------------------------------------------------------------
    void RerunSink::set_time(const double t_seconds)
    {
        impl_->stream.set_time_duration_secs(Impl::kTimelineName, t_seconds);
    }

    // ----- log_pose -------------------------------------------------------------
    void RerunSink::log_pose(const std::string_view entity_path,
                             const Pose2D& pose)
    {
        const auto x = static_cast<float>(pose.x());
        const auto y = static_cast<float>(pose.y());
        const auto yaw = static_cast<float>(pose.yaw());

        const auto translation = rerun::Vec3D{x, y, 0.0F};
        const auto rotation = rerun::RotationAxisAngle{
            {0.0F, 0.0F, 1.0F}, rerun::Angle::radians(yaw)
        };

        impl_->stream.log(
            std::string{entity_path},
            rerun::Transform3D{}.with_translation(translation).with_rotation(rotation)
        );
    }

    // ----- log_twist ------------------------------------------------------------
    void RerunSink::log_twist(const std::string_view entity_path, const Twist2D& twist)
    {
        const std::string base{entity_path};
        impl_->stream.log(base + "/v", rerun::Scalars{twist.v()});
        impl_->stream.log(base + "/w", rerun::Scalars{twist.w()});
    }

    // ----- log_scalar -----------------------------------------------------------
    void RerunSink::log_scalar(const std::string_view entity_path, const double value)
    {
        impl_->stream.log(std::string{entity_path}, rerun::Scalars{value});
    }

    // ----- log_axes / log_axes_static ------------------------------------------
    void RerunSink::log_axes(const std::string_view entity_path, const float length)
    {
        impl_->stream.log(std::string{entity_path}, make_axes(length));
    }

    void RerunSink::log_axes_static(const std::string_view entity_path, const float length)
    {
        impl_->stream.log_static(std::string{entity_path}, make_axes(length));
    }

    // ----- log_trail_point / clear_trail ---------------------------------------
    void RerunSink::log_trail_point(const std::string_view trail_path, const double x, const double y)
    {
        // 单点 Points3D 写入: 不需要 C++ 侧累积 vector.
        // Rerun 按 (entity_path, timestamp) 索引存储, viewer 通过
        // visible time range 同时显示一段时间内的所有点, 拼出轨迹效果.
        const std::array<rerun::Position3D, 1> point{

            rerun::Position3D{
                static_cast<float>(x), static_cast<float>(y), 0.0F
            }
        };
        impl_->stream.log(std::string{trail_path}, rerun::Points3D{point});
    }

    void RerunSink::clear_trail(const std::string_view trail_path)
    {
        // Clear archetype: 把当前时间点的实体数据清空.
        // 不清历史(历史属于过去的时间戳, 由 viewer 的 time range 决定可见性).
        // 一般在 episode 切换时调用, 配合 set_time 把新 episode 标到新时间起点.
        impl_->stream.log(std::string{trail_path}, rerun::Clear::FLAT);
    }
}
