module;

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <rerun.hpp>

module mininav.viz.rerun_sink;

import mininav.core.types;

namespace mininav
{
    namespace
    {
        // ---------------------------------------------------------------------
        // ensure_viewer_in_path: 把项目 venv 的 bin 目录前置到当前进程 PATH。
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
    }

    // ---------------------------------------------------------------------------
    // RerunSink::Impl
    // ---------------------------------------------------------------------------
    struct RerunSink::Impl
    {
        rerun::RecordingStream stream;
        std::vector<rerun::Vec2D> trail_points;

        // 仿真时间轴的固定名称,所有 set_time 都用这一根轴。
        // 单位是秒,对应 SimStateV0::t。
        static constexpr auto kTimelineName = "sim_time";

        explicit Impl(std::string_view application_id)
            : stream{std::string{application_id}}
        {
        }
    };

    // ----- 构造函数 -----------------------------------------------------------

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

    RerunSink::RerunSink(const std::string_view application_id,
                         const std::filesystem::path& rrd_path)
        : impl_{std::make_unique<Impl>(application_id)}
    {
        if (rrd_path.has_parent_path())
        {
            std::filesystem::create_directories(rrd_path.parent_path());
        }

        const std::string path_str = rrd_path.string();
        const auto result = impl_->stream.save(path_str);
        if (result.is_err())
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

    // ----- 时间轴 -------------------------------------------------------------

    void RerunSink::set_time(const double t_seconds) const noexcept
    {
        impl_->stream.set_time_duration_secs(Impl::kTimelineName, t_seconds);
    }

    // ----- log_pose -----------------------------------------------------------

    void RerunSink::log_pose(const std::string_view entity_path,
                             const Pose2D& pose) const noexcept
    {
        const float x = static_cast<float>(pose.x());
        const float y = static_cast<float>(pose.y());
        const float yaw = static_cast<float>(pose.yaw());

        const auto translation = rerun::Vec3D{x, y, 0.0F};
        const auto rotation = rerun::RotationAxisAngle{
            {0.0F, 0.0F, 1.0F}, rerun::Angle::radians(yaw)
        };

        impl_->stream.log(
            std::string{entity_path},
            rerun::Transform3D{}.with_translation(translation).with_rotation(rotation)
        );

        impl_->trail_points.emplace_back(x, y);

        const std::string trail_path = std::string{entity_path} + "/trail";
        impl_->stream.log(
            trail_path,
            rerun::LineStrips2D{rerun::components::LineStrip2D{impl_->trail_points}}
        );
    }

    // ----- log_twist ----------------------------------------------------------

    void RerunSink::log_twist(const std::string_view entity_path,
                              const Twist2D& twist) const noexcept
    {
        const std::string base{entity_path};
        impl_->stream.log(base + "/v", rerun::Scalars{twist.v()});
        impl_->stream.log(base + "/w", rerun::Scalars{twist.w()});
    }

    // ----- log_scalar ---------------------------------------------------------

    void RerunSink::log_scalar(const std::string_view entity_path,
                               const double value) const noexcept
    {
        impl_->stream.log(std::string{entity_path}, rerun::Scalars{value});
    }

    // ----- log_axes -----------------------------------------------------------

    void RerunSink::log_axes(const std::string_view entity_path,
                             const float length) const noexcept
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

        impl_->stream.log(
            std::string{entity_path},
            rerun::Arrows3D::from_vectors(vectors)
            .with_origins(origins)
            .with_colors(colors)
        );
    }
}
