module;

#include <filesystem>
#include <memory>
#include <string_view>

export module mininav.viz.rerun_sink;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // RerunSink: 对 Rerun C++ SDK 的薄封装
    // ---------------------------------------------------------------------------
    class RerunSink
    {
    public:
        enum class Mode
        {
            Spawn,
            Save,
        };

        explicit RerunSink(std::string_view application_id);

        RerunSink(std::string_view application_id, const std::filesystem::path& rrd_path);

        ~RerunSink();

        RerunSink(const RerunSink&) = delete;
        RerunSink& operator=(const RerunSink&) = delete;
        RerunSink(RerunSink&&) noexcept;
        RerunSink& operator=(RerunSink&&) noexcept;

        void set_time(double t_seconds) const noexcept;

        void log_pose(std::string_view entity_path, const Pose2D& pose) const noexcept;
        void log_twist(std::string_view entity_path, const Twist2D& twist) const noexcept;
        void log_scalar(std::string_view entity_path, double value) const noexcept;
        void log_axes(std::string_view entity_path, float length = 0.5F) const noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
