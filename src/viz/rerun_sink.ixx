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
        explicit RerunSink(std::string_view application_id);

        RerunSink(std::string_view application_id, const std::filesystem::path& rrd_path);

        ~RerunSink();

        RerunSink(const RerunSink&) = delete;
        RerunSink& operator=(const RerunSink&) = delete;
        RerunSink(RerunSink&&) noexcept;
        RerunSink& operator=(RerunSink&&) noexcept;

        /// 设置当前帧的时间轴游标. 之后的所有 log 都打在这个时间点.
        void set_time(double t_seconds);

        // ---- 每帧动态数据 ----
        void log_pose(std::string_view entity_path, const Pose2D& pose);
        void log_twist(std::string_view entity_path, const Twist2D& twist);
        void log_scalar(std::string_view entity_path, double value);
        void log_axes(std::string_view entity_path, float length = 0.5F);
        void log_axes_static(std::string_view entity_path, float length = 0.5F);

        void log_trail_point(std::string_view trail_path, double x, double y);

        void clear_trail(std::string_view trail_path);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
