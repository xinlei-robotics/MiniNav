module;

#include <string_view>

export module mininav.viz.sink;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // VizSink: 后端无关的可视化下沉接口(纯虚)。
    //
    // 它把"把一帧仿真状态画到某处"抽象成一组与具体后端无关的操作。RerunSink 是
    // 它的 Rerun 实现;测试可注入 gmock 的 MockVizSink,在不启动任何 Viewer 的
    // 前提下验证上层(sim_state_log / 主循环)对该接口的调用次数与参数。
    //
    // 能被 mock 本身就证明:可视化逻辑与 Rerun 后端已被这层接口隔离干净
    // (PIMPL 之上再加一层依赖倒置)。sim_state_log 只认 VizSink&,不再认 Rerun。
    // ---------------------------------------------------------------------------
    class VizSink
    {
    public:
        VizSink() = default;
        virtual ~VizSink();   // out-of-line,锚定 vtable / typeinfo 于 viz 库

        VizSink(const VizSink&) = default;
        VizSink& operator=(const VizSink&) = default;
        VizSink(VizSink&&) = default;
        VizSink& operator=(VizSink&&) = default;

        /// 设置当前帧的时间轴游标;之后的所有 log 都打在这个时间点。
        virtual void set_time(double t_seconds) = 0;

        // ---- 每帧动态数据 ----
        virtual void log_pose(std::string_view entity_path, const Pose2D& pose) = 0;
        virtual void log_twist(std::string_view entity_path, const Twist2D& twist) = 0;
        virtual void log_scalar(std::string_view entity_path, double value) = 0;
        virtual void log_axes(std::string_view entity_path, float length) = 0;
        virtual void log_axes_static(std::string_view entity_path, float length) = 0;
        virtual void log_trail_point(std::string_view trail_path, double x, double y) = 0;

        virtual void clear_trail(std::string_view trail_path) = 0;
    };
}
