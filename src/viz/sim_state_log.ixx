module;

#include <string_view>

export module mininav.viz.sim_state_log;

import mininav.core.types;
import mininav.viz.rerun_sink;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // 调用约定:
    //   - 每个 register_*_statics 应在仿真循环开始前调用一次.
    //   - 之后每帧调用对应的 log_to_rerun.
    //   - 切换 episode (重置仿真) 时, 调用 reset_trails_* 清空轨迹.
    // ---------------------------------------------------------------------------

    /// 注册仿真的静态可视化(世界轴 / 机器人本体轴).
    void register_v0_statics(RerunSink& sink, std::string_view entity_root);
    void register_v1_statics(RerunSink& sink, std::string_view entity_root);

    // 重置轨迹数据 (调用 RerunSink::clear_trail), 以便新 episode 从空轨迹开始.
    void reset_trails_v0(RerunSink& sink, std::string_view entity_root);
    void reset_trails_v1(RerunSink& sink, std::string_view entity_root);

    // 将仿真状态 log 到 Rerun (调用 RerunSink 的 log_* 方法).
    void log_to_rerun(RerunSink& sink, const SimStateV0& state, std::string_view entity_root);
    void log_to_rerun(RerunSink& sink, const SimStateV1& state, std::string_view entity_root);
}
