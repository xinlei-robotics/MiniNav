module;

#include <string_view>

export module mininav.viz.sim_state_log;

import mininav.core.types;
import mininav.viz.sink;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // 调用约定:
    //   - 每个 register_*_statics 应在仿真循环开始前调用一次.
    //   - 之后每帧调用对应的 log_to_rerun.
    //   - 切换 episode (重置仿真) 时, 调用 reset_trails_* 清空轨迹.
    // ---------------------------------------------------------------------------

    /// 注册仿真的静态可视化(世界轴 + truth / odom / ekf 三路本体轴).
    void register_statics(VizSink& sink, std::string_view entity_root);

    // 重置轨迹数据 (调用 VizSink::clear_trail), 以便新 episode 从空轨迹开始.
    void reset_trails(VizSink& sink, std::string_view entity_root);

    // 将仿真状态 log 到可视化下沉 (调用 VizSink 的 log_* 方法).
    // 负责所有可由 SimState 直接导出的实体: truth / odom / ekf 三路 pose+trail、
    // cmd / true_velocity twist、encoder 读数、odom 漂移诊断。
    // 需要状态外信息的 cmd_traj 参考轨迹与 bias 估计/真值对照曲线, 由 app 主循环单独 log。
    void log_to_rerun(VizSink& sink, const SimState& state, std::string_view entity_root);
}
