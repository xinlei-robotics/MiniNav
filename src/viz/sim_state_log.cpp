module;

#include <string>
#include <string_view>

module mininav.viz.sim_state_log;

import mininav.core.types;
import mininav.viz.rerun_sink;

namespace mininav
{
    void log_to_rerun(RerunSink& sink,
                      const SimStateV0& state,
                      const std::string_view entity_root) noexcept
    {
        // 实体树设计:
        //   /world          (隐式根)
        //   ├── /origin     世界坐标轴
        //   └── /robot      机器人(由 log_pose 注入 Transform3D,随仿真移动)
        //       ├── /body   机器人本体坐标轴
        //       ├── /trail  轨迹折线
        //       └── /cmd    控制量曲线
        //
        // 每帧都重新 log 世界轴，简单直接。但理论上世界轴可以用 static log
        const std::string body_path = std::string{entity_root} + "/body";
        const std::string cmd_path = std::string{entity_root} + "/cmd";

        sink.log_axes("/world/origin", 1.0F);
        sink.log_pose(entity_root, state.pose);
        sink.log_axes(body_path, 0.5F);
        sink.log_twist(cmd_path, state.twist);
    }
}