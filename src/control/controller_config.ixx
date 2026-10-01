module;

#include <string_view>

export module mininav.control.controller_config;

import mininav.control.pure_pursuit;
import mininav.control.velocity_smoother;
import mininav.control.goal_checker;
import mininav.control.progress_checker;

export namespace mininav::control
{
    // ---------------------------------------------------------------------------
    // ControllerConfig: config/nav.yaml 的 controller 段(对应 Nav2 controller_server
    // 的频率 + RPP 插件参数 + velocity_smoother 的加速度上限)。YAML 里是平铺的
    // key,解析时分到三个子结构。
    // ---------------------------------------------------------------------------
    struct ControllerConfig
    {
        double control_frequency{20.0}; // Hz,控制器节拍;两拍之间零阶保持
        PurePursuitConfig pursuit{};
        VelocitySmootherConfig smoother{};
    };

    // 各段的解析:文本是该段的 YAML 映射(nav.yaml 由 app 按段拆开后传入)。
    // 缺失字段取默认值;未知 key、类型错误、取值不合法一律抛 std::runtime_error。
    // 空文本 = 全部默认。
    [[nodiscard]] ControllerConfig parse_controller_config(std::string_view yaml_text);
    [[nodiscard]] GoalCheckerConfig parse_goal_checker_config(std::string_view yaml_text);
    [[nodiscard]] ProgressCheckerConfig parse_progress_checker_config(std::string_view yaml_text);
}
