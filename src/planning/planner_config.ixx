module;

#include <string>
#include <string_view>

export module mininav.planning.planner_config;

import mininav.planning.grid_types;

export namespace mininav::planning
{
    // 把 planner.yaml 文本解析成 PlannerConfig。缺失字段回退到 PlannerConfig 默认值;
    // 非法取值(未知 heuristic、connectivity 非 4/8)抛 std::runtime_error。
    [[nodiscard]] PlannerConfig parse_planner_config(std::string_view yaml_text);

    // 从文件读取并解析 planner.yaml。文件打不开时抛 std::runtime_error。
    [[nodiscard]] PlannerConfig load_planner_config(const std::string& path);

    // 把 PlannerConfig 序列化回 YAML 文本(用于 round-trip 验证 / 落盘)。
    [[nodiscard]] std::string to_yaml(const PlannerConfig& cfg);
}
