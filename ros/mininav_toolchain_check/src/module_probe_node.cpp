// ---------------------------------------------------------------------------
// ModuleProbeNode: 工具链检查用的组件节点。
//
// 同一个翻译单元里先 #include rclcpp,再 import MiniNav 模块(规则:所有 #include
// 写在 import 之前);构造时用 core 的严格解析器读 mininav_core 安装的 robot.yaml,
// 打印几个字段。日志里那一行就是 launch 测试等的信号。
// ---------------------------------------------------------------------------

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <string>

import mininav.core.robot_description;

namespace mininav_toolchain_check
{
    class ModuleProbeNode : public rclcpp::Node
    {
    public:
        explicit ModuleProbeNode(const rclcpp::NodeOptions& options)
            : rclcpp::Node("module_probe", options)
        {
            const std::string path =
                ament_index_cpp::get_package_share_directory("mininav_core") + "/config/robot.yaml";
            const mininav::RobotDescription robot = mininav::load_robot_description(path);

            RCLCPP_INFO(get_logger(),
                        "robot.yaml from mininav_core: wheel_radius=%.4f m, wheel_base=%.4f m, "
                        "ticks_per_rev=%lld, circumscribed_radius=%.4f m",
                        robot.wheel_radius, robot.wheel_base,
                        static_cast<long long>(robot.ticks_per_rev),
                        robot.footprint.circumscribed_radius());
        }
    };
} // namespace mininav_toolchain_check

RCLCPP_COMPONENTS_REGISTER_NODE(mininav_toolchain_check::ModuleProbeNode)
