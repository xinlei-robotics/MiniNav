module;

#include <cstdint>
#include <string>
#include <string_view>

export module mininav.core.robot_description;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // Footprint: 矩形底盘外形(车体系,几何中心在差速轴中点)。
    // 碰撞检测与膨胀半径用外接圆近似:r = √((l/2)² + (w/2)²)。
    // ---------------------------------------------------------------------------
    struct Footprint
    {
        double length{0.0}; // m,沿车头方向
        double width{0.0};  // m

        [[nodiscard]] double circumscribed_radius() const noexcept;
    };

    // ---------------------------------------------------------------------------
    // RobotLimits: 执行器饱和(硬件能力上限)。控制器"允许用多少"(加速度上限、
    // 期望速度)属于导航参数,不在这里。
    // ---------------------------------------------------------------------------
    struct RobotLimits
    {
        double max_linear_vel{0.0};  // m/s
        double max_angular_vel{0.0}; // rad/s
    };

    // ---------------------------------------------------------------------------
    // RobotDescription: 机器人参数的单一来源(config/robot.yaml)。
    //
    // 仿真(被控对象、传感器、EKF 的观测模型)与 V6 实车共用同一份描述;
    // 占位值在实车实测后替换,结论以参数化形式给出即可复用。
    // ---------------------------------------------------------------------------
    struct RobotDescription
    {
        double wheel_radius{0.0};       // m
        double wheel_base{0.0};         // m,左右轮中心距
        std::int64_t ticks_per_rev{0};  // 编码器每圈 tick 数
        Footprint footprint{};
        RobotLimits limits{};
        double actuator_time_constant{0.0}; // s,电机 + 驱动的一阶滞后 τ;0 = 无滞后

        // 每 tick 对应的轮缘弧长 2π·r / ticks_per_rev [m]。表达式与
        // WheelEncoderModel 内部完全一致,保证两边逐位相同。
        [[nodiscard]] double distance_per_tick() const noexcept;
    };

    // 解析 robot.yaml 文本。全部字段必填;未知 key(含 footprint / limits 内层)、
    // 缺失字段、非正值(actuator_time_constant 允许 0)一律抛 std::runtime_error ——
    // 机器人描述写错时静默回退默认值,只会把错误推迟到仿真结果里。
    [[nodiscard]] RobotDescription parse_robot_description(std::string_view yaml_text);

    // 从文件读取并解析。文件打不开时抛 std::runtime_error。
    [[nodiscard]] RobotDescription load_robot_description(const std::string& path);
}
