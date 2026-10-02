export module mininav.control.velocity_smoother;

import mininav.core.types;

export namespace mininav::control
{
    struct VelocitySmootherConfig
    {
        double max_accel{0.5};         // m/s²,线加速度上限(加减速对称)
        double max_angular_accel{3.0}; // rad/s²
    };

    // 参数合法性检查;不合法抛 std::invalid_argument。
    void validate(const VelocitySmootherConfig& cfg);

    // ---------------------------------------------------------------------------
    // VelocitySmoother: 对齐 nav2_velocity_smoother,按加速度上限限制指令变化率。
    //
    // 两个分量按同一比例 s ∈ [0, 1] 收缩(Nav2 的 scale_velocities = true):
    //   out = last + s·(target − last),s = min(1, a·dt/|Δv|, α·dt/|Δω|)。
    // 若各自独立限幅,从静止起步时 v 受限而 ω 不受限,指令曲率 ω/v 会被放大数倍,
    // 车会偏离控制器算出的圆弧;按同一比例收缩则从静止起步时曲率不变。
    // ---------------------------------------------------------------------------
    class VelocitySmoother
    {
    public:
        explicit VelocitySmoother(const VelocitySmootherConfig& cfg);

        // 以上一拍输出为基准限制变化率,返回本拍输出(dt 为控制周期)。
        [[nodiscard]] Twist2D smooth(const Twist2D& target, double dt) noexcept;

        // 设定"上一拍输出"(如新目标开始时的实际速度)。
        void reset(const Twist2D& current) noexcept { last_ = current; }

        [[nodiscard]] const Twist2D& last() const noexcept { return last_; }

    private:
        VelocitySmootherConfig cfg_;
        Twist2D last_{};
    };
}
