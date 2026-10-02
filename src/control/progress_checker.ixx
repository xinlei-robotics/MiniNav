export module mininav.control.progress_checker;

import mininav.core.types;
import mininav.control.controller;

export namespace mininav::control
{
    struct ProgressCheckerConfig
    {
        double required_movement{0.20}; // m,时间窗内必须离开基准点的距离
        double time_allowance{10.0};    // s
    };

    // 参数合法性检查;不合法抛 std::invalid_argument。
    void validate(const ProgressCheckerConfig& cfg);

    // ---------------------------------------------------------------------------
    // SimpleProgressChecker: 对齐 Nav2 的 SimpleProgressChecker。
    // 记一个基准位姿与时间;车离开基准点超过 required_movement 就把基准更新为当前
    // 位姿;距上次更新超过 time_allowance 仍未离开则判为卡住。第一次调用只设基准。
    // ---------------------------------------------------------------------------
    class SimpleProgressChecker final : public ProgressChecker
    {
    public:
        explicit SimpleProgressChecker(const ProgressCheckerConfig& cfg);

        void reset() override { has_baseline_ = false; }
        [[nodiscard]] bool check(const Pose2D& pose, double t) override;

    private:
        ProgressCheckerConfig cfg_;
        bool has_baseline_{false};
        Pose2D baseline_pose_{};
        double baseline_time_{0.0};
    };
}
