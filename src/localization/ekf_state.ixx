module;

#include <Eigen/Core>

export module mininav.localization.ekf_state;

export namespace mininav
{
    using Vec6 = Eigen::Vector<double, 6>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;

    // ---------------------------------------------------------------------------
    // StateIdx: 6D 状态向量各分量的下标
    //
    //   x = [ px, py, θ, v, ω, b_ω ]^T
    // ---------------------------------------------------------------------------
    enum StateIdx : int
    {
        kPx = 0,
        kPy = 1,
        kTheta = 2,
        kV = 3,
        kOmega = 4,
        kBiasOmega = 5,
    };

    inline constexpr int kStateDim = 6;

    // ---------------------------------------------------------------------------
    // EkfState6: 一个高斯置信(belief)的完整描述 —— 均值 + 协方差。
    //
    // 采用 Thrun《Probabilistic Robotics》约定:
    //   mu    ↔ μ   (state mean)
    //   Sigma ↔ Σ   (state covariance)
    // ---------------------------------------------------------------------------
    struct EkfState6
    {
        Vec6 mu{Vec6::Zero()};
        Mat6 Sigma{Mat6::Identity()};
    };
}
