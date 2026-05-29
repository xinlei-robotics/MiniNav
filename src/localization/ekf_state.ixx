module;

#include <Eigen/Core>

export module mininav.localization.ekf_state;

export namespace mininav
{
    using Vec5 = Eigen::Vector<double, 5>;
    using Mat5 = Eigen::Matrix<double, 5, 5>;

    // ---------------------------------------------------------------------------
    // StateIdx: 5D 状态向量各分量的下标
    //
    //   x = [ px, py, θ, v, ω ]^T
    // ---------------------------------------------------------------------------
    enum StateIdx : int
    {
        kPx = 0,
        kPy = 1,
        kTheta = 2,
        kV = 3,
        kOmega = 4,
    };

    inline constexpr int kStateDim = 5;

    // ---------------------------------------------------------------------------
    // EkfState5: 一个高斯置信(belief)的完整描述 —— 均值 + 协方差。
    //
    // 采用 Thrun《Probabilistic Robotics》约定:
    //   mu    ↔ μ   (state mean)
    //   Sigma ↔ Σ   (state covariance)
    // ---------------------------------------------------------------------------
    struct EkfState5
    {
        Vec5 mu{Vec5::Zero()};
        Mat5 Sigma{Mat5::Identity()};
    };
}
