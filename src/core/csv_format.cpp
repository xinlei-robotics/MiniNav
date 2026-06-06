module;

#include <ios>
#include <limits>
#include <sstream>
#include <string>

module mininav.core.csv_format;

import mininav.core.types;

namespace mininav
{
    namespace
    {
        void configure_stream(std::ostringstream& os)
        {
            os.precision(std::numeric_limits<double>::max_digits10);
            os << std::scientific;
        }
    }

    std::string csv_header(const SimStateV0&)
    {
        return "t,x,y,yaw,v,w";
    }

    std::string csv_row(const SimStateV0& record)
    {
        std::ostringstream os;
        configure_stream(os);
        os << record.t << ','
            << record.pose.x() << ','
            << record.pose.y() << ','
            << record.pose.yaw() << ','
            << record.cmd.v() << ','
            << record.cmd.w();
        return os.str();
    }

    std::string csv_header(const SimStateV1&)
    {
        return "t,"
            "cmd_v,cmd_w,"
            "true_v,true_w,"
            "truth_x,truth_y,truth_yaw,"
            "enc_dl,enc_dr,"
            "odom_x,odom_y,odom_yaw";
    }

    std::string csv_row(const SimStateV1& record)
    {
        std::ostringstream os;
        configure_stream(os);
        os << record.t << ','
            << record.cmd.v() << ',' << record.cmd.w() << ','
            << record.true_velocity.v() << ',' << record.true_velocity.w() << ','
            << record.truth_pose.x() << ','
            << record.truth_pose.y() << ','
            << record.truth_pose.yaw() << ','
            << record.enc_dticks.left << ','
            << record.enc_dticks.right << ','
            << record.odom_pose.x() << ','
            << record.odom_pose.y() << ','
            << record.odom_pose.yaw();

        return os.str();
    }

    // -------------------------------------------------------------------------
    // SimStateV2: V1 的 13 列 + imu_omega + 6 列 EKF 均值 + 7 列协方差 + 2 列 NIS = 28 列。
    //
    // ekf_cov 是 6×6, 索引约定: 0=px 1=py 2=θ 3=v 4=ω 5=b_ω。
    //
    // 协方差量只导出 7 个:
    //   - 位置 2×2 子块的三项 (xx, yy, xy)，供 3σ 不确定椭圆使用。;
    //   - 另 4 个对角项 (θθ, vv, ωω, b_ωb_ω)，供 3σ 包络曲线 / bias 学习曲线使用。
    // -------------------------------------------------------------------------
    std::string csv_header(const SimStateV2&)
    {
        return "t,"
            "cmd_v,cmd_w,"
            "true_v,true_w,"
            "truth_x,truth_y,truth_yaw,"
            "enc_dl,enc_dr,"
            "imu_omega,"
            "odom_x,odom_y,odom_yaw,"
            "ekf_x,ekf_y,ekf_yaw,ekf_v,ekf_omega,ekf_bias_omega,"
            "ekf_sigma_xx,ekf_sigma_yy,ekf_sigma_xy,"
            "ekf_sigma_thth,ekf_sigma_vv,ekf_sigma_ww,ekf_sigma_bb,"
            "nis_encoder,nis_imu";
    }

    std::string csv_row(const SimStateV2& record)
    {
        std::ostringstream os;
        configure_stream(os);
        os << record.t << ','
            << record.cmd.v() << ',' << record.cmd.w() << ','
            << record.true_velocity.v() << ',' << record.true_velocity.w() << ','
            << record.truth_pose.x() << ','
            << record.truth_pose.y() << ','
            << record.truth_pose.yaw() << ','
            << record.enc_dticks.left << ','
            << record.enc_dticks.right << ','
            << record.imu_omega << ','
            << record.odom_pose.x() << ','
            << record.odom_pose.y() << ','
            << record.odom_pose.yaw() << ','
            // EKF mean: [px, py, θ, v, ω, b_ω]
            << record.ekf_mean(0) << ','
            << record.ekf_mean(1) << ','
            << record.ekf_mean(2) << ','
            << record.ekf_mean(3) << ','
            << record.ekf_mean(4) << ','
            << record.ekf_mean(5) << ','
            // EKF covariance(选取项)
            << record.ekf_cov(0, 0) << ',' // sigma_xx
            << record.ekf_cov(1, 1) << ',' // sigma_yy
            << record.ekf_cov(0, 1) << ',' // sigma_xy
            << record.ekf_cov(2, 2) << ',' // sigma_thth
            << record.ekf_cov(3, 3) << ',' // sigma_vv
            << record.ekf_cov(4, 4) << ',' // sigma_ww
            << record.ekf_cov(5, 5) << ',' // sigma_bb
            << record.nis_encoder << ','
            << record.nis_imu;

        return os.str();
    }
}
