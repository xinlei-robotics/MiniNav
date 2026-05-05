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

    [[nodiscard]] std::string csv_header(const SimStateV1&)
    {
        return "t,"
            "cmd_v,cmd_w,"
            "true_v,true_w,"
            "truth_x,truth_y,truth_yaw,"
            "enc_dl,enc_dr,"
            "odom_x,odom_y,odom_yaw";
    }

    [[nodiscard]] std::string csv_row(const SimStateV1& record)
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
}
