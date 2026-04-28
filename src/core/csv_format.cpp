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
            << record.twist.v() << ','
            << record.twist.w();
        return os.str();
    }
}
