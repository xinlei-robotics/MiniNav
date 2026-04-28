module;

#include <string>

export module mininav.core.csv_format;

import mininav.core.types;

export namespace mininav
{
    [[nodiscard]] std::string csv_header(const SimStateV0&);
    [[nodiscard]] std::string csv_row(const SimStateV0& record);
}
