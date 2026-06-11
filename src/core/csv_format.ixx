module;

#include <string>

export module mininav.core.csv_format;

import mininav.core.types;

export namespace mininav
{
    [[nodiscard]] std::string csv_header(const SimState&);
    [[nodiscard]] std::string csv_row(const SimState& record);
}
