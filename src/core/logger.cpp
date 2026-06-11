module;

#include <iostream>
#include <string_view>

module mininav.core.logger;

namespace mininav::log
{
    void log(const LogLevel level, const std::string_view message) noexcept
    {
        switch (level)
        {
        case LogLevel::Info:
            std::cout << "[INFO] " << message << '\n';
            break;
        case LogLevel::Warning:
            std::cout << "[WARNING] " << message << '\n';
            break;
        case LogLevel::Error:
            std::cerr << "[ERROR] " << message << '\n';
            break;
        }
    }

    void info(const std::string_view message) noexcept
    {
        log(LogLevel::Info, message);
    }

    void warning(const std::string_view message) noexcept
    {
        log(LogLevel::Warning, message);
    }

    void error(const std::string_view message) noexcept
    {
        log(LogLevel::Error, message);
    }
}
