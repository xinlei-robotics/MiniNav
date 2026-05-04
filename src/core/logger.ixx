module;

#include <iostream>
#include <string_view>

export module mininav.core.logger;

export namespace mininav::log
{
    enum class LogLevel { Info, Warning, Error };

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
    };
}
