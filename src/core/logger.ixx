module;

#include <string_view>

export module mininav.core.logger;

export namespace mininav::log
{
    enum class LogLevel { Info, Warning, Error };

    void log(LogLevel level, std::string_view message) noexcept;
    void info(std::string_view message) noexcept;
    void warning(std::string_view message) noexcept;
    void error(std::string_view message) noexcept;
}
