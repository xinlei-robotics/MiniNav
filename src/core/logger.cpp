module;

#include <string_view>

#include <spdlog/spdlog.h>

module mininav.core.logger;

namespace mininav::log
{
    // spdlog 作为后端:logger.ixx 的对外接口保持不变,调用方零改动即获得
    // 分级、彩色、带时间戳的结构化日志。message 始终作为参数(而非格式串)
    // 传入,避免其中的 '{' / '}' 被 fmt 误当占位符解析。
    void log(const LogLevel level, const std::string_view message) noexcept
    {
        try
        {
            switch (level)
            {
            case LogLevel::Info:
                spdlog::info("{}", message);
                break;
            case LogLevel::Warning:
                spdlog::warn("{}", message);
                break;
            case LogLevel::Error:
                spdlog::error("{}", message);
                break;
            }
        }
        catch (...)
        {
            // 日志失败不应影响主流程,且本函数承诺 noexcept。
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
