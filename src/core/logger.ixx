module;

#include <iostream>
#include <string_view>

export module mininav.core.logger;

export namespace mininav {

enum class LogLevel { Info, Warning, Error };

class Logger {
public:
  void log(LogLevel level, std::string_view message) const noexcept {
    switch (level) {
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
  void info(std::string_view message) const noexcept {
    log(LogLevel::Info, message);
  }
  void warning(std::string_view message) const noexcept {
    log(LogLevel::Warning, message);
  }
  void error(std::string_view message) const noexcept {
    log(LogLevel::Error, message);
  }
};
} // namespace mininav