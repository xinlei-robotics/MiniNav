# ---------------------------------------------------------------------------
# spdlog.cmake
#
# spdlog:快速、分级、可路由的 C++ 日志后端,V3 起替换 core/logger 的内置实现。
# 沿用 CLI11 / yaml-cpp 的混合模式:本机装了走 find_package,否则 FetchContent 下载。
# SYSTEM:把 spdlog 头文件当系统库处理,不触发本项目的严格警告。
#
# 仅 core 库(logger 实现单元)链接它;logger.ixx 的对外接口不暴露任何 spdlog
# 类型,因此 spdlog 是纯实现细节,以 PRIVATE 链接进 core,不外泄给下游。
#
# 使用前需要确保:
#   - cmake_minimum_required >= 3.28(本项目已满足)
#   - 若本机未安装 spdlog,则网络可访问 github.com
# ---------------------------------------------------------------------------

include(FetchContent)

set(MININAV_SPDLOG_VERSION "1.15.3" CACHE STRING
        "spdlog release tag to fetch when not found on system")

# 用内置 fmt(不外挂系统 fmt),减少额外依赖面。
set(SPDLOG_FMT_EXTERNAL     OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS      OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLE    OFF CACHE BOOL "" FORCE)
set(SPDLOG_INSTALL          OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v${MININAV_SPDLOG_VERSION}
        GIT_SHALLOW TRUE
        EXCLUDE_FROM_ALL
        SYSTEM
        # FIND_PACKAGE_ARGS: 先尝试 find_package(spdlog),失败后 FetchContent。
        FIND_PACKAGE_ARGS NAMES spdlog
)

FetchContent_MakeAvailable(spdlog)
