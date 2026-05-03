# ---------------------------------------------------------------------------
# rerun.cmake
#
# 使用前需要确保:
#   - cmake_minimum_required >= 3.24
#   - 网络可访问 github.com
#   - 本机已安装匹配版本的 Rerun Viewer
# ---------------------------------------------------------------------------

include(FetchContent)

set(MININAV_RERUN_VERSION "0.31.4" CACHE STRING
        "Rerun SDK version. Must match the locally installed Rerun Viewer.")

set(RERUN_DOWNLOAD_AND_BUILD_ARROW ON CACHE BOOL
        "Let Rerun download and build Apache Arrow (recommended for first-time setup).")

FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/download/${MININAV_RERUN_VERSION}/rerun_cpp_sdk.zip
        # SYSTEM: 把 rerun_sdk 当系统库处理,其头文件不触发本项目的严格警告。
        SYSTEM

        EXCLUDE_FROM_ALL
        # FIND_PACKAGE_ARGS: 先尝试 find_package(rerun_sdk),失败后 FetchContent。
        FIND_PACKAGE_ARGS NAMES rerun_sdk
)

FetchContent_MakeAvailable(rerun_sdk)