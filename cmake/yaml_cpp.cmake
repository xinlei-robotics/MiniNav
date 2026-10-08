# ---------------------------------------------------------------------------
# yaml_cpp.cmake
#
# yaml-cpp:YAML 解析 / 生成,用于 V3 的 planner.yaml 与 map.yaml 配置。
# 沿用 CLI11 的混合模式:本机装了走 find_package,否则 FetchContent 下载。
# SYSTEM:把 yaml-cpp 头文件当系统库处理,不触发本项目的严格警告。
#
# 使用前需要确保:
#   - cmake_minimum_required >= 3.28(本项目已满足)
#   - 若本机未安装 yaml-cpp,则网络可访问 github.com
# ---------------------------------------------------------------------------

set(MININAV_YAML_CPP_VERSION "0.8.0" CACHE STRING
        "yaml-cpp release tag to fetch when not found on system")

if (MININAV_REQUIRE_SYSTEM_DEPS)
    # ROS 构建只认系统安装的 yaml-cpp,找不到就失败、不回退到下载:插件要与 ROS / Nav2
    # 进程里已加载的是同一份(理由同 spdlog.cmake)。
    find_package(yaml-cpp REQUIRED)
else ()
    include(FetchContent)

    set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_INSTALL       OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(
            yaml-cpp
            GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
            GIT_TAG ${MININAV_YAML_CPP_VERSION}
            GIT_SHALLOW TRUE
            EXCLUDE_FROM_ALL
            SYSTEM
            # FIND_PACKAGE_ARGS: 先尝试 find_package(yaml-cpp),失败后 FetchContent。
            FIND_PACKAGE_ARGS NAMES yaml-cpp
    )

    FetchContent_MakeAvailable(yaml-cpp)
endif ()

# 统一暴露 yaml-cpp::yaml-cpp 别名:老版本 find_package 只给非命名空间 target,
# 让下游一律链接命名空间形式,屏蔽来源差异。
if (TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
    add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
