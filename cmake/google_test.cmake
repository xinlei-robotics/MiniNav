# ---------------------------------------------------------------------------
# google_test.cmake
#
# 通过 FetchContent 引入 GoogleTest,锁定到一个稳定 release tag。
# 使用前需要确保:
#   - cmake_minimum_required >= 3.28(本项目已满足)
#   - 网络可访问 github.com
# ---------------------------------------------------------------------------

include(FetchContent)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.15.2
        GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(googletest)

# 启用 CTest 集成
include(GoogleTest)