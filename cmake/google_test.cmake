# ---------------------------------------------------------------------------
# google_test.cmake
#
# 通过 FetchContent 引入 GoogleTest,锁定到一个稳定 release tag。
# 使用前需要确保:
#   - cmake_minimum_required >= 3.28(本项目已满足)
#   - 网络可访问 github.com
#
# 调用方式:在根 CMakeLists 中 include(google_test) 后,GTest::gtest_main
# 与 GTest::gmock_main 即可作为 link target 使用,gtest_discover_tests()
# 也立即可用。
# ---------------------------------------------------------------------------

include(FetchContent)

# Windows 下需要让 GTest 使用动态 CRT,与项目其他部分一致
# (WSL/Linux 上此设置无副作用)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# 不为 GoogleTest 安装目标生成 install rule,避免污染 install tree
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