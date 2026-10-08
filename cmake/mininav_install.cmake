# ---------------------------------------------------------------------------
# mininav_install.cmake
#
# MININAV_INSTALL=ON 时把不依赖 ROS 的算法库安装并导出为 CMake 包 MiniNav,供 ROS 2
# 工作空间里的 ament 包 find_package(MiniNav) 后 import 模块:
#   - 静态库(PIC)       → lib/
#   - 模块接口 .ixx      → include/mininav/<lib>/(下游的 CMake 用它们重新生成 BMI)
#   - 导出与配置文件     → lib/cmake/MiniNav/
# viz(Rerun)、sim 与测试不安装。
#
# 需要 CMake >= 3.31:3.28 导入已安装的模块接口时,为其合成的 BMI 目标拿不到
# $<COMPILE_ONLY:…> 依赖的头文件路径,跨导入目标的模块依赖也找不到,两个目标消费同一个
# 导入目标时 Ninja 还会报重复规则(实测 3.28.3 失败、3.31.6 通过)。
# ---------------------------------------------------------------------------

if (CMAKE_VERSION VERSION_LESS 3.31)
    message(FATAL_ERROR
            "MININAV_INSTALL requires CMake >= 3.31 (found ${CMAKE_VERSION}): older versions "
            "cannot import installed C++ module interfaces reliably.")
endif ()

# 导出的目标链接 Eigen / spdlog / yaml-cpp,下游必须能在系统里找到同一份;
# FetchContent 下载的副本不在任何导出集里,也不该进 ROS 进程。
if (NOT MININAV_REQUIRE_SYSTEM_DEPS)
    message(FATAL_ERROR
            "MININAV_INSTALL requires MININAV_REQUIRE_SYSTEM_DEPS=ON: the exported libraries "
            "link Eigen, spdlog and yaml-cpp, which consumers must find on the system.")
endif ()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(_mininav_install_cmake_dir ${CMAKE_INSTALL_LIBDIR}/cmake/MiniNav)

foreach (_lib IN ITEMS core sensors simulation localization planning control)
    # 写进导出文件(IMPORTED_CXX_MODULES_COMPILE_FEATURES):下游按 C++23 重新编译模块接口。
    target_compile_features(${_lib} PUBLIC cxx_std_23)

    install(TARGETS ${_lib}
            EXPORT MiniNavTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            FILE_SET CXX_MODULES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mininav/${_lib}
    )
endforeach ()

install(EXPORT MiniNavTargets
        NAMESPACE MiniNav::
        DESTINATION ${_mininav_install_cmake_dir}
        CXX_MODULES_DIRECTORY cxx-modules
)

configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/MiniNavConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/MiniNavConfig.cmake
        INSTALL_DESTINATION ${_mininav_install_cmake_dir}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/MiniNavConfig.cmake
        DESTINATION ${_mininav_install_cmake_dir}
)
