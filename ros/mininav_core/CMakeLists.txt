# ---------------------------------------------------------------------------
# ros/mininav_core/CMakeLists.txt
#
# 包装包:在 ROS 2 工作空间里构建仓库根目录的 MiniNav 项目,只要不依赖 ROS 的算法库,
# 并安装:
#   - CMake 包 MiniNav(静态库 + 模块接口 + 导出文件,见 cmake/mininav_install.cmake);
#   - maps/ 与 config/ → share/mininav_core/(sim 与 ROS 节点读同一份数据)。
# 根目录不放 package.xml:核心不是 ROS 包,而且 colcon 在某个目录认出包后就不再往下找。
#
# 用 ament_cmake 而不是 build_type: cmake:colcon 不把后者的安装前缀加进
# AMENT_PREFIX_PATH,也不登记 ament 索引,launch 文件里的 FindPackageShare('mininav_core')
# 就找不到 maps/ 与 config/。核心库本身仍然不依赖 ROS,ament 只出现在这一层。
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.31)

project(mininav_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

get_filename_component(MININAV_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# 关掉 viz,sim 与测试随之关闭,不拉 Rerun / CLI11 / GoogleTest(它们归核心 CI)。
# spdlog / yaml-cpp / Eigen 只认系统里的,与 ROS / Nav2 进程里已加载的是同一份。
set(MININAV_WITH_VIZ OFF)
set(MININAV_REQUIRE_SYSTEM_DEPS ON)
set(MININAV_INSTALL ON)
add_subdirectory(${MININAV_ROOT} mininav)

install(DIRECTORY ${MININAV_ROOT}/maps/
        DESTINATION share/${PROJECT_NAME}/maps
        PATTERN src EXCLUDE
)
install(DIRECTORY ${MININAV_ROOT}/config/
        DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
