# ---------------------------------------------------------------------------
# ros/mininav_toolchain_check/CMakeLists.txt
#
# 工具链检查包(V5 PR0):用最小的组件节点与 Nav2 控制器插件回答四个问题——
#   1. ament 包能否 find_package(MiniNav) 并 import 模块;
#   2. 同一个翻译单元能否先 #include ROS 头文件再 import 模块;
#   3. GCC 编译的 component_container / controller_server 能否加载并调用 Clang 编译的 .so;
#   4. .so 里的 spdlog / yaml-cpp 是否与 ROS 进程里已加载的是同一份。
# 不含任何业务逻辑;真正的节点与插件在各自的包里。
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.31)

project(mininav_toolchain_check LANGUAGES CXX)

# 与 MiniNav 一致:C++23、不开 GNU 扩展(c++23 与 gnu++23 的 BMI 不能混用)。
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(MiniNav REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 与核心相同的警告集;colcon 默认 RelWithDebInfo,所以 -Werror 不按 Debug 区分。
# 导入目标的头文件按 SYSTEM 处理,警告只管我们自己的代码。
function(mininav_ros_set_warnings target_name)
    target_compile_options(${target_name} PRIVATE
            -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endfunction()

# Jazzy 的 nav2_core 不导出 CMake 目标,只给头文件目录与依赖库变量。
function(mininav_ros_link_nav2_core target_name)
    target_include_directories(${target_name} SYSTEM PRIVATE ${nav2_core_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${nav2_core_LIBRARIES})
endfunction()

# ------------------------------------------------------------------
# 组件节点:读 mininav_core 安装的 robot.yaml 并打印(问题 1、2、3)
# ------------------------------------------------------------------
add_library(module_probe_node SHARED src/module_probe_node.cpp)
target_link_libraries(module_probe_node PRIVATE
        MiniNav::core
        ament_index_cpp::ament_index_cpp
        rclcpp::rclcpp
        rclcpp_components::component
)
mininav_ros_set_warnings(module_probe_node)
rclcpp_components_register_node(module_probe_node
        PLUGIN "mininav_toolchain_check::ModuleProbeNode"
        EXECUTABLE module_probe
)

# ------------------------------------------------------------------
# 控制器插件:经 VelocitySmoother 刹停(问题 1、2、3、4)
# ------------------------------------------------------------------
add_library(stop_controller SHARED src/stop_controller.cpp)
target_link_libraries(stop_controller PRIVATE
        MiniNav::core
        MiniNav::control
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
)
mininav_ros_link_nav2_core(stop_controller)
mininav_ros_set_warnings(stop_controller)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS module_probe_node stop_controller
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# ------------------------------------------------------------------
# 测试
# ------------------------------------------------------------------
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)

    # pluginlib 加载插件,指令与直接调用库逐位相同。
    ament_add_gtest(test_stop_controller test/test_stop_controller.cpp)
    target_link_libraries(test_stop_controller
            MiniNav::core
            MiniNav::control
            pluginlib::pluginlib
            rclcpp::rclcpp
            rclcpp_lifecycle::rclcpp_lifecycle
            ${nav2_core_LIBRARIES}
    )
    target_include_directories(test_stop_controller SYSTEM PRIVATE ${nav2_core_INCLUDE_DIRS})
    mininav_ros_set_warnings(test_stop_controller)

    # 两个 .so 都不带 spdlog / yaml-cpp 的静态副本,动态依赖与 ROS 进程里的是同一份。
    ament_add_pytest_test(test_shared_deps test/test_shared_deps.py
            ENV
            MININAV_CHECK_LIBS=$<TARGET_FILE:module_probe_node>:$<TARGET_FILE:stop_controller>
    )

    # 原装的 component_container 与 controller_server 加载我们的 .so。
    add_launch_test(test/test_stock_loaders.py TIMEOUT 120)
endif ()

ament_package()
