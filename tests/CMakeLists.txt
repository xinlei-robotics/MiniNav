# ------------------------------------------------------------------
# core 模块的测试
# ------------------------------------------------------------------
add_executable(core_tests
        core/math_tests.cpp
        core/kinematics_tests.cpp
        core/trajectory_tests.cpp
        core/types_tests.cpp
        core/random_tests.cpp
        core/path_tests.cpp
        core/robot_description_tests.cpp
)

target_link_libraries(core_tests
        PRIVATE
        core
        GTest::gtest_main
)

# robot_description_tests 通过 PROJECT_ROOT_DIR 读取仓库里的 config/robot.yaml。
target_compile_definitions(core_tests PRIVATE
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

set_property(TARGET core_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(core_tests PROPERTIES LINKER_LANGUAGE CXX)

# 通过 GoogleTest 的 CMake 模块自动发现并注册每个 TEST() 用例
gtest_discover_tests(core_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "core"
)

# ------------------------------------------------------------------
# sensors 模块的测试
# ------------------------------------------------------------------
add_executable(sensors_tests
        sensors/actuator_model_tests.cpp
        sensors/wheel_encoder_tests.cpp
        sensors/imu_model_tests.cpp
)

target_link_libraries(sensors_tests
        PRIVATE
        sensors
        GTest::gtest_main
)

set_property(TARGET sensors_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(sensors_tests PROPERTIES LINKER_LANGUAGE CXX)

gtest_discover_tests(sensors_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "sensors"
)

# ------------------------------------------------------------------
# simulation 模块的测试(被控对象 Plant + 噪声档位表)
# ------------------------------------------------------------------
add_executable(simulation_tests
        simulation/plant_tests.cpp
)

target_link_libraries(simulation_tests
        PRIVATE
        simulation
        GTest::gtest_main
)

set_property(TARGET simulation_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(simulation_tests PROPERTIES LINKER_LANGUAGE CXX)

gtest_discover_tests(simulation_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "simulation"
)

# ------------------------------------------------------------------
# localization 模块的测试
# ------------------------------------------------------------------
add_executable(localization_tests
        localization/wheel_odometry_tests.cpp
        localization/ekf_predict_tests.cpp
        localization/ekf_jacobian_finite_diff_tests.cpp
        localization/ekf_integrator_consistency_tests.cpp
        localization/encoder_observation_tests.cpp
        localization/ekf_encoder_update_tests.cpp
        localization/ekf_imu_update_tests.cpp
        localization/ekf_fusion_rmse_tests.cpp
        localization/ekf_bias_observability_tests.cpp
        localization/ekf_nis_tests.cpp
        localization/ekf_pipeline_tests.cpp
)

target_link_libraries(localization_tests
        PRIVATE
        core
        sensors
        localization
        GTest::gtest_main
)

set_property(TARGET localization_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(localization_tests PROPERTIES LINKER_LANGUAGE CXX)

gtest_discover_tests(localization_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "localization"
)

# ------------------------------------------------------------------
# planning 模块的测试
# ------------------------------------------------------------------
add_executable(planning_tests
        planning/grid_types_tests.cpp
        planning/planner_config_tests.cpp
        planning/occupancy_grid_tests.cpp
        planning/map_io_tests.cpp
        planning/inflation_tests.cpp
        planning/astar_tests.cpp
)

target_link_libraries(planning_tests
        PRIVATE
        core
        planning
        GTest::gtest_main
)

# map_io_tests 通过 PROJECT_ROOT_DIR 定位 maps/ 下的手画测试地图。
target_compile_definitions(planning_tests PRIVATE
        PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)

set_property(TARGET planning_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(planning_tests PROPERTIES LINKER_LANGUAGE CXX)

gtest_discover_tests(planning_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "planning"
)

# ------------------------------------------------------------------
# control 模块的测试(Pure Pursuit + 平滑器 + 到达 / 卡住判定 + 配置解析)
# pure_pursuit_analytic_tests 用闭环仿真对照 docs/v4_plan.md §3 的解析结论。
# ------------------------------------------------------------------
add_executable(control_tests
        control/pure_pursuit_tests.cpp
        control/pure_pursuit_analytic_tests.cpp
        control/velocity_smoother_tests.cpp
        control/goal_checker_tests.cpp
        control/controller_config_tests.cpp
)

target_link_libraries(control_tests
        PRIVATE
        control
        GTest::gtest_main
)

set_property(TARGET control_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(control_tests PROPERTIES LINKER_LANGUAGE CXX)

gtest_discover_tests(control_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "control"
)

# ------------------------------------------------------------------
# viz 模块的测试(gmock:用 MockVizSink 隔离 Rerun 后端)
# ------------------------------------------------------------------
add_executable(viz_tests
        viz/viz_sink_log_tests.cpp
)

target_link_libraries(viz_tests
        PRIVATE
        core
        viz
        GTest::gmock_main
)

set_property(TARGET viz_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(viz_tests PROPERTIES LINKER_LANGUAGE CXX)

gtest_discover_tests(viz_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "viz"
)

# ------------------------------------------------------------------
# regression: golden CSV 回归护栏
#
# 端到端地跑 sim,把产出的 CSV 与 tests/golden/ 下提交的基线逐字段比较
# (规则见 tools/csv_compare.ixx 与 golden/README.md)。每个用例是一对 CTest:
#   regression.sim.<name>     运行 sim,FIXTURES_SETUP
#   regression.golden.<name>  csv_compare 比较,FIXTURES_REQUIRED
# 两者都带 regression 标签;单跑比较用例时 CTest 会自动带上它的 fixture。
# ------------------------------------------------------------------
add_library(csv_compare_lib STATIC)

target_sources(csv_compare_lib
        PRIVATE
        tools/csv_compare.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        tools/csv_compare.ixx
)

set_property(TARGET csv_compare_lib PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(csv_compare_lib PROPERTIES LINKER_LANGUAGE CXX)
mininav_set_warnings(csv_compare_lib)

add_executable(csv_compare tools/csv_compare_main.cpp)
target_link_libraries(csv_compare PRIVATE csv_compare_lib)
set_property(TARGET csv_compare PROPERTY CXX_SCAN_FOR_MODULES ON)
mininav_set_warnings(csv_compare)

add_executable(csv_compare_tests tools/csv_compare_tests.cpp)
target_link_libraries(csv_compare_tests PRIVATE csv_compare_lib GTest::gtest_main)
set_property(TARGET csv_compare_tests PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(csv_compare_tests PROPERTIES LINKER_LANGUAGE CXX)

gtest_discover_tests(csv_compare_tests
        DISCOVERY_TIMEOUT 300
        PROPERTIES
        LABELS "regression"
)

set(MININAV_GOLDEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/golden")
set(MININAV_GOLDEN_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/golden_out")

# 每次回归前清空上一轮产出:sim 若没写出文件,比较端不会误用旧 CSV。
add_test(NAME regression.clean
        COMMAND ${CMAKE_COMMAND} -E rm -rf "${MININAV_GOLDEN_OUT_DIR}")
set_tests_properties(regression.clean PROPERTIES
        FIXTURES_SETUP golden_clean
        LABELS "regression")

# update_golden 目标的命令,由 mininav_add_golden_test 逐条追加。
set(_mininav_golden_update_commands "")

# mininav_add_golden_test(<name> <golden-file> <subcommand> <sim args...>)
#   sim 在源码根目录运行,地图等路径用相对路径写(它们会进 CSV 头部元数据,
#   绝对路径会让基线依赖于检出位置)。--no-viz 与 --out 由本函数追加。
function(mininav_add_golden_test name golden_file)
    set(out "${MININAV_GOLDEN_OUT_DIR}/${golden_file}")

    add_test(NAME regression.sim.${name}
            COMMAND $<TARGET_FILE:sim> ${ARGN} --no-viz --out "${out}"
            WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
    set_tests_properties(regression.sim.${name} PROPERTIES
            FIXTURES_REQUIRED golden_clean
            FIXTURES_SETUP golden_${name}
            LABELS "regression")

    add_test(NAME regression.golden.${name}
            COMMAND $<TARGET_FILE:csv_compare> "${MININAV_GOLDEN_DIR}/${golden_file}" "${out}")
    set_tests_properties(regression.golden.${name} PROPERTIES
            FIXTURES_REQUIRED golden_${name}
            LABELS "regression")

    set(_mininav_golden_update_commands ${_mininav_golden_update_commands}
            COMMAND $<TARGET_FILE:sim> ${ARGN} --no-viz --out "${MININAV_GOLDEN_DIR}/${golden_file}"
            PARENT_SCOPE)
endfunction()

# EKF 模式:三档噪声各一条(high-noise 下在线 bias 估计会发散,正好锁住这条路径)。
mininav_add_golden_test(ekf_default_seed42 ekf_default_seed42.csv
        ekf --seed 42 --preset default)
mininav_add_golden_test(ekf_high_noise_seed7 ekf_high_noise_seed7.csv
        ekf --seed 7 --preset high-noise)
mininav_add_golden_test(ekf_low_noise_seed3 ekf_low_noise_seed3.csv
        ekf --seed 3 --preset low-noise)

# 规划模式:手画 office 与 500×500 的 office500(docs/v3_summary.md §7.1 的命令)。
mininav_add_golden_test(plan_office plan_office.csv
        plan --map maps/office.yaml --start 0.15,0.15 --goal 1.85,1.35
        --config config/planner.yaml)
mininav_add_golden_test(plan_office500 plan_office500.csv
        plan --map maps/office500.yaml --start 1.175,1.175 --goal 23.875,23.875
        --config config/planner.yaml)

# 刻意更新基线:cmake --build <build-dir> --target update_golden
# 不属于 all;更新 golden 的 PR 必须在描述里写明原因。
add_custom_target(update_golden
        ${_mininav_golden_update_commands}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Regenerating golden CSVs in tests/golden/ (state the reason in the PR)"
        VERBATIM)
add_dependencies(update_golden sim)
