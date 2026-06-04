# ------------------------------------------------------------------
# core 模块的测试
# ------------------------------------------------------------------
add_executable(core_tests
        core/math_tests.cpp
        core/kinematics_tests.cpp
        core/robot_model_tests.cpp
        core/trajectory_tests.cpp
        core/types_tests.cpp
        core/random_tests.cpp
)

target_link_libraries(core_tests
        PRIVATE
        core
        GTest::gtest_main
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
# localization 模块的测试
# ------------------------------------------------------------------
add_executable(localization_tests
        localization/wheel_odometry_tests.cpp
        localization/ekf_predict_tests.cpp
        localization/ekf_jacobian_finite_diff_tests.cpp
        localization/encoder_observation_tests.cpp
        localization/ekf_encoder_update_tests.cpp
        localization/ekf_imu_update_tests.cpp
        localization/ekf_fusion_rmse_tests.cpp
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