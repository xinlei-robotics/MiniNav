add_library(localization STATIC)

target_sources(localization
        PRIVATE
        wheel_odometry.cpp
        ekf.cpp
        PUBLIC
        FILE_SET CXX_MODULES
        FILES
        wheel_odometry.ixx
        ekf_state.ixx
        ekf.ixx
)

target_link_libraries(localization PUBLIC core)

set_property(TARGET localization PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(localization PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(localization)