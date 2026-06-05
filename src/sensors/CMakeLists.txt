add_library(sensors STATIC)

target_sources(sensors
        PRIVATE
        actuator_model.cpp
        wheel_encoder.cpp
        imu_model.cpp
        PUBLIC
        FILE_SET CXX_MODULES
        FILES
        actuator_model.ixx
        wheel_encoder.ixx
        imu_model.ixx
)

target_link_libraries(sensors PUBLIC core)

set_property(TARGET sensors PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(sensors PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(sensors)