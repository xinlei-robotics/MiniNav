# ---------------------------------------------------------------------------
# src/simulation/CMakeLists.txt
#
# simulation 库(V4 新增):被控对象 Plant —— 执行噪声、传感器仿真与真值积分 ——
# 以及噪声档位表。依赖 core 与 sensors;与 localization 互不依赖,二者之间只流过
# EncoderTicks 与陀螺标量。库名不叫 sim:可执行文件已占用该 target 名。
# ---------------------------------------------------------------------------

add_library(simulation STATIC)

target_sources(simulation
        PRIVATE
        plant.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        noise_presets.ixx
        plant.ixx
)

target_link_libraries(simulation PUBLIC core sensors)

set_property(TARGET simulation PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(simulation PROPERTIES LINKER_LANGUAGE CXX)

mininav_set_warnings(simulation)
