# ---------------------------------------------------------------------------
# src/viz/CMakeLists.txt
#
# viz 库:封装项目的可视化层,目前后端是 Rerun SDK。
# ---------------------------------------------------------------------------

add_library(viz STATIC)

target_sources(viz
        PRIVATE
        rerun_sink.cpp
        sim_state_log.cpp

        PUBLIC
        FILE_SET CXX_MODULES FILES
        rerun_sink.ixx
        sim_state_log.ixx
)

target_link_libraries(viz
        PUBLIC core
        PRIVATE rerun_sdk
)

set_property(TARGET viz PROPERTY CXX_SCAN_FOR_MODULES ON)
set_target_properties(viz PROPERTIES LINKER_LANGUAGE CXX)

set(_mininav_venv_bin "${CMAKE_SOURCE_DIR}/.venv/bin")
if (EXISTS "${_mininav_venv_bin}/rerun")
    message(STATUS "MiniNav: Rerun Viewer found at ${_mininav_venv_bin}/rerun")
    target_compile_definitions(viz PRIVATE
            MININAV_RERUN_VIEWER_BIN_DIR="${_mininav_venv_bin}"
    )
else ()
    message(WARNING
            "MiniNav: project venv '${_mininav_venv_bin}' not found. "
            "Spawn mode will rely on the system PATH to locate the Rerun Viewer. "
            "Run `python3 -m venv .venv && .venv/bin/pip install rerun-sdk==0.31.4` "
            "to enable IDE-launched spawn mode.")
    target_compile_definitions(viz PRIVATE
            MININAV_RERUN_VIEWER_BIN_DIR=""
    )
endif ()

mininav_set_warnings(viz)