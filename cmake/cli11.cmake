# CLI11: header-only command-line option parser.

include(FetchContent)

set(MININAV_CLI11_VERSION "2.4.2" CACHE STRING
        "CLI11 release tag to fetch when not found on system")

set(CLI11_PRECOMPILED OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CLI11_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
        CLI11
        GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
        GIT_TAG v${MININAV_CLI11_VERSION}
        GIT_SHALLOW TRUE
        EXCLUDE_FROM_ALL
        SYSTEM
        FIND_PACKAGE_ARGS NAMES CLI11
)

FetchContent_MakeAvailable(CLI11)