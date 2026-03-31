function(mininav_set_warnings target_name)
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${target_name}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wsign-conversion
        )
    elseif (MSVC)
        target_compile_options(${target_name}
            PRIVATE
                /W4
                /permissive-
        )
    endif()
endfunction()