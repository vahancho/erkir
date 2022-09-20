add_executable(unittest
    test.cpp
)

target_link_libraries(unittest PRIVATE erkir)

add_test(NAME unittests COMMAND unittest)

# Coverage support.
option(CODE_COVERAGE "Enable coverage reporting" ON)
if (CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Add required flags (GCC & LLVM/Clang)
    target_compile_options(polytest PUBLIC
        -O0        # no optimization
        -g         # generate debug info
        --coverage # sets all required flags
    )
    if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
        target_link_options(${TARGET} PUBLIC --coverage)
    else()
        target_link_libraries(${TARGET} PUBLIC --coverage)
    endif()
endif()
