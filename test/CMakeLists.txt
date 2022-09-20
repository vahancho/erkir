set (TARGET unittest)

add_executable(${TARGET} test.cpp)

target_link_libraries(${TARGET} PRIVATE erkir)

add_test(NAME unittests COMMAND ${TARGET})

# Coverage support.
option(CODE_COVERAGE "Enable coverage reporting" ON)
if (CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Add required flags (GCC & LLVM/Clang)
    target_compile_options(${TARGET} erkir PRIVATE
        -O0        # no optimization
        -g         # generate debug info
        --coverage # sets all required flags
    )
    if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
        target_link_options(${TARGET} erkir PRIVATE --coverage)
    else()
        target_link_libraries(${TARGET} erkir PRIVATE --coverage)
    endif()
endif()
