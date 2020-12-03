cmake_minimum_required(VERSION 3.9)

project(erkir)
set (CMAKE_CXX_STANDARD 11)

add_library(erkir
    include/cartesianpoint.h
    include/coordinate.h
    include/datum.h
    include/ellipsoidalpoint.h
    include/point.h
    include/sphericalpoint.h
    include/vector3d.h
    src/cartesianpoint.cpp
    src/coordinate.cpp
    src/datum.cpp
    src/ellipsoidalpoint.cpp
    src/point.cpp
    src/sphericalpoint.cpp
    src/vector3d.cpp
)
add_library(erkir::erkir ALIAS erkir)

add_executable(erkir_test
    test/test.cpp
)

target_link_libraries(erkir_test PRIVATE erkir)

# Includes
target_include_directories(erkir      PUBLIC  include)
target_include_directories(erkir_test PRIVATE include)
target_include_directories(erkir_test PRIVATE src)

enable_testing()
add_test(NAME Test COMMAND erkir_test)

# Coverage support.
option(CODE_COVERAGE "Enable coverage reporting" ON)
if (CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Add required flags (GCC & LLVM/Clang)
  target_compile_options(erkir INTERFACE
    -O0        # no optimization
    -g         # generate debug info
    --coverage # sets all required flags
  )
  if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
    target_link_options(erkir INTERFACE --coverage)
  else()
    target_link_libraries(erkir INTERFACE --coverage)
  endif()
endif()
