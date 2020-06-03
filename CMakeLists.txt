cmake_minimum_required(VERSION 3.12)

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

add_executable(erkir_test
    test/test.cpp
)

target_link_libraries(erkir_test PRIVATE erkir)

# Includes
target_include_directories(erkir      PRIVATE include)
target_include_directories(erkir_test PRIVATE include)
target_include_directories(erkir_test PRIVATE src)

enable_testing()
add_test(NAME Test COMMAND erkir_test)
