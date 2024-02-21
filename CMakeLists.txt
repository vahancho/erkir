cmake_minimum_required(VERSION 3.9)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The project definition
project(erkir VERSION 2.0.1
        DESCRIPTION "C++ library for geodesic and trigonometric calculations"
        LANGUAGES CXX)

# Make cache variables for install destinations
include(GNUInstallDirs)

# General options
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
option(ENABLE_TESTING "Enable unit test build" OFF)

if (NOT MSVC)
    # Specify the target architecture (Linux). For Windows based generator use rather
    # '-A Win32' or '-A x64 options'
    set(TARGET_ARCH x86 CACHE STRING "the target architecture")
    set_property(CACHE TARGET_ARCH PROPERTY STRINGS x86 x64)

    if (TARGET_ARCH STREQUAL "x86")
        set(CMAKE_C_FLAGS   -m32)
        set(CMAKE_CXX_FLAGS -m32)
    elseif(TARGET_ARCH STREQUAL "x64")
        set(CMAKE_C_FLAGS   -m64)
        set(CMAKE_CXX_FLAGS -m64)
    else()
        message(FATAL_ERROR "Incorrect target architecture specified. "
                            "It should be either x86 or x64")
    endif()

    # Get the architecture: x86 vs x64
    if(TARGET_ARCH STREQUAL "x64")
        set(BIT "x86_64")
    else()
        set(BIT "x86")
    endif()

    add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

# Append a postfix for the debug libraries
if (NOT CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX d)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if (ENABLE_TESTING)
    # Coverage support.
    option(CODE_COVERAGE "Enable coverage reporting" ON)
    if (CODE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        # Add required flags (GCC & LLVM/Clang)
	    add_compile_options(-O0        # no optimization
                            -g         # generate debug info
			                --coverage # sets all required flags
	    )
	    if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
	        add_link_options(--coverage)
	    else()
	        link_libraries(--coverage)
	    endif()
    endif()
endif()

add_subdirectory(src)

if (ENABLE_TESTING)
    include(CTest)
    enable_testing()

    add_subdirectory(test)
endif()
