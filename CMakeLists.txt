cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

#
# Project
#
project(
  erkir
  VERSION 3.0.0
  LANGUAGES CXX
  HOMEPAGE_URL https://github.com/vahancho/erkir
  DESCRIPTION "C++ library for geodesic and trigonometric calculations"
)

#
# Settings
#
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS FALSE)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#
# Options
#
include(CMakeDependentOption)
option(BUILD_SHARED_LIBS "Build shared library" TRUE)
option(ERKIR_BUILD_DOCS "Build documentation" FALSE)
option(ERKIR_BUILD_TESTS "Build tests" FALSE)
cmake_dependent_option(
  ERKIR_BUILD_TESTS_COVERAGE "Build tests coverage" FALSE "ERKIR_BUILD_TESTS" TRUE
)

#
# Checks
#
if(${ERKIR_BUILD_TESTS_COVERAGE})
  # Compiler (GNU, Clang and Flang)
  get_property(LANGUAGES GLOBAL PROPERTY ENABLED_LANGUAGES)
  foreach(lang ${LANGUAGES})
    # FIXME(carlocorradini) See https://github.com/bilke/cmake-modules/issues/84
    #
    # if(NOT "${CMAKE_${lang}_COMPILER_ID}" MATCHES "GNU|(Apple)?[Cc]lang|(LLVM)?[Ff]lang")
    if(NOT "${CMAKE_${lang}_COMPILER_ID}" MATCHES "GNU")
      message(WARNING "Tests coverage disabled because compiler ${lang} does not support it")
      set(KDIS_BUILD_TESTS_COVERAGE
          FALSE
          CACHE BOOL "Build tests coverage" FORCE
      )
      break()
    endif()
  endforeach()
endif()

#
# Debug postfix
#
if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
  set(CMAKE_DEBUG_POSTFIX d)
endif()

#
# Includes
#
include(GNUInstallDirs)
include(GenerateExportHeader)
include(CMakePackageConfigHelpers)
if(${ERKIR_BUILD_TESTS})
  include(CPM)
  if(${ERKIR_BUILD_TESTS_COVERAGE})
    # Enable C language
    enable_language(C)
    include(CodeCoverage)
  endif()
endif()

#
# Configuration file
#
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h @ONLY
)

#
# Headers & Sources
#
# Headers
set(ERKIR_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
set(ERKIR_HEADERS
    ${ERKIR_HEADERS_DIR}/cartesianpoint.h
    ${ERKIR_HEADERS_DIR}/config.h
    ${ERKIR_HEADERS_DIR}/coordinate.h
    ${ERKIR_HEADERS_DIR}/datum.h
    ${ERKIR_HEADERS_DIR}/ellipsoidalpoint.h
    ${ERKIR_HEADERS_DIR}/erkir.h
    ${ERKIR_HEADERS_DIR}/point.h
    ${ERKIR_HEADERS_DIR}/sphericalpoint.h
    ${ERKIR_HEADERS_DIR}/vector3d.h
)

# Sources
set(ERKIR_SOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(ERKIR_SOURCES
    ${ERKIR_SOURCES_DIR}/cartesianpoint.cpp
    ${ERKIR_SOURCES_DIR}/coordinate.cpp
    ${ERKIR_SOURCES_DIR}/datum.cpp
    ${ERKIR_SOURCES_DIR}/ellipsoidalpoint.cpp
    ${ERKIR_SOURCES_DIR}/point.cpp
    ${ERKIR_SOURCES_DIR}/sphericalpoint.cpp
    ${ERKIR_SOURCES_DIR}/vector3d.cpp
)

#
# Library
#
add_library(${PROJECT_NAME} ${ERKIR_SOURCES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
# Export header
string(TOUPPER ${PROJECT_NAME} ERKIR_PROJECT_NAME_UPPER)
generate_export_header(
  ${PROJECT_NAME} EXPORT_FILE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/export.h
  INCLUDE_GUARD_NAME ${ERKIR_PROJECT_NAME_UPPER}_EXPORT_H_
)
list(APPEND ERKIR_HEADERS ${ERKIR_HEADERS_DIR}/export.h)
# Include directories
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
# Properties
set_target_properties(
  ${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION} DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
)
# Compile options
target_compile_options(
  ${PROJECT_NAME} PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX> $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall
                          -Wextra -Wpedantic -Werror>
)

#
# Configuration
#
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

#
# Install
#
# Directories
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
# Headers
install(FILES ${ERKIR_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
# Help
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/README.md TYPE DOC)
# Export
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
# Package
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

#
# CPack
#
set(CPACK_GENERATOR DEB RPM TXZ TGZ)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PROJECT_DESCRIPTION})
set(CPACK_PACKAGE_FILE_NAME
    "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}.${CMAKE_SYSTEM_PROCESSOR}"
)
set(CPACK_PACKAGE_RPM_DIR "${CMAKE_CURRENT_BINARY_DIR}/_CPack_Packages/Linux/RPM")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Vahan Aghajanyan <vahancho@gmail.com>")
include(CPack)

#
# Documentation
#
if(${ERKIR_BUILD_DOCS})
  add_subdirectory(docs)
endif()

#
# Tests
#
if(${ERKIR_BUILD_TESTS})
  # Coverage
  if(${ERKIR_BUILD_TESTS_COVERAGE})
    append_coverage_compiler_flags_to_target(${PROJECT_NAME})
  endif()
  add_subdirectory(tests)
endif()
