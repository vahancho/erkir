set(TARGET erkir)

set(HEADERS
    ${CMAKE_SOURCE_DIR}/include/cartesianpoint.h
    ${CMAKE_SOURCE_DIR}/include/coordinate.h
    ${CMAKE_SOURCE_DIR}/include/datum.h
    ${CMAKE_SOURCE_DIR}/include/ellipsoidalpoint.h
    ${CMAKE_SOURCE_DIR}/include/export.h
    ${CMAKE_SOURCE_DIR}/include/point.h
    ${CMAKE_SOURCE_DIR}/include/sphericalpoint.h
    ${CMAKE_SOURCE_DIR}/include/vector3d.h
)

add_library(${TARGET}
    ${HEADERS}
    cartesianpoint.cpp
    coordinate.cpp
    datum.cpp
    ellipsoidalpoint.cpp
    point.cpp
    sphericalpoint.cpp
    vector3d.cpp
)

add_library(erkir::erkir ALIAS erkir)

# Append a postfix for the debug version of the library
set_target_properties(${TARGET} PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")

# Specify the include directories for the target
target_include_directories(${TARGET}
                           PUBLIC
                           "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

if (MSVC)
    target_compile_definitions(${TARGET} PUBLIC MAKEDLL)
endif()

###############################################################################
# The installation and packaging
#

# The installation directories
install(TARGETS  ${TARGET}
        EXPORT   ${TARGET}Targets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install header files
install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install help files
install(FILES ${PROJECT_SOURCE_DIR}/README.md TYPE DOC)

# Install the export
install(EXPORT ${TARGET}Targets
        FILE ${TARGET}Targets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET}
)

# Packaging
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET}
)

# Creating a Package Configuration File
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET}
)

# Creating a Package Version File
set_property(TARGET ${TARGET} PROPERTY VERSION ${CMAKE_PROJECT_VERSION})
set_property(TARGET ${TARGET} PROPERTY INTERFACE_${TARGET}_MAJOR_VERSION ${PROJECT_VERSION_MAJOR})
set_property(TARGET ${TARGET} APPEND PROPERTY COMPATIBLE_INTERFACE_STRING ${TARGET}_MAJOR_VERSION)

# Generate the version file for the configuration file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}ConfigVersion.cmake"
    VERSION "${CMAKE_PROJECT_VERSION}"
    COMPATIBILITY AnyNewerVersion
)

# Exporting targets from the build tree if we want our project to also be used
# from a build directory
export(EXPORT ${TARGET}Targets
       FILE "${CMAKE_CURRENT_BINARY_DIR}/../cmake/${TARGET}Targets.cmake"
)

# Set the name of the package
string(TOLOWER
       "${CMAKE_PROJECT_NAME}-${CMAKE_PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${BIT}-${CMAKE_BUILD_TYPE}"
       CPACK_PACKAGE_FILE_NAME)

# Enable packaging with CPack
include(CPack)
