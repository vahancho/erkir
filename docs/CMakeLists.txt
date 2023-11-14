find_package(Doxygen)

if(DOXYGEN_FOUND)
  set(DOXYGEN_IN "${CMAKE_SOURCE_DIR}/Doxyfile.in")
  set(DOXYGEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

  add_custom_target(
    ${PROJECT_NAME}_docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
else()
  message(STATUS "Doxygen need to be installed to generate the documentation")
endif()
