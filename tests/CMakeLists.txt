#
# Sources
#
set(ERKIR_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(ERKIR_TESTS_SOURCES ${ERKIR_TESTS_DIR}/test.cpp)

#
# GoogleTest
#
CPMAddPackage(
  NAME googletest
  GITHUB_REPOSITORY google/googletest
  GIT_TAG v1.12.0
  EXCLUDE_FROM_ALL TRUE
  SYSTEM TRUE
  OPTIONS "INSTALL_GTEST FALSE" "gtest_force_shared_crt TRUE"
)
include(GoogleTest)

#
# Test
#
enable_testing()
add_executable(${PROJECT_NAME}_tests ${ERKIR_TESTS_SOURCES})
target_link_libraries(
  ${PROJECT_NAME}_tests PRIVATE GTest::gtest_main $<LINK_LIBRARY:WHOLE_ARCHIVE,erkir::erkir>
)
target_compile_options(
  ${PROJECT_NAME}_tests PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
                                $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)
gtest_discover_tests(${PROJECT_NAME}_tests)

#
# Coverage
#
if(${ERKIR_BUILD_TESTS_COVERAGE})
  append_coverage_compiler_flags_to_target(${PROJECT_NAME}_tests)
  setup_target_for_coverage_lcov(
    NAME
    ${PROJECT_NAME}_coverage
    EXECUTABLE
    ${PROJECT_NAME}_tests
    DEPENDENCIES
    ${PROJECT_NAME}_tests
    erkir::erkir
    LCOV_ARGS
    "--no-external"
    EXCLUDE
    "build/*"
    "tests/*"
  )
endif()
