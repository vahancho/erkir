set (TARGET unittest)

add_executable(${TARGET} test.cpp)

target_link_libraries(${TARGET} PRIVATE erkir)

add_test(NAME unittests COMMAND ${TARGET})

