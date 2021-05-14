cmake_minimum_required(VERSION 3.16.3)

project(example VERSION 1.4.0 LANGUAGES C)

add_executable(meshAPItest test_app.c)

add_library(cmeshtest test.c)
target_link_libraries(cmeshtest cmeshapi)

target_link_libraries(meshAPItest cmeshapi cmeshtest)
target_compile_definitions(meshAPItest PUBLIC
                           -D_REENTRANT)

add_test(test_all meshAPItest)