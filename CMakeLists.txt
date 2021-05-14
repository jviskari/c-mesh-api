cmake_minimum_required(VERSION 3.16.3)
project(c-mesh-api-example VERSION 1.4.0 LANGUAGES C)
include(CTest)

add_subdirectory(lib)
add_subdirectory(example)
add_subdirectory(test)

