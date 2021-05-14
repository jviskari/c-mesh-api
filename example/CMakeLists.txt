cmake_minimum_required(VERSION 3.16.3)

project(example VERSION 1.4.0 LANGUAGES C)

add_executable(example main.c)
target_link_libraries(example cmeshapi)