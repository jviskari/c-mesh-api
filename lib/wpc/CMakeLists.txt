cmake_minimum_required(VERSION 3.16.3)

project(wpc VERSION 1.4.0 LANGUAGES C)

add_library(wpc
        ${CMAKE_CURRENT_SOURCE_DIR}/dsap.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/slip.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc_internal.c
        ${CMAKE_CURRENT_SOURCE_DIR}/attribute.c
        ${CMAKE_CURRENT_SOURCE_DIR}/msap.c
        ${CMAKE_CURRENT_SOURCE_DIR}/csap.c
)

target_include_directories(wpc
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../api>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_definitions(wpc PUBLIC
                           -D_REENTRANT)

target_link_libraries(wpc pthread platform)
