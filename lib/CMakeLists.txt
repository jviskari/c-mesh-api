cmake_minimum_required(VERSION 3.16.3)

project(c-mesh-api VERSION 1.4.0 LANGUAGES C)

add_library(cmeshapi
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/linux/serial_termios2.c
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/linux/logger.c
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/linux/serial.c
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/linux/platform.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc/dsap.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc/wpc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc/slip.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc/wpc_internal.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc/attribute.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc/msap.c
        ${CMAKE_CURRENT_SOURCE_DIR}/wpc/csap.c
)

target_include_directories(cmeshapi
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/api>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/platform>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/platform/linux>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/wpc/include>
)

target_compile_definitions(cmeshapi PUBLIC
                           -D_REENTRANT)

target_link_libraries(cmeshapi pthread)