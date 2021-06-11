cmake_minimum_required(VERSION 3.16.3)

project(serial_port VERSION 1.4.0 LANGUAGES C)

if(WIN32)
    add_library(win32
    ${CMAKE_CURRENT_SOURCE_DIR}/win32/logger.c
    ${CMAKE_CURRENT_SOURCE_DIR}/win32/platform.c
    ${CMAKE_CURRENT_SOURCE_DIR}/win32/winserial.c
    )

    target_include_directories(win32
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/win32>
    )

    target_compile_definitions(win32 PUBLIC
                    -D_REENTRANT)

    target_link_libraries(win32 wpc rs232)
    #use platform as dependency
    add_library(platform ALIAS win32)
else()
    add_library(linux
    ${CMAKE_CURRENT_SOURCE_DIR}/linux/serial_termios2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linux/logger.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linux/serial.c
    ${CMAKE_CURRENT_SOURCE_DIR}/linux/platform.c
    )

    target_include_directories(linux
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/linux>
    )

    target_compile_definitions(linux PUBLIC
                    -D_REENTRANT)

    target_link_libraries(linux wpc)
    #use platform as dependency
    add_library(platform ALIAS linux)
endif()