option(WPC_USE_PLATFORM_LOGGER "Use platform logger functionality" ON)

set(WPC_PLATFORM_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/platform.c
    ${CMAKE_CURRENT_LIST_DIR}/serial.c
    ${CMAKE_CURRENT_LIST_DIR}/serial_termios2.c
)

if(WPC_USE_PLATFORM_LOGGER)
    list(APPEND WPC_PLATFORM_SOURCES ${CMAKE_CURRENT_LIST_DIR}/logger.c)
endif()

add_library(wpc_platform STATIC ${WPC_PLATFORM_SOURCES})

target_include_directories(wpc_platform PRIVATE
    ${PROJECT_SOURCE_DIR}/api
    ${PROJECT_SOURCE_DIR}/platform
    ${PROJECT_SOURCE_DIR}/wpc/include
)

find_package(Threads REQUIRED)
target_link_libraries(wpc_platform PRIVATE Threads::Threads)
