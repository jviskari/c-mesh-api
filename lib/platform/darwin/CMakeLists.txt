option(WPC_USE_PLATFORM_LOGGER "Use platform logger functionality" ON)

add_library(wpc_platform STATIC
    ${CMAKE_CURRENT_LIST_DIR}/platform.c
    ${CMAKE_CURRENT_LIST_DIR}/serial.c
)

if(WPC_USE_PLATFORM_LOGGER)
    list(APPEND WPC_PLATFORM_SOURCES ${CMAKE_CURRENT_LIST_DIR}/logger.c)
endif()

target_include_directories(wpc_platform PRIVATE
    ${PROJECT_SOURCE_DIR}/api
    ${PROJECT_SOURCE_DIR}/platform
    ${PROJECT_SOURCE_DIR}/wpc/include
)

find_package(Threads REQUIRED)
target_link_libraries(wpc_platform PRIVATE Threads::Threads)
