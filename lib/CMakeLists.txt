cmake_minimum_required(VERSION 3.16.3)

project(C-Mesh-API VERSION 1.4.0 LANGUAGES C)


if(WIN32)
    add_subdirectory(3rdparty/RS-232)
endif()

add_subdirectory(platform)
add_subdirectory(wpc)


#link the actual application to just cmeshapi
add_library(cmeshapi INTERFACE)
target_link_libraries(cmeshapi INTERFACE wpc pthread platform)