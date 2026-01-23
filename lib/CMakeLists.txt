cmake_minimum_required(VERSION 3.18)

project(c-mesh-api LANGUAGES C)

set(WPC_PLATFORM_DEFAULT "linux")
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(WPC_PLATFORM_DEFAULT "darwin")
endif()

set(WPC_PLATFORM ${WPC_PLATFORM_DEFAULT} CACHE STRING "Platform implementation")
option(WPC_BUILD_PROTO_API "Build wpc_proto library" OFF)

add_subdirectory(wpc)

if(${WPC_PLATFORM} STREQUAL "linux" OR ${WPC_PLATFORM} STREQUAL "darwin")
    add_subdirectory(platform/${WPC_PLATFORM})
    target_link_libraries(wpc wpc_platform)
endif()

if(WPC_BUILD_PROTO_API)
  add_subdirectory(wpc_proto)
  target_link_libraries(wpc_proto wpc)
endif()

