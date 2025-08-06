cmake_minimum_required(VERSION 3.18)

project(c-mesh-api LANGUAGES C)

set(WPC_PLATFORM "macos" CACHE STRING "Platform implementation")
option(WPC_BUILD_PROTO_API "Build wpc_proto library" OFF)

add_subdirectory(wpc)

if(${WPC_PLATFORM} STREQUAL "linux")
    add_subdirectory(platform/linux)
    target_link_libraries(wpc wpc_platform)
elseif(${WPC_PLATFORM} STREQUAL "macos")
    add_subdirectory(platform/macos)
    target_link_libraries(wpc wpc_platform)
endif()

if(WPC_BUILD_PROTO_API)
  add_subdirectory(wpc_proto)
  target_link_libraries(wpc_proto wpc)
endif()

