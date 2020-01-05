
cmake_minimum_required(VERSION 3.10)
project(msp)

message("System: ${CMAKE_SYSTEM_NAME}")
message("Build type: ${CMAKE_BUILD_TYPE}")

set(TARGET_BINARY_DIR  ${msp-onboard_SOURCE_DIR}/bin)


if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Debug)
endif(NOT CMAKE_BUILD_TYPE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  set(COMMON_CXX_FLAGS "-std=c++17 -fPIC -Wall -Wno-psabi")
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  add_definitions(-DMSP_DEBUG)
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif ()


# link additional libraries
link_libraries(stdc++fs)


# Dji-Onboard-SDK
include_directories("${CMAKE_INSTALL_PREFIX}/include/djiosdk")

# Mavlink-SDK
set(MAVLINK_LIB_HEADER_DIR ../mavlink/c_library_v2)

FILE(GLOB MAVLINK_LIB_HEADERS
        "${MAVLINK_LIB_HEADER_DIR}"
        "${MAVLINK_LIB_HEADER_DIR}/common"
        )

include_directories("${MAVLINK_LIB_HEADERS}")


FILE(GLOB COMMMON_LIB_HEADERS
        "${PROJECT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/lib"
        )

include_directories("${COMMMON_LIB_HEADERS}")
include_directories("${PROJECT_SOURCE_DIR}")

# submodules
if (DJI_OSDK)
    add_subdirectory(modules/dji)
endif(DJI_OSDK)
add_subdirectory(modules/sensors)
add_subdirectory(modules/mavlink)
add_subdirectory(modules/controller)

# executable
add_executable(msp-onboard main.cpp)
target_link_libraries(msp-onboard spdlog::spdlog)
target_link_libraries(msp-onboard sensors)
target_link_libraries(msp-onboard mavlink)
target_link_libraries(msp-onboard controller)


if (DJI_OSDK)
    target_link_libraries(msp-onboard dji djiosdk-core)
endif(DJI_OSDK)

install (TARGETS msp-onboard DESTINATION "${TARGET_BINARY_DIR}")

