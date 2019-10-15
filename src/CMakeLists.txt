
cmake_minimum_required(VERSION 3.10)
project(msp)

message("System: ${CMAKE_SYSTEM_NAME}")
message("Build type: ${CMAKE_BUILD_TYPE}")

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  set(COMMON_CXX_FLAGS "-std=c++11 -pthread -fPIC")
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0 -Wall") #-Werror
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif ()


## Dji-Onboard-SDK
set(OSDK_LIB_HEADER_DIR ../Onboard-SDK/osdk-core)
set(OSDK_LIB_DIR ../Onboard-SDK/libs)

FILE(GLOB OSDK_LIB_HEADERS
        "${OSDK_LIB_HEADER_DIR}/api/inc"
        "${OSDK_LIB_HEADER_DIR}/modules/inc/payload"
        "${OSDK_LIB_HEADER_DIR}/modules/inc/flight"
        "${OSDK_LIB_HEADER_DIR}/protocol/inc"
        "${OSDK_LIB_HEADER_DIR}/hal/inc"
        "${OSDK_LIB_HEADER_DIR}/utility/inc"
        "${OSDK_LIB_HEADER_DIR}/default/inc"
        "${OSDK_LIB_HEADER_DIR}/platform/linux/inc"
        )
include_directories("${OSDK_LIB_HEADERS}")

# Mavlink-SDK
set(MAVLINK_LIB_HEADER_DIR ../mavlink/v1.0)

FILE(GLOB MAVLINK_LIB_HEADERS
        "${MAVLINK_LIB_HEADER_DIR}/"
        "${MAVLINK_LIB_HEADER_DIR}/common"
        )

include_directories("${MAVLINK_LIB_HEADERS}")

# subdirectories
add_subdirectory(modules/mavlink)

# common includes
include_directories("${PROJECT_SOURCE_DIR}/lib")

# common libraries
link_directories("${OSDK_LIB_DIR}")

# executable
add_executable(msp-onboard main.cpp)
target_link_libraries(msp-onboard pthread)
target_link_libraries(msp-onboard mavlink)

if (DJI_OSDK)
    target_link_libraries(msp libdjiosdk-core)
endif(DJI_OSDK)

install (TARGETS msp-onboard DESTINATION "${msp-onboard_SOURCE_DIR}/bin")

