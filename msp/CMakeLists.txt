
cmake_minimum_required(VERSION 3.10)
project(msp)


# Add module path for <name>.cmake conf files
set(CURRENT_CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
 
## Set compiler flags accordingly to the build type


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
FILE(GLOB OSDK_LIB_HEADERS
        ../Onboard-SDK/osdk-core/api/inc
        ../Onboard-SDK/osdk-core/modules/inc/payload
        ../Onboard-SDK/osdk-core/modules/inc/flight
        ../Onboard-SDK/osdk-core/protocol/inc
        ../Onboard-SDK/osdk-core/hal/inc
        ../Onboard-SDK/osdk-core/utility/inc
        ../Onboard-SDK/osdk-core/default/inc
        ../Onboard-SDK/osdk-core/platform/linux/inc
        )


include_directories("${OSDK_LIB_HEADERS}")
include_directories("${PROJECT_SOURCE_DIR}/lib")

# add the executable

add_executable(msp src/main.cpp)
target_link_libraries(msp pthread)
target_link_libraries(msp "${PROJECT_SOURCE_DIR}/lib/libdjiosdk-core.a")


################
# Installation #
################

set(INSTALL_BIN_DIR bin)

install (TARGETS msp DESTINATION bin)
install (FILES MathFunctions.h DESTINATION include)
