
# https://cmake.org/cmake-tutorial/

cmake_minimum_required (VERSION 3.10)
project(msp-onboard)

set (MSP_VERSION_MAJOR 1)
set (MSP_VERSION_MINOR 0)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
    option(DJI_OSDK "Use of dji osdk" ON)
    add_definitions(-DDJI_OSDK)
endif()

# Update submodules as needed
# see https://cliutils.gitlab.io/modern-cmake/chapters/projects/submodule.html
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/Onboard-SDK/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()


if (DJI_OSDK)
    add_subdirectory(Onboard-SDK)
endif ()

add_subdirectory(src)
