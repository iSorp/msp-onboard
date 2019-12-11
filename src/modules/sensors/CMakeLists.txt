# *  @Copyright (c) 2019 Simon Waelti

set (sensors_src
    sensors.cpp
    )


add_library(sensors "${sensors_src}")
add_subdirectory(bmp280)


target_link_libraries(sensors PRIVATE spdlog::spdlog)
target_link_libraries(sensors PRIVATE bmp280)

target_include_directories (sensors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
