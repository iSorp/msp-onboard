set (sensors_src
    sensors.cpp
    )


add_library(sensors "${sensors_src}")
add_subdirectory(bmp280)


target_include_directories (sensors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sensors bmp280)

