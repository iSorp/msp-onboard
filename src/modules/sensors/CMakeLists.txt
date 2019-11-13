set (sensors_src
    sensors.cpp
    )


include_directories(bmp280)
add_subdirectory(bmp280)
add_library(sensors "${sensors_src}")


target_include_directories (sensors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sensors PRIVATE bmp280)