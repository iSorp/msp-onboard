set (bmp280_src
    bmp280.c
    bmp280i2c.c
    )



add_definitions(-DBMP280)

add_library(bmp280 "${bmp280_src}")
target_include_directories (bmp280 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})