set (bmp280_src
    bmp280.c
    bmp280i2c.c
    )

set_source_files_properties(bmp280.c PROPERTIES LANGUAGE CXX )
set_source_files_properties(bmp280i2c.c PROPERTIES LANGUAGE CXX )

add_library(bmp280 "${bmp280_src}")
target_include_directories (bmp280 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})