set (dji_src 
    dji_mavlink.cpp
    dji_mav_receiver.cpp
    )


add_library(dji "${dji_src}")

target_include_directories (dji PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#target_link_libraries(dji PRIVATE mavlink controller)
target_link_libraries(dji PRIVATE mavlink)