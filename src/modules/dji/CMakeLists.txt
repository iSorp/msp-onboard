set (dji_src 
    dji_linux_environment.cpp
    dji_linux_helpers.cpp
    dji_mavlink.cpp
    dji_mobile_interface.cpp
    dji_mission_interface.cpp
    )


add_library(dji "${dji_src}")

target_include_directories (dji PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(dji PRIVATE mavlink controller)
target_link_libraries(dji PRIVATE mavlink)