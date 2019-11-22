set (dji_src 
    dji_linux_environment.cpp
    dji_linux_helpers.cpp
    dji_mavlink.cpp
    dji_mobile_interface.cpp
    dji_mission_interface.cpp
    dji_vehicle.cpp
    )


add_library(dji "${dji_src}")

target_link_libraries(dji PRIVATE spdlog::spdlog)
target_link_libraries(dji PRIVATE mavlink controller)
target_link_libraries(dji PRIVATE mavlink)

target_include_directories (dji PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})