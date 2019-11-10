set (mavlink_src 
    mav_mavlink.cpp
    mav_mavlink_udp.cpp
    mav_command.cpp
    mav_mission.cpp
    mav_ftp.cpp
    )


add_library(mavlink "${mavlink_src}")

target_include_directories (mavlink PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(mavlink PRIVATE controller)