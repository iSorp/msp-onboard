set (mavlink_src 
    mav_mavlink.cpp
    mav_receiver.cpp
    mav_mission.cpp
    mav_ftp.cpp
    )


add_library(mavlink "${mavlink_src}")

target_include_directories (mavlink PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
