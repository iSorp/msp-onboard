set (mavlink_src 
    mav_mavlink.cpp 
    mav_mission.cpp
    )

add_library(mavlink "${mavlink_src}")

target_include_directories (mavlink PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
