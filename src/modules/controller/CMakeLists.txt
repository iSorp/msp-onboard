set (controller_src 
    controller_init.cpp
    controller_idle.cpp
    controller_mission.cpp
    )


add_library(controller "${controller_src}")

target_include_directories (controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

