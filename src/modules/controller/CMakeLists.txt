# Copyright (c) 2019 Samuel Ackermann, Simon Wälti

set (controller_src 
    controller.cpp
    controller_init.cpp
    controller_idle.cpp
    controller_command.cpp
    controller_mission.cpp
    mspvehicle.cpp
    )


add_library(controller "${controller_src}")

target_link_libraries(controller PRIVATE spdlog::spdlog)
target_link_libraries(controller PRIVATE mavlink)
target_link_libraries(controller PRIVATE sensors)

target_include_directories (controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})