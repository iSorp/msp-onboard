#pragma once

#include <thread>
#include "controller.h"


struct MspVehicle {

    public:
        //virtual EResult cmdCallback(EVehicleCmd cmd, void* data, size_t len) = 0;
        
};


class MspMockVehicle : public MspVehicle {

    public:
        MspMockVehicle() : MspVehicle() {};
        void initialize();

        EResult handleStateRequest();
        EResult runWaypointMission();
        EResult pauseWaypointMission();
        EResult resumeWaypointMission();

    private:
        std::thread runner;

        void missionRun();
};
