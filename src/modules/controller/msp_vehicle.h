#pragma once

#include <thread>
#include "controller.h"


struct MspVehicle {

        
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
        
        void missionRun();
};
