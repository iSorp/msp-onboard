#pragma once

#include "controller.h"

using MspVehicle = struct {};

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
