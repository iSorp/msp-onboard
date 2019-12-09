#pragma once

#include "controller_def.h"
#include "mav_mavlink.h"

struct MspVehicle {

    public:
        MspVehicle() {}

        virtual ~MspVehicle() {};

        virtual EResult command(EVehicleCmd cmd, void* data, size_t len)=0;
        virtual void initialize()=0;
};

class MspMockVehicle : public MspVehicle {

    public:
        MspMockVehicle() : MspVehicle() {};

        void initialize() override;

        EResult command(EVehicleCmd cmd, void* data, size_t len) override;

    private:
        std::vector<mavlink_mission_item_t*> wp_list;
        volatile bool exit;

        void missionRun();

        EResult runWaypointMission();
        EResult pauseWaypointMission();
        EResult resumeWaypointMission();
        EResult stopWaypointMission();
};
