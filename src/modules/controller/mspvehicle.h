/**
    MspVehicle: Defines a abstract wrapper structure for a physical or logical vehicle.
    MspMockVehicle: Defines a logical mock vehicle to thest the interfaces and logical steps of MspController. This
    Vehicle creates a new thread for simulating waypoint handling.
    @file mspvehicle.cpp
    @version 1.0
    @author Simon Waelti
    @version 1.0 1.12.2019
*/

#pragma once

#include "controller_def.h"
#include "mav_mavlink.h"

struct MspVehicle {

    public:
        MspVehicle() {}

        virtual ~MspVehicle() {};

        virtual void initialize()=0;
        virtual EResult command(EVehicleCmd cmd, void* data, size_t len)=0;
};

class MspMockVehicle : public MspVehicle {

    public:
        MspMockVehicle() : MspVehicle() {};

        /**
            Intializes the vehicle and notifies the controller about the state.
        */
        void initialize() override;

        /**
            Calls the requestet command function
            @param EVehicleCmd cmd
            @param void* data
            @param size_t len
            @return EResult
        */
        EResult command(EVehicleCmd cmd, void* data, size_t len) override;

    private:
        std::vector<mavlink_mission_item_t*> wp_list;
        volatile bool exit;

        /**
            Runner function for the waypint handling thread. On each waypoint the MspController is notified.
        */
        void missionRun();
        /**
            Simulates the start of a new waypoint mission. This function is synchronized with the waypoint handling thread.
            @return EResult
        */
        EResult runWaypointMission();
        /**
            Simulates the pausing of a running waypoint mission.
            @return EResult
        */
        EResult pauseWaypointMission();
        /**
            Simulates the resuming of a running waypoint mission.
            @return EResult
        */
        EResult resumeWaypointMission();
        /**
            Simulates the stopping of a running waypoint mission.
            @return EResult
        */
        EResult stopWaypointMission();
};
