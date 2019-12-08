#pragma once

#include <array>
#include <dji_vehicle.hpp>

#include "mspvehicle.h"
#include "dji_mavlink.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class MspDjiVehicle : public MspVehicle {

    public:
        MspDjiVehicle(Vehicle* vehicleDJI, Mavlink* mavlinkDJI);
        ~MspDjiVehicle() override;
        
        EResult command(EVehicleCmd cmd, void* data, size_t len) override;
        uint8_t status;
        void initialize() override;

    private:

        Vehicle* vehicle;
        MavlinkDJI* mavlink;
        std::vector<WayPointSettings> wp_list;
        

        bool setUpSubscription(Vehicle* vehicle, int responseTimeout, int pkgIndex, int freq, std::vector<TopicName> topicList);
        bool teardownSubscription(Vehicle* vehicle, const int pkgIndex, int responseTimeout);

        void createWaypoints();
        EResult handleStateRequest();
        EResult uploadWaypoints();
        EResult runWaypointMission(); 
        EResult pauseWaypointMission();
        EResult resumeWaypointMission();
        EResult takePicture(void* data);
};