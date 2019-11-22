        
#include "dji_vehicle.h"

static MspDjiVehicle* mspVehicle;

static EResult cmdCallback(EVehicleCmd cmd, void* data, size_t len) {
    
}

void
MspDjiVehicle::initialize(Vehicle* vehicleDJI, LinuxSetup* linuxEnvironment, Mavlink* mavlinkDJI) {
    MavlinkDJI* mavlink = static_cast<MavlinkDJI*>(mavlinkDJI);
    mspVehicle = this;
}

