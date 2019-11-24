        
#include "dji_mspvehicle.h"
#include "dji_mission_interface.h"
#include "dji_mobile_interface.h"

Vehicle* vehicle;
MavlinkDJI* mavlink;

void
MspDjiVehicle::initialize(Vehicle* vehicleDJI, LinuxSetup* linuxEnvironment, Mavlink* mavlinkDJI) {
    
    mavlink = static_cast<MavlinkDJI*>(mavlinkDJI);
    vehicle = vehicleDJI;

    // set callback for controller command data
    MspController::getInstance()->vehicleCmd = &cmdMissionCallback;

    // set callback for mavlink send data
    if (mavlink){
        mavlink->sendDataCallback = &sendDataToMSDK;
    }
    
    // register callback for mobile data
    if (vehicle){
        vehicle->mobileDevice->setFromMSDKCallback(mobileCallback, linuxEnvironment);
    } 
}