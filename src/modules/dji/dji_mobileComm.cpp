#include "dji_mobileComm.h"


static MavlinkDJI* mavlink;
static Vehicle* vehicle;

bool
setupMSDKComm(Vehicle* vehicleDJI, LinuxSetup* linuxEnvironment, MavlinkDJI* mavlinkDJI)
{
    // registered dji vehicle
    vehicle = vehicleDJI;

    // mavlink handler
    mavlink = mavlinkDJI;

    // set callback for mavlink send data
    mavlink->sendDataCallback = &sendDataToMSDK;

    // register callback for mobile data
    vehicle->mobileDevice->setFromMSDKCallback(mobileCallback, linuxEnvironment);
}

void
mobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
    // LinuxSetup* linuxEnvironment = (LinuxSetup*)userData;
    // mobile_data_id = *(reinterpret_cast<uint16_t*>(&recvFrame.recvData.raw_ack_array));

    // route received data to dji mavlink receiver
    mavlink->setBuffer(&recvFrame.recvData.raw_ack_array, MAX_INCOMING_DATA_SIZE);
}

void
sendDataToMSDK(const uint8_t* data, uint8_t len)
{
    //vehicle->moc->sendDataToMSDK(data, len);
    vehicle->mobileDevice->sendDataToMSDK(data, len);
}