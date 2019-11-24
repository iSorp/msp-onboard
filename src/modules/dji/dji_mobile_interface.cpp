#include "dji_mobile_interface.h"

void
mobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {

    // route received data to dji mavlink receiver
    if (mavlink) {
        mavlink->setBuffer(recvFrame.recvData.raw_ack_array, MAX_INCOMING_DATA_SIZE);
    }
}

void
sendDataToMSDK(uint8_t* data, uint8_t len)
{
    //vehicle->moc->sendDataToMSDK(data, len);
    if (vehicle) {
        vehicle->mobileDevice->sendDataToMSDK(data, len);
    }
}