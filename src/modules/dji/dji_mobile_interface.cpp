#include <algorithm>
#include "dji_mspvehicle.h"
#include "dji_mobile_interface.h"


#define OSDK_DATA_MAX_SIZE 100


//-------------------------------------------------------------
// Callbacks
//-------------------------------------------------------------
void
mobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {

    MavlinkDJI* mavlink = static_cast<MavlinkDJI*>(userData);

    // send received data to dji mavlink receiver
    if (mavlink) {
        mavlink->setBuffer(recvFrame.recvData.raw_ack_array, MAX_INCOMING_DATA_SIZE);
    }
}

//-------------------------------------------------------------
// MobileSDK communication
//-------------------------------------------------------------
void
sendDataToMobile(uint8_t* data, uint8_t len, void* userData)
{
    Vehicle* vehicle = static_cast<Vehicle*>(userData);
    if (vehicle) {
        uint8_t writePos = 0;
        if (len > OSDK_DATA_MAX_SIZE) {
            while (writePos < len) {
                size_t length = std::min(len - writePos, OSDK_DATA_MAX_SIZE);
                uint8_t buf[length] = {};
                memcpy (&buf, &data[writePos], length);
                writePos += length;
                vehicle->mobileDevice->sendDataToMSDK(buf, length);
            }
        } else {
            vehicle->mobileDevice->sendDataToMSDK(data, len);
        }
    }
}