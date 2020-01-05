#include <algorithm>
#include "dji_mspvehicle.h"
#include "dji_mobile_interface.h"


static size_t OSDK_DATA_MAX_SIZE = 100;


//-------------------------------------------------------------
// Callbacks
//-------------------------------------------------------------
void
mobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {

    MavlinkDJI* mavlink = static_cast<MavlinkDJI*>(userData);

    // send received data to dji mavlink receiver
    if (mavlink) {
        int len = recvFrame.recvInfo.len;
        /*for (int i = 0; i < len; i++) {
            printf("%0x", recvFrame.recvData.raw_ack_array[i]);
        }
        printf("\n");*/
        
        size_t size = std::min(len, 100);
        mavlink->setBuffer(recvFrame.recvData.raw_ack_array, size);//MAX_INCOMING_DATA_SIZE);
    }
}

//-------------------------------------------------------------
// MobileSDK communication
//-------------------------------------------------------------
void
sendDataToMobile(uint8_t* data, size_t len, void* userData)
{
    Vehicle* vehicle = static_cast<Vehicle*>(userData);
    if (vehicle) {
        size_t writePos = 0;
        if (len > 100) {
            while (writePos < len) {                
                size_t length = std::min(len - writePos, OSDK_DATA_MAX_SIZE);
                uint8_t buf[length] = {};
                memcpy (&buf, &data[writePos], length);
                writePos += length;
                vehicle->mobileDevice->sendDataToMSDK(buf, length);
            }
        } else {
            uint8_t size = len;
            vehicle->mobileDevice->sendDataToMSDK(data, size);
        }
    }
}