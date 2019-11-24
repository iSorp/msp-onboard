#pragma once


#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

#include "dji_mobile_interface.h"
#include "dji_mavlink.h"

void mobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
void sendDataToMSDK(uint8_t* data, uint8_t len);

extern Vehicle* vehicle;
extern MavlinkDJI* mavlink;
