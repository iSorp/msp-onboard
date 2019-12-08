#pragma once

#include <dji_vehicle.hpp>


void
mobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
void 
sendDataToMobile(uint8_t* data, uint8_t len, void* userData);