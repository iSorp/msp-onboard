#pragma once

#include <dji_vehicle.hpp>


void
mobileCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData);
void 
sendDataToMobile(uint8_t* data, size_t len, void* userData);