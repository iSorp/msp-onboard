#pragma once


#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

#include "dji_mobile_interface.h"
#include "dji_mavlink.h"

bool
setupMSDKComm(Vehicle* vehicle, LinuxSetup* linuxEnvironment, MavlinkDJI* mavlinkDJI);