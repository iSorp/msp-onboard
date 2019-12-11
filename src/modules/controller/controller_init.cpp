/**
    Implementation of the Init state class.
    @file controller_init.cpp
    @version 1.0
    @author Simon Waelti
    @version 1.0 1.12.2019
*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "controller.h"
#include "sensors.h"


//-------------------------------------------------------------
// Class Init 
//-------------------------------------------------------------
void
MspController::Init::entry() {

    // Initialize oboard-controller Sensors
    MspSensors::getInstance()->initialize();
}

void 
MspController::Init::vehicleNotification(EVehicleNotification notification, VehicleData data) {
    if (notification == EVehicleNotification::MSP_VHC_STATE) {
        VehicleInfoData* vehicleInfo = (VehicleInfoData*)data;

        bool available = vehicleInfo->state >= EVehicleState::MSP_VHC_AVAILABLE;
        if (available) {
            context->setState(&context->stateIdle);
        }
        else {
            spdlog::critical("vehicle not in proper state");
            std::exit(1);
        } 
    }
}