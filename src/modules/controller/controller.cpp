#include "controller.h"

//-------------------------------------------------------------
// Singleton Class MspController 
//-------------------------------------------------------------
MspController *MspController::instance = 0;

MspController *
MspController::getInstance() {
    if (!instance)
        instance = new MspController;
    return instance;
}

void
MspController::initialize(Mavlink* mavlink) {
    
    this->mavlink = mavlink;

    //missionItems.reserve(MAX_MISSION_ITEM_COUNT);
    //missionItems.resize(MAX_MISSION_ITEM_COUNT);

    // State machine entrypoint
    setState(&stateInit);
}

/*
* Execute state command
*/
EResult
MspController::cmdExecute(uint16_t command, mavlink_command_long_t cmd) {
    return state->cmdExecute(command, cmd);
}

/*
* Notification from vehicle
*/
void
MspController::vehicleNotification(EVehicleNotification notification) {
    state->vehicleNotification(notification);
}
       
// Mission functionalities
//-------------------------------------------------------------

bool
MspController::missionIsActive() {
    return typeid(*state) == typeid(MspController::Mission);
}

EResult 
MspController::missionDelete() {
    if (typeid(*state) == typeid(MspController::Idle))
    {
        missionItems.clear();
        std::cout << "mission deleted";
        return EResult::MSP_SUCCESS;
    }
    else {
        std::cout << "controller in wrong  state";
        return EResult::MSP_FAILED;
    }
}


EResult
MspController::missionAddItem(mavlink_mission_item_t wp){
    EResult res = EResult::MSP_FAILED;
    if (typeid(*state) == typeid(MspController::Idle))
    {
        if (missionItems.size() <= MAX_MISSION_ITEM_COUNT) {
            missionItems.push_back(wp);

            res = EResult::MSP_SUCCESS;
        }
        else {
            res = EResult::MSP_FAILED;
        }   
    }
    else {
        res = EResult::MSP_FAILED;
    }
    return res;
}