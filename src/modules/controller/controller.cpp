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

    // State machine entrypoint
    setState(&stateInit);
}

/*
* Execute state command
*/
EResult
MspController::cmdExecute(uint16_t command) {
    return state->cmdExecute(command);
}

/*
* handle message
*/
EResult
MspController::handleMessage(uint16_t message) {
    EResult res = EResult::FAILED;

    switch (message)
    {
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        return missionDelete();
        break;
    default:
        break;
    }

    return res;
}

       
// Mission functionalities
//-------------------------------------------------------------
EResult 
MspController::missionDelete() {
    if (typeid(*state) == typeid(MspController::Idle))
    {
        missionItems.clear();
        std::cout << "mission deleted";
        return EResult::SUCCESS;
    }
    else {
        std::cout << "controller in wrong  state";
        return EResult::FAILED;
    }
}

void
MspController::missionAddItem(mavlink_mission_item_t wp){
    if (typeid(*state) == typeid(MspController::Idle))
    {
        missionItems.push_back(wp);
    }
}