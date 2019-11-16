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

void 
MspController::setState(State *_state) {

    spdlog::debug("MSPController::setState(" + std::string(typeid(*_state).name()) + ")");
    _state->exit();
    state = _state; 
    state->entry();
};

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
MspController::vehicleNotification(EVehicleNotification notification, VehicleData data) {
    state->vehicleNotification(notification, data);
}
       
// Mission functionalities
//-------------------------------------------------------------


/*
* Gets the behavior mission item for a certain key. This item contains the behavior for the flight,
* like speed and delay
*/
mavlink_mission_item_t* 
MspController::getMissionBehaviorItem(int key) {
    mavlink_mission_item_t item;
    if (missionItemMap.count(key) > 0) {
        std::vector<mavlink_mission_item_t> items = missionItemMap[key];
        for (int i = 0; i < items.size(); i++) {
            if (items[i].command == MAV_CMD_USER_1) {
                return &items[i];
            }
        }
    } 
    return NULL;
} 

std::vector<mavlink_mission_item_t>* 
MspController::getMissionItem(int key) {
    if (missionItemMap.count(key) > 0) { 
        return &missionItemMap[key];
    }
    return NULL;
} 
int 
MspController::getMissionItemCount(){
    return missionItemMap.size();
}


bool
MspController::missionIsActive() {
    return typeid(*state) == typeid(MspController::Mission);
}

EResult 
MspController::missionDelete() {
    if (typeid(*state) == typeid(MspController::Idle))
    {
        missionItemMap.clear();
        return EResult::MSP_SUCCESS;
    }
    else {
        spdlog::warn("MSPController::missionDelete, wrong state");
        return EResult::MSP_FAILED;
    }
}

EResult
MspController::missionAddItem(int key, mavlink_mission_item_t wp){
    EResult res = EResult::MSP_FAILED;
    if (typeid(*state) == typeid(MspController::Idle))
    {
        if (missionItemMap.size() <= MAX_MISSION_ITEM_COUNT) {
            missionItemMap[key].push_back(wp);

            res = EResult::MSP_SUCCESS;
        }
        else {
            res = EResult::MSP_FAILED;
            spdlog::error("MSPController::missionAddItem, size > MAX_MISSION_ITEM_COUNT");
        }   
    }
    else {
        spdlog::warn("MSPController::missionAddItem, wrong state");
        res = EResult::MSP_FAILED;
    }
    return res;
}