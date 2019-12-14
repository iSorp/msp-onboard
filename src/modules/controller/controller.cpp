/**
    Implementation of the MspController class.
    @file controller.cpp
    @version 1.0
    @author Simon Waelti
    @version 1.0 1.12.2019
*/

#include <mutex>
#include "controller.h"

static std::mutex command_mutex;

//-------------------------------------------------------------
// Singleton Class MspController 
//-------------------------------------------------------------
MspController *MspController::instance = 0;

// Return the singleton instance
MspController *
MspController::getInstance() {
    if (!instance)
        instance = new MspController;
    return instance;
}

// Initializes the Controller and sets the state machines init state. 
void
MspController::initialize(Mavlink* _mavlink) {
    this->mavlink    = _mavlink;

    // State machine entrypoint
    setState(&stateInit);
}

// Returns the current vehicle state represented as Mavlink state.
MAV_STATE 
MspController::getMavState() {
    if (state == nullptr) {
        return MAV_STATE::MAV_STATE_UNINIT;
    } else if (typeid(*state) == typeid(MspController::Init)){
        return MAV_STATE::MAV_STATE_BOOT;
    }else{
        return MAV_STATE::MAV_STATE_ACTIVE;
    }
}

// Returns the current vehicle mode represented as Mavlink mode.
uint8_t 
MspController::getMavMode() {
    if (vehicleInfo.state == EVehicleState::MSP_VHC_SIMULATION) {
        vehicleInfo.mode |= MAV_MODE_FLAG_HIL_ENABLED;
    }
    return vehicleInfo.mode;
}

// Sets the current state of the state machine.
void 
MspController::setState(State* _state) {

    spdlog::debug("MSPController::setState(" + std::string(typeid(*_state).name()) + ")");
    _state->exit();
    state = _state; 
    state->entry();
};

// Sets the vehicle command callback function.
void 
MspController::setVehicleCommandCallback(VehicleCommandCallback callback, VehicleData vehicleData) {
    vCmdCbHandler.callback = callback;
    vCmdCbHandler.vehicleData = vehicleData;
}

//------------------------------------------------------------- 
// Command and notification interface
//-------------------------------------------------------------

// Sets the a vehicle command.
EResult
MspController::setCommand(uint16_t command, mavlink_command_long_t cmd) {
    // make sure only one vehicle notification call is handled
    std::lock_guard<std::mutex> guard(command_mutex);
    return state->setCommand(command, cmd);
}

// Notification from vehicle is called by callback threads
void
MspController::vehicleNotification(EVehicleNotification notification, VehicleData data) {

    // make sure only one vehicle notification call is handled
    std::lock_guard<std::mutex> guard(command_mutex);

    if (notification == EVehicleNotification::MSP_VHC_STATE){
        VehicleInfoData* vehicleInfo = (VehicleInfoData*)data;
        this->vehicleInfo = *vehicleInfo;
    }
    state->vehicleNotification(notification, data);
}

// Sets the a vehicle command.
EResult 
MspController::setVehicleCommand(EVehicleCmd command, void* data, size_t len) {

    EResult res = vCmdCbHandler.callback(command, vCmdCbHandler.vehicleData, data, len);
    return res;
}

// Sets the a vehicle command.
EResult 
MspController::setVehicleCommand(EVehicleCmd command) {
    return setVehicleCommand(command, NULL, 0);
}


//------------------------------------------------------------- 
// Mission flight control
//-------------------------------------------------------------
int16_t 
MspController::getCurrentWp() {
    if (missionIsActive()) {
        return stateMission.getCurrent();
    }
    else {
        return -1;
    }
}

//------------------------------------------------------------- 
// Mission repository
//-------------------------------------------------------------
/*
* Gets the behavior mission item for a certain key. This item contains the behavior for the flight,
* like speed, delay and position.
*/
mavlink_mission_item_t* 
MspController::getMissionBehaviorItem(int key) {
    if (missionItemMap.count(key) > 0) {
        std::vector<mavlink_mission_item_t>* items = &missionItemMap[key];
        for (uint8_t i = 0; i < items->size(); i++) {
            if (items->at(i).command == MAV_CMD_USER_1) {
                return &items->at(i);
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
size_t 
MspController::getMissionItemCount(){
    return missionItemMap.size();
}

bool
MspController::missionIsActive() {
    bool b = typeid(*state) == typeid(MspController::Mission);
    return b;
}

EResult 
MspController::missionDelete() {
  if (!missionIsActive())
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
    if (!missionIsActive())
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