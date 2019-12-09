#include <mutex>
#include "controller.h"

std::mutex notify_mutex;

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
MspController::initialize(Mavlink* _mavlink) {
    this->mavlink    = _mavlink;

    // State machine entrypoint
    setState(&stateInit);
}

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

uint8_t 
MspController::getMavMode() {

    setVehicleCommand(EVehicleCmd::MSP_CMD_READ_STATE);

    if (vehicleInfo.state == EVehicleState::MSP_VHC_SIMULATION) {
        vehicleInfo.mode |= MAV_MODE_FLAG_HIL_ENABLED;
    }

    return vehicleInfo.mode;
}

void 
MspController::setState(State* _state) {

    spdlog::debug("MSPController::setState(" + std::string(typeid(*_state).name()) + ")");
    _state->exit();
    state = _state; 
    state->entry();
};

//------------------------------------------------------------- 
// Command and notification interface
//-------------------------------------------------------------
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

    // make sure only one vehicle notification call is handled
    std::lock_guard<std::mutex> guard(notify_mutex);

    if (notification == EVehicleNotification::MSP_VHC_STATE){
        VehicleInfoData* vehicleInfo = (VehicleInfoData*)data;
        this->vehicleInfo = *vehicleInfo;
    }
    state->vehicleNotification(notification, data);
}

void 
MspController::setVehicleCommandCallback(VehicleCommandCallback callback, VehicleData vehicleData) {
    vCmdCbHandler.callback = callback;
    vCmdCbHandler.vehicleData = vehicleData;
}

EResult 
MspController::setVehicleCommand(EVehicleCmd command) {
    return setVehicleCommand(command, NULL, 0);
}

EResult 
MspController::setVehicleCommand(EVehicleCmd command, void* data, size_t len) {
    return vCmdCbHandler.callback(command, vCmdCbHandler.vehicleData, data, len);
}

//------------------------------------------------------------- 
// Mission repository
//-------------------------------------------------------------
/*
* Gets the behavior mission item for a certain key. This item contains the behavior for the flight,
* like speed and delay
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