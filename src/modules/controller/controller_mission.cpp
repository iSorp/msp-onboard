#include <iomanip>

// https://github.com/nlohmann/json
#include "json.hpp"

#include "helper.h"
#include "controller.h"
#include "sensors.h"

/*
static void 
writeWpResult(waypointReachedData_t* wpdata, int num, sensor_t* sensor...) {
    nlohmann::json j;
    va_list valist;
    sensor_t* s;
    va_start(valist, num);

    nlohmann::json sensor_array = nlohmann::json::array();

    for (int i = 0; i < num; i++) {
        s =  va_arg(valist, sensor_t*);
        std::string strIndex = std::to_string(i);
        sensor_array.push_back({
            {"id", s->id},
            {"value", s->value},
        });
    }
    j["sensors"] = sensor_array;
    
    j["x"] = wpdata->x;
    j["y"] = wpdata->y;
    j["z"] = wpdata->z;

    std::ofstream ofs("wp" + std::to_string(wpdata->index) + "data.json");
    ofs << std::setw(4) << j << std::endl;
}
*/

static void 
writeWpResult(waypointReachedData_t* wpdata, std::vector<SensorValue> sensors) {
    nlohmann::json j;
    nlohmann::json sensor_array = nlohmann::json::array();

    for (int i = 0; i < sensors.size(); i++) {
        std::string strIndex = std::to_string(i);
        sensor_array.push_back({
            {"id", sensors[i].id},
            {"value", sensors[i].value},
        });
    }
    j["sensors"] = sensor_array;
    
    j["x"] = wpdata->longitude;
    j["y"] = wpdata->latitude;
    j["z"] = wpdata->altitude;

    std::ofstream ofs("wp" + std::to_string(wpdata->index) + "data.json");
    ofs << std::setw(4) << j << std::endl;
}


//-------------------------------------------------------------
// Class Mission 
//-------------------------------------------------------------
void 
MspController::Mission::entry() {
    // TODO some initialization
    
}
void 
MspController::Mission::exit() {
    // if all WP are done, end mission
    sendMissionItemReached(-1);
}

EResult 
MspController::Mission::cmdExecute(uint16_t command, mavlink_command_long_t cmd){
    EResult res = EResult::MSP_FAILED;
    switch (command)
    {
    case MAV_CMD_MISSION_START:
        res = missionStart();
        break;
    case MAV_CMD_DO_PAUSE_CONTINUE:
        res = missionPauseContinue(cmd.param1 == 0);
        break;
    default:
        spdlog::warn("MspController::Mission::cmdExecute, command not available");
        res =  EResult::MSP_INVALID;
        break;
    }
    return res;
}

EResult 
MspController::Mission::missionStart() {
    spdlog::info("MspController::Mission::missionStart");

    // Upload mission data
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS, NULL, 0);

    // Start mission
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_START, NULL, 0);
    return EResult::MSP_SUCCESS;
}

EResult 
MspController::Mission::missionPauseContinue(bool pause) {
    spdlog::info("MspController::Mission::missionPauseContinue(" + std::to_string(pause) + ")");

    if (pause) {
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_PAUSE, NULL, 0);
    }
    else {
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_RESUME, NULL, 0);
    }    

    return EResult::MSP_SUCCESS;
}

EResult 
MspController::Mission::missionStop() {
    spdlog::info("MspController::Mission::missionStop");
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_STOP, NULL, 0);

    return EResult::MSP_SUCCESS;
}

void 
MspController::Mission::vehicleNotification(EVehicleNotification notification, VehicleData data) {
    spdlog::info("MspController::Mission::vehicleNotification(" + std::to_string(notification) + ")");
    if (notification == EVehicleNotification::MSP_VHC_WAY_POINT_REACHED) {
        // Pause mission
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_PAUSE, NULL, 0);

        // Handle wpreached data
        waypointReachedData_t* wpdata = static_cast<waypointReachedData_t*>(data);     
        if (wpdata) {

            // Send mission item reached mavlink messsage
            sendMissionItemReached(wpdata->index);

            // Export position and sensor data
            std::vector<mavlink_mission_item_t>* items = MspController::getInstance()->getMissionItem(wpdata->index);
            if (items) {
                std::vector<SensorValue> sensors;
                for (int i = 0; i < items->size(); i++){
                    mavlink_mission_item_t item = (*items)[i];

                    // TODO handle Camera commands
                    if (item.command == MAV_CMD_USER_1) continue;
                    if (item.command == MAV_CMD_USER_2) {
                        int id = (int)item.param1;
                        sensors.push_back(MspSensors::getInstance()->getSensorValue(id));
                    }
                }
                if (sensors.size() > 0) {
                    writeWpResult(wpdata, sensors);
                }               
            }
        }

        // check for next WP


        // resume mission
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_RESUME, NULL, 0);

        // if option to origin -> execute
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN, NULL, 0);

        // otherwise hower and set idle state
        context->setState(&context->stateIdle);
    }
}

void 
MspController::Mission::sendMissionItemReached(int seq) {
    
    Mavlink* mavlink = MspController::getInstance()->mavlink;

    // Send current mission item
    mavlink_mission_current_t wpc;
    wpc.seq = seq;
    mavlink_msg_mission_current_send_struct(mavlink->getChannel(), &wpc);
}