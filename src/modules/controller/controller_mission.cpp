#include <string>
#include <sstream>
#include <iomanip>

#include "json.hpp"
#include "helper.h"
#include "controller.h"
#include "sensors.h"

static void
writeWpResult(waypointReachedData_t* wpdata, std::vector<SensorValue> sensors, std::vector<std::string> pictures);


//-------------------------------------------------------------
// Class Mission 
//-------------------------------------------------------------
void 
MspController::Mission::entry() {

    // Ckeck if mission items are valid (coordinates, actions)
    validateMissionItems(); 
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
    EResult ret = EResult::MSP_FAILED;
    // Upload mission data
    ret = MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS, NULL, 0);
    if (ret != EResult::MSP_SUCCESS){
        context->setState(&context->stateIdle);
        return ret;
    }

    // Start mission
    ret = MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_START, NULL, 0);
    if (ret != EResult::MSP_SUCCESS){
        context->setState(&context->stateIdle);
        return ret;
    }

    return ret;
}

EResult 
MspController::Mission::missionPauseContinue(bool pause) {
    spdlog::info("MspController::Mission::missionPauseContinue(" + std::to_string(pause) + ")");
    EResult ret = EResult::MSP_FAILED;

    if (pause) {
        ret = MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_PAUSE, NULL, 0);
    }
    else {
        ret = MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_RESUME, NULL, 0);
    }    

    return ret;
}

EResult 
MspController::Mission::missionStop() {
    spdlog::info("MspController::Mission::missionStop");
    EResult ret = EResult::MSP_FAILED;
    ret = MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_STOP, NULL, 0);

    return ret;
}

void 
MspController::Mission::vehicleNotification(EVehicleNotification notification, VehicleData data) {
    spdlog::info("MspController::Mission::vehicleNotification(" + std::to_string(notification) + ")");

    switch (notification)
    {
    case EVehicleNotification::MSP_VHC_WAY_POINT_REACHED:
        handleWpReached(data);
        break;
    
    default:
        break;
    }
}

//-------------------------------------------------------------
// Mission helpers
//-------------------------------------------------------------
void 
MspController::Mission::handleWpReached(VehicleData data) {
    // Pause mission
    EResult ret = EResult::MSP_FAILED;
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_PAUSE, NULL, 0);

    // Handle wpreached data
    waypointReachedData_t* wpdata = static_cast<waypointReachedData_t*>(data);     
    if (wpdata) {

        // Send mission item reached mavlink messsage
        sendMissionItemReached(wpdata->index);
        executeAction(wpdata);
    }

    // TODO
    bool nextWPavailable = true;
    bool optionOrigin = false;

    // check for next WP
    if (nextWPavailable) {
        // resume mission
        ret = MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_RESUME, NULL, 0);
        if (ret != EResult::MSP_SUCCESS){
            context->setState(&context->stateIdle);
            return;
        }
    }
    else {
        // if option to origin -> execute
        if (optionOrigin){
            ret = MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN, NULL, 0);
            if (ret != EResult::MSP_SUCCESS){
                context->setState(&context->stateIdle);
                return;
            }
        } else{
            // otherwise hower and set idle state
            context->setState(&context->stateIdle);
        }
    }
}

void 
MspController::Mission::executeAction(waypointReachedData_t* wpdata) {
    
    // Export position and sensor/picture data
    std::vector<mavlink_mission_item_t>* items = MspController::getInstance()->getMissionItem(wpdata->index);
    if (items) {
        std::vector<SensorValue> sensors;
        std::vector<std::string> pictures;

        for (size_t i = 0; i < items->size(); i++) {
            mavlink_mission_item_t item = (*items)[i];
            int id = (int)item.param1;
    sensors.push_back(MspSensors::getInstance()->getSensorValue(id));
            switch (item.command)
            {
            case MAV_CMD_USER_2:
                sensors.push_back(MspSensors::getInstance()->getSensorValue(id));
                break;
            case MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:
                MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_TAKE_PICTURE, &item, 0);
                break;
            
            default:
                break;
            }
        }             
        writeWpResult(wpdata, sensors, pictures);            
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

void 
MspController::Mission::validateMissionItems() {
    
    // TODO validate coordinates (GEO fence) 
    for (int i = 0; i < MspController::getInstance()->getMissionItemCount(); i++)
    {
        mavlink_mission_item_t* item = MspController::getInstance()->getMissionBehaviorItem(i);
    }

    if (false) {
        context->setState(&context->stateIdle);
    }
}


static void
writeWpResult(waypointReachedData_t* wpdata, std::vector<SensorValue> sensors, std::vector<std::string> pictures) {

    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);
    std::stringstream dstream;
    // 2009-06-15T13:45:30 
    dstream << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' <<  now->tm_mday 
        << 'T' << now->tm_hour << ':' << now->tm_min << ':' << now->tm_sec;

    nlohmann::json j;
    nlohmann::json sensor_array = nlohmann::json::array();

    // Sensor values: e.g i2c sensors
    for (size_t i = 0; i < sensors.size(); i++) {
        std::string strIndex = std::to_string(i);
        sensor_array.push_back({
            {"id", sensors[i].id},
            {"value", sensors[i].value},
            {"mime", "text/plain" },
        });
    }

    // pictures paths
    for (size_t i = 0; i < pictures.size(); i++) {
        std::string strIndex = std::to_string(i);
        sensor_array.push_back({
            {"value", pictures[i] },
            {"mime", "image/png" },
        });
    }

    j["sensors"] = sensor_array;
    
    j["x"] = wpdata->longitude;
    j["y"] = wpdata->latitude;
    j["z"] = wpdata->altitude;
    j["date"] = dstream.str();
    std::string file_path;
    file_path.append(WP_EXPORT_PATH);
    file_path.append("/wp");
    file_path.append(std::to_string(wpdata->index));
    file_path.append(".json");

    std::ofstream ofs(file_path);
    ofs << std::setw(4) << j << std::endl;
}

