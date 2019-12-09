#include <string>
#include <sstream>
#include <iomanip>

#include "json.hpp"
#include "helper.h"
#include "controller.h"
#include "sensors.h"


//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------
static void
writeWpResult(WaypointReachedData* wpdata, std::vector<SensorValue> sensors, std::vector<std::string> pictures) {

    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);
    std::stringstream dstream;
    // 2009-06-15T13:45:30 
    dstream << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' <<  now->tm_mday 
        << 'T' << now->tm_hour << ':' << now->tm_min << ':' << now->tm_sec;

    nlohmann::json j;
    nlohmann::json sensor_array = nlohmann::json::array();

    // Sensor values: value is always a string
    for (size_t i = 0; i < sensors.size(); i++) {
        std::string strIndex = std::to_string(i);
        sensor_array.push_back({
            {"id", sensors[i].id},
            {"value", sensors[i].value},
            {"mime", "text/plain" },
        });
    }

    // picture id
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
    ret = MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS);
    if (ret != EResult::MSP_SUCCESS){
        context->setState(&context->stateIdle);
        return ret;
    }

    // Start mission
    ret = MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_START);
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
        ret = MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_PAUSE);
    }
    else {
        ret = MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_RESUME);
    }    

    return ret;
}

EResult 
MspController::Mission::missionStop() {
    spdlog::info("MspController::Mission::missionStop");
    EResult ret = EResult::MSP_FAILED;
    ret = MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_STOP);

    return ret;
}

void 
MspController::Mission::vehicleNotification(EVehicleNotification notification, VehicleData data) {

    spdlog::debug("MspController::Mission::vehicleNotification(" + std::to_string(notification) + ")");

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
    EResult ret = EResult::MSP_FAILED;

    // Handle wpreached data
    WaypointReachedData* wpdata = static_cast<WaypointReachedData*>(data);  
    if (wpdata) {
        spdlog::debug("MspController::Mission::handleWpReached(" + std::to_string(wpdata->index) + ")");
        
        // get current BehaviorItem and execute assigned waypoint actions
        mavlink_mission_item_t* item = MspController::getInstance()->getMissionBehaviorItem(wpdata->index);
        if (item) {

            // check for autocontinue 
            // false: pause mission -> resume mission after actions completed
            // true: execute waypoint actions while mission continues
            if(!item->autocontinue){
                // Pause mission
                spdlog::debug("MspController::Mission::handleWpReached, mission pause");
                MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_PAUSE);
            }

            // Send mission item reached mavlink messsage
            sendMissionItemReached(wpdata->index);

            // Execute waypoint action and stor data to json file
            executeAction(wpdata);

            if (wpdata->index + 1 < MspController::getInstance()->getMissionItemCount()) {
                // resume mission
                ret = MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_RESUME);
                if (ret != EResult::MSP_SUCCESS){
                    spdlog::debug("MspController::Mission::handleWpReached, mission resume");
                    context->setState(&context->stateIdle);
                    return;
                }
            }
            else {
                // goto origin if the max number of waypoints is reached
                MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN);
                context->setState(&context->stateIdle);
            }
        }
        else {
            // goto origin if no mission item for the current waypoint is found
            MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN);
            context->setState(&context->stateIdle);
        }
    }
}

void 
MspController::Mission::executeAction(WaypointReachedData* wpdata) {
    
    std::vector<mavlink_mission_item_t>* items = MspController::getInstance()->getMissionItem(wpdata->index);
    if (items) {
        std::vector<SensorValue> sensors;
        std::vector<std::string> pictures;

        // execute all waypoint actions
        for (size_t i = 0; i < items->size(); i++) {
            mavlink_mission_item_t item = (*items)[i];
            int id = (int)item.param1;
   
            switch (item.command)
            {
            case MAV_CMD_USER_2:
                // read sensor values 
                sensors.push_back(MspSensors::getInstance()->getSensorValue(id));
                break;
            case MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE:

                // take a picture (SDCard filesystem can not be accessed within the OSDK context, 
                // so the assigement picture-waypoint is only in the MSDK possible (media manager))
                MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_TAKE_PICTURE, &item, 0);
                pictures.push_back(std::to_string(id));
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
    // Currently done by DJI vehicle itselfe!!!
    /*for (int i = 0; i < MspController::getInstance()->getMissionItemCount(); i++)
    {
        mavlink_mission_item_t* item = MspController::getInstance()->getMissionBehaviorItem(i);
    }*/

    /*if (false) {
        context->setState(&context->stateIdle);
    }*/
}
