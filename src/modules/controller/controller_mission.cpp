/**
    Implementation of the Mission state class.
    @file controller_mission.cpp
    @version 1.0
    @author Simon Waelti
    @version 1.0 1.12.2019
*/

#include <string>
#include <sstream>
#include <iomanip>

#include "json.hpp"
#include "helper.h"
#include "controller.h"
#include "sensors.h"
#include "sensor_def.h"

//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------

/**
    Assignes the sensor data with position and time, the result is store in a json file
    (wp0.json, wp1.json, ...)
    @param wpdata information generated by the vehicle
    @param sensors set of sensors (values and ids)
    @param pictures set of ids, pictures musst be assigned to the way point results in the mobile app, OSDK does not allow access to the filesystem.
*/
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
            {"command_id", sensors[i].command }, // TODO id
        });
    }

    // picture id
    for (size_t i = 0; i < pictures.size(); i++) {
        std::string strIndex = std::to_string(i);
        sensor_array.push_back({
            {"id", 0 },
            {"value", pictures[i] },
            {"command_id", sensors[i].command  },
        });
    }

    j["sensors"] = sensor_array;
    
    j["x"] = wpdata->longitude;
    j["y"] = wpdata->latitude;
    j["z"] = wpdata->altitude;
    j["date"] = dstream.str();
    j["seq"] = std::to_string(wpdata->index);
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

    // Check if mission items are valid (coordinates, actions)
    validateMissionItems(); 
    missionActive       = false;
    userCommandPaused   = false;
}

void 
MspController::Mission::exit() {
    missionActive       = false;
    userCommandPaused   = false;
}

// Starts a way point mission, state transfer to idle on error
EResult 
MspController::Mission::missionStart() {

    spdlog::info("MspController::Mission::missionStart");
    EResult ret = EResult::MSP_FAILED;
    currenindex = 0;
    
    // Upload mission data
    ret = context->setVehicleCommand(EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS);
    if (ret != EResult::MSP_SUCCESS){
        context->setState(&context->stateIdle);
        return ret;
    }

    // Start mission
    ret = context->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_START);
    if (ret != EResult::MSP_SUCCESS){
        context->setState(&context->stateIdle);
        return ret;
    }

    missionActive = true;
    return ret;
}

// Pauses/resumes a way point mission, state transfer to idle on error
EResult 
MspController::Mission::missionPauseContinue() {
    spdlog::info("MspController::Mission::missionPauseContinue");
    EResult ret = EResult::MSP_FAILED;

    if (missionActive) {
        userCommandPaused = true; // exit mission state if vehicle notifies paused
        ret = context->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_PAUSE);
    }
    else {
        missionActive = true;
        ret = context->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_RESUME);
    }    

    return ret;
}

//------------------------------------------------------------- 
// Command and notification interface
//-------------------------------------------------------------
EResult 
MspController::Mission::setCommand(uint16_t command, mavlink_command_long_t cmd){
    EResult res = EResult::MSP_FAILED;
    switch (command)
    {
    case MAV_CMD_MISSION_START:
        res = missionStart();
        break;
    case MAV_CMD_DO_PAUSE_CONTINUE:
        res = missionPauseContinue();
        break;
    default:
        spdlog::warn("MspController::Mission::setCommand, command not available");
        res =  EResult::MSP_INVALID;
        break;
    }
    return res;
}

/**
    Is called when a way point is reached.
    This function manages the logic for executing actions and next waypoints, 
    if the number of way point is reached, a state transfer to idle is performed.
*/
void 
MspController::Mission::vehicleNotification(EVehicleNotification notification, VehicleData data) {

    switch (notification)
    {
    case EVehicleNotification::MSP_VHC_WAY_POINT_REACHED:
        handleWpReached(data);
        break;  
    case EVehicleNotification::MSP_VHC_MISSION_STOPPED:
        context->mavlink->setCmdResult(EResult::MSP_SUCCESS);
        context->setState(&context->stateIdle);
        break;
    case EVehicleNotification::MSP_VHC_MISSION_PAUSED:
        context->mavlink->setCmdResult(EResult::MSP_SUCCESS);
        if (userCommandPaused) {
            context->setState(&context->stateIdle);
        }
        break;
    case EVehicleNotification::MSP_VHC_MISSION_RESUMED:
        context->mavlink->setCmdResult(EResult::MSP_SUCCESS);
        break;
    default:
        break;
    }
}

//-------------------------------------------------------------
// Mission helpers
//-------------------------------------------------------------

/**
    Performs all action of the current way point. 
*/
void 
MspController::Mission::handleWpReached(VehicleData data) {
    EResult ret = EResult::MSP_FAILED;

    // Handle wpreached data
    WaypointReachedData* wpdata = static_cast<WaypointReachedData*>(data);  
    if (wpdata) {
        spdlog::debug("MspController::Mission::handleWpReached(" + std::to_string(wpdata->index) + ")");
        
        // get current BehaviorItem and execute assigned waypoint actions
        currenindex = wpdata->index;
        mavlink_mission_item_t* item = context->getMissionBehaviorItem(wpdata->index);
        if (item) {

            // check for autocontinue 
            // false: pause mission -> resume mission after actions completed
            // true: execute waypoint actions while mission continues
            if(!item->autocontinue){
                // Pause mission
                spdlog::debug("MspController::Mission::handleWpReached, mission pause");
                context->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_PAUSE);
            }

            // Send mission item reached mavlink messsage
            sendMissionItemReached(wpdata->index);

            // Execute waypoint action and stor data to json file
            executeAction(wpdata);

            if (wpdata->index + 1 < context->getMissionItemCount()) {
                // resume mission
                ret = context->setVehicleCommand(EVehicleCmd::MSP_CMD_MISSION_RESUME);
                if (ret != EResult::MSP_SUCCESS && ret != EResult::MSP_PROGRESS){
                    spdlog::debug("MspController::Mission::handleWpReached, mission resume");
                    context->setState(&context->stateIdle);
                    return;
                }
            }
            else {
                // goto origin if the max number of waypoints is reached
                currenindex -1;
                sendMissionItemReached(-1);
                context->setVehicleCommand(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN);
                context->setState(&context->stateIdle);
            }
        }
        else {
            // goto origin if no mission item for the current waypoint is found
            currenindex -1;
            sendMissionItemReached(-1);
            context->setVehicleCommand(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN);
            context->setState(&context->stateIdle);
        }
    }
}

// Performs all action of the current way point.
void 
MspController::Mission::executeAction(WaypointReachedData* wpdata) {
    
    std::vector<mavlink_mission_item_t>* items = context->getMissionItem(wpdata->index);
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
                context->setVehicleCommand(EVehicleCmd::MSP_CMD_TAKE_PICTURE, &item, 0);
                pictures.push_back(std::to_string(item.command));
                break;
            default:
                break;
            }
        }             
        writeWpResult(wpdata, sensors, pictures);            
    } 
}

// Sends a mission item reached mavlink message
void 
MspController::Mission::sendMissionItemReached(int seq) {
    
    Mavlink* mavlink = context->mavlink;

    mavlink_mission_item_reached_t item;
    item.seq = seq;
    mavlink_msg_mission_item_reached_send_struct(mavlink->getChannel(), &item);

    // Send current mission item
    mavlink_mission_current_t wpc;
    wpc.seq = seq;
    mavlink_msg_mission_current_send_struct(mavlink->getChannel(), &wpc);
}

/**
    Validates all mission items.
    (this function is not used due the fact, dji performs already a validation itself)
*/
void 
MspController::Mission::validateMissionItems() {
    
    // TODO validate coordinates (GEO fence) 
    // Currently done by DJI vehicle itselfe!!!
    /*for (int i = 0; i < context->getMissionItemCount(); i++)
    {
        mavlink_mission_item_t* item = context->getMissionBehaviorItem(i);
    }*/

    /*if (false) {
        context->setState(&context->stateIdle);
    }*/
}
