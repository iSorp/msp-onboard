#include <iomanip>

// https://github.com/nlohmann/json
#include "json.hpp"

#include "helper.h"
#include "controller.h"
#include "sensors.h"


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
        res =  EResult::MSP_INVALID;
        break;
    }
    return res;
}

EResult 
MspController::Mission::missionStart() {


      // Combine and stor data
        nlohmann::json j = {
            {"pi", 3.141},
            {"happy", true},
            {"name", "Niels"},
            {"nothing", nullptr},
            {"answer", {
                {"everything", 42}
            }},
            {"list", {1, 0, 2}},
            {"object", {
                {"currency", "USD"},
                {"value", 42.99}
            }}
        };

        std::ofstream o("pretty.json");
        o << std::setw(4) << j << std::endl;

    // Upload mission data
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS, NULL, 0);

    // Start mission
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_START, NULL, 0);

    return EResult::MSP_SUCCESS;
}

EResult 
MspController::Mission::missionPauseContinue(bool pause) {

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
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_STOP, NULL, 0);

    return EResult::MSP_SUCCESS;
}

void 
MspController::Mission::vehicleNotification(EVehicleNotification notification) {

    if (notification == EVehicleNotification::MSP_VHC_WAY_POINT_REACHED) {
        // Pause mission
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_MISSION_PAUSE, NULL, 0);

        sendMissionItemReached(1);

        // read DJI telemetrie data

        // read sensor data
        double sensor_value = getSensorValue(0);

        // Combine and stor data
        nlohmann::json j = {
            {"pi", 3.141},
            {"happy", true},
            {"name", "Niels"},
            {"nothing", nullptr},
            {"answer", {
                {"everything", 42}
            }},
            {"list", {1, 0, 2}},
            {"object", {
                {"currency", "USD"},
                {"value", 42.99}
            }}
        };

        std::ofstream o("pretty.json");
        o << std::setw(4) << j << std::endl;


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
