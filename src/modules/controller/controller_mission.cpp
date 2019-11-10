#include "controller.h"


//-------------------------------------------------------------
// Class Mission 
//-------------------------------------------------------------
void 
MspController::Mission::entry() {
    // TODO some initialization
    
}

EResult 
MspController::Mission::cmdExecute(uint16_t command){
    switch (command)
    {
    default:
        return EResult::INVALID;
        break;
    }
}


EResult 
MspController::Mission::missionStart() {

    void* data;
    size_t len;

    // Upload mission data
    MspController::getInstance()->vehicleCmd(EVehicleCmd::UPLOAD_WAY_POINTS, data, len);

    // Start mission
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MISSION_START, NULL, 0);
}

EResult 
MspController::Mission::missionStop() {
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MISSION_STOP, NULL, 0);
}

void 
MspController::Mission::vehicleNotification(EVehicleNotification notification) {

    if (notification == EVehicleNotification::WAY_POINT_REACHED) {
        sendMissionItemReached(1);

        // read DJI telemetrie data

        // read sensor data

        // Combine and stor data

        // check for next WP

        // resume mission
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MISSION_RESUME, NULL, 0);

        // if option to origin -> execute
        MspController::getInstance()->vehicleCmd(EVehicleCmd::RETURN_TO_ORIGIN, NULL, 0);

        // otherwise hower and set idle state
        context->setState(&context->stateIdle);
    }
}

void 
MspController::Mission::exit() {
    // if all WP are done, end mission
    sendMissionItemReached(-1);
}

void 
MspController::Mission::sendMissionItemReached(int seq) {

    Mavlink* mavlink = MspController::getInstance()->mavlink;

    // Send mission item reached
	mavlink_mission_item_reached_t wp_reached;
	wp_reached.seq = seq;
	mavlink_msg_mission_item_reached_send_struct(mavlink->getChannel(), &wp_reached);

    // Send current mission item
    mavlink_mission_current_t wpc;
    wpc.seq = seq;
    mavlink_msg_mission_current_send_struct(mavlink->getChannel(), &wpc);
}
