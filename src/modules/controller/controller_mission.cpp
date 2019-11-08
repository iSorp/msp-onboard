#include "controller.h"


//-------------------------------------------------------------
// Class StateMission 
//-------------------------------------------------------------
void 
FlightController::StateMission::entry() {
    // dji request for upload mission item and start mission
    
}


void 
FlightController::StateMission::djiCallbackWpReached() {
    // read DJI telemetrie data

    // read sensor data

    // Combine and stor data

    // check for next WP

    // if all WP are done, end mission

    // Mavlink request for message Mission done

    // if option to origin -> execute

    // otherwise hower and set idle state
    context->setState(&context->stateIdle);
}

void 
FlightController::StateMission::exit() {
    
}



//-------------------------------------------------------------
// Class WPPending 
//-------------------------------------------------------------
void 
FlightController::WPPending::entry() {
    // transmit next WP to OSDK
    
}

void 
FlightController::WPPending::run() {
    // wait for messages

    // if WP reached change state
    context->setState(&context->stateWpReached)
}

//-------------------------------------------------------------
// Class WPReached 
//-------------------------------------------------------------
void 
FlightController::WPReached::entry() {
    // read DJI telemetrie data

    // read sensor data

    // Combine and stor data

    // check for next WP
    context->setState(&context->stateWpPending);


    // if all WP are done, end mission
    context->setState(&context->stateMisionDone);
}


//-------------------------------------------------------------
// Class MissionDone 
//-------------------------------------------------------------
void 
FlightController::MissionDone::entry() {
    // Mavlink request for message Mission done

    // if option to origin -> execute

    // otherwise hower and set idle state
    context->setState(&context->stateIdle);
}

