
#include "controller.h"

//-------------------------------------------------------------
// Singleton Class FlightController 
//-------------------------------------------------------------
FlightController *FlightController::instance = 0;

FlightController *
FlightController::getInstance() {
    if (!instance)
        instance = new FlightController;
    return instance;
}

void
FlightController::initialize() {

    // State machine entrypoint
    setState(&stateInit);
}

//-------------------------------------------------------------
// Class Init 
//-------------------------------------------------------------
void
FlightController::Init::entry() {
    
    // TODO initialize

    context->setState(&context->stateIdle);
}
