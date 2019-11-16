
#include <iostream>

#include "controller.h"
#include "sensors.h"


//-------------------------------------------------------------
// Class Init 
//-------------------------------------------------------------
void
MspController::Init::entry() {
    
    // TODO initialize
    MspSensors::getInstance()->initialize();
    
    context->setState(&context->stateIdle);
}
