
#include "controller.h"
#include "sensors.h"


//-------------------------------------------------------------
// Class Init 
//-------------------------------------------------------------
void
MspController::Init::entry() {
    
    // TODO initialize
    initializeSensors();

    context->setState(&context->stateIdle);
}
