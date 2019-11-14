
#include <iostream>

#include "controller.h"
#include "sensors.h"


//-------------------------------------------------------------
// Class Init 
//-------------------------------------------------------------
void
MspController::Init::entry() {
    
    // TODO initialize
    int ret = initializeSensors();
    if (ret != 0){
        std::cout << "sensor initialization failed";
    }
    
    context->setState(&context->stateIdle);
}
