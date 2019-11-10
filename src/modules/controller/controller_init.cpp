
#include "controller.h"

//-------------------------------------------------------------
// Class Init 
//-------------------------------------------------------------
void
MspController::Init::entry() {
    
    // TODO initialize

    context->setState(&context->stateIdle);
}
