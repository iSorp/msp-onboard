// System Includes
#include <stdio.h>

#include "mav_mavlink.h"
#include "mav_mission.h"


/*#include <cmath>
#include <stdio.h>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
*/

int main() {
	printf("%s", "hello world");
    
    Mavlink* mavlink = new Mavlink();
    MavlinkMissionManager* mission = new MavlinkMissionManager(mavlink);
    mavlink -> test(0,0);
    
    return 1;


}




