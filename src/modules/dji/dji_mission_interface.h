#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>


EResult cmdMissionCallback(EVehicleCmd cmd, void* data, size_t len);

void createWaypoints();
EResult handleStateRequest();
EResult uploadWaypoints();
EResult runWaypointMission(); 
EResult pauseWaypointMission();
EResult resumeWaypointMission();
EResult takePicture(void* data);

extern Vehicle* vehicle;