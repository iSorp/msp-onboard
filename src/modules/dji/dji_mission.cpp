#include <cmath>
#include <vector>
#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


bool
setUpSubscription(DJI::OSDK::Vehicle* vehicle, int responseTimeout) {
  
  return true;
}

bool
teardownSubscription(DJI::OSDK::Vehicle* vehicle, const int pkgIndex, int responseTimeout) {

  return true;
}

std::vector<DJI::OSDK::WayPointSettings> 
createWaypoints(DJI::OSDK::Vehicle* vehicle, int numWaypoints, float64_t distanceIncrement, float32_t start_alt) {
  

  return wpVector;
}

void
uploadWaypoints(Vehicle* vehicle, std::vector<DJI::OSDK::WayPointSettings>& wp_list, int responseTimeout) {

}

bool
runWaypointMission(Vehicle* vehicle, uint8_t numWaypoints, int responseTimeout) {

    return true;
}