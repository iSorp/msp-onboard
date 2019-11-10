#include <cmath>
#include <vector>
#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

#include "dji_mission_interface.h"
#include "controller.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

Vehicle* vehicle;
std::vector<WayPointSettings> wp_list;

int responseTimeout = 3;

const int DEFAULT_PACKAGE_INDEX = 0;

//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------
static void 
cmdCallback(EVehicleCmd cmd, void* data, size_t len);

static void
commandDataToOSDK();

static void
runWaypointMission(); 

static void
uploadWaypoints();

void
createWaypoints(void* data, size_t len);

static std::vector<DJI::OSDK::WayPointSettings>
generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment, int num_wp);

static void
setWaypointDefaults(WayPointSettings* wp);

static void
setWaypointInitDefaults(WayPointInitSettings* fdata);


//-------------------------------------------------------------
// Initialize Mission functionalities
//-------------------------------------------------------------
bool
setupDJIMission(Vehicle* vehicleDJI, LinuxSetup* linuxEnvironment)
{
    // registered dji vehicle
    vehicle = vehicleDJI;

    // set callback for controller command data
    MspController::getInstance()->vehicleCmd = &cmdCallback;
}

//-------------------------------------------------------------
// Calback command function from/To MSPController
//-------------------------------------------------------------
void 
cmdCallback(EVehicleCmd cmd, void* data, size_t len) {
    switch (cmd)
    {
    case EVehicleCmd::UPLOAD_WAY_POINTS:
        createWaypoints(data, len);
        break;

    case EVehicleCmd::MISSION_START:
        uploadWaypoints();
        runWaypointMission();
        break;
    
    default:
        break;
    }
}

void wayPointCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {

    MspController::getInstance()->getState()->vehicleNotification(EVehicleNotification::WAY_POINT_REACHED);
}


//-------------------------------------------------------------
// Mission upload and running
//-------------------------------------------------------------
void
uploadWaypoints() {

    // Waypoint Mission : Initialization
    WayPointInitSettings fdata;
    setWaypointInitDefaults(&fdata);

    fdata.indexNumber = wp_list.size() + 1; // We add 1 to get the aircarft back to the start.

    float64_t increment = 0.000001;
    float32_t start_alt = 10;

    ACK::ErrorCode initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);
    if (ACK::getError(initAck))
    {
        ACK::getErrorCodeMessage(initAck, __func__);
    }

    for (std::vector<WayPointSettings>::iterator wp = wp_list.begin(); wp != wp_list.end(); ++wp)
    {
        printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude, wp->longitude, wp->altitude);
        ACK::WayPointIndex wpDataACK = vehicle->missionManager->wpMission->uploadIndexData(&(*wp), responseTimeout);
        ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
    }

    // Add callback for waypoint management
    vehicle->missionManager->wpMission->setWaypointCallback(wayPointCallback, NULL);
}

void
runWaypointMission() {
    
    // Waypoint Mission: Start
    ACK::ErrorCode startAck = vehicle->missionManager->wpMission->start(responseTimeout);
    if (ACK::getError(startAck))
    {
        ACK::getErrorCodeMessage(startAck, __func__);
    }
    else
    {
        std::cout << "Starting Waypoint Mission.\n";
    }
}

//-------------------------------------------------------------
// Waypoint creation
//-------------------------------------------------------------
void
createWaypoints(void* data, size_t len) {

    float32_t start_alt = 10;

    // Create Start Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition broadcastGPosition;

    broadcastGPosition = vehicle->broadcast->getGlobalPosition();
    start_wp.latitude  = broadcastGPosition.latitude;
    start_wp.longitude = broadcastGPosition.longitude;
    start_wp.altitude  = start_alt;
    printf("Waypoint created at (LLA): %f \t%f \t%f\n",broadcastGPosition.latitude, broadcastGPosition.longitude, start_alt);

    wp_list = generateWaypointsPolygon(&start_wp, 1, len);
}

std::vector<DJI::OSDK::WayPointSettings>
generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment, int num_wp)
{
    // Let's create a vector to store our waypoints in.
    std::vector<DJI::OSDK::WayPointSettings> wp_list;

    // Some calculation for the polygon
    float64_t extAngle = 2 * M_PI / num_wp;

    // First waypoint
    start_data->index = 0;
    wp_list.push_back(*start_data);

    // Iterative algorithm
    for (int i = 1; i < num_wp; i++)
    {
        WayPointSettings  wp;
        WayPointSettings* prevWp = &wp_list[i - 1];
        setWaypointDefaults(&wp);
        wp.index     = i;
        wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
        wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
        wp.altitude  = (prevWp->altitude + 1);
        wp_list.push_back(wp);
    }

    // Come back home
    start_data->index = num_wp;
    wp_list.push_back(*start_data);

    return wp_list;
}

void
setWaypointDefaults(WayPointSettings* wp)
{
    wp->damping         = 0;
    wp->yaw             = 0;
    wp->gimbalPitch     = 0;
    wp->turnMode        = 0;
    wp->hasAction       = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber    = 0;
    wp->actionRepeat    = 0;
    for (int i = 0; i < 16; ++i)
    {
        wp->commandList[i]      = 0;
        wp->commandParameter[i] = 0;
    }
}

void
setWaypointInitDefaults(WayPointInitSettings* fdata)
{
    fdata->maxVelocity    = 10;
    fdata->idleVelocity   = 5;
    fdata->finishAction   = 0;
    fdata->executiveTimes = 1;
    fdata->yawMode        = 0;
    fdata->traceMode      = 0;
    fdata->RCLostAction   = 1;
    fdata->gimbalPitch    = 0;
    fdata->latitude       = 0;
    fdata->longitude      = 0;
    fdata->altitude       = 0;
}
