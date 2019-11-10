#pragma once

/**
 * Maximum number of multi topic instances
 */
#define ORB_MULTI_MAX_INSTANCES	2


#define SYSID   1
#define COMPID  1


#define MAX_MISSION_ITEM_COUNT 200

#define MAV_MAX_RETRIES 5
#define MAV_TIMEOUT 3000000 // 3 sec




enum EVehicleNotification {
    WAY_POINT_REACHED,
    VEHICLE_LANDED,

};

enum EVehicleCmd {

    MISSION_START,
    MISSION_PAUSE,
    MISSION_RESUME,
    MISSION_STOP,

    UPLOAD_WAY_POINTS,

    RETURN_TO_ORIGIN
};

enum EResult {
    // Command successful executed 
    SUCCESS,
    // Invalid command (not available)
    INVALID,
    // Controller (state) is busy, command can not be executed
    BUSY,
    // Command execution failed (wrong state)
    FAILED,
    // Command in progress, ends later
    PROGRESS
};

