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
    MSP_VHC_WAY_POINT_REACHED,
    MSP_VHC_LANDED,
    MSP_VHC_TAKEOFF,
};

enum EVehicleCmd {
    MSP_CMD_RETURN_TO_ORIGIN,
    MSP_CMD_TAKEOFF,
    MSP_CMD_LAND,


    MSP_CMD_MISSION_START,
    MSP_CMD_MISSION_PAUSE,
    MSP_CMD_MISSION_RESUME,
    MSP_CMD_MISSION_STOP,

    MSP_CMD_UPLOAD_WAY_POINTS,
};

enum EResult {
    // Command successful executed 
    MSP_SUCCESS,
    // Invalid command (not available)
    MSP_INVALID,
    // Controller (state) is busy, command can not be executed
    MSP_BUSY,
    // Command execution failed (wrong state)
    MSP_FAILED,
    // Command in progress, ends later
    MSP_PROGRESS
};

