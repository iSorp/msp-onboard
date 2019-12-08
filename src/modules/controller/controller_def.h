#pragma once

enum EVehicleNotification {
    MSP_VHC_STATE,
    MSP_VHC_WAY_POINT_REACHED,
    MSP_VHC_LANDED,
    MSP_VHC_TAKEOFF,
};

enum EVehicleCmd {
    MSP_CMD_READ_STATE,

    MSP_CMD_RETURN_TO_ORIGIN,
    MSP_CMD_TAKEOFF,
    MSP_CMD_LAND,


    MSP_CMD_MISSION_START,
    MSP_CMD_MISSION_PAUSE,
    MSP_CMD_MISSION_RESUME,
    MSP_CMD_MISSION_STOP,

    MSP_CMD_UPLOAD_WAY_POINTS,

    MSP_CMD_TAKE_PICTURE,
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

enum EVehicleState {
    MSP_VHC_AVAILABLE = 1,
    MSP_VHC_READY     = 2,
    MSP_VHC_MISSION   = 3,
};

struct vehicleStateData_t {        
    uint8_t state;      // bit mask for EVehicleState
};

struct waypointReachedData_t {        
    u_int16_t index;
    float longitude;    // x /*!< unit: rad */
    float latitude;     // y /*!< unit: rad */     
    float altitude;     // z /*!< WGS 84 reference ellipsoid */
};