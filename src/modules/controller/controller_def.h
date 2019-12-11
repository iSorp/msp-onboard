/**
    Definitions for MspController
    @file controller_def.h
    @version 1.0
    @author Simon Waelti
    @version 1.0 1.12.2019
*/


#pragma once

enum EVehicleNotification {
    MSP_VHC_STATE,
    MSP_VHC_WAY_POINT_REACHED,
    MSP_VHC_LANDED,
    MSP_VHC_TAKEOFF,
    MSP_VHC_MISSION_PAUSED,
    MSP_VHC_MISSION_RESUMED,
    MSP_VHC_MISSION_STOPPED,
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
    MSP_VHC_NOT_AVAILABLE = -1, // init state
    MSP_VHC_AVAILABLE,
    MSP_VHC_SIMULATION
};

typedef struct VehicleInfoData {        
    EVehicleState state;
    uint8_t mode;      // bit mask for MAV_MODE_FLAG
}VehicleInfoData;

typedef struct WaypointReachedData {        
    u_int16_t index;
    float longitude;    // x /*!< unit: rad */
    float latitude;     // y /*!< unit: rad */     
    float altitude;     // z /*!< WGS 84 reference ellipsoid */
} WaypointReachedData;