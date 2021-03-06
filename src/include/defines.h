#pragma once


/**
 * DJI
 */
#define DJI_USER_CONFIG "/etc/msp-onboard/UserConfig.txt"

/**
 * Mavlink
 */
#define ORB_MULTI_MAX_INSTANCES	2

#define SYSID   2
#define COMPID  1

#define MAX_MISSION_ITEM_COUNT  200
#define MAV_MAX_RETRIES         5
#define MAV_TIMEOUT             3000000 // 3 sec
#define MAV_HEARTBEAT_INTERVAL  1000000 // 1 sec


/**
 * Mission
 */
#define WP_EXPORT_PATH "/var/msp-onboard/export"


#define FTP_PATH "/var/msp-onboard/export"
