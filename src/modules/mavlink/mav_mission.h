#pragma once

#include "mav_service.h"
#include "defines.h"

struct Mavlink;

class MavlinkMissionManager : public MavlinkServiceManager
{
    public:
        MavlinkMissionManager(Mavlink *mavlink) : MavlinkServiceManager(mavlink),
            missionDownloadService(mavlink) { }

        void handle_message(const mavlink_message_t *msg) override;
        void run() override;
        
        void checkActiveMission();
        void activateMission();

    private:      

        class MissionDownloadService : public MavlinkService {
            public:
                MissionDownloadService(Mavlink *mavlink) : MavlinkService(mavlink),
                    missionDownloadInit(this), 
                    missionDownloadItem(this), 
                    missionDownloadEnd(this)
                { 
                    state = &missionDownloadInit;
                };

                // Variables
                int count = 0;
                int seq = 0;

                // Functions
                void handleMissionCount(const mavlink_message_t *msg);
                void handleMissionItem(const mavlink_message_t *msg);
                void handleMissionAck(const mavlink_message_t *msg);
                void handleMissionRequest(uint8_t sysid, uint8_t compid, uint16_t seq);
                void sendMissionAck(uint8_t sysid, uint8_t compid, uint8_t type);

                
                class MissionDownloadInit : public ServiceState<MissionDownloadService> {
                    public:
                        MissionDownloadInit(MissionDownloadService *context) : ServiceState(context) {};
                        void handleMessage(const mavlink_message_t *msg) override;
                        void entry() override;
                        void run() override;
                };

                class MissionDownloadItem : public ServiceState<MissionDownloadService> {
                    public:
                        MissionDownloadItem(MissionDownloadService *context) : ServiceState(context) {};

                        // Overrides
                        void handleMessage(const mavlink_message_t *msg) override;
                        void run() override;
                   
                };

                class MissionDownloadEnd : public ServiceState<MissionDownloadService> {
                    public:
                        MissionDownloadEnd(MissionDownloadService *context) : ServiceState(context) {};
                        void entry() override;
                };

                // States
                MissionDownloadInit missionDownloadInit;
                MissionDownloadItem missionDownloadItem;
                MissionDownloadEnd missionDownloadEnd;
        };     

        MissionDownloadService missionDownloadService;
};


