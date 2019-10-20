
#pragma once


#include "mavlink_bridge_header.h"
#include "defines.h"


class Mavlink;

class MavlinkMissionManager
{
    public:
        MavlinkMissionManager(Mavlink *mavlink) : mavlink(mavlink) {
            missionUploader = new MissionStateContext(mavlink);
        }

        ~MavlinkMissionManager() {
            delete missionUploader;
        }

        void handle_message(const mavlink_message_t *msg);
        void run();
        void checkActiveMission();
        void activateMission();
      
    protected:
        

    private:      
        class MissionStateContext;

        Mavlink *mavlink;
        MissionStateContext *missionUploader;


        class MissionState  {
            public:
                MissionState(MissionStateContext *context) : context(context) {};

                virtual void handleMessage(const mavlink_message_t *msg);
                virtual void entry();
                virtual void exit();
                virtual void run();

            protected:
                MissionStateContext *context;

                void handleMissionCount(const mavlink_message_t *msg);
                void handleMissionItemInt(const mavlink_message_t *msg);
                void handleMissionAck(const mavlink_message_t *msg);
                void handleMissionRequest(uint8_t sysid, uint8_t compid, uint16_t seq);
                void sendMissionAck(uint8_t sysid, uint8_t compid, uint8_t type);
        };

        class MissionUploadInit : public MissionState {
            public:
                MissionUploadInit(MissionStateContext *context) : MissionState(context) {};
                ~MissionUploadInit() {};

                // Overrides
                void handleMessage(const mavlink_message_t *msg) override;
                void entry() override;
                void run() override;
        };

        class MissionUploadItemInt : public MissionState {
            public:
                MissionUploadItemInt(MissionStateContext *context) : MissionState(context) {};
                ~MissionUploadItemInt() {};

                // Overrides
                void handleMessage(const mavlink_message_t *msg) override;
                void run() override;
        };

       class MissionUploadEnd : public MissionState {
            public:
                MissionUploadEnd(MissionStateContext *context) : MissionState(context) {};
                ~MissionUploadEnd() {};

                // Overrides
                void entry() override;
        };

        class MissionStateContext {
            public:
                MissionStateContext(Mavlink *mavlink) : 
                    mavlink(mavlink),
                    missionUploadInit(this),
                    missionUploadItemInt(this),
                    missionUploadEnd(this) 
                { 
                    state = &missionUploadInit;
                };

                ~MissionStateContext() { };

                Mavlink *mavlink;

                // States
                MissionUploadInit missionUploadInit;
                MissionUploadItemInt missionUploadItemInt;
                MissionUploadEnd missionUploadEnd;

                int count = 0;
                int seq = 0;
                int transferSysId = 0;
			    int transferCompId = 0;
                int repeatCounter = 0;
                const uint64_t sendResponseTimeout = 3000000;

                // Overrides
                //void run(const mavlink_message_t *msg);

                // Functions
                MissionState* getState() {return state; };
                void setState(MissionState *_state) {
                    _state->exit();
                    state = _state; 
                    state->entry();
                };
            private:

                // States
                MissionState *state;
        };     
};


