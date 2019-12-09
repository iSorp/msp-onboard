#pragma once

#include <list>
#include <map>

#include "spdlog/spdlog.h"
#include "defines.h"
#include "controller_def.h"
#include "mspvehicle.h"
#include "mav_mavlink.h"


typedef void* VehicleData;

// Typ for callback function which the data actually sends to the receiver

typedef EResult (*VehicleCommandCallback)(EVehicleCmd command, VehicleData vehicleData, void* userData, size_t len);
typedef struct VehicleCommandCallbackHandler{
    VehicleCommandCallback callback;
    VehicleData vehicleData;
}VehicleCommandCallbackHandler;

class MspController {
    
    
    struct State;

    public:
        static MspController *getInstance();

        // functions
        void initialize(Mavlink* mavlink);
        MAV_STATE getMavState();
        uint8_t getMavMode();

        void setVehicleCommandCallback(VehicleCommandCallback callback, VehicleData vehicleData);
        EResult setVehicleCommand(EVehicleCmd command);
        EResult setVehicleCommand(EVehicleCmd command, void* data, size_t len);
        void vehicleNotification(EVehicleNotification notification, VehicleData data);
        
        // user commands
        EResult setCommand(uint16_t command, mavlink_command_long_t cmd);

        bool missionIsActive();

        // Mission repository
        EResult missionDelete();
        EResult missionAddItem(int key, mavlink_mission_item_t wp);

        mavlink_mission_item_t* getMissionBehaviorItem(int key);
        std::vector<mavlink_mission_item_t>* getMissionItem(int key);
        size_t getMissionItemCount();

    protected:
        Mavlink* mavlink;

    private:
        MspController() : 
            stateInit(this),
            stateIdle(this),
            stateMission(this),
            stateCommand(this)
        {}

        static MspController *instance;
        VehicleCommandCallbackHandler vCmdCbHandler;
        State* state = nullptr;
        
        std::map<int, std::vector<mavlink_mission_item_t>> missionItemMap;
        VehicleInfoData vehicleInfo = {};

        // State functions 
        State* getState() {return state; };
        void setState(State *_state);

        struct State {
            public:
                State(MspController *context) : context(context) {};
                State() {}

                MspController *context;

                virtual void run() {};
                virtual EResult setCommand(uint16_t command, mavlink_command_long_t cmd){
                    return EResult::MSP_FAILED;
                }

                // State handling functions
                virtual void entry(){};
                virtual void exit(){};
    
                // commands from drone
                virtual void vehicleNotification(EVehicleNotification notification, VehicleData data){}
        };

        class Init: public State {
            public:
                Init(MspController *context) : State(context) { };
                void entry() override;
                void vehicleNotification(EVehicleNotification notification, VehicleData data) override;
        };

        class Idle : public State {
            public:
                Idle(MspController *context) : State(context) {};
                void entry() override;
                EResult setCommand(uint16_t command, mavlink_command_long_t cmd) override;
        };

        class Mission : public State {
            typedef void 
                (*SendDataCallback)(uint8_t* data, uint8_t len);

            public:
                Mission(MspController *context) : State(context) { };
                void entry() override;
                void exit() override;
                void vehicleNotification(EVehicleNotification notification, VehicleData data) override;
                EResult setCommand(uint16_t command, mavlink_command_long_t cmd) override;

            private:
                bool missionActive;
                bool userCommandPaused;

                EResult missionStart();
                EResult missionPauseContinue();
                void handleWpReached(VehicleData data);
                void executeAction(WaypointReachedData* wpdata);
                void sendMissionItemReached(int seq);
                void validateMissionItems();
        };

        class Command : public State {
            public:
                Command(MspController *context) : State(context) {};
                void vehicleNotification(EVehicleNotification notification, VehicleData data) override;
                EResult setCommand(uint16_t command, mavlink_command_long_t cmd) override;
        };

        Init stateInit;
        Idle stateIdle;
        Mission stateMission;
        Command stateCommand;

};
