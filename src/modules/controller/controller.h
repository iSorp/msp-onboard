#pragma once

#include <list>
#include <map>

#include "spdlog/spdlog.h"
#include "defines.h"
#include "controller_def.h"
#include "mav_mavlink.h"

class MspController {
    
    using VehicleCmdCallback = EResult (*)(EVehicleCmd cmd, void* data, size_t len);
    using VehicleData = void*;

    struct State;

    public:
        static MspController *getInstance();

        VehicleCmdCallback vehicleCmd = ([] (EVehicleCmd cmd, void* data, size_t len) -> EResult{ });
        void vehicleNotification(EVehicleNotification notification, VehicleData data);
        
        void initialize(Mavlink* mavlink);
        MAV_STATE getMavState();
        uint8_t getMavMode();
        

        // user commands
        EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd);

        bool missionIsActive();

        // Mission repository
        EResult missionDelete();
        EResult missionAddItem(int key, mavlink_mission_item_t wp);

        mavlink_mission_item_t* getMissionBehaviorItem(int key);
        std::vector<mavlink_mission_item_t>* getMissionItem(int key);
        int getMissionItemCount();

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
        State* state;
        
        std::map<int, std::vector<mavlink_mission_item_t>> missionItemMap;
        
        // State functions 
        State* getState() {return state; };
        void setState(State *_state);

        struct State {
            public:
                State(MspController *context) : context(context) {};
                State() {}

                MspController *context;

                virtual EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd){
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
        };

        class Idle : public State {
            public:
                Idle(MspController *context) : State(context) {};
                void entry() override;
                EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd) override;
        };

        class Mission : public State {
            typedef void 
                (*SendDataCallback)(uint8_t* data, uint8_t len);

            public:
                Mission(MspController *context) : State(context) { };
                void entry() override;
                void exit() override;
                void vehicleNotification(EVehicleNotification notification, VehicleData data) override;
                EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd) override;

            private:
                EResult missionStart();
                EResult missionStop();
                EResult missionPauseContinue(bool pause);
                void sendMissionItemReached(int seq);
                void validateMissionItems();
        };

        class Command : public State {
            public:
                Command(MspController *context) : State(context) {};
                void vehicleNotification(EVehicleNotification notification, VehicleData data) override;
                EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd) override;
        };

        Init stateInit;
        Idle stateIdle;
        Mission stateMission;
        Command stateCommand;

};
