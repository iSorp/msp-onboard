#pragma once

#include <list>

#include "defines.h"
#include "mav_mavlink.h"

class MspController {
    
    typedef void (*VehicleCmdCallback)(EVehicleCmd cmd, void* data, size_t len);

    enum class EState : uint32_t{
        MISSION_EXISTS  = (1 << 0),
        MISSION_ACTIVE  = (1 << 1),
        MISSION_RUNNING = (1 << 2)
    };

    struct State;

    public:
        static MspController *getInstance();

        VehicleCmdCallback vehicleCmd = nullptr;

        void vehicleNotification(EVehicleNotification notification);
        
        void initialize(Mavlink* mavlink);
        
        // user commands
        EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd);

        bool missionIsActive();
        EResult missionDelete();
        EResult missionAddItem(mavlink_mission_item_t wp);


        std::vector<mavlink_mission_item_t> missionItems;

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

        State *state;
        
        // State functions 
        State* getState() {return state; };
        void setState(State *_state) {
            _state->exit();
            state = _state; 
            state->entry();
        };

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
                virtual void vehicleNotification(EVehicleNotification notification){}
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
                void vehicleNotification(EVehicleNotification notification) override;
                EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd) override;

            private:
                EResult missionStart();
                EResult missionStop();
                EResult missionPauseContinue(bool pause);
                void sendMissionItemReached(int seq);
        };

        class Command : public State {
            public:
                Command(MspController *context) : State(context) {};
                void vehicleNotification(EVehicleNotification notification) override;
                EResult cmdExecute(uint16_t command, mavlink_command_long_t cmd) override;
        };

        Init stateInit;
        Idle stateIdle;
        Mission stateMission;
        Command stateCommand;

};
