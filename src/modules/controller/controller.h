#pragma once

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
        
        void initialize(Mavlink* mavlink);
        
        // user commands
        EResult cmdExecute(uint16_t command);
        EResult handleMessage(uint16_t message);

        void missionActivate(){}
        void missionAddItem(mavlink_mission_item_t wp);

        EState getStateFlag() {
            return drone_state;
        }

        // State functions 
        State* getState() {return state; };

        void setState(State *_state) {
            _state->exit();
            state = _state; 
            state->entry();
        };

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

        EState drone_state;
        State *state;
        std::vector<mavlink_mission_item_t> missionItems;
    
        void setStateFlag(EState flag) {}

        // functions
        EResult missionDelete();


        struct State {
            public:
                State(MspController *context) : context(context) {};
                State() {}

                MspController *context;

                virtual EResult cmdExecute(uint16_t command){
                    return EResult::FAILED;
                }

                // State handling functions
                virtual void entry(){};
                virtual void exit(){};


                // Mission commands
                virtual EResult missionStart(){}
                virtual EResult missionStop(){}
        
                // Drone commands
                virtual EResult returnToOrigin(){}
                virtual EResult takeOff(){}
                virtual EResult land(){}

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
                EResult cmdExecute(uint16_t command) override;
        };

        class Mission : public State {
            typedef void 
                (*SendDataCallback)(uint8_t* data, uint8_t len);

            public:
                Mission(MspController *context) : State(context) { };
                void entry() override;
                void exit() override;
                void vehicleNotification(EVehicleNotification notification) override;
                EResult cmdExecute(uint16_t command) override;
                EResult missionStart() override;
                EResult missionStop() override;

                void sendMissionItemReached(int seq);
        };

        class Command : public State {
            public:
                Command(MspController *context) : State(context) {};
                void vehicleNotification(EVehicleNotification notification);
                EResult cmdExecute(uint16_t command) override;
        };

        Init stateInit;
        Idle stateIdle;
        Mission stateMission;
        Command stateCommand;

};
