#pragma once

#include "mavlink.h"

enum class EState : uint32_t{
    MISSION_EXISTS  = (1 << 0),
    MISSION_ACTIVE  = (1 << 1),
    MISSION_RUNNING = (1 << 2)
};

class FlightController {
    
    struct State;

    public:
        static FlightController *getInstance();

        void initialize();


        // Mission commands
        void missionStart(){}
        void missionStop(){}
        void missionActivate(){}
        void missionValidate(){}
        void missionDelete(){}
        void missionAddItem(mavlink_mission_item_t wp){}

        // Drone commands
        void cmdReturnToOrigin(){}
        void cmdTakeOff(){}
        void cmdToLand(){}

        EState getStateFlag() {
            return drone_state;
        }

       
        // Functions 
        
        State* getState() {return state; };

        void setState(State *_state) {
            _state->exit();
            state = _state; 
            state->entry();
        };

    private:
        FlightController() : 
            stateInit(this),
            stateIdle(this),
            stateWpPending(this),
            stateWpReached(this),
            stateMisionDone(this),
            stateToOrigin(this)
        {}

        static FlightController *instance;

        EState drone_state;
        State *state;

    
        void setStateFlag(EState flag) {
        
        }

        struct State {
            public:
                State(FlightController *context) : context(context) {};
                State() {}

                FlightController *context;

                virtual void entry() {};
                virtual void exit() {};

                virtual void missionStart(){}
                virtual void missionStop(){}
                virtual void returnToOrigin(){}
        };

        class Init: public State {
            public:
                Init(FlightController *context) : State(context) { };
                void entry() override;
        };

        class Idle : public State {
            public:
                Idle(FlightController *context) : State(context) {};
                void entry() override;
                void missionStart() override;
                void returnToOrigin() override;
        };

        class WPPending : public State {
            public:
                WPPending(FlightController *context) : State(context) { };
                void entry() override;
        };
 
        class WPReached : public State {
            public:
                WPReached(FlightController *context) : State(context) { };
                void entry() override;
        };

        class MissionDone : public State {
            public:
                MissionDone(FlightController *context) : State(context) {};
                void entry() override;
        };

        class ToOrigin : public State {
            public:
                ToOrigin(FlightController *context) : State(context) {};
        };



        Init stateInit;
        Idle stateIdle;
        WPPending stateWpPending;
        WPReached stateWpReached;
        MissionDone stateMisionDone;
        ToOrigin stateToOrigin;

};
