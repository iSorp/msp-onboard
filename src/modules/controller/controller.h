/**
    @file controller.h
    @brief
    Definition of the MspController containing a state machine for vehicle commands.

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/

#pragma once

#include <list>
#include <map>

#include "spdlog/spdlog.h"
#include "defines.h"
#include "controller_def.h"
#include "mspvehicle.h"
#include "mav_mavlink.h"

/**
    Generic pointer which represends data send by a VehicleCommand
    @see VehicleCommandCallback
*/
typedef void* VehicleData;

/**
    Typ for callback function which actually sends the data to the receiver
    @param command 
    @param vehicleData 
    @param userData 
    @param len 
*/
typedef EResult (*VehicleCommandCallback)(EVehicleCmd command, VehicleData vehicleData, void* userData, size_t len);

/**
    Typ for callback function handler, contains callback function and data defined by the vehicle
*/
typedef struct VehicleCommandCallbackHandler{
    VehicleCommandCallback callback;
    VehicleData vehicleData;
}VehicleCommandCallbackHandler;


/**
    The MspController class implements the singleton pattern. It contains following componets:
    - State machine for vehicle commands
    - In-Memory repository for waypoint data

    The controller is passive, means it lives only of external threads (vehicle callback and mavlink).

*/
class MspController {
    
    struct State;

    public:
        static MspController *getInstance();

        /**
            Initializes the Controller and sets the state machines init state. 
            @param mavlink
        */
        void initialize(Mavlink* mavlink);
        /**
            Returns the current vehicle state represented as Mavlink state.
            @return MAV_STATE
        */
        MAV_STATE getMavState();
        /**
            Returns the current vehicle mode represented as Mavlink mode.
            @see MAV_MODE_FLAG
            @return mode
        */
        uint8_t getMavMode();
        /**
            Sets the vehicle command callback function.
            @param callback
            @param vehicleData
        */
        void setVehicleCommandCallback(VehicleCommandCallback callback, VehicleData vehicleData);
        /**
            Vehicles should notify message by calling this function. this function is synchronized with setCommand. This avoids a concurrent manipulating of 
            the state machine.
            @see EResult setCommand(uint16_t command, mavlink_command_long_t cmd);
            @param notification
            @param data pointer to the caller object
        */
        void vehicleNotification(EVehicleNotification notification, VehicleData data);
        /**
            Sets a mavlink command, this function is synchronized with vehicleNotification. This avoids a concurrent manipulating of 
            the state machine.
            @see EResult void vehicleNotification(EVehicleNotification notification, VehicleData data);
            @param command encoded Mavlink command
            @param cmd Mavlink command message
        */
        EResult setCommand(uint16_t command, mavlink_command_long_t cmd);


        //------------------------------------------------------------- 
        // Mission flight control
        //-------------------------------------------------------------
        int16_t getCurrentWp();


        //------------------------------------------------------------- 
        // Mission repository
        //-------------------------------------------------------------
        /**
            Returns the state whether a mission is active or not
            @return state
        */
        bool missionIsActive();
        /**
            Deletes the current mission (way points)
            @return EResult
        */
        EResult missionDelete();
        /**
            Adds a way p0int to the current mission
            @return EResult
        */
        EResult missionAddItem(int key, mavlink_mission_item_t wp);
        /**
            Return a pointer to a certain mission behavior item
            (mission item which only contains navigation properties, no actions).
            @return mavlink_mission_item_t
        */
        mavlink_mission_item_t* getMissionBehaviorItem(int key);
        /**
            Return a pointer to a certain mission item set 
            (the first item in the set is allway the behavior item).
            @return mavlink_mission_item_t
        */
        std::vector<mavlink_mission_item_t>* getMissionItem(int key);
        /**
            Returns the number of mission items(behavior)
            @return size_t
        */
        size_t getMissionItemCount();


    private:
        MspController() : 
            stateInit(this),
            stateIdle(this),
            stateMission(this),
            stateCommand(this)
        {}

        static MspController *instance;
        VehicleCommandCallbackHandler vCmdCbHandler;
        Mavlink* mavlink;
        State* state = nullptr;
        
        std::map<int, std::vector<mavlink_mission_item_t>> missionItemMap;
        VehicleInfoData vehicleInfo = {};


        /**
            Sets the a vehicle command.
            @param command
        */
        EResult setVehicleCommand(EVehicleCmd command);
        /**
            Sets the a vehicle command.
            @param command
            @param data
            @param len
        */
        EResult setVehicleCommand(EVehicleCmd command, void* data, size_t len);

        //------------------------------------------------------------- 
        // State machine
        //-------------------------------------------------------------
         /**
            Returns the current state of the state machine
            @return State
        */
        State* getState() {return state; };
        /**
            Sets the current state of the state machine.
            @param state
        */
        void setState(State *state);

        /**
            Abstract state class
        */
        struct State {
            public:
                State(MspController *context) : context(context) {};
                State() {}

                MspController *context;

                /**
                    Sets a mavlink command, this function is called by the MspController.
                    @see MspController::setCommand
                    @param command encoded Mavlink command
                    @param cmd Mavlink command message
                */
                virtual EResult setCommand(uint16_t command, mavlink_command_long_t cmd){
                    return EResult::MSP_FAILED;
                }
                /**
                    Vehicles should notify message by calling this function, this function is called by the MspController.
                    @see MspController::vehicleNotification;
                    @param command encoded Mavlink command
                    @param cmd Mavlink command message
                */
                virtual void vehicleNotification(EVehicleNotification notification, VehicleData data){}

                /**
                    State entry function
                */
                virtual void entry(){};
                /**
                    State exit function
                */
                virtual void exit(){};
        };

        /**
            Class for initializing the state machine.
            Initializes also the Sensormanager.
        */
        class Init: public State {
            public:
                Init(MspController *context) : State(context) { };
                void entry() override;
                void vehicleNotification(EVehicleNotification notification, VehicleData data) override;
        };
        /**
            Idle state if no command/mission is active. First state after initialization.
        */
        class Idle : public State {
            public:
                Idle(MspController *context) : State(context) {};
                void entry() override;
                EResult setCommand(uint16_t command, mavlink_command_long_t cmd) override;
        };
        /**
            State to perform mission activities, such as mission start, stop, pause/resume, performing waypoint actions.
        */
        class Mission : public State {
            typedef void 
                (*SendDataCallback)(uint8_t* data, uint8_t len);

            public:
                Mission(MspController *context) : State(context) { };
                void entry() override;
                void exit() override;
                void vehicleNotification(EVehicleNotification notification, VehicleData data) override;
                EResult setCommand(uint16_t command, mavlink_command_long_t cmd) override;
                int16_t getCurrent() {
                    return currenindex;
                }

            private:
                bool missionActive;
                bool userCommandPaused;
                int16_t currenindex;
                /**
                    Starts a way point mission, state transfer to idle on error
                    @return EResult
                */
                EResult missionStart();
                /**
                    Pauses/resumes a way point mission, state transfer to idle on error
                    @return EResult
                */
                EResult missionPauseContinue();
                /**
                    Is called when a way point is reached.
                    This function manages the logic for executing actions and next waypoints, 
                    if the number of way point is reached, a state transfer to idle is performed.
                    @param data
                */
                void handleWpReached(VehicleData data);                
                /**
                    Performs all action of the current way point. 
                    @param wpdata
                */
                void executeAction(WaypointReachedData* wpdata);
                /**
                    Sends a mission item reached mavlink message
                    @param seq mission item sequece (current index)
                */
                void sendMissionItemReached(int seq);
                /**
                    Validates all mission items.
                    (this function is not used due the fact, dji performs already a validation itself)
                */
                void validateMissionItems();
        };
        /**
            State to perform commands like landing, takoff, ..
        */
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
