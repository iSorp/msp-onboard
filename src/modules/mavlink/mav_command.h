/**
    @file mav_command.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        


#pragma once

#include <atomic>
#include "defines.h"
#include "controller_def.h"
#include "mav_service.h"

struct Mavlink;

class MavlinkCommandManager : public MavlinkServiceManager
{
    public:
        MavlinkCommandManager(Mavlink *mavlink) : MavlinkServiceManager(mavlink),
            commandService(mavlink) { }

        void handleMessages(const mavlink_message_t *msg) override;
        void run() override;

        void setCmdResult(EResult result){
            commandService.cmdResult = result;
        }

    private:      

        class CommandService : public MavlinkService {
            public:
                CommandService(Mavlink *mavlink) : MavlinkService(mavlink),
                    commandInit(this),
                    commandReceive(this), 
                    commandProgress(this),
                    commandEnd(this)
                { 
                    state = &commandInit;
                };

                // Variables
                std::atomic<EResult> cmdResult;
                
                uint16_t currentCmd = 0;
                int confirmation = 0;
                int progress = 0;
                int seq = 0;

                // Functions
                void sendCmdAck(uint16_t cmd, uint8_t result);
                void handleCmdErrorResultAndInit(EResult result, uint16_t command);

                class CommandInit : public ServiceState<CommandService> {
                    public:
                        CommandInit(CommandService *context) : ServiceState(context) {};
                        void entry() override;
                };

                class CommandReceive : public ServiceState<CommandService> {
                    public:
                        CommandReceive(CommandService *context) : ServiceState(context) {};
                        void handleMessage(const mavlink_message_t *msg) override;
                };

                class CommandProgress : public ServiceState<CommandService> {
                    public:
                        CommandProgress(CommandService *context) : ServiceState(context) {};
                        void run() override;
                };
                class CommandEnd : public ServiceState<CommandService> {
                    public:
                        CommandEnd(CommandService *context) : ServiceState(context) {};
                        void entry() override;
                };


                // States
                CommandInit commandInit;
                CommandReceive commandReceive;
                CommandProgress commandProgress;
                CommandEnd commandEnd;
        };     

        CommandService commandService;
};


