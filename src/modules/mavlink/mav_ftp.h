/**
    @file mav_ftp.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        


#pragma once
#include <iostream>
#include <fstream>

#include "mavlink_types.h"
#include "mav_service.h"
#include "mav_ftp_protocol.h"

struct Mavlink;

class MavlinkFtpManager : public MavlinkServiceManager
{
    public:
        MavlinkFtpManager(Mavlink *mavlink) : MavlinkServiceManager(mavlink),
            fileUploadService(mavlink),
            listDirectoryService(mavlink) {}
        void handleMessages(const mavlink_message_t *msg) override;
        void run() override;

        // Functions

        // Classes
        class FileUploadService : public MavlinkService {
            public:
                FileUploadService(Mavlink *mavlink) : MavlinkService(mavlink), 
                    fileUploadInit(this), 
                    fileUploadWrite(this), 
                    fileUploadEnd(this)
                { 
                    state = &fileUploadInit;
                };
                
                // Variables
                std::ifstream file;
                uint32_t file_size = 0;
                uint16_t seq = 0;
                uint16_t seq_rec = 0;
                uint8_t session = 0;
                uint8_t reqCode;
                const char *cpath;

                // Functions
                void sendNakFailure(uint8_t sysid, uint8_t compid, uint8_t failure); 
                void sendOpenSession();
                void sendData(char *buffer, int size);
                void sendEofData();
                void sendAckData();

                // States
                class FileUploadInit : public ServiceState<FileUploadService> {
                    public:
                        FileUploadInit(FileUploadService *context) : ServiceState(context) {};
                        void handleMessage(const mavlink_message_t *msg) override;
                        void entry() override;
                };

                class FileUploadWrite : public ServiceState<FileUploadService> {
                    public:
                        FileUploadWrite(FileUploadService *context) : ServiceState(context) {};
                        void handleMessage(const mavlink_message_t *msg) override;
                        void run() override;
                };

                class FileUploadEnd : public ServiceState<FileUploadService> {
                    public:
                        FileUploadEnd(FileUploadService *context) : ServiceState(context) {};
                        void handleMessage(const mavlink_message_t *msg) override;
                        void entry() override;
                        void run() override;
                };

                // States
                FileUploadInit fileUploadInit;
                FileUploadWrite fileUploadWrite;
                FileUploadEnd fileUploadEnd;
        };     

        class ListDirectoryService : public MavlinkService {
            public:
                ListDirectoryService(Mavlink *mavlink) : MavlinkService(mavlink),  
                    listDirectoryInit(this),
                    listDirectoryWrite(this)
                { 
                    state = &listDirectoryInit;
                };
                
                // Variables
                uint16_t offset = 0;
                std::string entries;

                // Functions
                void sendData(const char *buffer, const size_t size);
                void sendEofData();

                class ListDirectoryInit : public ServiceState<ListDirectoryService> {
                    public:
                        ListDirectoryInit(ListDirectoryService *context) : ServiceState(context) {};
                };
                class ListDirectoryWrite : public ServiceState<ListDirectoryService> {
                    public:
                        ListDirectoryWrite(ListDirectoryService *context) : ServiceState(context) {};
                        void handleMessage(const mavlink_message_t *msg) override;
                        void entry() override;
                        void run() override;
                };

                // States
                ListDirectoryInit listDirectoryInit;
                ListDirectoryWrite listDirectoryWrite;
        };   
    private:      
        FileUploadService fileUploadService;
        ListDirectoryService listDirectoryService;

};


