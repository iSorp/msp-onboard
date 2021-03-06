/**
    @file mav_ftp.cpp
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        


#include <stdio.h>
#include <string>
#include <iostream>
#include <filesystem>
#include <string>

#include "mav_ftp.h"
#include "mav_mavlink.h"
#include "helper.h"

using namespace std;
namespace fs = std::filesystem;


static void 
eraseSubStr(std::string & mainStr, const std::string & toErase) {
	size_t pos = mainStr.find(toErase);
	if (pos != std::string::npos)
	{
		mainStr.erase(pos, toErase.length());
	}
}

//-------------------------------------------------------------
// Class MavlinkFtpManager
//-------------------------------------------------------------
void
MavlinkFtpManager::run() {
    fileUploadService.getState()->run();
}

void
MavlinkFtpManager::handleMessages(const mavlink_message_t *msg) {
    if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
        mavlink_file_transfer_protocol_t ftp;
        mavlink_msg_file_transfer_protocol_decode(msg, &ftp);

        // Initialize state machine iff no session exists (only one session possible)
        if (ftp.payload[CODE] == OpenFileRO) {
            if (fileUploadService.session <= 0) {
                spdlog::info("MavlinkFtpManager::handleMessages,  start file upload");
                fileUploadService.setState(&fileUploadService.fileUploadInit);
            }
            // send error on file open request
            else {
                spdlog::warn("MavlinkFtpManager::handleMessages, no session available");
                fileUploadService.sendNakFailure(msg->sysid, msg->compid, NOSESS);
            }
        }


        // list directory request
        if (ftp.payload[CODE] == ListDirectory) {
            if (typeid(*listDirectoryService.getState()) == typeid(ListDirectoryService::ListDirectoryInit)) {
            
                spdlog::info("MavlinkFtpManager::handleMessages,  start list directory");
                listDirectoryService.setState(&listDirectoryService.listDirectoryWrite);
            }
        }

        fileUploadService.getState()->handleMessage(msg);
        listDirectoryService.getState()->handleMessage(msg);
    }  
}

//-------------------------------------------------------------
// Class FileUploadService 
//-------------------------------------------------------------
void
MavlinkFtpManager::FileUploadService::sendNakFailure(uint8_t sysid, uint8_t compid, uint8_t failure) {
    mavlink_file_transfer_protocol_t ftp;
    ftp.target_system       = sysid;
    ftp.target_component    = compid;
    memcpy(&ftp.payload[SEQ], &seq, 2);
    ftp.payload[SESS]       = session; 
    ftp.payload[CODE]       = NAK;
    ftp.payload[SIZE]       = 1;
    ftp.payload[DATA]       = failure; 
    mavlink_msg_file_transfer_protocol_send_struct(mavlink->getChannel(), &ftp);
}

void
MavlinkFtpManager::FileUploadService::sendOpenSession() {

    // Build message payload
    mavlink_file_transfer_protocol_t ftp;

    ftp.target_system       = transferSysId;
    ftp.target_component    = transferCompId;

    //reinterpret_cast
    memcpy(&ftp.payload[SEQ], &seq, 2);
    ftp.payload[SESS]       = session; 
    ftp.payload[CODE]       = ACK; 
    ftp.payload[SIZE]       = 4;
    ftp.payload[REQCODE]    = reqCode;
    
    memcpy(&ftp.payload[DATA], &file_size, sizeof(file_size));

    // send message: ACK( session, size=4, data=len(file) )
    mavlink_msg_file_transfer_protocol_send_struct(mavlink->getChannel(), &ftp);
}

void
MavlinkFtpManager::FileUploadService::sendData(char *buffer, int size) {

    // Build message payload
    mavlink_file_transfer_protocol_t ftp;

    ftp.target_system       = transferSysId;
    ftp.target_component    = transferCompId;
    memcpy(&ftp.payload[SEQ], &seq, 2);
    ftp.payload[SESS]       = session; 
    ftp.payload[CODE]       = ACK; 
    ftp.payload[SIZE]       = size;
    ftp.payload[REQCODE]    = reqCode;
    
    memcpy(&ftp.payload[DATA], buffer, size);

    // send message: ACK( session, size=4, data=len(file) )
    mavlink_msg_file_transfer_protocol_send_struct(mavlink->getChannel(), &ftp);
}

/**
 * Message send: NAK(session, size=1, data=EOF)
 */
void
MavlinkFtpManager::FileUploadService::sendEofData() {

    // Build message payload
    mavlink_file_transfer_protocol_t ftp;

    ftp.target_system       = transferSysId;
    ftp.target_component    = transferCompId;
    memcpy(&ftp.payload[SEQ], &seq, 2);
    ftp.payload[SESS]       = session; 
    ftp.payload[CODE]       = NAK; 
    ftp.payload[SIZE]       = 1;
    ftp.payload[REQCODE]    = reqCode;
    ftp.payload[DATA]       = NEOF;
    
    // send message: NAK
    mavlink_msg_file_transfer_protocol_send_struct(mavlink->getChannel(), &ftp);
}

/**
 * Message send: ACK()
 */
void
MavlinkFtpManager::FileUploadService::sendAckData() {

    // Build message payload
    mavlink_file_transfer_protocol_t ftp;

    ftp.target_system       = transferSysId;
    ftp.target_component    = transferCompId;
    memcpy(&ftp.payload[SEQ], &seq, 2);
    ftp.payload[CODE]       = ACK;
    ftp.payload[REQCODE]    = TERM;
    
    // send message: ACK( session, size=4, data=len(file) )
    mavlink_msg_file_transfer_protocol_send_struct(mavlink->getChannel(), &ftp);
}

//-------------------------------------------------------------
// Class FileUploadInit 
//-------------------------------------------------------------
void
MavlinkFtpManager::FileUploadService::FileUploadInit::entry() { 
    context->seq = 0;
    context->file_size = 0;
    context->session = 0;
    context->cpath = NULL;
    context->transferSysId = 0;
    context->transferCompId = 0;
    context->repeatCounter = 0;

    if (context->file.is_open()){
        context->file.close();
    }
}

void
MavlinkFtpManager::FileUploadService::FileUploadInit::handleMessage(const mavlink_message_t *msg) { 
    mavlink_file_transfer_protocol_t ftp;
    mavlink_msg_file_transfer_protocol_decode(msg, &ftp);

    // message received: OpenFileRO( data[0]=filePath, size=len(filePath) )
    if (ftp.payload[CODE] == OpenFileRO) {
        spdlog::debug("FileUploadInit::handleMessages, opening file");

        context->transferSysId  = msg->sysid;
        context->transferCompId = msg->compid;
        context->seq_rec = *(uint16_t *)&ftp.payload[SEQ];

        // Open requested file
        std::string file_path;
        file_path.append(FTP_PATH);
        file_path.append("/");
        file_path.append((char*)&ftp.payload[DATA]);
        const char *cstr = file_path.c_str();
        context->cpath = cstr;
        context->file.open(string(context->cpath), ios::binary);
        if (context->file.is_open()) {
            spdlog::debug("FileUploadInit::handleMessages, file found and open");
            
            // get length of file:
            context->file.seekg (0, context->file.end);
            context->file_size  = context->file.tellg();
            context->file.seekg (0, context->file.beg);

            // TODO Session creation
            context->session    = 123;             

            // save last received code for later responses
            context->reqCode    = ftp.payload[CODE];

            // send message: ACK( session, size=4, data=len(file) )
            context->sendOpenSession();
            
            // change state for writing data chunks
            context->setState(&context->fileUploadWrite);
        }
        else {
            spdlog::warn("FileUploadInit::handleMessage, file not open");
            context->sendNakFailure(context->transferSysId, context->transferCompId, FNF);
            
            // initialize state machine (closes session, closes file)
            context->setState(&context->fileUploadInit);
        }
    }
}

//-------------------------------------------------------------
// Class FileUploadWrite 
//-------------------------------------------------------------

void
MavlinkFtpManager::FileUploadService::FileUploadWrite::handleMessage(const mavlink_message_t *msg) { 
    mavlink_file_transfer_protocol_t ftp;
    mavlink_msg_file_transfer_protocol_decode(msg, &ftp);

    // resend message: ACK( session, size=4, data=len(file) ) 
    // if no message has transfered yet
    if (ftp.payload[CODE] == OpenFileRO) {
        if (context->seq > 0) {
            spdlog::debug("FileUploadWrite::FileUploadWrite, OpenFileRO already received but no data transfered yet -> send open session");
            context->sendOpenSession();
        }
        else {
            spdlog::warn("FileUploadWrite::handleMessage, file upload already started: OpenFileRO and seq > 0");
            context->sendNakFailure(context->transferSysId, context->transferCompId, FAIL);
            // initialize state machine (closes session, closes file)
            context->setState(&context->fileUploadInit);
        }
    }

    // Message received: ReadFile(session, size, offset)
    if (ftp.payload[CODE] == ReadFile) {

        if (context->file.is_open()) {
            char buffer[DATA_SIZE];

            // save last received code for later responses
            context->reqCode = ftp.payload[CODE];
            context->seq = *(uint16_t*)&ftp.payload[SEQ];
                  
            // find file offset, read and write data
            uint32_t offset = *(uint32_t*)&ftp.payload[OFFSET];

            spdlog::debug("FileUploadWrite::handleMessages, file open, send data offset: " + std::to_string(offset));

            context->file.seekg(offset); 
            if (context->file) {
                context->file.read(buffer, ftp.payload[SIZE]);
                int size = context->file.gcount();
                context->sendData(buffer, size);
            }
            else {
                // After last message
                // Send message: NAK(session, size=1, data=EOF)
                context->setState(&context->fileUploadEnd);
            }
        }
        else {
            spdlog::warn("FileUploadWrite::handleMessages, file not anymore open");
            context->sendNakFailure(context->transferSysId, context->transferCompId, FNF);
        }
    }
}

void
MavlinkFtpManager::FileUploadService::FileUploadWrite::run() { 

    if ((microsSinceEpoch() - context->mavlink->getSendTime()) > context->sendResponseTimeout) {
        if (context->repeatCounter < MAV_MAX_RETRIES) {

            // resend message: ACK( session, size=4, data=len(file) ) 
            // if no message has transfered yet
            if (context->seq <= 0) {
               // context->sendOpenSession();
            }
            else {
                // TODO resend Message:  ACK( session, size=4, data=len(file) )
            }  
            ++context->repeatCounter;
        }
        else {
            spdlog::warn("FileUploadWrite::run, max retries reached");
            context->sendNakFailure(context->transferSysId, context->transferCompId, FAIL);
            // initialize state machine (closes session, closes file)
            context->setState(&context->fileUploadInit);
        }
    }
}

//-------------------------------------------------------------
// Class FileUploadEnd 
//-------------------------------------------------------------
void
MavlinkFtpManager::FileUploadService::FileUploadEnd::entry() { 
    // Message send: NAK(session, size=1, data=EOF)
    context->sendEofData();
}

void
MavlinkFtpManager::FileUploadService::FileUploadEnd::handleMessage(const mavlink_message_t *msg) { 
    mavlink_file_transfer_protocol_t ftp;
    mavlink_msg_file_transfer_protocol_decode(msg, &ftp);

    // Wait for message TerminateSession(session)
    if (ftp.payload[CODE] == TERM) {
        spdlog::info("FileUploadEnd::handleMessages, file upload success");

        context->sendAckData();

        // intialize state machine, close session
        context->setState(&context->fileUploadInit);
    }
}

void
MavlinkFtpManager::FileUploadService::FileUploadEnd::run() { 

    if ((microsSinceEpoch() - context->mavlink->getSendTime()) > context->sendResponseTimeout) {
        if (context->repeatCounter < MAV_MAX_RETRIES) {
            // TODO resend Message: NAK(session, size=1, data=EOF)
            ++context->repeatCounter;
        }
        else {
            spdlog::warn("FileUploadEnd::run, max retries reached");
            context->sendNakFailure(context->transferSysId, context->transferCompId, FAIL);
            // initialize state machine (closes session, closes file)
            context->setState(&context->fileUploadInit);
        }
    }
}

//-------------------------------------------------------------
// Class ListDirectoryService 
//-------------------------------------------------------------

void
MavlinkFtpManager::ListDirectoryService::sendData(const char *buffer, const size_t size) {

    // Build message payload
    mavlink_file_transfer_protocol_t ftp;

    ftp.target_system       = transferSysId;
    ftp.target_component    = transferCompId;
    ftp.payload[CODE]       = ACK; 
    ftp.payload[SIZE]       = size;
    
    memcpy(&ftp.payload[DATA], buffer, size);

    // send message: ACK(size, data=entries_at_offset_...)
    mavlink_msg_file_transfer_protocol_send_struct(mavlink->getChannel(), &ftp);
}

/**
 * Message send: NAK(session, size=1, data=EOF)
 */
void
MavlinkFtpManager::ListDirectoryService::sendEofData() {

    // Build message payload
    mavlink_file_transfer_protocol_t ftp;

    ftp.target_system       = transferSysId;
    ftp.target_component    = transferCompId;
    ftp.payload[CODE]       = NAK; 
    ftp.payload[SIZE]       = 1;
    ftp.payload[DATA]       = NEOF;
    
    // send message: NACK(size=1, data[0]=EOF)
    mavlink_msg_file_transfer_protocol_send_struct(mavlink->getChannel(), &ftp);
}

//-------------------------------------------------------------
// Class ListDirectoryWrite 
//-------------------------------------------------------------
void
MavlinkFtpManager::ListDirectoryService::ListDirectoryWrite::entry() { 
    context->offset = 0;
    context->entries = "";
}

void
MavlinkFtpManager::ListDirectoryService::ListDirectoryWrite::handleMessage(const mavlink_message_t *msg) { 
    mavlink_file_transfer_protocol_t ftp;
    mavlink_msg_file_transfer_protocol_decode(msg, &ftp);

    // Message received: ListDirectory( data[0]=path, size=len(path), offset=0 )
    if (ftp.payload[CODE] == ListDirectory) {

        int offset = 0;
        int count = 0;
        string path(FTP_PATH);
        string s;
        string entries;
        string entryPath;
        
        for (const auto & entry : fs::directory_iterator(path)) {
            ++count;
            if (count <= context->offset) continue;

            entryPath = entry.path();
            eraseSubStr(entryPath, path+"/");

            if (entry.is_directory()) {
                s = "D"+entryPath+"\\0";
            }
            if (entry.is_regular_file()) {
                s = "F"+entryPath+"\\t"+ to_string(entry.file_size()) + "\\0";
            }

            if (s.size() + entries.size() < DATA_SIZE) {
                entries.append(s);
                ++offset;
            }
            else {
                break;
            }
        }
        
        context->offset += offset;

        if (offset > 0) {
            spdlog::debug(entries);
            context->sendData(entries.c_str(), entries.size());
        }
        else {
            spdlog::debug("ListDirectoryWrite, Eof");
            context->sendEofData();
            context->setState(&context->listDirectoryInit);
        }
    }
}

void
MavlinkFtpManager::ListDirectoryService::ListDirectoryWrite::run() { 

    if ((microsSinceEpoch() - context->mavlink->getSendTime()) > context->sendResponseTimeout) {
        if (context->repeatCounter < MAV_MAX_RETRIES) {
            ++context->repeatCounter;
        }
        else {
            spdlog::warn("FileUploadWrite::run, max retries reached");
            //context->sendNakFailure(context->transferSysId, context->transferCompId, FAIL);
            // initialize state machine (closes session, closes file)
            context->setState(&context->listDirectoryInit);
        }
    }
}
