#include <stdio.h>
//#include <string.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <future> 

#include "mav_ftp.h"
#include "mav_mavlink.h"
#include "helper.h"

using namespace std;

//-------------------------------------------------------------
// Class MavlinkFtpManager
//-------------------------------------------------------------
void
MavlinkFtpManager::run() {
    fileUploadService.getState()->run();
}

void
MavlinkFtpManager::handle_message(const mavlink_message_t *msg)
{
    if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
        mavlink_file_transfer_protocol_t ftp;
        mavlink_msg_file_transfer_protocol_decode(msg, &ftp);

        // Initialize state machine iff no session exists (only one sessio possible)
        if (fileUploadService.session <= 0) {
            spdlog::info("MavlinkFtpManager::handle_message,  start file upload");
            fileUploadService.setState(&fileUploadService.fileUploadInit);
        }
        // send error on file open request
        else if (ftp.payload[CODE] == OpenFileRO) {
            spdlog::warn("MavlinkFtpManager::handle_message, no session available");
            fileUploadService.sendNakFailure(msg->sysid, msg->compid, NOSESS);
        }

        fileUploadService.getState()->handleMessage(msg);
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
    
    // send message: ACK( session, size=4, data=len(file) )
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
        spdlog::debug("FileUploadInit::handle_message, opening file");

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
            spdlog::debug("FileUploadInit::handle_message, file found and open");
            
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

            spdlog::debug("FileUploadWrite::handle_message, file open, send data offset: " + std::to_string(offset));

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
            spdlog::warn("FileUploadWrite::handle_message, file not anymore open");
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
        spdlog::info("FileUploadEnd::handle_message, file upload success");

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
