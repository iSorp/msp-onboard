/**
    @file mav_ftp_protocol.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        

#pragma once

#include <sys/types.h>

const static int MESSAGE_SIZE    = 251;
const static int INFO_SIZE       = 12;
const static int DATA_SIZE       = 239; // 251-12

// FTP Message header indexes
const static uint8_t SEQ         = 0x00; // 0->1
const static uint8_t SESS        = 0x02;
const static uint8_t CODE        = 0x03;
const static uint8_t SIZE        = 0x04;
const static uint8_t REQCODE     = 0x05;
const static uint8_t BURST       = 0x06;
const static uint8_t PAD         = 0x07;
const static uint8_t OFFSET      = 0x08;  // 8->11
const static uint8_t DATA        = 0x0C;  // 12->251


// OpCodes/Command
const static uint8_t TERM               = 0x01;
const static uint8_t ListDirectory      = 0x03;
const static uint8_t OpenFileRO         = 0x04;
const static uint8_t ReadFile           = 0x05;
const static uint8_t RemoveDirectory    = 0x10;
const static uint8_t ACK                = 0x80;    // 128
const static uint8_t NAK                = 0x81;    // 129

// NAK Error Information payload data[0]
const static uint8_t FAIL       = 0x01;
const static uint8_t FAILERNO   = 0x02;
const static uint8_t INVDSIZE   = 0x03;
const static uint8_t INVSESS    = 0x04;
const static uint8_t NOSESS     = 0x05;
const static uint8_t NEOF       = 0x06;
const static uint8_t UNKNOWN    = 0x07;
const static uint8_t EXISTS     = 0x08;
const static uint8_t PROTECTED  = 0x09;
const static uint8_t FNF        = 0x0A;

