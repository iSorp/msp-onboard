/**
    @file mav_service.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/    

#pragma once

#include <typeinfo>

#include "spdlog/spdlog.h"
#include "defines.h"
#include "mavlink_types.h"

struct Mavlink;
struct MavlinkService;

struct MavlinkServiceManager {
    public:
        MavlinkServiceManager(Mavlink *mavlink) : 
            mavlink(mavlink) {}

        ~MavlinkServiceManager() {};

        public:
            virtual void run() {};
            virtual void handleMessages(const mavlink_message_t *msg) {};

    protected:
        Mavlink *mavlink;
};

struct ServiceStateInterface {
    public: 
        virtual void handleMessage(const mavlink_message_t *msg){};
        virtual void entry(){};
        virtual void exit(){};
        virtual void run(){};
};

template <class T = MavlinkService> class ServiceState : public ServiceStateInterface {
    public:
        ServiceState(T *context) : context(context) {};
        ServiceState() {}

        T *context;
};

struct MavlinkService {
    public:
        MavlinkService(Mavlink *mavlink) : mavlink(mavlink) { }

        // Functions
        ServiceStateInterface* getState() {return state; };

        void setState(ServiceStateInterface *_state) {
            spdlog::debug("MavlinkService::setState(" + std::string(typeid(*_state).name()) + ")");
            _state->exit();
            state = _state;
            state->entry();
        };

    protected:
        Mavlink *mavlink;
        const uint64_t sendResponseTimeout = MAV_TIMEOUT;
        int retries = 0;
        int transferSysId = 0;
        int transferCompId = 0;
        int repeatCounter = 0;

        ServiceStateInterface *state;
};
