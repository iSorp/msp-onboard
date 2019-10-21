#include <stdlib.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>

#include "mavlink_bridge_header.h"
#include "mav_mavlink.h"
#include "mav_mission.h"

class MavlinkDJI : public Mavlink {
    public: 
        MavlinkDJI() { }
        ~MavlinkDJI();

        void beginSend() override { }
        void sendBytes(const uint8_t *buf, unsigned packet_len) override;
        int sendPacket() override;

    protected:
        void init() override;
        void readMessage() override;

    private:
};