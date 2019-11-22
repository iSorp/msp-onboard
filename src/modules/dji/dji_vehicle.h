#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

#include "msp_vehicle.h"
#include "dji_mavlink.h"

class MspDjiVehicle : public MspVehicle {

    public:
        MspDjiVehicle() : MspVehicle() {};
        void initialize(Vehicle* vehicleDJI, LinuxSetup* linuxEnvironment, Mavlink* mavlinkDJI);
       // EResult cmdCallback(EVehicleCmd cmd, void* data, size_t len) override;

    private:
        MavlinkDJI* mavlink;
        Vehicle* vehicle;
};