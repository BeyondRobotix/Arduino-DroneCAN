/*
 * Arduino DroneCAN API v1.3
 * This is an example app using the INA libary from https://github.com/RobTillaart/INA239
 * You'll need to download the library
 * this code is untested, and is just for illustration on how you could use the library. Hopefully I can test it with the INA soon : )
 */

#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include "INA239.h"
#include <simple_dronecanmessages.h>

INA239 INA(5, &SPI);

std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 127, 0, 127}};

DroneCAN dronecan;

uint32_t looptime = 0;

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    DroneCANonTransferReceived(dronecan, ins, transfer);
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)

{
    return false || DroneCANshoudlAcceptTransfer(ins, out_data_type_signature, data_type_id, transfer_type, source_node_id);
}

void setup()
{
    // the following block of code should always run first. Adjust it at your own peril!
    app_setup();
    IWatchdog.begin(2000000); 
    Serial.begin(115200);
    dronecan.version_major = 1;
    dronecan.version_minor = 0;
    dronecan.init(
        onTransferReceived, 
        shouldAcceptTransfer, 
        custom_parameters,
        "Beyond Robotix Node"
    );
    // end of important starting code

    SPI.begin();

    if (!INA.begin())
    {
        Serial.println("Could not connect. Fix and Reboot");
        while (1)
            ;
    }

    // set your shunt resistance here
    INA.setMaxCurrentShunt(10, 0.015);

    while (true)
    {
        const uint32_t now = millis();

        // send our battery message at 10Hz
        if (now - looptime > 100)
        {
            looptime = millis();

            // construct dronecan packet
            uavcan_equipment_power_BatteryInfo pkt{};
            pkt.voltage = INA.getBusVoltage();
            pkt.current = INA.getMilliAmpere() / 1000;
            pkt.temperature = INA.getTemperature();

            sendUavcanMsg(dronecan.canard, pkt);
        }

        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}
