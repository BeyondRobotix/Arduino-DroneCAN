#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>

#include "Adafruit_MCP9600.h"
#include "Wire.h"

// set up your parameters here with default values. NODEID should be kept
std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 100, 0, 127},
    {"DEVICE_ID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 0, 0, 127},
    {"BATT_EN", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 0, 0, 1},
};

DroneCAN dronecan;

uint32_t loop1time = 0;
uint32_t loop2time = 0;
uint32_t looptime1hz = 0;
int device_id = 0;
int batt_en = 0;

/*
MCP9600 specific setup
*/
#define I2C_ADDRESS (0x66)
Adafruit_MCP9600 mcp;

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
        "BR-Node-Temperature");
    // end of important starting code

    if (!mcp.begin(I2C_ADDRESS))
    {
        uint32_t deadloop = 0;
        while (1)
        {
            const uint32_t now = millis();
            if (now - deadloop > 1000)
            {
                deadloop = millis();
                dronecan.debug("MCP9600 not found", 0);
                Serial.println("Sensor not found. Check wiring!");
            }
            dronecan.cycle();
            IWatchdog.reload();
        }
    }

    // Set the thermocouple type
    mcp.setThermocoupleType(MCP9600_TYPE_K);

    // we use a while true loop instead of the arduino "loop" function since that causes issues.
    while (true)
    {
        const uint32_t now = millis();

        if (now - loop1time > 100)
        {
            loop1time = millis();

            if (batt_en)
            {

                uavcan_equipment_power_BatteryInfo pkt{};

                pkt.battery_id = device_id;
                pkt.current = 0;
                pkt.voltage = 0;
                pkt.temperature = mcp.readThermocouple();

                sendUavcanMsg(dronecan.canard, pkt);
            }
        }

        if (now - loop2time > 1000)
        {
            loop2time = millis();

            uavcan_equipment_device_Temperature pkt{};

            pkt.temperature = mcp.readThermocouple();
            pkt.device_id = device_id;

            sendUavcanMsg(dronecan.canard, pkt);
        }

        if (now - looptime1hz > 1000)
        {
            looptime1hz = millis();
            batt_en = dronecan.getParameter("BATT_EN");
            device_id = dronecan.getParameter("DEVICE_ID");
        }

        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}
