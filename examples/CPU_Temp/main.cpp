
/*
API version v1.3
*/

#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>

// set up your parameters here with default values. NODEID should be kept
std::vector<DroneCAN::parameter> custom_parameters = {
    { "NODEID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 100,  0, 127 },
    { "PARM_1", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
};

DroneCAN dronecan;

uint32_t looptime = 0;

/*
This function is called when we receive a CAN message, and it's accepted by the shouldAcceptTransfer function.
We need to do boiler plate code in here to handle parameter updates and so on, but you can also write code to interact with sent messages here.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    DroneCANonTransferReceived(dronecan, ins, transfer);
}

/*
For this function, we need to make sure any messages we want to receive follow the following format with
UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID as an example
 */
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
    
    // an example of getting and setting parameters within the code
    dronecan.setParameter("PARM_1", 50.0f);
    Serial.print("PARM_1 value: ");
    Serial.println(dronecan.getParameter("PARM_1"));

    while (true)
    {
        const uint32_t now = millis();

        // send our battery message at 10Hz
        // Don't use delay() since we need to call dronecan.cycle() as much as possible
        if (now - looptime > 100)
        {
            looptime = millis();

            // collect MCU core temperature data
            int32_t vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
            int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

            // construct dronecan packet
            uavcan_equipment_power_BatteryInfo pkt{};
            pkt.voltage = analogRead(PA1);
            pkt.current = analogRead(PA0);
            pkt.temperature = cpu_temp;

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
