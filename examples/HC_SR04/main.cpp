
/*
API version v1.3
*/

#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <HCSR04.h>

std::vector<DroneCAN::parameter> custom_parameters = {
    { "NODEID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 100,  0, 127 },
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

const byte triggerPin = PA8;
const byte echoPin = PA9;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);


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


    while (true)
    {
        const uint32_t now = millis();

        // send our battery message at 10Hz
        if (now - looptime > 100)
        {
            looptime = millis();

            float distance = distanceSensor.measureDistanceCm();

            Serial.println(distance);

            uavcan_equipment_range_sensor_Measurement pkt{};

            pkt.range = distance;
            
            uavcan_CoarseOrientation orientation{};
            orientation.fixed_axis_roll_pitch_yaw[0] = 0;
            orientation.fixed_axis_roll_pitch_yaw[1] = 0;
            orientation.fixed_axis_roll_pitch_yaw[2] = 0;
            orientation.orientation_defined = true;
            pkt.beam_orientation_in_body_frame = orientation;

            pkt.sensor_id = 0;

            pkt.timestamp.usec = micros();
            
            pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;

            uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE];
            uint32_t len = uavcan_equipment_range_sensor_Measurement_encode(&pkt, buffer);
            static uint8_t transfer_id;
            canardBroadcast(&dronecan.canard,
                            UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                            UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                            &transfer_id,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            buffer,
                            len);
        }
        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}
