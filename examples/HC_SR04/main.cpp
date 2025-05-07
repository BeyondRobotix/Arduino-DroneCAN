
#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <HCSR04.h>

std::vector<DroneCAN::parameter> custom_parameters = {
    { "NODEID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 127,  0, 127 },
    { "PARM_1", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
    { "PARM_2", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
    { "PARM_3", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
    { "PARM_4", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
    { "PARM_5", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
    { "PARM_6", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
    { "PARM_7", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
};

DroneCAN dronecan;

uint32_t looptime = 0;

/*
This function is called when we receive a CAN message, and it's accepted by the shouldAcceptTransfer function.
We need to do boiler plate code in here to handle parameter updates and so on, but you can also write code to interact with sent messages here.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{

    // switch on data type ID to pass to the right handler function
    // if (transfer->transfer_type == CanardTransferTypeRequest)
    // check if we want to handle a specific service request
    switch (transfer->data_type_id)
    {

    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
    {
        uavcan_equipment_ahrs_MagneticFieldStrength pkt{};
        uavcan_equipment_ahrs_MagneticFieldStrength_decode(transfer, &pkt);
        // Serial.print(pkt.magnetic_field_ga[0], 4);
        // Serial.print(" ");
        // Serial.print(pkt.magnetic_field_ga[1], 4);
        // Serial.print(" ");
        // Serial.print(pkt.magnetic_field_ga[2], 4);
        // Serial.print(" ");
        // Serial.println();
        break;
    }
    }

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
    if (transfer_type == CanardTransferTypeBroadcast)
    {
        // Check if we want to handle a specific broadcast packet
        switch (data_type_id)
        {
        case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE;
            return true;
        }
        }
    }

    return false || DroneCANshoudlAcceptTransfer(ins, out_data_type_signature, data_type_id, transfer_type, source_node_id);
}

const byte triggerPin = PA8;
const byte echoPin = PA9;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);


void setup()
{
    // to use debugging tools, remove app_setup and set FLASH start from 0x800A000 to 0x8000000 in ldscript.ld
    // this will over-write the bootloader. To use the bootloader again, reflash it and change above back.
    app_setup(); // needed for coming from a bootloader, needs to be first in setup
    Serial.begin(115200);
    dronecan.version_major = 1;
    dronecan.version_minor = 0;
    dronecan.init(
        onTransferReceived, 
        shouldAcceptTransfer, 
        custom_parameters,
        "Beyond Robotix Node"
    );
    IWatchdog.begin(2000000); // if the loop takes longer than 2 seconds, reset the system

    // an example of getting and setting parameters within the code
    dronecan.setParameter("PARM_1", 69);
    Serial.print("PARM_1 value: ");
    Serial.println(dronecan.getParameter("PARM_1"));

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
