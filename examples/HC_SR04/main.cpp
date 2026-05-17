#include <Arduino.h>
#include <dronecan.h>
#include <HCSR04.h>

std::vector<DroneCAN::parameter> custom_parameters = {
    { "NODEID", DroneCAN::INT, 100, 0, 127 },
};

DroneCAN dronecan;

uint32_t looptime = 0;

const byte triggerPin = PA8;
const byte echoPin = PA9;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);


void setup()
{
    // the following block of code should always run first. Adjust it at your own peril!
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    dronecan.init(
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
