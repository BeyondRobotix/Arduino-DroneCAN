/*
Just shows PA8 commanding a servo signal! Doesn't respond to CAN servo commands.
*/


#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>
#include <Servo.h>


Servo myservo; 

// set up your parameters here with default values. NODEID should be kept
std::vector<DroneCAN::parameter> custom_parameters = {
    { "NODEID", DroneCAN::INT,  100,  0, 127 },
    { "PARM_1", DroneCAN::REAL, 0.0f, 0.0f, 100.0f },
    { "PARM_2", DroneCAN::REAL, 0.0f, 0.0f, 100.0f },
    { "PARM_3", DroneCAN::REAL, 0.0f, 0.0f, 100.0f },
    { "PARM_4", DroneCAN::REAL, 0.0f, 0.0f, 100.0f },
    { "PARM_5", DroneCAN::REAL, 0.0f, 0.0f, 100.0f },
    { "PARM_6", DroneCAN::REAL, 0.0f, 0.0f, 100.0f },
    { "PARM_7", DroneCAN::REAL, 0.0f, 0.0f, 100.0f },
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
    dronecan.setParameter("PARM_1", 50.0f); // set a parameter to 50.0
    Serial.print("PARM_1 value: ");
    Serial.println(dronecan.getParameter("PARM_1"));

    myservo.attach(PA_8);

    int i = 1000;

    // we use a while true loop instead of the arduino "loop" function since that causes issues.
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

            myservo.write(i++);
        }

        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}
