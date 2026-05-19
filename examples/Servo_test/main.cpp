/*
Listens for DroneCAN ArrayCommand messages and drives a servo on PA8 to the
commanded position. The actuator ID this node responds to is set by the
ACTUATOR_ID parameter (default 0).

command_value is expected in the range -1.0 to 1.0, which maps to 0-180 degrees.
This matches the range sent by ArduPilot for servo outputs over DroneCAN.
*/

#include <Arduino.h>
#include <dronecan.h>
#include <Servo.h>

Servo myservo;

std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID", DroneCAN::INT, 100, 0, 127},
    {"ACTUATOR_ID", DroneCAN::INT, 0, 0, 14},
};

DroneCAN dronecan;

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    switch (transfer->data_type_id)
    {
    case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
    {
        uavcan_equipment_actuator_ArrayCommand pkt{};
        uavcan_equipment_actuator_ArrayCommand_decode(transfer, &pkt);

        int actuator_id = (int)dronecan.getParameter("ACTUATOR_ID");

        for (uint8_t i = 0; i < pkt.commands.len; i++)
        {
            if (pkt.commands.data[i].actuator_id == actuator_id)
            {
                // map -1.0..1.0 to 0..180 degrees
                float angle = (pkt.commands.data[i].command_value + 1.0f) * 90.0f;
                myservo.write((int)constrain(angle, 0, 180));
                break;
            }
        }
        break;
    }
    }

    DroneCANonTransferReceived(dronecan, ins, transfer);
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeBroadcast)
    {
        switch (data_type_id)
        {
        case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID:
            *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
            return true;
        }
    }

    return false || DroneCANshouldAcceptTransfer(ins, out_data_type_signature, data_type_id, transfer_type, source_node_id);
}

void setup()
{
    // the following block of code should always run first. Adjust it at your own peril!
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    dronecan.init(
        onTransferReceived,
        shouldAcceptTransfer,
        custom_parameters,
        "Beyond Robotix Servo");
    // end of important starting code

    myservo.attach(PA_8);
}

void loop()
{
    dronecan.cycle();
    IWatchdog.reload();
}
