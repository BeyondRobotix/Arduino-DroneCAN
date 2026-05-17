/*
Demonstrates receiving DroneCAN messages using the full callback API.

Listens for MagneticFieldStrength broadcast messages and prints the X, Y, Z
field values (in Gauss) to Serial at whatever rate the sender transmits them.

Use this as a template when you need to receive any DroneCAN message type —
copy the pattern in onTransferReceived and shouldAcceptTransfer, substituting
the message ID, signature, struct, and decode call for your target message.
*/

#include <Arduino.h>
#include <dronecan.h>

std::vector<DroneCAN::parameter> custom_parameters = {
    { "NODEID", DroneCAN::INT, 100, 0, 127 },
};

DroneCAN dronecan;

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    switch (transfer->data_type_id)
    {
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
    {
        uavcan_equipment_ahrs_MagneticFieldStrength pkt{};
        uavcan_equipment_ahrs_MagneticFieldStrength_decode(transfer, &pkt);

        Serial.print("Mag X: "); Serial.print(pkt.magnetic_field_ga[0]);
        Serial.print("  Y: ");   Serial.print(pkt.magnetic_field_ga[1]);
        Serial.print("  Z: ");   Serial.println(pkt.magnetic_field_ga[2]);
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
        case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
            *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE;
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
        "Beyond Robotix Listener"
    );
    // end of important starting code

    // we use a while true loop instead of the arduino "loop" function since that causes issues.
    while (true)
    {
        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}
