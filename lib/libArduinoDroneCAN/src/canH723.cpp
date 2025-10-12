#ifdef ARDUINO_NUCLEO_H723ZG
#include "Arduino.h"
#include "canH723.h"

// --- Message RAM Configuration ---
// These values MUST be defined as macros before including the ACANFD_STM32.h header.
// The library's internal headers use these macros to configure the FDCAN peripherals.
#define FDCAN1_MESSAGE_RAM_WORD_SIZE 800
#define FDCAN2_MESSAGE_RAM_WORD_SIZE 800
#define FDCAN3_MESSAGE_RAM_WORD_SIZE 800

// The ACANFD_STM32 library requires this main header to be included in one .cpp file
// to instantiate the CAN objects (fdcan1, fdcan2, fdcan3).
#include <ACANFD_STM32.h>

// --- FDCAN peripheral handles ---
// An array to easily select the CAN interface at runtime.
static ACANFD_STM32* can_ifaces[] = {&fdcan1, &fdcan2, &fdcan3};
// Pointer to the currently active CAN interface.
static ACANFD_STM32* active_can_iface = nullptr;
// Maximum number of supported CAN interfaces on the H723ZG.
const int MAX_CAN_IFACES = sizeof(can_ifaces) / sizeof(can_ifaces[0]);


/**
 * @brief Initializes the FDCAN controller.
 *
 * @param bitrate The desired bitrate from the BITRATE enum.
 * @param can_iface_index Selects the CAN interface. 0 for FDCAN1, 1 for FDCAN2, 2 for FDCAN3.
 * @return true on success, false on failure.
 */
bool CANInit(BITRATE bitrate, int can_iface_index) {
    can_iface_index = 0;
    if (can_iface_index < 0 || can_iface_index >= MAX_CAN_IFACES) {
        return false;
    }

    active_can_iface = can_ifaces[can_iface_index];

    uint32_t nominal_bitrate = 0;
    switch (bitrate) {
        case CAN_50KBPS:
            nominal_bitrate = 50 * 1000;
            break;
        case CAN_100KBPS:
            nominal_bitrate = 100 * 1000;
            break;
        case CAN_125KBPS:
            nominal_bitrate = 125 * 1000;
            break;
        case CAN_250KBPS:
            nominal_bitrate = 250 * 1000;
            break;
        case CAN_500KBPS:
            nominal_bitrate = 500 * 1000;
            break;
        case CAN_1000KBPS:
        default:
            nominal_bitrate = 1000 * 1000;
            break;
    }

    // Configure settings for CAN FD with a 4x data rate multiplier.
    // This is a common configuration for ArduPilot UAVCAN nodes.
    ACANFD_STM32_Settings settings(nominal_bitrate, DataBitRateFactor::x4);
    settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;

    // Start the FDCAN peripheral. The return code is 0 on success.
    settings.mTxPin = PD_1;
    settings.mRxPin = PD_0;
    uint32_t error_code = active_can_iface->beginFD(settings);

    return error_code == 0;
}

/**
 * @brief Sends a CAN message using the initialized FDCAN peripheral.
 *
 * This function converts a CanardCANFrame to the library's CANFDMessage format.
 * It forces all outgoing messages to use the Extended ID format for Ardupilot compatibility.
 *
 * @param tx_msg A pointer to the CanardCANFrame to be sent.
 */
void CANSend(const CanardCANFrame *tx_msg) {
    if (!active_can_iface || !tx_msg) {
        return;
    }

    CANFDMessage message;
    // ArduPilot's CAN drivers exclusively use extended frames.
    message.ext = true;
    message.id = tx_msg->id & CANARD_CAN_EXT_ID_MASK;
    message.len = tx_msg->data_len;
    message.type = CANFDMessage::CAN_DATA;

    memcpy(message.data, tx_msg->data, tx_msg->data_len);

    // Attempt to send the message. This is non-blocking.
    uint32_t send_status = active_can_iface->tryToSendReturnStatusFD(message);
    if (send_status != 0) {
        Serial.println("Failed to send CAN message");
    }
}


/**
 * @brief Receives a CAN message if one is available.
 *
 * This function checks the RX FIFO 0, and if a message is present, it populates
 * the provided CanardCANFrame struct.
 *
 * @param rx_msg A pointer to a CanardCANFrame that will be filled with the received data.
 */
void CANReceive(CanardCANFrame *rx_msg) {
    if (!active_can_iface || !rx_msg) {
        return;
    }

    CANFDMessage message;
    // Check RX FIFO 0 for a new message.
    if (active_can_iface->receiveFD0(message)) {
        // Populate the Canard frame from the received library message.
        
        rx_msg->id = (0x1FFFFFFFU & message.id) & 0x1FFFFFFFU;
        rx_msg->id |= 1U << 31; // https://github.com/ArduPilot/ardupilot/blob/4d31a7320a1d2c38e2d742ae63c34f914febaa8f/libraries/AP_HAL_ChibiOS/CanIface.cpp#L570
   
        rx_msg->data_len = message.len;

        memcpy(rx_msg->data, message.data, message.len);
    } else {
        // No message received, set ID to 0 to indicate an invalid/empty frame.
        rx_msg->id = 0;
        rx_msg->data_len = 0;
    }
}

/**
 * @brief Checks for available CAN messages.
 *
 * @return The number of messages pending in the driver's software receive FIFO 0.
 */
uint8_t CANMsgAvail(void) {
    if (!active_can_iface) {
        return 0;
    }
    // Return the number of messages available in RX FIFO 0.
    return active_can_iface->availableFD0();
}

#endif // ARDUINO_NUCLEO_H723ZG

