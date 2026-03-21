#ifdef CANH7
#include "Arduino.h"
#include "canH743.h"

// --- Message RAM Configuration ---
// These values MUST be defined as macros before including the ACANFD_STM32.h header.
// The library's internal headers use these macros to configure the FDCAN peripherals.
// H743 has 2560 words of Message RAM shared between FDCAN1 and FDCAN2.
// FDCAN2's start offset = FDCAN1_MESSAGE_RAM_WORD_SIZE, so give FDCAN1 zero
// to let FDCAN2 use the entire 2560-word region.
#define FDCAN1_MESSAGE_RAM_WORD_SIZE 0
#define FDCAN2_MESSAGE_RAM_WORD_SIZE 2560

// The ACANFD_STM32 library requires this main header to be included in one .cpp file
// to instantiate the CAN objects (fdcan1, fdcan2).
#include <ACANFD_STM32.h>

// A static pointer to the active CAN driver instance (FDCAN1 or FDCAN2).
// This allows the C-style API functions to interact with the C++ CAN object.
static ACANFD_STM32 *gCANDriver = nullptr;
static uint32_t gBeginFDStatus = 0xFFFF; // stored for debug

// This mask is used to extract the 29-bit extended ID from a canard frame ID.
#define CAN_EXT_ID_MASK 0x1FFFFFFFU


/**
 * @brief Initializes the FDCAN controller.
 *
 * @param bitrate The desired bitrate from the BITRATE enum.
 * @param can_iface_index Selects the CAN interface. 0 for FDCAN1, 1 for FDCAN2 on the NUCLEO-H743ZI2.
 * @return true on success, false on failure.
 */
bool CANInit(BITRATE bitrate, int can_iface_index) {

    // Select the FDCAN peripheral instance based on the provided index.
    if (can_iface_index == 1) {
        gCANDriver = &fdcan1;
    } else if (can_iface_index == 2) {
        gCANDriver = &fdcan2;
    } else {
        // If an invalid index is provided, fail initialization.
        return false;
    }

    // Determine the nominal bitrate from the enum.
    uint32_t desiredBitrate = 1000 * 1000; // Default to 1Mbit/s
    switch (bitrate) {
        case CAN_50KBPS:   desiredBitrate = 50 * 1000; break;
        case CAN_100KBPS:  desiredBitrate = 100 * 1000; break;
        case CAN_125KBPS:  desiredBitrate = 125 * 1000; break;
        case CAN_250KBPS:  desiredBitrate = 250 * 1000; break;
        case CAN_500KBPS:  desiredBitrate = 500 * 1000; break;
        case CAN_1000KBPS: desiredBitrate = 1000 * 1000; break;
    }

    ACANFD_STM32_Settings settings(desiredBitrate, DataBitRateFactor::x1);

    settings.mTxPin = PB_6;
    settings.mRxPin = PB_5;

    const uint32_t status = gCANDriver->beginFD(settings);
    gBeginFDStatus = status;

    return (status == 0);
}

/**
 * @brief Sends a CAN message using the initialized FDCAN peripheral.
 *
 * This function mimics the behavior of the canL431 driver, forcing all
 * outgoing messages to use the Extended ID format for Ardupilot compatibility.
 *
 * @param tx_msg A pointer to the CanardCANFrame to be sent.
 */
void CANSend(const CanardCANFrame *tx_msg) {
    if (!gCANDriver) {
        return; // Do nothing if the driver is not initialized.
    }

    CANFDMessage message;
    
    // Force Extended ID format, matching the canL431 driver's behavior.
    message.ext = true;
    message.id = tx_msg->id & CAN_EXT_ID_MASK; // Mask to get the 29-bit ID.

    // Copy the data payload and length.
    message.len = tx_msg->data_len;
    for (uint8_t i = 0; i < message.len; i++) {
        message.data[i] = tx_msg->data[i];
    }
    
    // Specify a classic CAN data frame.
    message.type = CANFDMessage::CAN_DATA;

    // Use the non-blocking send method from the library.
    gCANDriver->tryToSendReturnStatusFD(message);
}

/**
 * @brief Receives a CAN message if one is available.
 *
 * This function checks the RX FIFO, and if a message is present, it populates
 * the provided CanardCANFrame struct. It only processes extended frames to maintain
 * compatibility with the supplied L431 driver's logic.
 *
 * @param rx_msg A pointer to a CanardCANFrame that will be filled with the received data.
 */
void CANReceive(CanardCANFrame *rx_msg) {
    if (!gCANDriver || !gCANDriver->availableFD0()) {
        return; // Do nothing if driver is not initialized or FIFO is empty.
    }

    CANFDMessage message;
    if (gCANDriver->receiveFD0(message)) {
        // Only process extended frames, as implied by the L431 driver logic.
        if (message.ext) {
            // Populate the CanardCANFrame for the application.
            // Set the EFF flag (bit 31) for canard/Ardupilot compatibility.
            rx_msg->id = message.id | CANARD_CAN_FRAME_EFF;
            rx_msg->data_len = message.len;
            
            for (int i = 0; i < rx_msg->data_len; i++) {
                rx_msg->data[i] = message.data[i];
            }

            // Set the canfd flag if CAN-FD is enabled in canard build configuration.
            #if CANARD_ENABLE_CANFD
                rx_msg->canfd = (message.type == CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH || message.type == CANFDMessage::CANFD_NO_BIT_RATE_SWITCH);
            #endif
        }
        // Standard frames are ignored.
    }
}

/**
 * @brief Checks for available CAN messages.
 *
 * @return The number of messages pending in the driver's software receive FIFO 0.
 */
uint8_t CANMsgAvail(void) {
    if (!gCANDriver) {
        return 0;
    }
    // Return the fill level of the driver's receive FIFO.
    return gCANDriver->driverReceiveFIFO0Count();
}

#endif // CANH7

