#ifdef CANH743
#include "Arduino.h"
#include "canH743.h"

// This implementation relies on the ACANFD_STM32 library.
// It includes the board-specific object and settings files you provided.
// Please ensure the ACANFD_STM32 library is available in your project's include path.
static const uint32_t FDCAN1_MESSAGE_RAM_WORD_SIZE = 2560;
static const uint32_t FDCAN2_MESSAGE_RAM_WORD_SIZE = 2560; // FDCAN2 not used

// A static pointer to the active CAN driver instance (FDCAN1 or FDCAN2).
// This allows the C-style API functions to interact with the C++ CAN object.
static ACANFD_STM32 *gCANDriver = nullptr;

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
    if (can_iface_index == 2) {
        gCANDriver = &fdcan1;
    } else if (can_iface_index == 1) {
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

    // Configure FDCAN settings. We use the constructor that takes the arbitration bitrate
    // and a data bitrate factor. Setting the factor to x1 disables CAN-FD's bitrate switching,
    // ensuring we operate in classic CAN mode, matching the L431 driver.
    ACANFD_STM32_Settings settings(desiredBitrate, DataBitRateFactor::x1);

    settings.mTxPin = PD_1;
    settings.mRxPin = PH_14;

    // The ACANFD library's beginFD() method configures and starts the peripheral.
    // It returns 0 on success.
    const uint32_t status = gCANDriver->beginFD(settings);
    
    if (status == 0) {
        return true; // Initialization successful
    } else {
        gCANDriver = nullptr; // Nullify pointer on failure
        return false; // Initialization failed
    }
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
    uint32_t ret = gCANDriver->tryToSendReturnStatusFD(message);
    if (ret != 0)
    {
        Serial.println(ret);
    }
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

