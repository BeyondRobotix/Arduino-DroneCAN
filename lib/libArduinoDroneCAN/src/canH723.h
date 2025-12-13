#ifdef ARDUINO_NUCLEO_H723ZG
#ifndef CAN_DRIVER_H7_
#define CAN_DRIVER_H7_

#include <canard.h>
#include <ACANFD_STM32_from_cpp.h>

/**
 * @brief Defines standard CAN bitrates.
 * This enum provides a list of common bitrates for CAN communication.
 */
enum BITRATE
{
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};

/**
 * @brief Initializes the FDCAN controller with a specified bitrate and interface.
 * @param bitrate The desired communication speed from the BITRATE enum.
 * @param can_iface_index Selects the hardware CAN interface (0 for FDCAN1, 1 for FDCAN2, etc.).
 * @return Returns true if initialization is successful, false otherwise.
 */
bool CANInit(BITRATE bitrate, int can_iface_index);

/**
 * @brief Sends a CAN frame.
 * This function queues a CAN frame for transmission. It's non-blocking.
 * @param tx_msg A pointer to the CanardCANFrame structure containing the message to be sent.
 */
void CANSend(const CanardCANFrame *tx_msg);

/**
 * @brief Receives a CAN frame.
 * If a message is available in the FIFO, this function populates the provided struct with its data.
 * @param rx_msg A pointer to a CanardCANFrame structure to be filled with the received message data.
 */
void CANReceive(CanardCANFrame *rx_msg);

/**
 * @brief Checks for available CAN messages in the receive buffer.
 * @return The number of messages currently pending in the RX FIFO.
 */
uint8_t CANMsgAvail(void);

#endif // CAN_DRIVER_H7_
#endif // DARDUINO_NUCLEO_H723ZG
