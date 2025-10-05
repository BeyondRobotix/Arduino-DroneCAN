#ifdef CANH7
#ifndef CAN_DRIVER_H7_
#define CAN_DRIVER_H7_

#include "stm32h7xx.h"
#include "canard.h" // Use canard's native frame definition

/**
 * @brief Configures the GPIO pins for the CAN peripheral.
 */
void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed = 3);

/**
 * @brief Checks if a message is available in the receive FIFO.
 * @return Number of messages available.
 */
uint8_t CANMsgAvail(void);

/**
 * @brief Sends a CAN frame.
 * The frame format (Classic vs FD) is determined by the CanardCANFrame struct.
 */
void CANSend(const CanardCANFrame *tx_frame);

/**
 * @brief Receives a CAN frame.
 * Populates the CanardCANFrame struct with the received data.
 */
void CANReceive(CanardCANFrame *rx_frame);

/**
 * @brief Configures a single standard ID filter.
 */
void CANSetFilter(uint8_t index, uint8_t fifo, uint32_t id, uint32_t mask);

enum CAN_BITRATE
{
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS,
    // CAN FD bitrates
    CAN_FD_500KBPS_2000KBPS,
    CAN_FD_1000KBPS_2000KBPS,
    CAN_FD_1000KBPS_4000KBPS,
    CAN_FD_1000KBPS_8000KBPS,
};

#if CANARD_ENABLE_CANFD
/**
 * @brief Initializes the FDCAN peripheral with support for CAN FD using a bitrate enum.
 * @param bitrate The bitrate for the CAN bus (nominal and data).
 * @param remap   GPIO remapping index.
 */
bool CANInit(CAN_BITRATE bitrate, int remap);

/**
 * @brief Initializes the FDCAN peripheral with support for CAN FD.
 * @param nominal_bitrate The bitrate for the arbitration phase (e.g., 1000000 for 1Mbps).
 * @param data_bitrate    The bitrate for the data phase (e.g., 8000000 for 8Mbps).
 * @param remap           GPIO remapping index.
 */
bool CANInit(uint32_t nominal_bitrate, uint32_t data_bitrate, int remap);
#else
/**
 * @brief Initializes the FDCAN peripheral for Classic CAN operation only using a bitrate enum.
 * @param bitrate The bitrate for the CAN bus.
 * @param remap   GPIO remapping index.
 */
bool CANInit(CAN_BITRATE bitrate, int remap);

/**
 * @brief Initializes the FDCAN peripheral for Classic CAN operation only.
 * @param bitrate The bitrate for the CAN bus (e.g., 1000000 for 1Mbps).
 * @param remap   GPIO remapping index.
 */
bool CANInit(uint32_t bitrate, int remap);
#endif

struct Timings
{
    uint16_t sample_point_permill;
    uint16_t prescaler;
    uint8_t sjw;
    uint8_t bs1;
    uint8_t bs2;

    Timings()
        : sample_point_permill(0), prescaler(0), sjw(0), bs1(0), bs2(0)
    {
    }
};

#endif // CAN_DRIVER_H7_
#endif // CANH7