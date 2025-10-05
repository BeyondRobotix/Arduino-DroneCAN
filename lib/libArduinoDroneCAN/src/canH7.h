#ifndef CANH7_COMPATIBLE_H
#define CANH7_COMPATIBLE_H

// Assume standard H7 HAL includes and canard.h are available in your build system
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include <canard.h> // Required for CanardCANFrame
#include "Arduino.h"

// --- Types from canL431.h ---
enum BITRATE
{
    CAN_50KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};

struct CAN_bit_timing_config_t
{
    uint8_t TS2;
    uint8_t TS1;
    uint8_t BRP;
};

enum CAN_FORMAT
{
    STANDARD_FORMAT = 0,
    EXTENDED_FORMAT
};

enum CAN_TYPE
{
    DATA_FRAME = 0,
    REMOTE_FRAME
};

// --- Global HAL Handle (For use by interrupt handlers) ---
extern FDCAN_HandleTypeDef hfdcan1;

// --- Function Prototypes (Matching existing API) ---

/**
 * @brief Initializes the FDCAN peripheral in Classic CAN mode (8-byte payload).
 * @param bitrate The desired communication bitrate.
 * @param remap Placeholder for GPIO remap (handled in HAL_FDCAN_MspInit).
 * @retval true on success, false on failure.
 */
bool CANInit(enum BITRATE bitrate, int remap);

/**
 * @brief Sends a CAN message.
 * @param CAN_tx_msg Pointer to the CanardCANFrame structure containing the message.
 */
void CANSend(const CanardCANFrame *CAN_tx_msg);

/**
 * @brief Retrieves a received CAN message from Rx FIFO 0.
 * @param CAN_rx_msg Pointer to the CanardCANFrame structure to hold the received message.
 */
void CANReceive(CanardCANFrame *CAN_rx_msg);

/**
 * @brief Checks if a message is available in Rx FIFO 0.
 * @retval Number of available messages (0 to 8).
 */
uint8_t CANMsgAvail(void);

/**
 * @brief Configures a single filter to accept a standard or extended ID.
 * @param id The ID to filter on.
 */
void CANSetFilter(uint16_t id);

/**
 * @brief Configures multiple filters (deprecated, only uses the first ID).
 * @param ids Array of IDs to filter on.
 * @param num Number of IDs in the array.
 */
void CANSetFilters(uint16_t *ids, uint8_t num);


#endif // CANH7_COMPATIBLE_H