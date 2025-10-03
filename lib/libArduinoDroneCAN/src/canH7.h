#ifdef CANH7
#ifndef CAN_DRIVER
#define CAN_DRIVER
#include <canard.h>
#include "stm32h7xx.h"

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
    uint16_t BRP;
};

/* Symbolic names for formats of CAN message                                 */
enum
{
    STANDARD_FORMAT = 0,
    EXTENDED_FORMAT
} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
enum
{
    DATA_FRAME = 0,
    REMOTE_FRAME
} CAN_FRAME;

void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed = 3);
uint8_t CANMsgAvail(void);
void CANSend(const CanardCANFrame *CAN_tx_msg);
void CANReceive(CanardCANFrame *CAN_rx_msg);
void CANSetFilter(uint8_t index, uint8_t fifo, uint32_t id, uint32_t mask);
bool CANInit(BITRATE bitrate, int remap);

#endif // CAN_DRIVER
#endif // CANH7