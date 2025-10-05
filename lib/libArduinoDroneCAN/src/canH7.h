#ifdef CANH7
#ifndef CAN_DRIVER_H743
#define CAN_DRIVER_H743
#include <canard.h>

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

/* Symbolic names for formats of CAN message */
enum
{
    STANDARD_FORMAT = 0,
    EXTENDED_FORMAT
} CAN_FORMAT;

/* Symbolic names for type of CAN message */
enum
{
    DATA_FRAME = 0,
    REMOTE_FRAME
} CAN_FRAME;

extern CAN_bit_timing_config_t can_configs[6];

uint8_t CANMsgAvail(void);
void CANSend(const CanardCANFrame *CAN_tx_msg);
void CANReceive(CanardCANFrame *CAN_rx_msg);
void CANSetFilters(uint16_t *ids, uint8_t num);
void CANSetFilter(uint16_t id);
bool CANInit(BITRATE bitrate, int remap);

#endif // CAN_DRIVER_H743
#endif // CANH743