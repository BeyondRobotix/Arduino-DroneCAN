#ifdef CANH7
#include "Arduino.h"
#include "canH7.h"

// Example bit timing configs for STM32H7 FDCAN (update as needed)
CAN_bit_timing_config_t can_configs[6] = {
    {2, 13, 60},   // 50kbps
    {2, 13, 30},   // 100kbps
    {2, 13, 24},   // 125kbps
    {2, 13, 12},   // 250kbps
    {2, 13, 6},    // 500kbps
    {2, 13, 3}     // 1000kbps
};

// Helper: Setup GPIO for FDCAN (update for your board/pins)
void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed = 3)
{
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = (index > 7) ? 1 : 0;
    if (index > 7) _index4 = (index - 8) * 4;

    uint32_t mask = 0xF << _index4;
    addr->AFR[ofs] &= ~mask;
    addr->AFR[ofs] |= (afry << _index4);

    mask = 0x3 << _index2;
    addr->MODER &= ~mask;
    addr->MODER |= (0x2 << _index2);

    addr->OSPEEDR &= ~mask;
    addr->OSPEEDR |= (speed << _index2);

    addr->OTYPER &= ~(0x1 << index);
    addr->PUPDR &= ~mask;
}

// Helper: Setup FDCAN filter (single standard filter example)
void CANSetFilter(uint16_t id)
{
    // Enter FDCAN init mode
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {}

    // Set filter: Accept only 'id', store in RX FIFO 0
    FDCAN1->SIDFC = (0x0 << 2) | (1 << 16); // Start at offset 0, 1 filter
    uint32_t *filter_ram = (uint32_t *)(SRAMCAN_BASE);
    filter_ram[0] = (0x2U << 30) | (0x1U << 27) | (id << 16) | 0x0; // Classic, FIFO0, id, mask=0

    // Leave init mode
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) {}
}

// Check for available CAN messages
uint8_t CANMsgAvail(void)
{
    return (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL) ? 1 : 0;
}

// Send CAN frame
void CANSend(const CanardCANFrame *CAN_tx_msg)
{
    uint32_t index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    uint32_t *buffer = (uint32_t *)(SRAMCAN_BASE + (index * 18 * 4)); // FDCAN_FRAME_BUFFER_SIZE = 18

    // Extended frame format
    buffer[0] = (FDCAN_TX_ELEMENT_IDE | (CAN_tx_msg->id & 0x1FFFFFFF));
    buffer[1] = (CAN_tx_msg->data_len << 16) | (index << 24);

    // Copy data
    for (uint8_t i = 0; i < 8; i++) {
        ((uint8_t *)&buffer[2])[i] = CAN_tx_msg->data[i];
    }

    // Request transmission
    FDCAN1->TXBAR = (1 << index);

    // Wait for mailbox empty (optional, can be interrupt-driven)
    while (FDCAN1->TXBTO & (1 << index)) {}
}

// Receive CAN frame
void CANReceive(CanardCANFrame *CAN_rx_msg)
{
    if (!CANMsgAvail()) return;

    uint32_t index = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> 8;
    uint32_t *frame_ptr = (uint32_t *)(SRAMCAN_BASE + (index * 18 * 4));

    // Extended frame format
    CAN_rx_msg->id = frame_ptr[0] & 0x1FFFFFFF;
    CAN_rx_msg->data_len = (frame_ptr[1] >> 16) & 0xF;
    for (uint8_t i = 0; i < 8; i++) {
        CAN_rx_msg->data[i] = ((uint8_t *)&frame_ptr[2])[i];
    }

    // Acknowledge FIFO entry
    FDCAN1->RXF0A = index;
}

// Set multiple filters (example: accept all IDs in array)
void CANSetFilters(uint16_t *ids, uint8_t num)
{
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {}

    FDCAN1->SIDFC = (0x0 << 2) | (num << 16);
    uint32_t *filter_ram = (uint32_t *)(SRAMCAN_BASE);
    for (uint8_t i = 0; i < num; i++) {
        filter_ram[i] = (0x2U << 30) | (0x1U << 27) | (ids[i] << 16) | 0x0;
    }

    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) {}
}

// CAN initialization
bool CANInit(BITRATE bitrate, int remap)
{
    // Enable FDCAN clock (update for STM32H7)
    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
    RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;

    // Setup GPIOs for remap (update pins as needed)
    if (remap == 0) {
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
        CANSetGpio(GPIOA, 11, 9); // RX PA11 AF9
        CANSetGpio(GPIOA, 12, 9); // TX PA12 AF9
    }
    // Add other remap cases as needed...

    // Enter FDCAN init mode
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {}

    // Set bit timing
    FDCAN1->NBTP = (((can_configs[bitrate].TS2 - 1) << FDCAN_NBTP_NTSEG2_Pos) |
                    ((can_configs[bitrate].TS1 - 1) << FDCAN_NBTP_NTSEG1_Pos) |
                    ((can_configs[bitrate].BRP - 1) << FDCAN_NBTP_NBRP_Pos) |
                    (0 << FDCAN_NBTP_NSJW_Pos)); // SJW=1

    // Setup default filter (accept all)
    CANSetFilter(0);

    // Leave init mode
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) {}

    // Check for normal mode
    return true;
}
#endif