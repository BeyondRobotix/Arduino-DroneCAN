#ifdef CANH7
#include "Arduino.h"
#include "canH7.h"

// Example bit timing configs for STM32H7 FDCAN (update as needed)
CAN_bit_timing_config_t can_configs[6] = {
    {2, 13, 100},   // 50kbps
    {2, 13, 50},   // 100kbps
    {2, 13, 40},   // 125kbps
    {2, 13, 20},   // 250kbps
    {2, 13, 10},    // 500kbps
    {1, 8, 8}     // 1000kbps
};

// Helper: Setup GPIO for FDCAN (update for your board/pins)
void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed = 3)
{
    // Enable GPIO port clock
    if (addr == GPIOA) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    else if (addr == GPIOB) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    else if (addr == GPIOC) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
    else if (addr == GPIOD) RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    else if (addr == GPIOE) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    // Add other GPIO ports if needed

    // Configure GPIO pin
    uint32_t moder = addr->MODER;
    moder &= ~(GPIO_MODER_MODE0 << (index * 2));
    moder |= (2U << (index * 2)); // Alternate function
    addr->MODER = moder;

    uint32_t otyper = addr->OTYPER;
    otyper &= ~(GPIO_OTYPER_OT0 << index); // Push-pull
    addr->OTYPER = otyper;

    uint32_t ospeedr = addr->OSPEEDR;
    ospeedr &= ~(GPIO_OSPEEDR_OSPEED0 << (index * 2));
    ospeedr |= (speed << (index * 2)); // High speed
    addr->OSPEEDR = ospeedr;

    uint32_t pupdr = addr->PUPDR;
    pupdr &= ~(GPIO_PUPDR_PUPD0 << (index * 2)); // No pull-up, pull-down
    addr->PUPDR = pupdr;

    uint8_t afr_index = (index < 8) ? 0 : 1;
    uint32_t afr_shift = (index < 8) ? (index * 4) : ((index - 8) * 4);
    uint32_t afr = addr->AFR[afr_index];
    afr &= ~(0xFU << afr_shift);
    afr |= (afry << afr_shift);
    addr->AFR[afr_index] = afr;
}

/**
 * Initializes the CAN filter registers.
 *
 * The FDCAN peripheral has a message RAM area for filters.
 * This function is designed to be compatible with the bxCAN implementation in canL431.cpp
 *
 * @preconditions   - This register can be written only when the filter initialization mode is set (CCE=1, INIT=1) in the FDCAN_CCCR register.
 * @params: index   - Specified filter index (0-27 for standard filters).
 * @params: scale   - (Ignored for FDCAN) Kept for compatibility.
 * @params: mode    - (Ignored for FDCAN) Kept for compatibility. Assumes mask mode.
 * @params: fifo    - Select filter assigned FIFO (0 or 1).
 * @params: bank1   - Filter ID.
 * @params: bank2   - Filter Mask.
 *
 */
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2)
{
    if (index >= 28) { // FDCAN supports up to 28 standard filters
        return;
    }

    // Enter filter configuration mode
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {}
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    // Configure filter list start address and size
    // We assume filters start at the beginning of the message RAM
    // and we configure the size to be index + 1.
    // This might be better done once at init.
    FDCAN1->SIDFC = (0 << 2) | ((index + 1) << 16); // FLSSA=0, LSS=index+1

    uint32_t *filter_ram = (uint32_t *)(SRAMCAN_BASE);

    // Standard filter element format for "Classic" (ID/Mask) filter:
    // S0[31:30] SFEC: 01=Store in FIFO0, 10=Store in FIFO1
    // S0[29:27] SFT:  010=Classic filter
    // S0[26:16] SFID1: filter ID
    // S0[15:11] reserved
    // S0[10:0]  SFID2: filter mask

    uint32_t sfec = (fifo == 0) ? 0b01 : 0b10;
    uint32_t sft = 0b010; // Classic filter
    uint32_t sfid1 = bank1 & 0x7FF;
    uint32_t sfid2 = bank2 & 0x7FF;

    uint32_t filter_element = (sfec << 30) | (sft << 27) | (sfid1 << 16) | sfid2;
    
    filter_ram[index] = filter_element;

    // Leave filter configuration mode
    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
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
    // Wait for a free Tx buffer. This is a busy wait and could be improved with a timeout.
    while (FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) {}

    uint32_t index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    
    // The size of one buffer element is dependent on configuration, 18 words for 64-byte payload.
    uint32_t *buffer = (uint32_t *)(SRAMCAN_BASE + (index * 18 * 4));

    // T0: ID and flags. We send an extended frame, which is standard for DroneCAN.
    buffer[0] = (1U << 30) | (CAN_tx_msg->id & 0x1FFFFFFF); // Set XTD (IDE) bit for 29-bit extended ID

    // T1: DLC. For classic CAN, FDF (FD Format) and BRS (Bit Rate Switch) are 0.
    buffer[1] = (uint32_t)(CAN_tx_msg->data_len) << 16;

    // Copy data using word-writes for efficiency, similar to the canL431.cpp implementation.
    // Note: This assumes data is 8 bytes. libcanard for DroneCAN v0 uses up to 8 bytes.
    uint32_t *tx_data_ptr = &buffer[2];
    tx_data_ptr[0] = ((uint32_t)CAN_tx_msg->data[3] << 24) |
                     ((uint32_t)CAN_tx_msg->data[2] << 16) |
                     ((uint32_t)CAN_tx_msg->data[1] << 8) |
                     ((uint32_t)CAN_tx_msg->data[0]);
    tx_data_ptr[1] = ((uint32_t)CAN_tx_msg->data[7] << 24) |
                     ((uint32_t)CAN_tx_msg->data[6] << 16) |
                     ((uint32_t)CAN_tx_msg->data[5] << 8) |
                     ((uint32_t)CAN_tx_msg->data[4]);

    // Request transmission for the buffer at the calculated index.
    FDCAN1->TXBAR = (1 << index);

    // Wait for the transmission to complete. This is blocking.
    // A timeout is included to prevent an infinite loop.
    volatile int count = 0;
    while (!(FDCAN1->TXBTO & (1 << index)) && count++ < 1000000) {}

    // The TXBTO (Tx Buffer Transmission Occurred) flag could be cleared here if needed,
    // by writing a 1 to it: FDCAN1->TXBTO = (1 << index);
}

// Receive CAN frame
void CANReceive(CanardCANFrame *CAN_rx_msg)
{
    if (!CANMsgAvail()) {
        return;
    }

    // Get the read index from FIFO 0 Status register
    uint32_t index = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> 8;

    // Calculate the address of the Rx buffer in Message RAM
    // The size of one buffer element is dependent on configuration, 18 words for 64-byte payload.
    uint32_t *frame_ptr = (uint32_t *)(SRAMCAN_BASE + (index * 18 * 4));

    // R0 contains ID and flags
    uint32_t r0 = frame_ptr[0];
    // R1 contains DLC and other info
    uint32_t r1 = frame_ptr[1];

    // For DroneCAN, we expect extended frames.
    // The XTD bit is bit 30 in the Rx Buffer Element R0.
    if (r0 & (1U << 30)) { // Check XTD bit
        CAN_rx_msg->id = r0 & 0x1FFFFFFF; // Mask for 29-bit extended ID
        // For compatibility with some HALs (e.g. ArduPilot), an extra flag is added.
        // This is seen in canL431.cpp and the reference CanIface.cpp.
        CAN_rx_msg->id |= 1U << 31; 
    } else {
        // Standard frame received, handle if necessary
        CAN_rx_msg->id = (r0 >> 18) & 0x7FF;
    }

    // Extract Data Length Code from R1
    CAN_rx_msg->data_len = (r1 >> 16) & 0xF;

    // Copy data payload, respecting the DLC
    uint8_t *rx_data_ptr = (uint8_t *)&frame_ptr[2];
    for (uint8_t i = 0; i < CAN_rx_msg->data_len; i++) {
        CAN_rx_msg->data[i] = rx_data_ptr[i];
    }

    // Acknowledge the message to release the FIFO buffer
    FDCAN1->RXF0A = index;
}

// CAN initialization
bool CANInit(BITRATE bitrate, int remap)
{
    // Enable FDCAN clock
    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
    RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;

    // Setup GPIOs for remap
    if (remap == 0) {
        CANSetGpio(GPIOA, 11, 9); // RX PA11 AF9
        CANSetGpio(GPIOA, 12, 9); // TX PA12 AF9
    }
    // Add other remap cases as needed...

    // Enter FDCAN init mode
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) {}

    // Enable configuration changes
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    // Set bit timing
    FDCAN1->NBTP = (((can_configs[bitrate].TS2 - 1) << FDCAN_NBTP_NTSEG2_Pos) |
                    ((can_configs[bitrate].TS1 - 1) << FDCAN_NBTP_NTSEG1_Pos) |
                    ((can_configs[bitrate].BRP - 1) << FDCAN_NBTP_NBRP_Pos) |
                    (0 << FDCAN_NBTP_NSJW_Pos)); // SJW=1

    // Setup default filter (accept all)
    CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

    // Leave init mode
    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE; // Disable config changes
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;

    // Wait for normal mode with a timeout, similar to canL431.cpp
    uint16_t TimeoutMilliseconds = 1000;
    bool normal_mode = false;
    for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++)
    {
        if (!(FDCAN1->CCCR & FDCAN_CCCR_INIT))
        {
            normal_mode = true;
            break;
        }
        // A small delay could be added here if running on hardware
        // delay(1); 
    }
    
    return normal_mode;
}
#endif