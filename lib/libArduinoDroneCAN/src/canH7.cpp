#ifdef CANH7
#include "Arduino.h"
#include "canH7.h"
#include <string.h> // For memset

// Defines based on CANFDIface.cpp for STM32H7
#define FDCAN_FRAME_BUFFER_SIZE 18 // FIFO elements are spaced at 18 words for 64-byte payload
#define FDCAN_MAX_STD_FILTERS 28
#define FDCAN_MAX_EXT_FILTERS 8 // Example allocation
#define FDCAN_RX_FIFO0_ELEMENTS 8
#define FDCAN_TX_BUFFER_ELEMENTS 4

// Message RAM layout
static uint32_t FDCAN_message_ram_offset = 0;
struct {
    uint32_t StandardFilterSA;
    uint32_t ExtendedFilterSA;
    uint32_t RxFIFO0SA;
    uint32_t TxFIFOQSA;
    uint32_t EndAddress;
} MessageRam;

// Helper to compute timings, adapted from CANFDIface.cpp
static bool compute_timings(const uint32_t target_bitrate, CAN_bit_timing_config_t& out_timings) {
    if (target_bitrate < 1) {
        return false;
    }
    const uint32_t pclk = 80000000; // Assuming 80MHz FDCAN clock from CANFDIface.cpp
    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;
    static const int MaxSamplePointLocation = 900;

    const uint32_t prescaler_bs = pclk / target_bitrate;
    uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
        if (bs1_bs2_sum <= 2) {
            return false; // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U)) {
        return false; // No solution
    }

    uint8_t bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8;
    uint8_t bs2 = bs1_bs2_sum - bs1;
    uint16_t sample_point_permill = (uint16_t)(1000 * (1 + bs1) / (1 + bs1 + bs2));

    if (sample_point_permill > MaxSamplePointLocation) {
        bs1 = (7 * bs1_bs2_sum - 1) / 8;
        bs2 = bs1_bs2_sum - bs1;
    }
    
    if ((bs1 < 1) || (bs1 > MaxBS1) || (bs2 < 1) || (bs2 > MaxBS2)) {
        return false;
    }

    out_timings.BRP = (uint16_t)prescaler;
    out_timings.TS1 = bs1;
    out_timings.TS2 = bs2;
    // SJW is not calculated here but set to 1 in CANInit, which is a reasonable default.
    return true;
}


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
 * This function is corrected to properly configure a single standard filter.
 * For multiple/extended filters, this logic should be expanded.
 */
void CANSetFilter(uint8_t index, uint8_t fifo, uint32_t id, uint32_t mask)
{
    if (index >= FDCAN_MAX_STD_FILTERS) {
        return;
    }

    // Enter filter configuration mode
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    uint32_t timeout = 1000;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT) && timeout-- > 0) {}
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    // Standard filter element for classic CAN (ID/Mask)
    // S0[31:30] SFEC: 01=Store in FIFO0, 10=Store in FIFO1
    // S0[29:27] SFT:  010=Classic filter
    // S0[26:16] SFID1: filter ID
    // S0[15:0]  SFID2: filter mask (for classic filter type)
    uint32_t sfec = (fifo == 0) ? 0b01 : 0b10;
    uint32_t sft = 0b010; // Classic filter
    uint32_t sfid1 = id & 0x7FF;
    uint32_t sfid2 = mask & 0x7FF;

    uint32_t filter_element = (sfec << 30) | (sft << 27) | (sfid1 << 16) | sfid2;
    
    uint32_t *filter_ram = (uint32_t *)MessageRam.StandardFilterSA;
    filter_ram[index] = filter_element;

    // Leave filter configuration mode
    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    timeout = 1000;
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) && timeout-- > 0) {}
}

// Check for available CAN messages in FIFO0
uint8_t CANMsgAvail(void)
{
    return (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL);
}

// Send CAN frame
void CANSend(const CanardCANFrame *CAN_tx_msg)
{
    // Wait for a free Tx buffer, with a timeout.
    uint32_t timeout = 1000000;
    while ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) && timeout-- > 0) {}
    if (timeout == 0) {
        // Handle timeout error, maybe increment a counter
        return;
    }

    uint32_t index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    
    // Calculate Tx element address
    uint32_t *buffer = (uint32_t *)(MessageRam.TxFIFOQSA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));

    // T0: ID and flags. We send an extended frame, which is standard for DroneCAN.
    buffer[0] = (1U << 30) | (CAN_tx_msg->id & 0x1FFFFFFF); // Set XTD (IDE) bit for 29-bit extended ID

    // T1: DLC. For classic CAN, FDF (FD Format) and BRS (Bit Rate Switch) are 0.
    buffer[1] = (uint32_t)(CAN_tx_msg->data_len) << 16;

    // Copy data using word-writes for efficiency
    uint32_t *tx_data_ptr = &buffer[2];
    // Clear the data area first
    memset(tx_data_ptr, 0, CAN_tx_msg->data_len);
    memcpy(tx_data_ptr, CAN_tx_msg->data, CAN_tx_msg->data_len);

    // Request transmission for the buffer at the calculated index.
    FDCAN1->TXBAR = (1 << index);
}

// Receive CAN frame
void CANReceive(CanardCANFrame *CAN_rx_msg)
{
    if (CANMsgAvail() == 0) {
        return;
    }

    // Get the read index from FIFO 0 Status register
    uint32_t index = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;

    // Calculate the address of the Rx buffer in Message RAM
    uint32_t *frame_ptr = (uint32_t *)(MessageRam.RxFIFO0SA + (index * FDCAN_FRAME_BUFFER_SIZE * 4));

    // R0 contains ID and flags
    uint32_t r0 = frame_ptr[0];
    // R1 contains DLC and other info
    uint32_t r1 = frame_ptr[1];

    // The XTD bit is bit 30 in the Rx Buffer Element R0.
    if (r0 & (1U << 30)) { // Check XTD bit for extended frame
        CAN_rx_msg->id = r0 & 0x1FFFFFFF; // Mask for 29-bit extended ID
        CAN_rx_msg->id |= 1U << 31; // Compatibility flag
    } else {
        // Standard frame
        CAN_rx_msg->id = (r0 >> 18) & 0x7FF;
    }

    // Extract Data Length Code from R1
    CAN_rx_msg->data_len = (r1 >> 16) & 0xF;

    // Copy data payload
    uint8_t *rx_data_ptr = (uint8_t *)&frame_ptr[2];
    memcpy(CAN_rx_msg->data, rx_data_ptr, CAN_rx_msg->data_len);

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
    uint32_t timeout = 1000;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT) && timeout-- > 0) {}
    if (timeout == 0) return false;

    // Enable configuration changes
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    // Set bit timing
    CAN_bit_timing_config_t timings;
    uint32_t rate_kbps[] = {50000, 100000, 125000, 250000, 500000, 1000000};
    if (!compute_timings(rate_kbps[bitrate], timings)) {
        // Fallback to old hardcoded values if compute_timings fails
        CAN_bit_timing_config_t can_configs[6] = {
            {2, 13, 100}, {2, 13, 50}, {2, 13, 40}, {2, 13, 20}, {2, 13, 10}, {1, 8, 8}
        };
        timings = can_configs[bitrate];
    }
    
    FDCAN1->NBTP = (((timings.TS2 - 1) << FDCAN_NBTP_NTSEG2_Pos) |
                    ((timings.TS1 - 1) << FDCAN_NBTP_NTSEG1_Pos) |
                    ((timings.BRP - 1) << FDCAN_NBTP_NBRP_Pos) |
                    (0 << FDCAN_NBTP_NSJW_Pos)); // SJW=1

    // Setup Message RAM - adapted from CANFDIface.cpp
    memset(&MessageRam, 0, sizeof(MessageRam));
    FDCAN_message_ram_offset = 0;

    // Standard filters
    FDCAN1->SIDFC = (FDCAN_message_ram_offset << 2) | (FDCAN_MAX_STD_FILTERS << 16);
    MessageRam.StandardFilterSA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
    FDCAN_message_ram_offset += FDCAN_MAX_STD_FILTERS;

    // Extended filters (example, can be adjusted)
    FDCAN1->XIDFC = (FDCAN_message_ram_offset << 2) | (FDCAN_MAX_EXT_FILTERS << 16);
    MessageRam.ExtendedFilterSA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
    FDCAN_message_ram_offset += FDCAN_MAX_EXT_FILTERS * 2; // Extended filters take 2 words

    // Rx FIFO 0
    FDCAN1->RXF0C = (FDCAN_message_ram_offset << 2) | (FDCAN_RX_FIFO0_ELEMENTS << 16);
    MessageRam.RxFIFO0SA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
    FDCAN_message_ram_offset += FDCAN_RX_FIFO0_ELEMENTS * FDCAN_FRAME_BUFFER_SIZE;

    // Tx Buffers
    FDCAN1->TXBC = (FDCAN_message_ram_offset << 2) | (FDCAN_TX_BUFFER_ELEMENTS << 24);
    MessageRam.TxFIFOQSA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
    FDCAN_message_ram_offset += FDCAN_TX_BUFFER_ELEMENTS * FDCAN_FRAME_BUFFER_SIZE;

    // Global Filter Configuration: Accept non-matching standard frames into FIFO0, reject extended.
    FDCAN1->GFC = 0x02 << 4 | 0x01;

    // Set a default "accept all" standard filter at index 0
    CANSetFilter(0, 0, 0x0, 0x0);

    // Leave init mode
    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;

    timeout = 1000;
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) && timeout-- > 0) {}
    
    return timeout > 0;
}
#endif