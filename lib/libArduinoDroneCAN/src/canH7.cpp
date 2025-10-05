#ifdef CANH7
#include "Arduino.h"
#include "canH7.h"
#include <stm32h7xx.h>

#define DEBUG 0
#define AF9 0x09

// Timing configurations for different bitrates
// Assuming PLL2_Q = 80MHz for FDCAN kernel clock
CAN_bit_timing_config_t can_configs[6] = {
    {4, 11, 100}, // 50 kbps:  80MHz / (100 * (1+11+4)) = 50k
    {4, 11, 50},  // 100 kbps: 80MHz / (50 * (1+11+4)) = 100k
    {4, 11, 40},  // 125 kbps: 80MHz / (40 * (1+11+4)) = 125k
    {4, 11, 20},  // 250 kbps: 80MHz / (20 * (1+11+4)) = 250k
    {4, 11, 10},  // 500 kbps: 80MHz / (10 * (1+11+4)) = 500k
    {4, 11, 5}    // 1 Mbps:   80MHz / (5 * (1+11+4)) = 1M
};

// FDCAN memory configuration
#define FDCAN_MEM_START_ADDR          0x4000AC00UL
#define FDCAN_MEM_END_ADDR            0x4000D3FFUL

// Standard filters (not used, set to 0)
#define FDCAN_11B_FILTER_EL_CNT       0UL
#define FDCAN_11B_FILTER_OFFSET       0UL

// Extended filters configuration
#define FDCAN_29B_FILTER_EL_CNT       1UL
#define FDCAN_29B_FILTER_EL_SIZE      8UL
#define FDCAN_29B_FILTER_OFFSET       0UL
#define FDCAN_29B_FILTER_START_ADDR   FDCAN_MEM_START_ADDR

// RX FIFO 0 configuration
#define FDCAN_RX_FIFO_0_EL_CNT        32UL
#define FDCAN_RX_FIFO_0_HEAD_SIZE     8UL
#define FDCAN_RX_FIFO_0_DATA_SIZE     8UL
#define FDCAN_RX_FIFO_0_EL_SIZE       16UL
#define FDCAN_RX_FIFO_0_OFFSET        (FDCAN_29B_FILTER_OFFSET + FDCAN_29B_FILTER_EL_CNT * 2)
#define FDCAN_RX_FIFO_0_START_ADDR    (FDCAN_29B_FILTER_START_ADDR + FDCAN_29B_FILTER_EL_CNT * FDCAN_29B_FILTER_EL_SIZE)

// TX FIFO configuration
#define FDCAN_TX_FIFO_EL_CNT          16UL
#define FDCAN_TX_FIFO_HEAD_SIZE       8UL
#define FDCAN_TX_FIFO_DATA_SIZE       8UL
#define FDCAN_TX_FIFO_EL_SIZE         16UL
#define FDCAN_TX_FIFO_OFFSET          (FDCAN_RX_FIFO_0_OFFSET + FDCAN_RX_FIFO_0_EL_CNT * 4)
#define FDCAN_TX_FIFO_START_ADDR      (FDCAN_RX_FIFO_0_START_ADDR + FDCAN_RX_FIFO_0_EL_CNT * FDCAN_RX_FIFO_0_EL_SIZE)

struct can_fifo_element
{
    uint32_t word0;
    uint32_t word1;
    uint32_t word2;
    uint32_t word3;
};

#define STM32_CAN_TIR_TXRQ (1U << 0U)
#define STM32_CAN_RIR_RTR  (1U << 1)
#define STM32_CAN_RIR_IDE  (1U << 2)
#define STM32_CAN_TIR_RTR  (1U << 1U)
#define STM32_CAN_TIR_IDE  (1U << 2U)

#define CAN_EXT_ID_MASK    0x1FFFFFFFU
#define CAN_STD_ID_MASK    0x000007FFU

/**
 * Print registers for debugging.
 */
void printRegister(char *buf, uint32_t reg)
{
    if (DEBUG == 0)
        return;
    Serial.print(buf);
    Serial.print("0x");
    Serial.print(reg, HEX);
    Serial.println();
}

/**
 * Initializes the CAN GPIO registers.
 */
void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed = 3)
{
    uint8_t _index2 = index * 2;
    uint8_t _index4 = index * 4;
    uint8_t ofs = 0;
    uint8_t setting;

    if (index > 7)
    {
        _index4 = (index - 8) * 4;
        ofs = 1;
    }

    uint32_t mask;
    
    // Reset and set alternate function
    mask = 0xF << _index4;
    addr->AFR[ofs] &= ~mask;
    setting = afry;
    mask = setting << _index4;
    addr->AFR[ofs] |= mask;

    // Reset and set mode to alternate function
    mask = 0x3 << _index2;
    addr->MODER &= ~mask;
    setting = 0x2; // Alternate function mode
    mask = setting << _index2;
    addr->MODER |= mask;

    // Reset and set speed
    mask = 0x3 << _index2;
    addr->OSPEEDR &= ~mask;
    setting = speed;
    mask = setting << _index2;
    addr->OSPEEDR |= mask;

    // Reset Output push-pull
    mask = 0x1 << index;
    addr->OTYPER &= ~mask;

    // Reset port pull-up/pull-down
    mask = 0x3 << _index2;
    addr->PUPDR &= ~mask;
}

/**
 * Initializes the CAN filter registers.
 */
void CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2)
{
    if (index > 0) return; // Only using one filter for extended IDs
    
    uint32_t *ptr = (uint32_t*)FDCAN_29B_FILTER_START_ADDR;
    
    // For extended filter: Filter configuration + ID1
    *ptr++ = (1UL << 29) | 0x0; // Store in FIFO 0 if filter matches, accept all IDs
    *ptr++ = 0x0; // Mask - accept all
}

/**
 * Convert DLC to data length
 */
uint8_t dlcToDataLength(uint8_t dlc)
{
    if (dlc <= 8) return dlc;
    else if (dlc == 9) return 12;
    else if (dlc == 10) return 16;
    else if (dlc == 11) return 20;
    else if (dlc == 12) return 24;
    else if (dlc == 13) return 32;
    else if (dlc == 14) return 48;
    return 64;
}

/**
 * Initializes the CAN controller with specified bit rate.
 */
bool CANInit(BITRATE bitrate, int remap)
{
    // Reset FDCAN2
    FDCAN2->CCCR = FDCAN_CCCR_INIT;
    
    // Enable clocks
    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN; // Enable FDCAN clock
    
    // Configure GPIO based on remap option
    if (remap == 0)
    {
        // PA11 (RX), PA12 (TX) - Default FDCAN2 pins
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
        CANSetGpio(GPIOA, 11, AF9); // PA11 -> FDCAN2_RX
        CANSetGpio(GPIOA, 12, AF9); // PA12 -> FDCAN2_TX
    }
    else if (remap == 2)
    {
        // PB8 (RX), PB9 (TX) - Alternate FDCAN2 pins
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
        CANSetGpio(GPIOB, 5, AF9);  // PB8 -> FDCAN2_RX
        CANSetGpio(GPIOB, 6, AF9);  // PB9 -> FDCAN2_TX
    }
    else if (remap == 1)
    {
        // PD0 (RX), PD1 (TX) - Another alternate for FDCAN2
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN;
        CANSetGpio(GPIOH, 14, AF9);
        CANSetGpio(GPIOD, 1, AF9); 
    }
    else if (remap == 3)
    {
        // PH13 (RX), PH14 (TX) - Yet another alternate for FDCAN2
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN;
        CANSetGpio(GPIOH, 13, AF9); // PH13 -> FDCAN2_RX
        CANSetGpio(GPIOH, 14, AF9); // PH14 -> FDCAN2_TX
    }
    
    // Set FDCAN clock source to PLL2Q (80 MHz assumed)
    RCC->D2CCIP1R &= ~RCC_D2CCIP1R_FDCANSEL_Msk;
    RCC->D2CCIP1R |= RCC_D2CCIP1R_FDCANSEL_1; // PLL2Q selected
    
    // Enter initialization mode
    FDCAN2->CCCR |= FDCAN_CCCR_INIT;
    while ((FDCAN2->CCCR & FDCAN_CCCR_INIT) == 0);
    
    // Enable configuration change
    FDCAN2->CCCR |= FDCAN_CCCR_CCE;
    
    // Set Classic CAN mode (not FD)
    FDCAN2->CCCR &= ~FDCAN_CCCR_FDOE;
    
    // Disable automatic retransmission for compatibility with L431 driver
    FDCAN2->CCCR &= ~FDCAN_CCCR_DAR;
    
    // Clear message RAM
    uint32_t *ram_ptr;
    for (ram_ptr = (uint32_t*)FDCAN_MEM_START_ADDR; ram_ptr < (uint32_t*)FDCAN_MEM_END_ADDR; ram_ptr++)
    {
        *ram_ptr = 0;
    }
    
    // Configure bit timing
    FDCAN2->NBTP = 0; // Clear first
    FDCAN2->NBTP |= ((can_configs[bitrate].TS2 - 1) << FDCAN_NBTP_NTSEG2_Pos) |
                    ((can_configs[bitrate].TS1 - 1) << FDCAN_NBTP_NTSEG1_Pos) |
                    ((can_configs[bitrate].BRP - 1) << FDCAN_NBTP_NBRP_Pos) |
                    (3 << FDCAN_NBTP_NSJW_Pos); // Sync jump width = 4
    
    // Configure filters - accept all extended IDs
    FDCAN2->GFC = 0; // Accept all frames by default
    FDCAN2->SIDFC = 0; // No standard filters
    
    // Setup extended filter
    FDCAN2->XIDFC = (FDCAN_29B_FILTER_EL_CNT << FDCAN_XIDFC_LSE_Pos) | 
                    (FDCAN_29B_FILTER_OFFSET << FDCAN_XIDFC_FLESA_Pos);
    
    // Initialize filter to accept all
    CANSetFilter(0, 1, 0, 0, 0x0, 0x0);
    
    // Configure RX FIFO 0
    FDCAN2->RXF0C = (FDCAN_RX_FIFO_0_OFFSET << FDCAN_RXF0C_F0SA_Pos) |
                    (FDCAN_RX_FIFO_0_EL_CNT << FDCAN_RXF0C_F0S_Pos);
    
    // Configure TX buffer/FIFO
    FDCAN2->TXBC = (FDCAN_TX_FIFO_EL_CNT << FDCAN_TXBC_TFQS_Pos) |
                   (FDCAN_TX_FIFO_OFFSET << FDCAN_TXBC_TBSA_Pos);
    
    // No TX event FIFO
    FDCAN2->TXEFC = 0;
    
    // Enable RX FIFO 0 new message interrupt
    FDCAN2->IE |= FDCAN_IE_RF0NE;
    
    // Enable interrupt line 0
    FDCAN2->ILE |= FDCAN_ILE_EINT0;
    
    // Leave initialization mode
    FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;
    
    // Wait for normal mode
    const uint32_t now = millis();

    while (millis() - now < 1000)
    {
        if ((FDCAN2->CCCR & FDCAN_CCCR_INIT) == 0)
        {
            Serial.println("FDCAN2 initialized successfully");
            return true;
        }
        FDCAN2->CCCR &= ~FDCAN_CCCR_INIT;
    }
    
    Serial.println("FDCAN2 initialization failed!");
    return false;
}

/**
 * Decodes CAN messages from the data registers.
 */
void CANReceive(CanardCANFrame *CAN_rx_msg)
{
    // Get the fill level and get index
    uint8_t rx_fifo_get_index = (uint8_t)((FDCAN2->RXF0S >> 8) & 0x3F);
    
    struct can_fifo_element *fifo;
    fifo = (struct can_fifo_element*)(FDCAN_RX_FIFO_0_START_ADDR + rx_fifo_get_index * FDCAN_RX_FIFO_0_EL_SIZE);
    
    // Extract ID (always extended for UAVCAN/DroneCAN)
    CAN_rx_msg->id = (fifo->word0 & CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
    
    // Extract data length
    uint8_t dlc = (fifo->word1 >> 16) & 0x0F;
    CAN_rx_msg->data_len = dlcToDataLength(dlc);
    if (CAN_rx_msg->data_len > 8) CAN_rx_msg->data_len = 8; // Limit to 8 for classic CAN
    
    // Extract data bytes
    CAN_rx_msg->data[0] = (uint8_t)((fifo->word2 >> 0) & 0xFF);
    CAN_rx_msg->data[1] = (uint8_t)((fifo->word2 >> 8) & 0xFF);
    CAN_rx_msg->data[2] = (uint8_t)((fifo->word2 >> 16) & 0xFF);
    CAN_rx_msg->data[3] = (uint8_t)((fifo->word2 >> 24) & 0xFF);
    CAN_rx_msg->data[4] = (uint8_t)((fifo->word3 >> 0) & 0xFF);
    CAN_rx_msg->data[5] = (uint8_t)((fifo->word3 >> 8) & 0xFF);
    CAN_rx_msg->data[6] = (uint8_t)((fifo->word3 >> 16) & 0xFF);
    CAN_rx_msg->data[7] = (uint8_t)((fifo->word3 >> 24) & 0xFF);
    
    // Acknowledge the FIFO entry
    FDCAN2->RXF0A = rx_fifo_get_index;
}

/**
 * Encodes and sends CAN messages.
 */
void CANSend(const CanardCANFrame *CAN_tx_msg)
{
    // Check if TX FIFO is full
    if (FDCAN2->TXFQS & FDCAN_TXFQS_TFQF)
    {
        Serial.println("TX FIFO Full!");
        return;
    }
    
    // Get put index for TX FIFO
    uint8_t tx_index = (FDCAN2->TXFQS >> 16) & 0x1F;
    
    struct can_fifo_element *fifo;
    fifo = (struct can_fifo_element*)(FDCAN_TX_FIFO_START_ADDR + tx_index * FDCAN_TX_FIFO_EL_SIZE);
    
    // Set up the ID field (extended ID for UAVCAN)
    fifo->word0 = (CAN_tx_msg->id & CAN_EXT_ID_MASK) | (1UL << 30); // XTD bit for extended
    
    // Set up control field
    fifo->word1 = (CAN_tx_msg->data_len << 16); // DLC
    
    // Copy data
    fifo->word2 = ((uint32_t)CAN_tx_msg->data[3] << 24) |
                  ((uint32_t)CAN_tx_msg->data[2] << 16) |
                  ((uint32_t)CAN_tx_msg->data[1] << 8) |
                  ((uint32_t)CAN_tx_msg->data[0]);
    
    fifo->word3 = ((uint32_t)CAN_tx_msg->data[7] << 24) |
                  ((uint32_t)CAN_tx_msg->data[6] << 16) |
                  ((uint32_t)CAN_tx_msg->data[5] << 8) |
                  ((uint32_t)CAN_tx_msg->data[4]);
    
    // Request transmission
    FDCAN2->TXBAR |= (1UL << tx_index);
    
    // Wait for transmission to complete (with timeout)
    volatile int count = 0;
    while ((FDCAN2->TXBTO & (1UL << tx_index)) == 0 && count++ < 1000000);
    
    if (count >= 1000000)
    {
        Serial.print("Send timeout! ESR: ");
        Serial.print(FDCAN2->PSR);
        Serial.print(" ECR: ");
        Serial.println(FDCAN2->ECR);
    }
}

/**
 * Returns whether there are CAN messages available.
 */
uint8_t CANMsgAvail(void)
{
    // Check RX FIFO 0 fill level
    return (FDCAN2->RXF0S & 0x3F);
}

#endif // CANH743