#include "canH7.h"
#include <stdbool.h>
#include <string.h>

// -----------------------------------------------------------------------------
// FIX: Local Definitions for Missing HAL Time Segment Constants
// These constants are the register values (TQ - 1) expected by FDCAN_InitTypeDef
// -----------------------------------------------------------------------------
#define SRAMCAN_BASE 0x4000AC00U 

// Nominal Time Segment 1 (NTSEG1): 16 TQ -> value 15
#ifndef FDCAN_NTSEG1_16TQ
#define FDCAN_NTSEG1_16TQ ((uint32_t)15U)
#endif
// Nominal Time Segment 2 (NTSEG2): 3 TQ -> value 2
#ifndef FDCAN_NTSEG2_3TQ
#define FDCAN_NTSEG2_3TQ ((uint32_t)2U)
#endif
// Nominal Sync Jump Width (NSJW): 1 TQ -> value 0
#ifndef FDCAN_NSJW_1TQ
#define FDCAN_NSJW_1TQ ((uint32_t)0U)
#endif
// Data Time Segment 1 (DTSEG1): 1 TQ -> value 0 (Used for Classic CAN fallback)
#ifndef FDCAN_DTSEG1_1TQ
#define FDCAN_DTSEG1_1TQ ((uint32_t)0U)
#endif
// Data Time Segment 2 (DTSEG2): 1 TQ -> value 0 (Used for Classic CAN fallback)
#ifndef FDCAN_DTSEG2_1TQ
#define FDCAN_DTSEG2_1TQ ((uint32_t)0U)
#endif


// --- Configuration Constants (Assumed from previous steps) ---
#define FDCAN_NUM_STD_FILTERS 1 
#define FDCAN_NUM_EXT_FILTERS 0
#define FDCAN_NUM_RXFIFO0_ELEMENTS 8
#define FDCAN_NUM_TX_QUEUE_ELEMENTS 7
#define FDCAN_ELEMENT_SIZE_WORDS 4

#define FDCAN_RX_FIFO0_ELEMENT_SIZE FDCAN_DATA_BYTES_8
#define FDCAN_TX_ELEMENT_SIZE FDCAN_DATA_BYTES_8

#define FDCAN_FILTER_BYTES (FDCAN_STD_FILTER_WORDS * 4)
#define FDCAN_RX_ELEMENT_BYTES (FDCAN_ELEMENT_SIZE_WORDS * 4)
#define FDCAN_TX_ELEMENT_BYTES (FDCAN_ELEMENT_SIZE_WORDS * 4)

// Helper to calculate size in 32-bit words (Header + 8-byte Data = 4 words total)
#define FDCAN_ELEMENT_WORDS 4 
#define FDCAN_STD_FILTER_WORDS 1

// --- Global Handle Definition ---
FDCAN_HandleTypeDef hfdcan1;
#define FDCAN_PERIPHERAL FDCAN1
#define RX_FIFO FDCAN_RX_FIFO0
#define TX_QUEUE_ELEMENTS 7

// --- Internal Data Structure for Bit Timing (Hardcoded for FDCAN @ 100MHz clock) ---
typedef struct
{
    uint32_t NominalPrescaler;
    uint32_t NominalTimeSeg1;
    uint32_t NominalTimeSeg2;
    uint32_t NominalSyncJumpWidth;
} FDCAN_Timing_Config_t;

// The values now use the locally defined constants above
const FDCAN_Timing_Config_t fdcancfg[6] = {
    // FDCAN Clock = 100 MHz. Total TQ = 20. Sample Point ~ 80% (3 TQ after 1 TQ Sync + 16 TQ TSeg1)
    [CAN_50KBPS] = {100, FDCAN_NTSEG1_16TQ, FDCAN_NTSEG2_3TQ, FDCAN_NSJW_1TQ},
    [CAN_100KBPS] = {50, FDCAN_NTSEG1_16TQ, FDCAN_NTSEG2_3TQ, FDCAN_NSJW_1TQ},
    [CAN_125KBPS] = {40, FDCAN_NTSEG1_16TQ, FDCAN_NTSEG2_3TQ, FDCAN_NSJW_1TQ},
    [CAN_250KBPS] = {20, FDCAN_NTSEG1_16TQ, FDCAN_NTSEG2_3TQ, FDCAN_NSJW_1TQ},
    [CAN_500KBPS] = {10, FDCAN_NTSEG1_16TQ, FDCAN_NTSEG2_3TQ, FDCAN_NSJW_1TQ},
    [CAN_1000KBPS] = {5, FDCAN_NTSEG1_16TQ, FDCAN_NTSEG2_3TQ, FDCAN_NSJW_1TQ}};

// --- FDCAN Initialization ---

bool CANInit(enum BITRATE bitrate, int remap)
{
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // FDCAN1 RX GPIO Configuration: PH14 -> FDCAN1_RX
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1; // AF9 for FDCAN1 on PH14
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    // FDCAN1 TX GPIO Configuration: PD1 -> FDCAN1_TX
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1; // AF9 for FDCAN1 on PD1
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_StatusTypeDef status;
    FDCAN_Timing_Config_t timing = fdcancfg[bitrate];

    uint32_t current_address = SRAMCAN_BASE;

    // Standard Filter List
    hfdcan1.msgRam.StandardFilterSA = current_address;
    current_address += FDCAN_NUM_STD_FILTERS * FDCAN_FILTER_BYTES;

    // Extended Filter List (0 elements)
    hfdcan1.msgRam.ExtendedFilterSA = current_address;

    // Rx FIFO 0 
    hfdcan1.msgRam.RxFIFO0SA = current_address;
    current_address += FDCAN_NUM_RXFIFO0_ELEMENTS * FDCAN_RX_ELEMENT_BYTES;

    // Rx FIFO 1 (0 elements)
    hfdcan1.msgRam.RxFIFO1SA = current_address;
    
    // Tx FIFO/Queue (CRITICAL: Needs to be set correctly)
    hfdcan1.msgRam.TxFIFOQSA = current_address;
    current_address += FDCAN_NUM_TX_QUEUE_ELEMENTS * FDCAN_TX_ELEMENT_BYTES;

    // Set unused or End addresses
    hfdcan1.msgRam.RxBufferSA = current_address;
    hfdcan1.msgRam.TxEventFIFOSA = current_address;
    hfdcan1.msgRam.TxBufferSA = current_address;
    hfdcan1.msgRam.TTMemorySA = current_address;
    hfdcan1.msgRam.EndAddress = current_address;

    // 1. De-Initialize (HAL recommended for re-init)
    if (HAL_FDCAN_DeInit(&hfdcan1) != HAL_OK)
        return false;

    // 2. Configure Handle
    hfdcan1.Instance = FDCAN_PERIPHERAL;

    // 3. Configure Initialization Structure for CLASSIC CAN (8-byte payload)
    hfdcan1.Instance = FDCAN_PERIPHERAL;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC; 
    hfdcan1.Init.DataPrescaler = 1; 
    hfdcan1.Init.DataTimeSeg1 = FDCAN_DTSEG1_1TQ;
    hfdcan1.Init.DataTimeSeg2 = FDCAN_DTSEG2_1TQ;

    // Apply Nominal Timings
    hfdcan1.Init.NominalPrescaler = timing.NominalPrescaler;
    hfdcan1.Init.NominalSyncJumpWidth = timing.NominalSyncJumpWidth;
    hfdcan1.Init.NominalTimeSeg1 = timing.NominalTimeSeg1;
    hfdcan1.Init.NominalTimeSeg2 = timing.NominalTimeSeg2;

    // 4. Configure Message RAM Layout for Init (Must match manual settings)
    hfdcan1.Init.StdFiltersNbr = FDCAN_NUM_STD_FILTERS;       
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = FDCAN_NUM_RXFIFO0_ELEMENTS;      
    hfdcan1.Init.RxFifo1ElmtsNbr = 0;
    
    // CRITICAL FIX: Ensure zero dedicated Tx buffers
    hfdcan1.Init.RxBuffersNbr = 0; 
    hfdcan1.Init.TxBuffersNbr = 0; 

    // Set Tx Queue Parameters
    hfdcan1.Init.TxFifoQueueElmtsNbr = FDCAN_NUM_TX_QUEUE_ELEMENTS;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION; // Use Queue mode

    // Payload Size must be 8 bytes for Classic CAN
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8; 
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    // 5. Initialize FDCAN
    status = HAL_FDCAN_Init(&hfdcan1);
    if (status != HAL_OK)
        return false;

    // 6. CONFIGURE GLOBAL FILTER TO ACCEPT ALL NON-MATCHING FRAMES (Standard and Extended)
    // This ensures all CAN traffic is received for the higher-level DroneCAN library to process.
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                          FDCAN_FILTER_TO_RXFIFO0,  // Non-matching Standard ID frames
                                          FDCAN_FILTER_TO_RXFIFO0,  // Non-matching Extended ID frames
                                          FDCAN_FILTER_TO_RXFIFO0,  // Non-matching Standard Remote frames
                                          FDCAN_FILTER_TO_RXFIFO0); // Non-matching Extended Remote frames
    if (status != HAL_OK)
        return false;

    // 7. Start the FDCAN peripheral
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        return false;

    // Check CCCR register (Configuration and Control Register)
    Serial.print("CCCR Register Value: 0x");
    Serial.println(hfdcan1.Instance->CCCR, HEX);

    // The INIT bit (CCCR[0]) must be 1 to be in Configuration Mode
    if (!(hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT))
    {
        Serial.println("FDCAN did not enter Configuration Mode (CCCR.INIT is 0). Clock issue?");
    }

    return true;
}

// --- FDCAN Transmission ---

/**
 * @brief Helper function to convert byte length (0-8) to FDCAN_DLC_x constant.
 */
static uint32_t getHalDlc(uint8_t len)
{
    if (len > 8)
        len = 8;
    return len << 16; // DLC encoding for 0-8 bytes in FDCAN
}

void CANSend(const CanardCANFrame *CAN_tx_msg)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};

    // 1. Map CanardCANFrame to FDCAN_TxHeaderTypeDef
    if (CAN_tx_msg->id & CANARD_CAN_FRAME_EFF)
    {
        // Correctly handles DroneCAN's required 29-bit Extended ID format
        txHeader.IdType = FDCAN_EXTENDED_ID;
        txHeader.Identifier = CAN_tx_msg->id & 0x1FFFFFFF;
    }
    else
    {
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.Identifier = CAN_tx_msg->id & 0x7FF;
    }

    txHeader.TxFrameType = (CAN_tx_msg->id & CANARD_CAN_FRAME_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;

    // Classic CAN mode settings for DroneCAN
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;

    txHeader.DataLength = getHalDlc(CAN_tx_msg->data_len);
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    // 2. Add the message to the Tx FIFO/Queue
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t *)CAN_tx_msg->data);
    if (status != HAL_OK)
    {
        // Handle error
    }
}

// --- FDCAN Reception ---

void CANReceive(CanardCANFrame *CAN_rx_msg)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_FDCAN_GetRxMessage(&hfdcan1, RX_FIFO, &rxHeader, rxData) == HAL_OK)
    {
        // 1. Map FDCAN_RxHeaderTypeDef to CanardCANFrame

        // ID and EFF flag
        CAN_rx_msg->id = rxHeader.Identifier;
        if (rxHeader.IdType == FDCAN_EXTENDED_ID)
        {
            // Correctly extracts the Extended ID and sets the flag for DroneCAN
            CAN_rx_msg->id |= CANARD_CAN_FRAME_EFF;
        }

        // RTR flag
        if (rxHeader.RxFrameType == FDCAN_REMOTE_FRAME)
        {
            CAN_rx_msg->id |= CANARD_CAN_FRAME_RTR;
        }

        // Data Length and Data
        uint8_t dlc = (uint8_t)((rxHeader.DataLength >> 16U) & 0xFU);
        CAN_rx_msg->data_len = dlc;

        // Copy data (max 8 bytes)
        memcpy(CAN_rx_msg->data, rxData, dlc);
    }
}

uint8_t CANMsgAvail(void)
{
    return (uint8_t)HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, RX_FIFO);
}

// --- FDCAN Filtering ---

void CANSetFilter(uint16_t id)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = id << 5;
    sFilterConfig.FilterID2 = 0x7FF << 5;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
}

void CANSetFilters(uint16_t *ids, uint8_t num)
{
    if (num > 0)
    {
        CANSetFilter(ids[0]);
    }
}