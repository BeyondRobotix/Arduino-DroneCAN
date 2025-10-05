#ifdef CANH7
#include "canH7.h"
#include <string.h> // For memset and memcpy
#include "Arduino.h"

#define DEBUG 1

#define SRAMCAN_BASE 0x4000AC00U

#if DEBUG
void printRegister(const char *buf, uint32_t reg)
{
    Serial.print(buf);
    Serial.print("0x");
    Serial.print(reg, HEX);
    Serial.println();
}
#endif

// --- Message RAM Configuration ---
#if CANARD_ENABLE_CANFD
#define FDCAN_DATA_FIELD_SIZE 64
#define FDCAN_ELEMENT_SIZE_WORDS 18 // In words for 64-byte data payload
#else
#define FDCAN_DATA_FIELD_SIZE 8
#define FDCAN_ELEMENT_SIZE_WORDS 4 // In words for 8-byte data payload
#endif

#define FDCAN_MAX_STD_FILTERS 78
#define FDCAN_MAX_EXT_FILTERS 40
#define FDCAN_RX_FIFO0_ELEMENTS 8
#define FDCAN_TX_BUFFER_ELEMENTS 7

#define MAX_FILTER_LIST_SIZE 78U       // 78 element Standard Filter List elements or 40 element Extended Filter List
#define FDCAN_NUM_RXFIFO0_SIZE 108U    // 6 Frames
#define FDCAN_TX_FIFO_BUFFER_SIZE 126U // 7 Frames
#define MESSAGE_RAM_END_ADDR 0x4000B5FC
#define FDCAN_FRAME_BUFFER_SIZE 18

#undef MIN
#undef MAX
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

static uint32_t FDCAN_message_ram_offset = 0;

// --- Helper Functions & Structs ---
struct CAN_bit_timing_config_t
{
    uint16_t BRP;
    uint8_t TS1;
    uint8_t TS2;
    uint8_t SJW;
};

struct MessageRAM
{
    uint32_t StandardFilterSA;
    uint32_t ExtendedFilterSA;
    uint32_t RxFIFO0SA;
    uint32_t RxFIFO1SA;
    uint32_t TxFIFOQSA;
    uint32_t EndAddress;
} MessageRam_;

#if CANARD_ENABLE_CANFD
static const uint8_t dlc_to_len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static uint8_t dlcToLen(uint8_t dlc) { return dlc_to_len[dlc & 0x0F]; }
static uint8_t lenToDlc(uint8_t len)
{
    if (len <= 8)
        return len;
    if (len <= 12)
        return 9;
    if (len <= 16)
        return 10;
    if (len <= 20)
        return 11;
    if (len <= 24)
        return 12;
    if (len <= 32)
        return 13;
    if (len <= 48)
        return 14;
    return 15;
}
#else
static uint8_t dlcToLen(uint8_t dlc) { return dlc; }
static uint8_t lenToDlc(uint8_t len) { return len; }
#endif

// --- Timing Calculation (implementation details) ---
bool computeTimings(const uint32_t target_bitrate, Timings &out_timings)
{
    if (target_bitrate < 1)
    {
        return false;
    }

    /*
     * Hardware configuration
     */
    const uint32_t pclk = 80U * 1000U * 1000U;

    static const int MaxBS1 = 16;
    static const int MaxBS2 = 8;

    /*
     * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
     *      CAN in Automation, 2003
     *
     * According to the source, optimal quanta per bit are:
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;

    static const int MaxSamplePointLocation = 900;

    /*
     * Computing (prescaler * BS):
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
     * let:
     *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    const uint32_t prescaler_bs = pclk / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    uint8_t bs1_bs2_sum = uint8_t(max_quanta_per_bit - 1);

    while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0)
    {
        if (bs1_bs2_sum <= 2)
        {
            return false; // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U))
    {
        return false; // No solution
    }

    /*
     * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
     * We need to find the values so that the sample point is as close as possible to the optimal value.
     *
     *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
     *   {{bs2 -> (1 + bs1)/7}}
     *
     * Hence:
     *   bs2 = (1 + bs1) / 7
     *   bs1 = (7 * bs1_bs2_sum - 1) / 8
     *
     * Sample point location can be computed as follows:
     *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
     *
     * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
     *   - With rounding to nearest
     *   - With rounding to zero
     */
    struct BsPair
    {
        uint8_t bs1;
        uint8_t bs2;
        uint16_t sample_point_permill;

        BsPair() : bs1(0),
                   bs2(0),
                   sample_point_permill(0)
        {
        }

        BsPair(uint8_t bs1_bs2_sum, uint8_t arg_bs1) : bs1(arg_bs1),
                                                       bs2(uint8_t(bs1_bs2_sum - bs1)),
                                                       sample_point_permill(uint16_t(1000 * (1 + bs1) / (1 + bs1 + bs2)))
        {
        }

        bool isValid() const
        {
            return (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);
        }
    };

    // First attempt with rounding to nearest
    BsPair solution(bs1_bs2_sum, uint8_t(((7 * bs1_bs2_sum - 1) + 4) / 8));

    if (solution.sample_point_permill > MaxSamplePointLocation)
    {
        // Second attempt with rounding to zero
        solution = BsPair(bs1_bs2_sum, uint8_t((7 * bs1_bs2_sum - 1) / 8));
    }

    /*
     * Final validation
     * Helpful Python:
     * def sample_point_from_btr(x):
     *     assert 0b0011110010000000111111000000000 & x == 0
     *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
     *     return (1+ts1+1)/(1+ts1+1+ts2+1)
     *
     */
    if ((target_bitrate != (pclk / (prescaler * (1 + solution.bs1 + solution.bs2)))) || !solution.isValid())
    {
        return false;
    }

    out_timings.sample_point_permill = solution.sample_point_permill;
    out_timings.prescaler = uint16_t(prescaler);
    out_timings.sjw = 1;
    out_timings.bs1 = uint8_t(solution.bs1);
    out_timings.bs2 = uint8_t(solution.bs2);
    return true;
}

#if CANARD_ENABLE_CANFD
static bool compute_fd_timings(const uint32_t target_bitrate, CAN_bit_timing_config_t &out_timings)
{
    static const struct
    {
        uint32_t bitrate;
        uint8_t p;
        uint8_t bs1;
        uint8_t bs2;
        uint8_t sjw;
    } T[] = {
        {1000000, 4, 14, 5, 5},
        {2000000, 2, 14, 5, 5},
        {4000000, 1, 14, 5, 5},
        {5000000, 1, 11, 4, 4},
        {8000000, 1, 6, 3, 3},
    };
    for (size_t i = 0; i < sizeof(T) / sizeof(T[0]); ++i)
    {
        if (T[i].bitrate == target_bitrate)
        {
            out_timings.BRP = T[i].p;
            out_timings.TS1 = T[i].bs1;
            out_timings.TS2 = T[i].bs2;
            out_timings.SJW = T[i].sjw;
            return true;
        }
    }
    return false;
}
#endif

/**
 * @brief Configures a GPIO pin for an Alternate Function (e.g., for CAN).
 * @param addr  Pointer to the GPIO port (e.g., GPIOD).
 * @param index The pin number (0-15).
 * @param afry  The alternate function number to set (e.g., 9 for FDCAN1).
 * @param speed The GPIO speed (e.g., 3 for Very High Speed).
 */
void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed)
{
    // 1. Enable the clock for the given GPIO Port
    if (addr == GPIOA) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    else if (addr == GPIOB) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    else if (addr == GPIOC) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
    else if (addr == GPIOD) RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    else if (addr == GPIOE) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    else if (addr == GPIOF) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;
    else if (addr == GPIOG) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;
    else if (addr == GPIOH) RCC->AHB4ENR |= RCC_AHB4ENR_GPIOHEN;

    // 2. Set Pin Mode to Alternate Function (binary 10)
    addr->MODER &= ~(3U << (index * 2)); // Clear the two mode bits for the pin
    addr->MODER |=  (2U << (index * 2)); // Set mode to Alternate Function

    // 3. Set Output Type to Push-Pull (binary 0)
    addr->OTYPER &= ~(1U << index);

    // 4. Set Pin Speed
    addr->OSPEEDR &= ~(3U << (index * 2)); // Clear the two speed bits
    addr->OSPEEDR |=  (speed << (index * 2)); // Set the desired speed

    // 5. Set Pull-up/Pull-down to None (binary 00)
    // This is typical for CAN, as the transceiver handles the bus state.
    addr->PUPDR &= ~(3U << (index * 2));

    // 6. Set the Alternate Function number
    uint8_t afr_register_index = (index < 8) ? 0 : 1; // Pins 0-7 use AFR[0], 8-15 use AFR[1]
    uint8_t afr_shift = (index % 8) * 4;           // Each pin's AF setting is 4 bits wide

    addr->AFR[afr_register_index] &= ~(0xFU << afr_shift); // Clear the 4 bits for our pin
    addr->AFR[afr_register_index] |=  (afry << afr_shift); // Set the AF number (e.g., AF9)
}

void _CANSetFilter(uint8_t index, uint8_t fifo, uint32_t id, uint32_t mask)
{
    // Correct format for Standard ID Filter Element (SFE):
    // [31:30] SFT: Standard Filter Type (0b10 = Classic filter with ID + mask)
    // [29:27] SFEC: Standard Filter Element Configuration (0b001 = Store in FIFO 0)
    // [26:16] SFID1: Standard Filter ID 1
    // [15:0]  SFID2: Standard Filter ID 2 (used as the mask in a classic filter)

    uint32_t filter_element = (2U << 30) |                      // SFT: Set as "Classic" filter type
                              (((fifo == 0) ? 1U : 2U) << 27) | // SFEC: Store in the selected FIFO
                              ((id & 0x7FF) << 16) |            // SFID1: The filter ID
                              (mask & 0x7FF);                   // SFID2: The filter mask

    // Write the correctly formatted element to the Message RAM
    ((uint32_t *)MessageRam_.StandardFilterSA)[index] = filter_element;
}

void CANSetFilter(uint8_t index, uint8_t fifo, uint32_t id, uint32_t mask)
{
    if (index >= FDCAN_MAX_STD_FILTERS)
        return;

    _CANSetFilter(index, fifo, id, mask);
}

uint8_t CANMsgAvail(void) { return (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL); }

void CANSend(const CanardCANFrame *tx_frame)
{
    for (volatile int i = 0; (FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) && i < 100000; i++)
        ;
    if (FDCAN1->TXFQS & FDCAN_TXFQS_TFQF)
    {
#if DEBUG
        Serial.println("CANSend H7: TX FIFO full!");
#endif
        return;
    }

    uint32_t index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    uint32_t *buffer = (uint32_t *)(MessageRam_.TxFIFOQSA + (index * FDCAN_ELEMENT_SIZE_WORDS * 4));

    buffer[0] = (tx_frame->id & CANARD_CAN_FRAME_EFF) ? (1U << 30) | (tx_frame->id & CANARD_CAN_EXT_ID_MASK) : (tx_frame->id & CANARD_CAN_STD_ID_MASK) << 18;
    buffer[1] = (uint32_t)lenToDlc(tx_frame->data_len) << 16;

#if CANARD_ENABLE_CANFD
    if (tx_frame->canfd)
    {
        buffer[1] |= (1U << 21); // FDF
        buffer[1] |= (1U << 20); // BRS
    }
#endif

    memcpy(&buffer[2], tx_frame->data, tx_frame->data_len);
    FDCAN1->TXBAR = (1 << index);
#if DEBUG
    Serial.print("CANSend H7: Sent frame with ID 0x");
    Serial.println(tx_frame->id, HEX);
#endif
}

void CANReceive(CanardCANFrame *rx_frame)
{
    if (CANMsgAvail() == 0)
    {
        rx_frame->data_len = 0;
        return;
    }

    uint32_t index = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
    uint32_t *frame_ptr = (uint32_t *)(MessageRam_.RxFIFO0SA + (index * FDCAN_ELEMENT_SIZE_WORDS * 4));

    uint32_t r0 = frame_ptr[0], r1 = frame_ptr[1];
    rx_frame->id = (r0 & (1U << 30)) ? (r0 & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF : (r0 >> 18) & CANARD_CAN_STD_ID_MASK;
    uint8_t dlc = (r1 >> 16) & 0xF;
    rx_frame->data_len = dlcToLen(dlc);

#if CANARD_ENABLE_CANFD
    rx_frame->canfd = (r1 & (1U << 21)) != 0;
#endif

    memcpy(rx_frame->data, &frame_ptr[2], rx_frame->data_len);
    FDCAN1->RXF0A = index;
#if DEBUG
    Serial.print("CANReceive H7: Received frame with ID 0x");
    Serial.println(rx_frame->id, HEX);
#endif
}

#if CANARD_ENABLE_CANFD
bool CANInit(uint32_t nominal_bitrate, uint32_t data_bitrate, int remap)
{
#else
bool CANInit(uint32_t bitrate, int remap)
{
#endif

    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
    RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;

    if (remap == 2)
    {
        CANSetGpio(GPIOH, 14, 9, 3);
        CANSetGpio(GPIOD, 1, 9, 3);
    }

    FDCAN1->CCCR &= ~FDCAN_CCCR_CSR; // exit sleep mode
    uint32_t start_ms = millis();
    while ((FDCAN1->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA)
    {
        if (millis() - start_ms > 100)
        {
            return false;
        }
    } // wait for wake up ack

    FDCAN1->CCCR |= FDCAN_CCCR_INIT; // Request driver init
    start_ms = millis();
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0)
    {
        if (millis() - start_ms > 100)
        {
            return false;
        }
    }

    FDCAN1->CCCR |= FDCAN_CCCR_CCE; // Enable Config change
    FDCAN1->IE = 0;                 // disable interupts

    Timings timings;

    if (!computeTimings(bitrate, timings))
    {
        return false;
    }

    FDCAN1->NBTP = (((timings.sjw - 1) << FDCAN_NBTP_NSJW_Pos) |
                    ((timings.bs1 - 1) << FDCAN_NBTP_NTSEG1_Pos) |
                    ((timings.bs2 - 1) << FDCAN_NBTP_NTSEG2_Pos) |
                    ((timings.prescaler - 1) << FDCAN_NBTP_NBRP_Pos));

    // FDCAN1->RXESC = 0; // Set for 8Byte Frames

    FDCAN1->RXESC = 0x777; // Support up to 64-byte long frames
    FDCAN1->TXESC = 0x7;   // Support up to 64-byte long frames

    // --- Start of Message RAM Configuration ---
    // Make sure the offset is initialized to 0.
    uint32_t FDCANMessageRAMOffset_ = 0;

    // --- NEW (THE FIX) ---
    // 1. Configure Global Filter: Reject non-matching frames.
    FDCAN1->GFC = (0x3U << 4) | (0x3U << 2);

    // 2. Configure Standard ID Filter List
    uint32_t num_std_filters = FDCAN_MAX_STD_FILTERS; // Let's allocate for the max
    FDCAN1->SIDFC = (FDCANMessageRAMOffset_ << 2) | (num_std_filters << 16);
    MessageRam_.StandardFilterSA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
    FDCANMessageRAMOffset_ += num_std_filters;
    // --- END NEW ---

    // 3. Rx FIFO 0 start address and element count
    uint32_t num_elements = MIN((FDCAN_NUM_RXFIFO0_SIZE / FDCAN_FRAME_BUFFER_SIZE), 64U);
    if (num_elements)
    {
        FDCAN1->RXF0C = (FDCANMessageRAMOffset_ << 2) | (num_elements << 16);
        MessageRam_.RxFIFO0SA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_elements * FDCAN_FRAME_BUFFER_SIZE;
    }

    // 4. Tx FIFO/queue start address and element count
    num_elements = MIN((FDCAN_TX_FIFO_BUFFER_SIZE / FDCAN_FRAME_BUFFER_SIZE), 32U);
    if (num_elements)
    {
        FDCAN1->TXBC = (FDCANMessageRAMOffset_ << 2) | (num_elements << 24);
        MessageRam_.TxFIFOQSA = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
        FDCANMessageRAMOffset_ += num_elements * FDCAN_FRAME_BUFFER_SIZE;
    }

    // 5. Check for RAM overflow
    MessageRam_.EndAddress = SRAMCAN_BASE + (FDCANMessageRAMOffset_ * 4U);
    if (MessageRam_.EndAddress > MESSAGE_RAM_END_ADDR)
    {
        return false;
    }

    // Clear all Interrupts
    FDCAN1->IR = 0x3FFFFFFF;

    CANSetFilter(0, 0, 0x0, 0x0); // Default accept-all filter

    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    start_ms = millis();
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 1)
    {
        FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
        if (millis() - start_ms > 1000)
        {
            Serial.println("------------------------------------");
            Serial.println("--- FDCAN Register Dump ---");

            // Core & Control Registers
            Serial.print("CCCR (Control): 0x");
            Serial.println(FDCAN1->CCCR, HEX);
            Serial.print("NBTP (Nominal Timing): 0x");
            Serial.println(FDCAN1->NBTP, HEX);

            // Status & Error Registers
            Serial.print("ECR (Error Counter): 0x");
            Serial.println(FDCAN1->ECR, HEX);
            Serial.print("PSR (Protocol Status): 0x");
            Serial.println(FDCAN1->PSR, HEX);

            // Message RAM Configuration (verify our writes)
            Serial.print("GFC (Global Filter): 0x");
            Serial.println(FDCAN1->GFC, HEX);
            Serial.print("SIDFC (Std ID Filter): 0x");
            Serial.println(FDCAN1->SIDFC, HEX);
            Serial.print("RXF0C (Rx FIFO 0 Cfg): 0x");
            Serial.println(FDCAN1->RXF0C, HEX);
            Serial.print("TXBC (Tx Buffer Cfg): 0x");
            Serial.println(FDCAN1->TXBC, HEX);

            // Interrupt Configuration
            Serial.print("IE (Interrupt Enable): 0x");
            Serial.println(FDCAN1->IE, HEX);
            Serial.print("ILS (Interrupt Line Sel): 0x");
            Serial.println(FDCAN1->ILS, HEX);
            Serial.println("------------------------------------");
            Serial.flush(); // Ensure the data is sent before the function returns false
            return false;
        }
    }

    return true;
}

bool CANInit(CAN_BITRATE bitrate, int remap)
{
    switch (bitrate)
    {

// FD rates
#if CANARD_ENABLE_CANFD
    case CAN_FD_500KBPS_2000KBPS:
        return CANInit(500000, 2000000, remap);
    case CAN_FD_1000KBPS_2000KBPS:
        return CANInit(1000000, 2000000, remap);
    case CAN_FD_1000KBPS_4000KBPS:
        return CANInit(1000000, 4000000, remap);
    case CAN_FD_1000KBPS_8000KBPS:
        return CANInit(1000000, 8000000, remap);
    default:
        return false; // Or handle error appropriately
#else
    case CAN_50KBPS:
        return CANInit(50000, remap);
    case CAN_100KBPS:
        return CANInit(100000, remap);
    case CAN_125KBPS:
        return CANInit(125000, remap);
    case CAN_250KBPS:
        return CANInit(250000, remap);
    case CAN_500KBPS:
        return CANInit(500000, remap);
    case CAN_1000KBPS:
        return CANInit(1000000, remap);
    default:
        return false; // Or handle error appropriately
#endif
    }
}
#endif
