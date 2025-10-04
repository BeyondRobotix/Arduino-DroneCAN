#ifdef CANH7
#include "canH7.h"
#include <string.h> // For memset and memcpy
#include "Arduino.h"

#define DEBUG 1

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

static uint32_t FDCAN_message_ram_offset = 0;
struct
{
    uint32_t StandardFilterSA;
    uint32_t ExtendedFilterSA;
    uint32_t RxFIFO0SA;
    uint32_t TxFIFOQSA;
} MessageRam;

// --- Helper Functions & Structs ---
struct CAN_bit_timing_config_t
{
    uint16_t BRP;
    uint8_t TS1;
    uint8_t TS2;
    uint8_t SJW;
};

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
static bool compute_timings(const uint32_t target_bitrate, CAN_bit_timing_config_t &out_timings)
{
    if (target_bitrate < 1)
        return false;
    const uint32_t pclk = 80000000; // Assuming 80MHz FDCAN clock
    const int max_quanta_per_bit = (target_bitrate >= 1000000) ? 10 : 17;
    const uint32_t prescaler_bs = pclk / target_bitrate;
    for (uint8_t bs = max_quanta_per_bit; bs > 2; bs--)
    {
        if ((prescaler_bs % bs) == 0)
        {
            const uint32_t prescaler = prescaler_bs / bs;
            if ((prescaler > 0) && (prescaler <= 512))
            {
                uint8_t bs1 = (bs * 7) / 8 - 1;
                uint8_t bs2 = bs - bs1 - 1;
                if (bs1 > 0 && bs1 <= 255 && bs2 > 0 && bs2 <= 128)
                {
                    out_timings.BRP = (uint16_t)prescaler;
                    out_timings.TS1 = bs1;
                    out_timings.TS2 = bs2;
                    out_timings.SJW = bs2 > 1 ? bs2 / 2 : 1;
                    return true;
                }
            }
        }
    }
    return false;
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

// --- Driver Functions ---
void CANSetGpio(GPIO_TypeDef *addr, uint8_t index, uint8_t afry, uint8_t speed)
{
    if (addr == GPIOA)
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    else if (addr == GPIOB)
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    else if (addr == GPIOC)
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
    else if (addr == GPIOD)
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
    else if (addr == GPIOE)
        RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    uint32_t moder = addr->MODER;
    moder &= ~(3U << (index * 2));
    moder |= (2U << (index * 2));
    addr->MODER = moder;
    addr->OTYPER &= ~(1U << index);
    uint32_t ospeedr = addr->OSPEEDR;
    ospeedr &= ~(3U << (index * 2));
    ospeedr |= (speed << (index * 2));
    addr->OSPEEDR = ospeedr;
    addr->PUPDR &= ~(3U << (index * 2));
    uint8_t afr_idx = (index < 8) ? 0 : 1;
    uint32_t afr_shift = (index % 8) * 4;
    uint32_t afr = addr->AFR[afr_idx];
    afr &= ~(0xFU << afr_shift);
    afr |= (afry << afr_shift);
    addr->AFR[afr_idx] = afr;
}

void _CANSetFilter(uint8_t index, uint8_t fifo, uint32_t id, uint32_t mask)
{
    uint32_t filter_element = (((fifo == 0) ? 1U : 2U) << 30) | (2U << 27) | ((id & 0x7FF) << 16) | (mask & 0x7FF);
    ((uint32_t *)MessageRam.StandardFilterSA)[index] = filter_element;
}

void CANSetFilter(uint8_t index, uint8_t fifo, uint32_t id, uint32_t mask)
{
    if (index >= FDCAN_MAX_STD_FILTERS)
        return;
#if DEBUG
    Serial.print("CANSetFilter H7: index=");
    Serial.print(index);
    Serial.print(" fifo=");
    Serial.print(fifo);
    Serial.print(" id=0x");
    Serial.print(id, HEX);
    Serial.print(" mask=0x");
    Serial.println(mask, HEX);
#endif
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    uint32_t start_ms = millis();
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT))
    {
        if (millis() - start_ms > 100) {
            return; // Timeout
        }
    }
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    _CANSetFilter(index, fifo, id, mask);

    FDCAN1->CCCR &= ~FDCAN_CCCR_CCE;
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    start_ms = millis();
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT))
    {
        if (millis() - start_ms > 100) {
            return; // Timeout
        }
    }
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
    uint32_t *buffer = (uint32_t *)(MessageRam.TxFIFOQSA + (index * FDCAN_ELEMENT_SIZE_WORDS * 4));

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
    uint32_t *frame_ptr = (uint32_t *)(MessageRam.RxFIFO0SA + (index * FDCAN_ELEMENT_SIZE_WORDS * 4));

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
#if DEBUG
    Serial.println("CANInit H7");
#endif

    RCC->APB1HENR |= RCC_APB1HENR_FDCANEN;
    RCC->APB1HRSTR |= RCC_APB1HRSTR_FDCANRST;
    RCC->APB1HRSTR &= ~RCC_APB1HRSTR_FDCANRST;

    if (remap == 0)
    {
        CANSetGpio(GPIOA, 11, 9);
        CANSetGpio(GPIOA, 12, 9);

    }
    else if (remap == 1)
    {
        CANSetGpio(GPIOH, 14, 9);
        CANSetGpio(GPIOD, 1, 9);
    }

    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    uint32_t start_ms = millis();
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT))
    {
        if (millis() - start_ms > 100) {
#if DEBUG
            Serial.println("Failed to enter init mode");
#endif
            return false;
        }
    }
#if DEBUG
    Serial.println("Entered init mode.");
#endif

#if CANARD_ENABLE_CANFD
    FDCAN1->CCCR |= FDCAN_CCCR_CCE | FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;
    CAN_bit_timing_config_t d_timings;
    if (!compute_fd_timings(data_bitrate, d_timings))
    {
#if DEBUG
        Serial.println("Failed to compute data timings");
#endif
        return false;
    }
    FDCAN1->DBTP = ((d_timings.SJW - 1) << 16) | ((d_timings.TS1 - 1) << 8) | ((d_timings.TS2 - 1) << 4) | (d_timings.BRP - 1);
    FDCAN1->TDCR = 10 << FDCAN_TDCR_TDCO_Pos; // Transmitter Delay Compensation
#if DEBUG
    printRegister("DBTP: ", FDCAN1->DBTP);
#endif
#else
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
#endif

    CAN_bit_timing_config_t n_timings;
#if CANARD_ENABLE_CANFD
    if (!compute_timings(nominal_bitrate, n_timings))
    {
#if DEBUG
        Serial.println("Failed to compute nominal timings");
#endif
        return false;
    }
#else
    if (!compute_timings(bitrate, n_timings))
    {
#if DEBUG
        Serial.println("Failed to compute timings");
#endif
        return false;
    }
#endif
    FDCAN1->NBTP = ((n_timings.SJW - 1) << 25) | ((n_timings.TS1 - 1) << 16) | ((n_timings.TS2 - 1) << 8) | (n_timings.BRP - 1);
#if DEBUG
    printRegister("NBTP: ", FDCAN1->NBTP);
    Serial.println("Bit timings set.");
#endif

#if CANARD_ENABLE_CANFD
    FDCAN1->RXESC = 0b111;
    FDCAN1->TXESC = 0b111; // 64 byte payloads
#endif

    memset(&MessageRam, 0, sizeof(MessageRam));
    FDCAN_message_ram_offset = 0;
    FDCAN1->SIDFC = (FDCAN_message_ram_offset << 2) | (FDCAN_MAX_STD_FILTERS << 16);
    MessageRam.StandardFilterSA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
    FDCAN_message_ram_offset += FDCAN_MAX_STD_FILTERS * 2;
    FDCAN1->XIDFC = (FDCAN_message_ram_offset << 2) | (FDCAN_MAX_EXT_FILTERS << 16);
    MessageRam.ExtendedFilterSA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
    FDCAN_message_ram_offset += FDCAN_MAX_EXT_FILTERS * 2;
    FDCAN1->RXF0C = (FDCAN_message_ram_offset << 2) | (FDCAN_RX_FIFO0_ELEMENTS << 16);
    MessageRam.RxFIFO0SA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
    FDCAN_message_ram_offset += FDCAN_RX_FIFO0_ELEMENTS * FDCAN_ELEMENT_SIZE_WORDS;
    FDCAN1->TXBC = (FDCAN_message_ram_offset << 2) | (FDCAN_TX_BUFFER_ELEMENTS << 24);
    MessageRam.TxFIFOQSA = SRAMCAN_BASE + (FDCAN_message_ram_offset * 4U);
#if DEBUG
    Serial.println("Message RAM configured.");
#endif

    FDCAN1->GFC = (0x02 << 4) | 0x01; // Accept non-matching standard into FIFO0, reject extended
    _CANSetFilter(0, 0, 0x0, 0x0);     // Default accept-all filter
#if DEBUG
    Serial.println("Default filter set.");
#endif

    FDCAN1->CCCR &= ~(FDCAN_CCCR_CCE | FDCAN_CCCR_INIT);
#if DEBUG
    Serial.println("Attempting to exit init mode...");
#endif
    start_ms = millis();
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT))
    {
        FDCAN1->CCCR &= ~(FDCAN_CCCR_CCE | FDCAN_CCCR_INIT);
        if (millis() - start_ms > 100) {
#if DEBUG
            Serial.println("CANInit H7 failed to exit init mode.");
#endif
            return false;
        }
    }
    
    bool success = !(FDCAN1->CCCR & FDCAN_CCCR_INIT);
#if DEBUG
    if (success)
    {
        Serial.println("CANInit H7 success.");
    }
    else
    {
        Serial.println("CANInit H7 failed.");
    }
#endif
    return success;
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
