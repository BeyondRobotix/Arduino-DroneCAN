/************************************************************************************
 * Adapted from ArduPilot flash driver (originally by Uros Platise, Andrew Tridgell,
 * and Siddharth Bharat Purohit) for use in Arduino-DroneCAN.
 *
 * Stripped to essentials: erase, write, and query — no ChibiOS dependencies.
 * Uses Arduino noInterrupts()/interrupts() for ISR protection.
 ************************************************************************************/

#include "flash.h"
#include <Arduino.h>
#include <string.h>

// ---- Map stm32duino family defines to shorter ArduPilot-style names ------------
// stm32duino defines STM32H7xx, STM32L4xx, STM32F4xx, STM32G4xx etc.
#if defined(STM32H7xx) && !defined(STM32H7)
#define STM32H7
#endif
#if defined(STM32L4xx) && !defined(STM32L4)
#define STM32L4
#endif
#if defined(STM32F4xx) && !defined(STM32F4)
#define STM32F4
#endif
#if defined(STM32G4xx) && !defined(STM32G4)
#define STM32G4
#endif

#ifndef BOARD_FLASH_SIZE
#error "You must define BOARD_FLASH_SIZE in kbytes (e.g. -DBOARD_FLASH_SIZE=256)"
#endif

#define KB(x) ((x) * 1024)
#define STM32_FLASH_BASE 0x08000000
#define STM32_FLASH_SIZE KB(BOARD_FLASH_SIZE)

// ---- Page layout per MCU family ------------------------------------------------

#if defined(STM32H7)
#define STM32_FLASH_NPAGES      (BOARD_FLASH_SIZE / 128)
#define STM32_FLASH_FIXED_PAGE_SIZE 128   // KB per page
#define STM32_FLASH_NBANKS      (BOARD_FLASH_SIZE / 1024)
#define STM32_FLASH_FIXED_PAGE_PER_BANK (1024 / STM32_FLASH_FIXED_PAGE_SIZE)

#elif defined(STM32F4)
  #if BOARD_FLASH_SIZE == 512
    #define STM32_FLASH_NPAGES 8
    static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = {
        KB(16), KB(16), KB(16), KB(16), KB(64), KB(128), KB(128), KB(128)};
  #elif BOARD_FLASH_SIZE == 1024
    #define STM32_FLASH_NPAGES 12
    static const uint32_t flash_memmap[STM32_FLASH_NPAGES] = {
        KB(16), KB(16), KB(16), KB(16), KB(64),
        KB(128), KB(128), KB(128), KB(128), KB(128), KB(128), KB(128)};
  #else
    #error "Unsupported BOARD_FLASH_SIZE for STM32F4"
  #endif

#elif defined(STM32L4)
#define STM32_FLASH_NPAGES      (BOARD_FLASH_SIZE / 2)
#define STM32_FLASH_FIXED_PAGE_SIZE 2
#define STM32_FLASH_NBANKS      2

#elif defined(STM32G4)
#define STM32_FLASH_NPAGES      (BOARD_FLASH_SIZE / 2)
#define STM32_FLASH_FIXED_PAGE_SIZE 2
#define STM32_FLASH_NBANKS      2

#else
#error "Unsupported MCU family for flash.cpp — define STM32H7, STM32L4, STM32F4, or STM32G4"
#endif

#ifndef STM32_FLASH_NBANKS
#define STM32_FLASH_NBANKS 2
#endif

// ---- Low-level register helpers ------------------------------------------------

static inline uint16_t getreg16(unsigned int addr)
{
    uint16_t retval;
    __asm__ __volatile__("\tldrh %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
    return retval;
}

static inline void putreg16(uint16_t val, unsigned int addr)
{
    __asm__ __volatile__("\tstrh %0, [%1]\n\t" : : "r"(val), "r"(addr));
}

static inline uint32_t getreg32(unsigned int addr)
{
    uint32_t retval;
    __asm__ __volatile__("\tldr %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
    return retval;
}

static inline void putreg32(uint32_t val, unsigned int addr)
{
    *(volatile uint32_t *)(addr) = val;
}

// ---- Flash state helpers -------------------------------------------------------

#ifndef FLASH_KEY1
#define FLASH_KEY1 0x45670123
#endif
#ifndef FLASH_KEY2
#define FLASH_KEY2 0xCDEF89AB
#endif

static void stm32_flash_wait_idle(void)
{
    __DSB();
#if defined(STM32H7)
    while ((FLASH->SR1 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE))
#if STM32_FLASH_NBANKS > 1
           || (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE))
#endif
    ) {
        // wait
    }
#else
    while (FLASH->SR & FLASH_SR_BSY) {
        // wait
    }
#endif
}

static void stm32_flash_clear_errors(void)
{
#if defined(STM32H7)
    FLASH->CCR1 = ~0;
#if STM32_FLASH_NBANKS > 1
    FLASH->CCR2 = ~0;
#endif
#else
    FLASH->SR = 0xF3;
#endif
}

static void stm32_flash_unlock(void)
{
    stm32_flash_wait_idle();
#if defined(STM32H7)
    if (FLASH->CR1 & FLASH_CR_LOCK) {
        FLASH->KEYR1 = FLASH_KEY1;
        FLASH->KEYR1 = FLASH_KEY2;
    }
#if STM32_FLASH_NBANKS > 1
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        FLASH->KEYR2 = FLASH_KEY1;
        FLASH->KEYR2 = FLASH_KEY2;
    }
#endif
#else
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
#endif

#ifdef FLASH_ACR_DCEN
    FLASH->ACR &= ~FLASH_ACR_DCEN;
#endif
}

static void stm32_flash_lock(void)
{
#if defined(STM32H7)
    if (FLASH->SR1 & FLASH_SR_QW) {
        FLASH->CR1 |= FLASH_CR_FW;
    }
#if STM32_FLASH_NBANKS > 1
    if (FLASH->SR2 & FLASH_SR_QW) {
        FLASH->CR2 |= FLASH_CR_FW;
    }
#endif
    stm32_flash_wait_idle();
    FLASH->CR1 |= FLASH_CR_LOCK;
#if STM32_FLASH_NBANKS > 1
    FLASH->CR2 |= FLASH_CR_LOCK;
#endif
#else
    stm32_flash_wait_idle();
    FLASH->CR |= FLASH_CR_LOCK;
#endif

#ifdef FLASH_ACR_DCEN
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
#endif
}

// ---- Page address / size helpers (non-uniform F4 support) ----------------------

#ifndef STM32_FLASH_FIXED_PAGE_SIZE
static uint32_t flash_pageaddr[STM32_FLASH_NPAGES];
static bool flash_pageaddr_initialised;
#endif

uint32_t stm32_flash_getpageaddr(uint32_t page)
{
    if (page >= STM32_FLASH_NPAGES) {
        return 0;
    }
#if defined(STM32_FLASH_FIXED_PAGE_SIZE)
    return STM32_FLASH_BASE + page * STM32_FLASH_FIXED_PAGE_SIZE * 1024;
#else
    if (!flash_pageaddr_initialised) {
        uint32_t address = STM32_FLASH_BASE;
        for (uint8_t i = 0; i < STM32_FLASH_NPAGES; i++) {
            flash_pageaddr[i] = address;
            address += stm32_flash_getpagesize(i);
        }
        flash_pageaddr_initialised = true;
    }
    return flash_pageaddr[page];
#endif
}

uint32_t stm32_flash_getpagesize(uint32_t page)
{
#if defined(STM32_FLASH_FIXED_PAGE_SIZE)
    (void)page;
    return STM32_FLASH_FIXED_PAGE_SIZE * 1024;
#else
    return flash_memmap[page];
#endif
}

uint32_t stm32_flash_getnumpages(void)
{
    return STM32_FLASH_NPAGES;
}

bool stm32_flash_ispageerased(uint32_t page)
{
    if (page >= STM32_FLASH_NPAGES) {
        return false;
    }
    uint32_t addr = stm32_flash_getpageaddr(page);
    uint32_t count = stm32_flash_getpagesize(page);
    for (; count; count -= 4, addr += 4) {
        if (getreg32(addr) != 0xFFFFFFFF) {
            return false;
        }
    }
    return true;
}

// ---- Erase ---------------------------------------------------------------------

bool stm32_flash_erasepage(uint32_t page)
{
    if (page >= STM32_FLASH_NPAGES) {
        return false;
    }

    noInterrupts();
    stm32_flash_wait_idle();
    stm32_flash_unlock();
    stm32_flash_clear_errors();

#if defined(STM32H7)
    if (page < STM32_FLASH_FIXED_PAGE_PER_BANK) {
        FLASH->SR1 = ~0;
        stm32_flash_wait_idle();
#ifdef FLASH_CR_PSIZE_1
        FLASH->CR1 = FLASH_CR_PSIZE_1 | (page << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
#else
        FLASH->CR1 = (page << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
#endif
        FLASH->CR1 |= FLASH_CR_START;
        while (FLASH->SR1 & FLASH_SR_QW)
            ;
    }
#if STM32_FLASH_NBANKS > 1
    else {
        FLASH->SR2 = ~0;
        stm32_flash_wait_idle();
#ifdef FLASH_CR_PSIZE_1
        FLASH->CR2 = FLASH_CR_PSIZE_1 | ((page - STM32_FLASH_FIXED_PAGE_PER_BANK) << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
#else
        FLASH->CR2 = ((page - STM32_FLASH_FIXED_PAGE_PER_BANK) << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
#endif
        FLASH->CR2 |= FLASH_CR_START;
        while (FLASH->SR2 & FLASH_SR_QW)
            ;
    }
#endif // NBANKS > 1

#elif defined(STM32F4)
    uint8_t snb = (((page % 12) << 3) | ((page / 12) << 7));
    FLASH->CR = FLASH_CR_PSIZE_1 | snb | FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;

#elif defined(STM32G4) || defined(STM32L4)
    FLASH->CR = FLASH_CR_PER;
    FLASH->CR |= page << FLASH_CR_PNB_Pos;
    FLASH->CR |= FLASH_CR_STRT;

#else
#error "Unsupported MCU for flash erase"
#endif

    stm32_flash_wait_idle();

#if defined(STM32H7)
    SCB_InvalidateDCache_by_Addr((void *)stm32_flash_getpageaddr(page),
                                  stm32_flash_getpagesize(page));
#endif

    stm32_flash_lock();
    interrupts();

    return stm32_flash_ispageerased(page);
}

// ---- Write (MCU-specific) ------------------------------------------------------

#if defined(STM32H7)

static bool stm32h7_check_all_ones(uint32_t addr, uint32_t words)
{
    for (uint32_t i = 0; i < words; i++) {
        if (getreg32(addr) != 0xFFFFFFFF) {
            return false;
        }
        addr += 4;
    }
    return true;
}

static bool stm32h7_flash_write32(uint32_t addr, const void *buf)
{
    volatile uint32_t *CR = &FLASH->CR1;
    volatile uint32_t *CCR = &FLASH->CCR1;
    volatile uint32_t *SR = &FLASH->SR1;
#if STM32_FLASH_NBANKS > 1
    if (addr - STM32_FLASH_BASE >= STM32_FLASH_FIXED_PAGE_PER_BANK * STM32_FLASH_FIXED_PAGE_SIZE * 1024) {
        CR = &FLASH->CR2;
        CCR = &FLASH->CCR2;
        SR = &FLASH->SR2;
    }
#endif
    stm32_flash_wait_idle();
    *CCR = ~0;
    *CR |= FLASH_CR_PG;

    const uint32_t *v = (const uint32_t *)buf;
    for (uint8_t i = 0; i < 8; i++) {
        while (*SR & (FLASH_SR_BSY | FLASH_SR_QW))
            ;
        putreg32(*v, addr);
        v++;
        addr += 4;
    }
    __DSB();
    stm32_flash_wait_idle();
    *CCR = ~0;
    *CR &= ~FLASH_CR_PG;
    return true;
}

static bool stm32_flash_write_h7(uint32_t addr, const void *buf, uint32_t count)
{
    uint8_t *b = (uint8_t *)buf;

    // H7 requires 256-bit (32-byte) aligned writes
    if ((count & 0x1F) || (addr & 0x1F)) {
        return false;
    }

    stm32_flash_unlock();
    bool success = true;

    while (count >= 32) {
        if (memcmp((void *)addr, b, 32) != 0) {
            if (!stm32h7_check_all_ones(addr, 8)) {
                success = false;
                break;
            }

            noInterrupts();
            bool ok = stm32h7_flash_write32(addr, b);
            interrupts();

            if (!ok) {
                success = false;
                break;
            }
            SCB_InvalidateDCache_by_Addr((void *)addr, 32);
            if (memcmp((void *)addr, b, 32) != 0) {
                success = false;
                break;
            }
        }
        addr += 32;
        count -= 32;
        b += 32;
    }

    stm32_flash_lock();
    return success;
}

#endif // STM32H7

#if defined(STM32F4)
static bool stm32_flash_write_f4(uint32_t addr, const void *buf, uint32_t count)
{
    uint8_t *b = (uint8_t *)buf;
    if (count & 1) {
        return false;
    }
    if ((addr + count) > STM32_FLASH_BASE + STM32_FLASH_SIZE) {
        return false;
    }

    noInterrupts();
    stm32_flash_unlock();
    stm32_flash_clear_errors();
    stm32_flash_wait_idle();

    while (count >= 4 && (addr & 3) == 0) {
        FLASH->CR &= ~(FLASH_CR_PSIZE);
        FLASH->CR |= FLASH_CR_PSIZE_1 | FLASH_CR_PG;
        uint32_t v1 = *(uint32_t *)b;
        putreg32(v1, addr);
        __DSB();
        stm32_flash_wait_idle();
        if (getreg32(addr) != v1) {
            FLASH->CR &= ~(FLASH_CR_PG);
            stm32_flash_lock();
            interrupts();
            return false;
        }
        count -= 4;
        b += 4;
        addr += 4;
    }

    while (count >= 2) {
        FLASH->CR &= ~(FLASH_CR_PSIZE);
        FLASH->CR |= FLASH_CR_PSIZE_0 | FLASH_CR_PG;
        putreg16(*(uint16_t *)b, addr);
        __DSB();
        stm32_flash_wait_idle();
        if (getreg16(addr) != *(uint16_t *)b) {
            FLASH->CR &= ~(FLASH_CR_PG);
            stm32_flash_lock();
            interrupts();
            return false;
        }
        count -= 2;
        b += 2;
        addr += 2;
    }

    FLASH->CR &= ~(FLASH_CR_PG);
    stm32_flash_lock();
    interrupts();
    return true;
}
#endif // STM32F4

#if defined(STM32G4) || defined(STM32L4)
static bool stm32_flash_write_g4(uint32_t addr, const void *buf, uint32_t count)
{
    uint32_t *b = (uint32_t *)buf;
    if ((count & 7) || (addr & 7)) {
        return false;
    }
    if ((addr + count) > STM32_FLASH_BASE + STM32_FLASH_SIZE) {
        return false;
    }

    // skip already-programmed double-words
    while (count >= 8 && getreg32(addr) == b[0] && getreg32(addr + 4) == b[1]) {
        count -= 8;
        addr += 8;
        b += 2;
    }
    if (count == 0) {
        return true;
    }

    noInterrupts();
    stm32_flash_unlock();
    stm32_flash_wait_idle();

    while (count >= 8) {
        FLASH->CR = FLASH_CR_PG;
        putreg32(b[0], addr);
        putreg32(b[1], addr + 4);
        stm32_flash_wait_idle();
        FLASH->SR |= FLASH_SR_EOP;
        FLASH->CR = 0;
        if (getreg32(addr) != b[0] || getreg32(addr + 4) != b[1]) {
            stm32_flash_lock();
            interrupts();
            return false;
        }
        count -= 8;
        b += 2;
        addr += 8;
    }

    stm32_flash_lock();
    interrupts();
    return true;
}
#endif // STM32G4 || STM32L4

// ---- Public write dispatch -----------------------------------------------------

bool stm32_flash_write(uint32_t addr, const void *buf, uint32_t count)
{
#if defined(STM32H7)
    return stm32_flash_write_h7(addr, buf, count);
#elif defined(STM32F4)
    return stm32_flash_write_f4(addr, buf, count);
#elif defined(STM32G4) || defined(STM32L4)
    return stm32_flash_write_g4(addr, buf, count);
#else
#error "Unsupported MCU for flash write"
#endif
}
