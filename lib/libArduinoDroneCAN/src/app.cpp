
#include "app.h"

/*
 *   This does everything we need to allow the app to start from the bootloader
 *   If your app is bootloaderless, you don't need this. You'll need to set your program start to 0x08000000 in the linker in ldscript.ld though
 *
 *   Why we need to re-set SCB->VTOR: Arduino's per-family SystemInit()
 *   unconditionally sets VTOR back to FLASH_BASE (0x08000000), which is the
 *   bootloader's vector table when a bootloader is installed. Without
 *   overriding it here, the first interrupt the app handles dispatches
 *   through the bootloader's table -> wrong handler -> crash.
 *
 *   Must match the bootloader-aware ldscript FLASH origin and the bootloader's
 *   APP_START_ADDRESS in BR_bootloader/src/board_config.h.
 */
#if defined(CANL431)
#define APP_BASE_ADDRESS 0x0800A000UL   /* L4: 40 KB after flash start */
#elif defined(CANH7) || defined(ARDUINO_NUCLEO_H723ZG)
#define APP_BASE_ADDRESS 0x08020000UL   /* H7: sector-1 boundary (128 KB) */
#endif

void app_setup()
{
#ifndef DISABLE_APP_SETUP

#ifdef CANL431
    SCB->VTOR = APP_BASE_ADDRESS;
#endif // CANL431

#if defined(CANH7) || defined(ARDUINO_NUCLEO_H723ZG)
    SCB->VTOR = APP_BASE_ADDRESS;

#endif // CANH7 || ARDUINO_NUCLEO_H723ZG

#endif // DISABLE_APP_SETUP
}
