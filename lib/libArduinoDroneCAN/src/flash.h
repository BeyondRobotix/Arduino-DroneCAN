#ifndef DRONECAN_FLASH_H
#define DRONECAN_FLASH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Get the memory-mapped address of a flash page
uint32_t stm32_flash_getpageaddr(uint32_t page);

// Get the size in bytes of a flash page
uint32_t stm32_flash_getpagesize(uint32_t page);

// Get total number of flash pages
uint32_t stm32_flash_getnumpages(void);

// Erase a flash page. Returns true on success.
bool stm32_flash_erasepage(uint32_t page);

// Write to flash. On H7 addr and count must be 32-byte aligned.
// Returns true on success.
bool stm32_flash_write(uint32_t addr, const void *buf, uint32_t count);

// Check if a page is fully erased (all 0xFF)
bool stm32_flash_ispageerased(uint32_t page);

#ifdef __cplusplus
}
#endif

#endif // DRONECAN_FLASH_H
