#ifndef DRONECAN_STORAGE_H
#define DRONECAN_STORAGE_H

#include <stdint.h>
#include <stddef.h>

/*
    Flash-based parameter storage.

    Uses the last page of internal flash. The layout is:

        [float param_0] [float param_1] ... [float param_N-1] [uint32_t MAGIC]

    padded to the flash write alignment of the target MCU (32 bytes on H7).
    Because flash can only be written once after an erase, every save
    erases the page and rewrites the full parameter set.

    STORAGE_FLASH_PAGE can be defined at build time to override the default
    (last page).
*/
class DroneCAN_Storage
{
public:
    // Load parameter values from persistent storage into the given array.
    // Returns true if the storage contained valid data, false otherwise
    // (in which case the array is left untouched so code defaults are kept).
    static bool load(float *values, size_t count);

    // Persist a single parameter value at the given index.
    // Internally this rewrites the entire page (reads current values,
    // patches the one that changed, erases, writes).
    static void save(size_t index, float value, size_t total_count);

    // Persist all parameter values at once.
    static void save_all(const float *values, size_t count);

private:
    static constexpr uint32_t STORAGE_MAGIC = 0x4443414E; // "DCAN"

    // Minimum write granularity — must match the target MCU.
    // H7 = 32 bytes, F4 = 2 bytes, G4/L4 = 8 bytes.  We round up
    // to the largest so the same logic works everywhere.
    static constexpr size_t WRITE_ALIGN = 32;

    // Return the flash page used for storage.
    static uint32_t storage_page();

    // Round a byte count up to the next WRITE_ALIGN boundary.
    static size_t align_up(size_t n);

    // Write the full blob (params + magic) to the storage page.
    // Erases first, then writes.  Returns true on success.
    static bool write_page(const float *values, size_t count);
};

#endif // DRONECAN_STORAGE_H
