# CPU Temperature Example

This is the simplest example and a good starting point for a new node. It reads the STM32's internal temperature sensor and ADC pins, then broadcasts a DroneCAN `BatteryInfo` message at 10Hz containing:

- `voltage` — raw ADC reading from PA1
- `current` — raw ADC reading from PA0
- `temperature` — MCU core temperature in degrees Celsius

It also demonstrates reading and writing parameters via `dronecan.getParameter()` and `dronecan.setParameter()`.

## Dependencies

No extra libraries required. Uses the STM32 internal ADC (`AVREF`, `ATEMP`) which is available on all supported boards.
