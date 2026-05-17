# INA239 Current Sensor

Reads voltage, current, and temperature from an INA239 SPI current/power monitor and broadcasts them as a DroneCAN `BatteryInfo` message at 10Hz.

The INA239 is initialised with a 10A max current range and a 15mΩ shunt resistor — adjust the `INA.setMaxCurrentShunt()` call in `main.cpp` to match your hardware.

> **Note:** This example is untested. It is provided as an illustration of how to integrate an SPI sensor with the DroneCAN library.

## Dependencies

Install the INA239 library by RobTillaart:
https://github.com/RobTillaart/INA239

Download the zip and place the unzipped folder in the `lib` directory of your project. Make sure there are no nested folders inside (this can happen when unzipping from GitHub).

Alternatively, install it via the PlatformIO library manager.
