# Thermocouple MCP9600

Integrates an MCP9600 I2C thermocouple amplifier as a DroneCAN node. The node is configured for a K-type thermocouple and broadcasts:

- `Temperature` message at 1Hz — always sent, uses the `DEVICE_ID` parameter as the device ID
- `BatteryInfo` message at 10Hz — only sent when the `BATT_EN` parameter is set to `1`

The `DEVICE_ID` and `BATT_EN` parameters can be changed at runtime via a DroneCAN ground station (e.g. Mission Planner or QGroundControl).

The MCP9600 is expected at I2C address `0x66`. If the sensor is not found on boot, the node will log a debug message over DroneCAN and keep retrying.

## Dependencies

Install the following libraries. Download the zips and place the unzipped folders in the `lib` directory of your project. Make sure there are no nested folders inside (this can happen when unzipping from GitHub). Alternatively, install via the PlatformIO library manager.

- Adafruit MCP9600: https://github.com/adafruit/Adafruit_MCP9600/releases/tag/2.0.4
- Adafruit BusIO (required by MCP9600 library): https://github.com/adafruit/Adafruit_BusIO/releases/tag/1.17.2
