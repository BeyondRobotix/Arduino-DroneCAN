# Servo Test

Listens for DroneCAN `ArrayCommand` messages and drives a servo on PA8 to the commanded position. This also demonstrates how to use the callback API (`onTransferReceived` / `shouldAcceptTransfer`) to receive DroneCAN messages.

The actuator ID this node responds to is configured via the `ACTUATOR_ID` parameter (default `0`), which can be changed at runtime via a ground station.

`command_value` is expected in the range `-1.0` to `1.0`, which maps to `0`–`180` degrees. This matches the range sent by ArduPilot for servo outputs over DroneCAN.

## Dependencies

No extra libraries required. The `Servo` library is included with the STM32 Arduino core.
