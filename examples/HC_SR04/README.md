# HC-SR04 Rangefinder

Reads distance from an HC-SR04 ultrasonic sensor and broadcasts it as a DroneCAN `RangeSensor` measurement at 10Hz.

The sensor is wired to:
- Trigger: PA8
- Echo: PA9

## Dependencies

Install the HC-SR04 Arduino library by Martinsos:
https://github.com/Martinsos/arduino-lib-hc-sr04

Download the zip and place the unzipped folder in the `lib` directory of your project. Make sure there are no nested folders inside (this can happen when unzipping from GitHub).

Alternatively, install it via the PlatformIO library manager.
