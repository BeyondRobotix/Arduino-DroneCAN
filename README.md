# Arduino DroneCAN

This repository allows easy integration of sensors to be used with Ardupilot and PX4 via DroneCAN. Sensors can be integrated within minutes by using pre-existing Arduino libraries for sensors, and through this library which abstracts the DroneCAN layer so you can focus on sending and receiving messages.

By using the Arduino core and PlatformIO with pre-configured board setups, you can start developing instantly.

This isn't intended to be used in the same way as AP_Periph, which supports a bunch of Ardupilot sensors all at once and is adaptable without writing code. This is intended for writing a DroneCAN interface for any sensor or system, without having to deal with a large code base and lots of boilerplate.

## Features

- Send DroneCAN messages ✅
- Receive DroneCAN messages ✅
- Standard DroneCAN under the hood (allocation, node info) ✅
- DroneCAN Parameters ✅
- Firmware update over CAN ✅
- CANFD ❌ (coming soon)
- Multiple CAN ports ❌

## Code Usage

Apart from calling usual init functions and our library update function, sending a DroneCAN message boils down to this:
```cpp
uavcan_equipment_power_BatteryInfo pkt{};
pkt.voltage = analogRead(PA1);
pkt.current = analogRead(PA0);

sendUavcanMsg(dronecan.canard, pkt);
```

See [Beyond Robotix Gitbooks ArduinoCAN documentation](https://beyond-robotix.gitbook.io/docs/can-node-system/arduino-dronecan) for more information!

We have a big tutorial here which runs through integrating a sensor: [Arduino DroneCAN tutorial](https://beyond-robotix.gitbook.io/docs/can-node-system/arduino-dronecan/thermocouple-tutorial)

## PlatformIO Management

Board definitions, variants, linker scripts, and the bundled bootloader binaries are provided by the [br_platformio_hwdef](https://github.com/BeyondRobotix/br_platformio_hwdef) platform, referenced directly from `platformio.ini`:

```ini
[env]
platform = https://github.com/BeyondRobotix/br_platformio_hwdef.git
framework = arduino
```

PlatformIO fetches the platform on first build — no separate install step. To pick up new changes to the platform, force a refetch with `pio pkg uninstall --platform br-stm32 -g`.

The DroneCAN library itself is pulled in the same way, pinned to a tag:

```ini
lib_deps = https://github.com/BeyondRobotix/libArduinoDroneCAN.git#v1.0.0
```

## Currently Supported Hardware

This repository is plug and play with the Beyond Robotix CAN node series!

See [Beyond Robotix Can Node](https://www.beyondrobotix.com/products/micro-can-node-development-bundle)

<img src="./assets/BRlogo.png" width="50%">

<img src="./assets/node-and-carrier.png" width="80%">

- More hardware options coming soon! 2 H7 based. Contact admin@beyondrobotix.com for early info.


We've got some handy docs for the node hardware and some more software explaination and examples here [CAN node gitbook](https://beyond-robotix.gitbook.io/docs/can-node-system)

## Support

If you get stuck with this repository, the discussions section will allow us to help out.

For dedicated engineering support on your application, contact [admin@beyondrobotix.com](admin@beyondrobotix.com). We can also quote for writing custom firmware for you!
