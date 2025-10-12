## CANFD Library for STM32 G4 and H7

Handled boards:

* NUCLEO-G431KB (1 CAN FD),
*  NUCLEO-G474RE (3 CAN FD),
*  NUCLEO-H743ZI2 (2 CAN FD).

### Compatibility with other ACAN libraries

This library is fully compatible with other ACAN libraries, it uses a very similar API and the same `CANFDMessage` and `CANMessage`classes for handling messages.

### ACANFD_STM32 library description

The driver supports many bit rates, as standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, and 1 Mbit/s. An efficient CAN bit timing calculator finds settings for them, but also for exotic bit rates as 833 kbit/s. If the wished bit rate cannot be achieved, the `beginFD` method does not configure the hardware and returns an error code.

> Driver API is fully described by the PDF file in the `extras` directory.

### Demo Sketches

Demo sketches are provided for all handled boards in the `examples` directory.


