# Magnetometer Listener

Demonstrates how to receive DroneCAN messages using the full callback API (`onTransferReceived` / `shouldAcceptTransfer`).

Listens for `MagneticFieldStrength` broadcast messages on the bus and prints the X, Y, Z field values in Gauss to Serial.

Use this as a template when you need to receive any DroneCAN message type — copy the pattern in `onTransferReceived` and `shouldAcceptTransfer`, substituting the message ID, signature, struct, and decode call for your target message.

## When to use the callback API

The simplified `dronecan.init(parameters, name)` used in most examples handles everything internally. Use the 4-argument form shown here when your node needs to **receive** messages from other nodes on the bus.

## Dependencies

No extra libraries required.
