# Captured UART frames

Each hexadecimal file contains one complete frame in wire order, including its checksum and footer.

The F3000 fixtures use zero timestamps and recalculated checksums.

- `f3000-telemetry.hex` uses the frame layout of an F3000 UART capture with synthetic readings.
- `f3000-firmware.hex` and `f3000-identification.hex` reproduce the metadata frame lengths from the same inverter. The firmware frame retains the model name and uses placeholder versions. The identification frame uses ASCII zeros for the device identifier.
- `t10-g3-original.hex` is a 245-byte capture with a published field breakdown. [Capture](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/22) [Field breakdown](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/24)
