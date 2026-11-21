# Captured UART frames

Each hexadecimal file contains one complete frame in wire order, including its checksum and footer.

The F3000 fixtures use zero timestamps and recalculated checksums.

- `f3000-telemetry.hex` contains synthetic readings in the F3000 UART frame layout.
- `f3000-firmware.hex` and `f3000-identification.hex` reproduce the metadata frame lengths from the same inverter. The firmware frame retains the model name and uses placeholder versions. The identification frame uses ASCII zeros for the device identifier.
- `t-series-original-165.hex` joins the UART debug lines published by the original project author. The capture declares 152 payload bytes and preserves the checksum from the screenshot. [Original UART capture](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/42)
- `t10-g3-original.hex` is a 245-byte capture with a published field breakdown. [Capture](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/22) [Field breakdown](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/24)
- `t10-g3-mqtt-firmware.hex` and `t10-g3-mqtt-telemetry.hex` are consecutive messages forwarded to MQTT. The replay sends both in one burst to check that the firmware frame leaves the next telemetry frame intact. [MQTT captures](https://community.home-assistant.io/t/foxess-inverter-modbus/459313?page=2)
- `t10-g3-rs485-telemetry.hex` is a 245-byte RS485 sniffer capture with separately checked daily and lifetime energy values. [RS485 capture](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/79) [Energy values](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/80)
