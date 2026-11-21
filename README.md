# FoxESS Solar Esphome
Read out FoxESS Inverters to Home Assistant using ESPHome.

![FoxESS](resources/images/foxess.png)

Compatible with:
- Foxess T-series (including T10-G3 and other extended-frame models)
- Foxess F-series

The decoder uses the original measurement offsets for 163- and 245-byte frames.
A 251-byte frame uses the six-byte shift reported for later T10-G3 firmware.
[245-byte capture](https://community.home-assistant.io/t/foxess-inverter-modbus/459313/22)
[251-byte field report](https://community.home-assistant.io/t/foxess-inverter-modbus/459313?page=9)

```yaml
uart:
  tx_pin: GPIO16
  rx_pin: GPIO17
  baud_rate: 9600

sensor:
   - platform: foxess_solar
    phase_a:
      voltage:
        name: "FoxESS Phase A Voltage"
      current:
        name: "FoxESS Phase A Current"
      active_power:
        name: "FoxESS Phase A Power"
      frequency:
        name: "FoxESS Phase A Frequency"

    pv1:
      voltage:
        name: "FoxESS PV1 Voltage"
      current:
        name: "FoxESS PV1 Current"
      active_power:
        name: "FoxESS PV1 Power"

    total_energy_production:
      name: "Total Energy Production"
    energy_production_day:
      name: "Today Energy Production"

    generation_power:
      name: "FoxESS Generation Power"
    grid_power:
      name: "FoxESS Grid Power"
    loads_power:
      name: "FoxESS Loads Power"

    inverter_status:
      name: "FoxESS Status Code"

    inverter_temp:
      name: "FoxESS Inverter Temp"
    ambient_temp:
      name: "FoxESS Ambient Temp"
    boost_temp:
      name: "FoxESS Boost Temp"
```

## Instructions
- Copy the example YAML files to your ESPHome directory in Home Assistant
- Fill out individual data in the secrets.yaml file (SSID, password, etc.)
- Optionally comment sensors you don't need or uncomment sensors you do need

## Configuration variables

- **uart_id** (*Optional*, ID): Manually specify the ID of the UART hub.
- **flow_control_pin** selects the GPIO that holds the MAX485 transceiver in receive mode at startup. Defaults to GPIO4.
- **inverter_timeout** sets how long to wait for a valid frame before reporting the inverter offline. It defaults to five minutes and accepts a minimum of one second. The offline transition happens on the next polling update after the timeout. Live measurements report `NaN` while offline, and energy counters keep their last valid readings.
- **update_interval** sets how often the component checks for a communication timeout. Defaults to one second. UART reception runs in the main loop.
- **inverter_status** (*Optional*): Status code of the inverter (0: offline, 1: online, 2: error, 99: waiting for response)
- **phase_a** (*Optional*): Sensors related to first phase of the inverter
  - **current** (*Optional*): Current flowing to the grid (A)
  - **voltage** (*Optional*): Grid voltage (V)
  - **active_power** (*Optional*): Power delivered to the grid (W)
  - **frequency** (*Optional*): Grid frequency (Hz)
- **phase_b** (*Optional*): Sensors related to second phase of the inverter
  - See **phase_a**
- **phase_c** (*Optional*): Sensors related to third phase of the inverter
  - See **phase_a**

- **pv1** (*Optional*): Sensors related to the first group PV cells
  - **current** (*Optional*): Current flowing from PV1 (A)
  - **voltage** (*Optional*): PV1 Voltage (V)
  - **active_power** (*Optional*): PV1 power production (W)
- **pv2** (*Optional*): Sensors related to the second group PV cells
  - See **pv1**
- **pv3** (*Optional*): Sensors related to the third group PV cells
  - See **pv1**
- **pv4** (*Optional*): Sensors related to the fourth group PV cells
  - See **pv1**

- **total_energy_production** (*Optional*): Total energy produced during lifetime of inverter (kWh)
- **energy_production_day** (*Optional*): Total energy produced today (kWh)
- **generation_power** (*Optional*): Current total power generation (W)
- **grid_power** (*Optional*): Current export power to grid. Required to have a meter connected to the inverter (SDM230). (W)
- **loads_power** (*Optional*): (W)
- **inverter_temp** (*Optional*): Inverter temperature (°C)
- **boost_temp** (*Optional*): Boost temperature (°C)
- **ambient_temp** (*Optional*): Ambient temperature (°C)

## Development checks

Run the frame parser tests with a C++17 compiler and the address and undefined behavior sanitizers.

```sh
bash tests/run_parser_tests.sh
```

## Hardware setup
The hardware setup including a wiring diagram can be found in the [Wiki](https://github.com/assembly12/Foxess-T-series-ESPHome-Home-Assistant/wiki/Hardware-setup).

Designing a custom pcb and enclosure is next on my to do list. I'll update here with the corresponding gerber and stl files when done.

There is some more info being send (like error messages and so on), however this is really not to usefull so I left it out of this component.

Some basic electronics skills (like soldering) are needed to realize this project. I do not take any responsibility for the use of this custom component or anything that it written down in this repository. Use at your own risk.
