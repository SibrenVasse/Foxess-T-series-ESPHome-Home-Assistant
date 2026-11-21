"""Build the ESPHome host target and exercise the component through a test UART."""

import math
import os
import pty
import re
import subprocess
import tempfile
import time
from collections import defaultdict
from contextlib import ExitStack
from pathlib import Path

COMPONENTS = Path(__file__).resolve().parents[1] / "esphome" / "components"
FIXTURES = Path(__file__).with_name("fixtures")
NODE_NAME = "foxess-uart-test"
FIRST_FRAGMENT_SIZE = 80
FRAGMENT_READ = "Read the first UART fragment"
STATE_LOG = re.compile(r"'([^']+)': Received new state ([^\s\x1b]+)")


def telemetry_frame(size, power, *, fault=False, pv_limit=False):
    extended_shift = 6 if size == 251 else 0
    data = bytearray(size)
    data[:3] = b"\x7e\x7e\x02"
    data[7:9] = (size - 13).to_bytes(2, "big")
    data[9:11] = (-120).to_bytes(2, "big", signed=True)
    generation_offset = 15 if extended_shift else 11
    data[generation_offset : generation_offset + 2] = power.to_bytes(2, "big")
    phase_offset = 15 + extended_shift
    voltage = 2413 if extended_shift else 2304
    data[phase_offset : phase_offset + 2] = voltage.to_bytes(2, "big")
    pv_offset = 39 + extended_shift
    data[pv_offset : pv_offset + 4] = b"\xff\xff\xff\xff" if pv_limit else b"\x17\x70\x00\x71"
    energy_offset = 69 + extended_shift
    data[energy_offset : energy_offset + 6] = b"\x01\x41\xf0\x00\x00\x00"
    if fault:
        data[156 + extended_shift] = 1

    checksum = 0xFFFF
    for byte in data[2:-4]:
        checksum ^= byte
        for _ in range(8):
            checksum = (checksum >> 1) ^ (0xA001 if checksum & 1 else 0)
    data[-4:-2] = checksum.to_bytes(2, "little")
    data[-2:] = b"\xe7\xe7"
    return data


def wait_for_output(process, log, check, description):
    deadline = time.monotonic() + 5
    while process.poll() is None and time.monotonic() < deadline:
        result = check(log.read_text())
        if result is not None:
            return result
        time.sleep(0.01)
    raise AssertionError(f"Expected {description}.\n{log.read_text()}")


def wait_for_state(process, log, name, expected):
    def read_states(output):
        states = defaultdict(list)
        for sensor, value in STATE_LOG.findall(output):
            states[sensor].append(float(value))
        if states[name]:
            value = states[name][-1]
            matches = (
                math.isnan(value) if math.isnan(expected)
                else math.isclose(value, expected, rel_tol=1e-6)
            )
            if matches:
                return states

    return wait_for_output(process, log, read_states, f"{name} to publish {expected}")


def check_uart(process, log, uart):
    states = wait_for_state(process, log, "Mode", 99)
    assert not states["Generation"]

    # Queue the remainder of a valid frame during a pause longer than the inter-byte timeout.
    first_frame = telemetry_frame(163, 0, pv_limit=True)
    os.write(uart, first_frame[:FIRST_FRAGMENT_SIZE])
    wait_for_output(process, log, lambda output: True if FRAGMENT_READ in output else None, FRAGMENT_READ)
    os.write(uart, first_frame[FIRST_FRAGMENT_SIZE:])
    states = wait_for_state(process, log, "Mode", 1)
    assert states["Generation"] == [0]
    assert states["Grid"] == [-120]
    assert math.isclose(states["Phase Voltage"][-1], 230.4, rel_tol=1e-6)
    assert math.isclose(states["PV Power"][-1], 65535**2 * 0.01, rel_tol=1e-6)
    assert math.isclose(states["Total Energy"][-1], 0xF0000000 * 0.1, rel_tol=1e-6)

    os.write(uart, telemetry_frame(163, 3500))
    wait_for_state(process, log, "Generation", 3500)

    # The captured 245-byte T10-G3 frame uses the same field offsets as the 163-byte frame.
    captured = bytes.fromhex((FIXTURES / "t10-g3-original.hex").read_text())
    os.write(uart, captured)
    states = wait_for_state(process, log, "Total Energy", 2185)
    assert states["Generation"][-1] == 6120
    assert math.isclose(states["Phase Voltage"][-1], 239.5, rel_tol=1e-6)
    assert math.isclose(states["PV Power"][-1], 5124.48, rel_tol=1e-6)
    assert states["Daily Energy"][-1] == 13

    os.write(uart, telemetry_frame(251, 2300, fault=True))
    states = wait_for_state(process, log, "Mode", 2)
    assert states["Generation"] == [0, 3500, 6120, 2300]
    assert math.isclose(states["Phase Voltage"][-1], 241.3, rel_tol=1e-6)
    assert math.isclose(states["PV Power"][-1], 6780, rel_tol=1e-6)
    assert len(states["Daily Energy"]) == len(states["Total Energy"]) == 4
    assert math.isclose(states["Daily Energy"][-1], 32.1, rel_tol=1e-6)
    assert math.isclose(states["Total Energy"][-1], 0xF0000000 * 0.1, rel_tol=1e-6)
    energy_states = (states["Daily Energy"], states["Total Energy"])

    # A valid checksum cannot make an unsupported frame update the sensors.
    os.write(uart, telemetry_frame(162, 9999))
    states = wait_for_state(process, log, "PV Power", math.nan)
    assert states["Mode"] == [99, 1, 2, 0]
    assert states["Generation"][:-1] == [0, 3500, 6120, 2300]
    for name in ("Generation", "Grid", "Phase Voltage", "PV Power"):
        assert math.isnan(states[name][-1])
    assert (states["Daily Energy"], states["Total Energy"]) == energy_states

    os.write(uart, telemetry_frame(163, 123))
    states = wait_for_state(process, log, "Mode", 1)
    assert states["Mode"] == [99, 1, 2, 0, 1]
    assert states["Generation"][-1] == 123


def main():
    with tempfile.TemporaryDirectory(prefix="foxess-uart-", dir="/tmp") as directory, ExitStack() as cleanup:
        work = Path(directory)
        uart, serial_port = pty.openpty()
        cleanup.callback(os.close, uart)
        cleanup.callback(os.close, serial_port)
        # ESPHome's host UART expects a device path with two components.
        port = work.with_suffix(".tty")
        port.symlink_to(os.ttyname(serial_port))
        cleanup.callback(port.unlink)

        build_dir = work / "build"
        configuration = work / "test.yaml"
        configuration.write_text(f"""\
esphome:
  name: {NODE_NAME}
  build_path: "{build_dir}"
  compile_process_limit: 4
  on_boot:
    priority: -100
    then:
      - lambda: |-
          auto wait_for_bytes = [](size_t count) {{
            const uint32_t start = millis();
            while (id(test_uart).available() < count && millis() - start < 3000) {{
              delay(1);
            }}
            return id(test_uart).available() >= count;
          }};
          if (!wait_for_bytes({FIRST_FRAGMENT_SIZE})) {{
            return;
          }}
          id(test_inverter).loop();
          ESP_LOGI("uart_test", "{FRAGMENT_READ}");
          delay(150);
          if (wait_for_bytes({163 - FIRST_FRAGMENT_SIZE})) {{
            id(test_inverter).loop();
          }}
host:
logger:
  level: VERBOSE
external_components:
  - source:
      type: local
      path: "{COMPONENTS}"
uart:
  id: test_uart
  port: "{port}"
  baud_rate: 9600
sensor:
  - platform: foxess_solar
    id: test_inverter
    inverter_timeout: 1s
    update_interval: 200ms
    inverter_status:
      name: "Mode"
    generation_power:
      name: "Generation"
    grid_power:
      name: "Grid"
    phase_a:
      voltage:
        name: "Phase Voltage"
    pv1:
      active_power:
        name: "PV Power"
    energy_production_day:
      name: "Daily Energy"
    total_energy_production:
      name: "Total Energy"
""")
        subprocess.run(["esphome", "compile", str(configuration)], cwd=work, check=True)

        program = build_dir / ".pioenvs" / NODE_NAME / "program"
        log = work / "uart.log"
        with log.open("wb") as output:
            process = subprocess.Popen([program], cwd=work, stdout=output, stderr=subprocess.STDOUT)
            try:
                check_uart(process, log, uart)
            finally:
                process.terminate()
                process.wait(timeout=5)
        print("UART integration checks passed")


if __name__ == "__main__":
    main()
