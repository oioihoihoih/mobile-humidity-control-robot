# Firmware map

The current HumiBot runtime uses three Arduino Uno sketches on the robot and
two ESP-01 sketches at the fixed sensing zones.

## Current runtime

| Role | Sketch | Notes |
| --- | --- | --- |
| SensorUno / I²C master | `uno_robot_esp01_rfid_relay/` | Network polling, RFID, local sensors, mission coordination |
| MotorUno / I²C `0x08` | `uno_line_tracker_motor_controller/` | Two-wheel line following and fail-safe motor control |
| ActuatorUno / I²C `0x09` | `uno_humidity_module_controller/` | Humidifier, dehumidifier, fan, and LCD |
| Fixed zone 2 | `zone2_esp01_direct/` | ESP-01 + DHT sensor reporter |
| Fixed zone 99 | `zone99_esp01_dht11/` | ESP-01 + DHT sensor reporter |

Copy each checked-in `*.example.h` file to the corresponding ignored private
header before compiling networked sketches. Never commit SSIDs, passwords,
access tokens, or site-specific addresses.

## Diagnostics

The following sketches temporarily replace runtime firmware while diagnosing
hardware. Lift the drive wheels and disconnect high-current loads before using
them.

- `i2c_bus_diagnostic/`, `i2c_passive_release/`, `i2c_slave_diagnostic/`
- `sensor_bus_release/`, `uno_sensor_pin_diagnostic/`
- `uno_esp01_at_bridge/`, `uno_esp01_autobaud_diagnostic/`
- `uno_motor_power_diagnostic/`, `uno_rc522_diagnostic/`
- `uno_server_gateway_esp01/` (optional USB/network handoff helper)

## Legacy prototypes

`robot/`, `zone_sensor/`, and `uno_dht11_esp8266/` preserve earlier ESP32 and
single-node experiments. They are not the current three-Uno build and are not
compiled by CI.

See [`../docs/hardware.md`](../docs/hardware.md) for wiring and power rules and
[`../docs/testing.md`](../docs/testing.md) for the verification matrix.
