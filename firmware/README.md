# Firmware map

The current HumiBot runtime uses three Arduino Uno sketches on the robot and
two ESP-01 sketches at the fixed sensing zones.

## Current runtime

| Role | Sketch | Notes |
| --- | --- | --- |
| SensorUno / I²C master | `uno_robot_esp01_rfid_relay/` | Network polling, RFID, route state, mission coordination |
| MotorUno / I²C `0x08` | `uno_line_tracker_motor_controller/` | Four-motor line following plus local rear HC-SR04 reverse safety; M1/M3 left, M2/M4 right |
| ActuatorUno / I²C `0x09` | `uno_humidity_module_controller/` | Local DHT22, humidifier, dehumidifier, fan, and LCD |
| Fixed zone 2 | `zone2_esp01_direct/` | ESP-01 + DHT sensor reporter |
| Fixed zone 99 | `zone99_esp01_dht11/` | ESP-01 + DHT sensor reporter |

Copy each checked-in `*.example.h` file to the corresponding ignored private
header before compiling networked sketches. Never commit SSIDs, passwords,
access tokens, or site-specific addresses.

Upload SensorUno and MotorUno from the same commit. The four-motor runtime uses
versioned movement commands `0x11/0x12` for absolute forward/straight reverse
and requires a `PROTOCOL_SYNC(7)` exact ACK before HOME calibration. Legacy
movement values `1/2` are invalid, so a mixed old/new pair remains stopped.

## Diagnostics

The following sketches temporarily replace runtime firmware while diagnosing
hardware. Lift the drive wheels and disconnect high-current loads before using
them.

- `i2c_bus_diagnostic/`, `i2c_passive_release/`, `i2c_slave_diagnostic/`
- `sensor_bus_release/`, `uno_sensor_pin_diagnostic/`
- `uno_esp01_at_bridge/`, `uno_esp01_autobaud_diagnostic/`
- `uno_motor_power_diagnostic/` (independent M1–M4 and forward/reverse test), `uno_rc522_diagnostic/`
- `uno_zone2_rfid_drive_diagnostic/` (SensorUno replacement for HOME sync → versioned 4WD forward → ZONE2 RFID stop; copy its example UID header first)
- `uno_server_gateway_esp01/` (optional USB/network handoff helper)

## Legacy prototypes

`robot/`, `zone_sensor/`, and `uno_dht11_esp8266/` preserve earlier ESP32 and
single-node experiments. They are not the current three-Uno build and are not
compiled by CI.

See [`../docs/hardware.md`](../docs/hardware.md) for wiring and power rules and
[`../docs/testing.md`](../docs/testing.md) for the verification matrix.
