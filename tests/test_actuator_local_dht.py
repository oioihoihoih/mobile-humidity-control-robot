"""ActuatorUno contract when DHT telemetry remains on SensorUno.

The legacy filename stays discoverable, while the assertions protect the
memory-saving design: ActuatorUno renders the validated 10-byte telemetry
frame and does not instantiate a second DHT driver.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "firmware"
    / "uno_humidity_module_controller"
    / "uno_humidity_module_controller.ino"
)


def function_body(source: str, name: str) -> str:
    match = re.search(rf"\bvoid\s+{re.escape(name)}\s*\(\s*\)\s*\{{", source)
    if match is None:
        raise AssertionError(f"function definition not found: {name}")
    opening = source.index("{", match.start())
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : index]
    raise AssertionError(f"unterminated function: {name}")


class ActuatorRemoteDhtSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")

    def test_actuator_has_no_local_dht_driver_or_d2_assignment(self) -> None:
        for forbidden in (
            "#include <DHT.h>",
            "DHT_PIN",
            "DHT_TYPE",
            "DHT_READ_INTERVAL_MS",
            "DHT dht(",
            "serviceLocalDht",
            "dht.begin()",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_existing_relay_lcd_and_i2c_contract_is_unchanged(self) -> None:
        for pattern in (
            r"I2C_ADDRESS\s*=\s*0x09",
            r"CONTROL_FRAME_MAGIC\s*=\s*0xA5",
            r"CONTROL_FRAME_SIZE\s*=\s*4",
            r"STATUS_REPLY_SIZE\s*=\s*6",
            r"DISPLAY_FRAME_MAGIC\s*=\s*0xD1",
            r"DISPLAY_FRAME_SIZE\s*=\s*10",
            r"HUMIDIFIER_RELAY_PIN\s*=\s*A0",
            r"PELTIER_RELAY_PIN\s*=\s*A1",
            r"COOLING_FAN_RELAY_PIN\s*=\s*7",
            r"LCD_SOFT_SDA_PIN\s*=\s*5",
            r"LCD_SOFT_SCL_PIN\s*=\s*4",
        ):
            self.assertRegex(self.source, pattern)

    def test_validated_display_frame_supplies_temperature_and_humidity(self) -> None:
        body = function_body(self.source, "serviceDisplayMailbox")
        for token in (
            "frame[0] != DISPLAY_FRAME_MAGIC",
            "crc8Atm(frame, DISPLAY_FRAME_SIZE - 1)",
            "static_cast<uint16_t>(frame[6])",
            "static_cast<uint16_t>(frame[7]) << 8",
            "nextHumidityTenths > 1000",
            "static_cast<uint16_t>(frame[4])",
            "static_cast<uint16_t>(frame[5]) << 8",
            "currentHumidityTenths = nextHumidityTenths;",
            "DISPLAY_INPUT_VALID",
        ):
            self.assertIn(token, body)

    def test_lcd_row_zero_uses_remote_validity_and_staleness(self) -> None:
        body = function_body(self.source, "formatDisplayLines")
        for token in (
            "telemetryStale",
            'setPaddedLine(0, "TELEMETRY STALE")',
            "telemetryDataValid",
            'setPaddedLine(0, "DHT22 ERROR")',
            "currentTemperatureTenths",
            "currentHumidityTenths",
        ):
            self.assertIn(token, body)

    def test_relay_safety_is_serviced_before_display_and_lcd(self) -> None:
        loop = function_body(self.source, "loop")
        command = loop.index("serviceCommandMailbox();")
        timer = loop.index("serviceActuatorTask();")
        display = loop.index("serviceDisplayMailbox();")
        lcd = loop.index("serviceLcd();")
        self.assertLess(command, timer)
        self.assertLess(timer, display)
        self.assertLess(display, lcd)


if __name__ == "__main__":
    unittest.main(verbosity=2)
