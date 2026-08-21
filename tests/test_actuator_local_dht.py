"""Source-contract tests for the ActuatorUno-local DHT22 split.

These tests are intentionally hardware-free.  They protect the pin/protocol
contract and, in particular, make sure a failed DHT read cannot stop a running
humidifier/dehumidifier task.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = (
    ROOT
    / "firmware/uno_humidity_module_controller/uno_humidity_module_controller.ino"
)


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


class ActuatorLocalDhtSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")

    def assert_source(self, pattern: str) -> None:
        self.assertRegex(self.source, re.compile(pattern, re.DOTALL))

    def test_existing_pin_and_i2c_protocol_contract_is_unchanged(self) -> None:
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
            self.assert_source(pattern)

    def test_dht22_is_local_on_d2_and_sampled_every_three_seconds(self) -> None:
        for pattern in (
            r"#include\s*<DHT\.h>",
            r"DHT_PIN\s*=\s*2",
            r"DHT_TYPE\s*=\s*DHT22",
            r"DHT_READ_INTERVAL_MS\s*=\s*3000",
            r"DHT\s+dht\s*\(\s*DHT_PIN\s*,\s*DHT_TYPE\s*\)",
            r"dht\.begin\s*\(\s*\)",
            r"now\s*-\s*lastDhtReadAt\s*<\s*DHT_READ_INTERVAL_MS",
        ):
            self.assert_source(pattern)

    def test_display_frame_keeps_compatibility_but_does_not_supply_dht_data(self) -> None:
        body = function_body(
            self.source, "void serviceDisplayMailbox()", "void serviceLocalDht()"
        )
        for token in (
            "frame[0] != DISPLAY_FRAME_MAGIC",
            "crc8Atm(frame, DISPLAY_FRAME_SIZE - 1)",
            "lastDisplaySeq = frame[1];",
            "currentDisplayState = nextState;",
            "currentZoneCode = nextZone;",
            "currentInputFlags = frame[8];",
        ):
            self.assertIn(token, body)

        for ignored_sensor_field in (
            "frame[4]",
            "frame[5]",
            "frame[6]",
            "frame[7]",
            "DISPLAY_INPUT_VALID",
        ):
            self.assertNotIn(ignored_sensor_field, body)

    def test_lcd_row_zero_uses_only_local_dht_values(self) -> None:
        body = function_body(
            self.source, "void formatDisplayLines()", "void scheduleFullLcdRender()"
        )
        for token in (
            "localDhtReadAttempted",
            "localDhtValid",
            'setPaddedLine(0, "DHT22 ERROR")',
            "localTemperatureTenths",
            "localHumidityTenths",
        ):
            self.assertIn(token, body)
        for forbidden in (
            "telemetryStale",
            "DISPLAY_INPUT_VALID",
            "currentTemperatureTenths",
            "currentHumidityTenths",
        ):
            self.assertNotIn(forbidden, body)

    def test_dht_failure_is_lcd_only_and_cannot_change_actuator_status(self) -> None:
        body = function_body(
            self.source, "void serviceLocalDht()", "void serviceDisplayStaleness()"
        )
        for token in (
            "localDhtValid = false;",
            "formatDisplayLines();",
            "scheduleFullLcdRender();",
            "local D2",
        ):
            self.assertIn(token, body)
        for forbidden in (
            "stopAllOutputs",
            "writeRelay",
            "publishActuatorState",
            "publishStatusReply",
            "actuatorStatus =",
            "statusReply[",
            "STATUS_ERROR",
            "dehumidifyStage =",
            "taskStartedAt =",
            "stageStartedAt =",
        ):
            self.assertNotIn(forbidden, body)

    def test_command_and_relay_timer_are_serviced_around_dht_read(self) -> None:
        loop = self.source[self.source.index("void loop()") :]
        first_command = loop.index("serviceCommandMailbox();")
        first_timer = loop.index("serviceActuatorTask();")
        dht = loop.index("serviceLocalDht();")
        second_command = loop.index("serviceCommandMailbox();", first_command + 1)
        second_timer = loop.index("serviceActuatorTask();", first_timer + 1)
        display = loop.index("serviceDisplayMailbox();")
        lcd = loop.index("serviceLcd();")

        self.assertLess(first_command, first_timer)
        self.assertLess(first_timer, dht)
        self.assertLess(dht, second_command)
        self.assertLess(second_command, second_timer)
        self.assertLess(second_timer, display)
        self.assertLess(display, lcd)


if __name__ == "__main__":
    unittest.main(verbosity=2)
