"""SensorUno-local DHT22 and remote LCD telemetry source contracts."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "firmware"
    / "uno_robot_esp01_rfid_relay"
    / "uno_robot_esp01_rfid_relay.ino"
)


def function_slice(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    return source[start : source.index(next_signature, start)]


class SensorLocalDhtSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")

    def test_dht22_uses_the_existing_sensor_uno_d4_pin(self) -> None:
        for pattern in (
            r"#include\s*<DHT\.h>",
            r"DHT_PIN\s*=\s*4",
            r"DHT_TYPE\s*=\s*DHT22",
            r"DHT_INTERVAL_MS\s*=\s*3000",
            r"DHT\s+dht\s*\(\s*DHT_PIN\s*,\s*DHT_TYPE\s*\)",
        ):
            self.assertRegex(self.source, pattern)
        self.assertIn("dht.begin();", self.source)

    def test_dht_read_failure_only_invalidates_telemetry(self) -> None:
        body = function_slice(
            self.source, "void updateDhtSensor()", "void serviceMotorLink()"
        )
        for token in (
            "dht.readHumidity()",
            "dht.readTemperature()",
            "dhtReadOk = false;",
            "dhtReadOk = true;",
            "sensorHumidity = humidity;",
            "sensorTemperature = temperature;",
        ):
            self.assertIn(token, body)
        for forbidden in (
            "stopMotorController",
            "stopModuleController",
            "sendMotorCommandChecked",
            "sendActuatorFrame",
        ):
            self.assertNotIn(forbidden, body)

    def test_display_frame_contains_little_endian_dht_tenths_and_valid_flag(self) -> None:
        body = function_slice(
            self.source,
            "void buildDisplayPayload(byte* payload)",
            "bool sendDisplayTelemetryFrame()",
        )
        for token in (
            "sensorTemperature",
            "sensorHumidity",
            "temperatureTenths",
            "humidityTenths",
            "payload[2] = static_cast<byte>(temperatureTenths & 0xFF);",
            "payload[3] = static_cast<byte>((temperatureTenths >> 8) & 0xFF);",
            "payload[4] = static_cast<byte>(humidityTenths & 0xFF);",
            "payload[5] = static_cast<byte>((humidityTenths >> 8) & 0xFF);",
            "if (dhtReadOk) flags |= DISPLAY_FLAG_DHT_VALID;",
        ):
            self.assertIn(token, body)

    def test_hc_sr04_driver_is_absent_from_sensor_uno(self) -> None:
        for forbidden in (
            "ULTRASONIC_ECHO_PIN",
            "ULTRASONIC_TRIG_PIN",
            "UltrasonicSampleStatus",
            "onUltrasonicEchoChange",
            "updateObstacleSensor",
            "obstacleDistanceCm",
            "pulseIn(",
        ):
            self.assertNotIn(forbidden, self.source)

    def test_motor_local_obstacle_status_pauses_the_rfid_direction_guard(self) -> None:
        self.assertGreaterEqual(
            self.source.count(
                "obstaclePauseActive = status == MOTOR_STATUS_OBSTACLE;"
            ),
            2,
        )

    def test_dht_never_runs_inside_software_serial_receive_waits(self) -> None:
        boundaries = (
            ("bool waitFor(", "bool sendAt("),
            ("bool collectHttpResponse()", "bool fetchCommandResponse()"),
        )
        for signature, next_signature in boundaries:
            body = function_slice(self.source, signature, next_signature)
            self.assertNotIn("updateDhtSensor();", body)
            self.assertIn("applyMotorLinkState();", body)


if __name__ == "__main__":
    unittest.main(verbosity=2)
