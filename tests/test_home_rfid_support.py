"""Regression contracts for the optional HOME RFID arrival path.

These checks intentionally read only tracked example/source files.  The local
``robot_network_config.h`` can contain the real installation UID and must never
be opened, copied into assertions, or committed by this test.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SENSOR_PATH = (
    ROOT
    / "firmware"
    / "uno_robot_esp01_rfid_relay"
    / "uno_robot_esp01_rfid_relay.ino"
)
EXAMPLE_CONFIG_PATH = SENSOR_PATH.with_name("robot_network_config.example.h")
REGISTRATION_PATH = (
    ROOT
    / "firmware"
    / "uno_home_rfid_registration"
    / "uno_home_rfid_registration.ino"
)
GITIGNORE_PATH = ROOT / ".gitignore"


def function_body(source: str, signature: str) -> str:
    """Return one C++ function including its braces, without regex nesting."""

    search_from = 0
    while True:
        start = source.index(signature, search_from)
        opening = source.index("{", start)
        semicolon = source.find(";", start, opening)
        if semicolon == -1:
            break
        search_from = semicolon + 1
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


class HomeRfidSourceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.sensor = SENSOR_PATH.read_text(encoding="utf-8")
        cls.example_config = EXAMPLE_CONFIG_PATH.read_text(encoding="utf-8")
        cls.registration = REGISTRATION_PATH.read_text(encoding="utf-8")
        cls.gitignore = GITIGNORE_PATH.read_text(encoding="utf-8")

    def test_home_uid_is_optional_and_real_value_stays_in_local_config(self) -> None:
        # Old local configs remain valid: an omitted HOME UID keeps the black
        # marker-only route instead of causing a compile failure.
        self.assertRegex(
            self.sensor,
            r"#ifndef\s+ROBOT_HOME_UID\s+"
            r"#define\s+ROBOT_HOME_UID\s+\"\"\s+#endif",
        )
        self.assertIn(
            "const char RFID_HOME_UID[] PROGMEM = ROBOT_HOME_UID;",
            self.sensor,
        )
        home_configured = function_body(
            self.sensor, "bool homeRfidConfigured()"
        )
        self.assertIn("pgm_read_byte(RFID_HOME_UID) != '\\0'", home_configured)

        # The tracked file contains a visibly synthetic example only.  The
        # installation-specific header, which may contain credentials and the
        # real UID, remains ignored.
        self.assertIn('#define ROBOT_HOME_UID "01 23 45 67"', self.example_config)
        self.assertRegex(
            self.gitignore,
            r"(?m)^firmware/uno_robot_esp01_rfid_relay/"
            r"robot_network_config\.h\s*$",
        )

    def test_calibration_marks_a_configured_home_tag_as_already_accepted(self) -> None:
        body = function_body(self.sensor, "bool performHomeCalibration()")
        assignment = re.search(
            r"lastAcceptedRfidStation\s*=\s*homeRfidConfigured\(\)\s*"
            r"\?\s*STATION_HOME\s*:\s*STATION_UNKNOWN\s*;",
            body,
        )
        self.assertIsNotNone(assignment)
        self.assertLess(body.index("routeCalibrated = true;"), assignment.start())
        self.assertIn("confirmedStation = STATION_HOME;", body)
        self.assertIn("expectedStation = STATION_ZONE2;", body)

    def test_lingering_home_tag_is_ignored_during_outbound_departure(self) -> None:
        body = function_body(
            self.sensor, "void processRouteRfid(RouteStation scannedStation)"
        )
        repeat_guard = re.search(
            r"if\s*\(scannedStation\s*==\s*lastAcceptedRfidStation\s*&&\s*"
            r"scannedStation\s*!=\s*expectedStation\)\s*return\s*;",
            body,
        )
        self.assertIsNotNone(repeat_guard)
        # The guard must run before STOP and before changing the accepted tag.
        self.assertLess(repeat_guard.start(), body.index("stopMotorController()"))
        self.assertLess(
            repeat_guard.start(), body.index("lastAcceptedRfidStation = scannedStation;")
        )

    def test_expected_home_tag_completes_homebound_route_with_outputs_off(self) -> None:
        body = function_body(
            self.sensor, "void processRouteRfid(RouteStation scannedStation)"
        )
        motor_stop = body.index("if (!stopMotorController())")
        mismatch = body.index("if (scannedStation != expectedStation)")
        home_branch = body.index("if (scannedStation == STATION_HOME")
        self.assertLess(motor_stop, mismatch)
        self.assertLess(mismatch, home_branch)

        home_body = body[home_branch : body.index("\n  if (confirmedStation", home_branch)]
        for token in (
            "targetStation == STATION_HOME",
            "routeHeading == HEADING_HOMEBOUND",
            "const bool moduleStopped = stopModuleController();",
            "expectedStation = STATION_ZONE2;",
            "taskActive = false;",
            "manualForwardActive = false;",
            "robotPhase = PHASE_IDLE;",
            'setCommandResult(F("COMPLETED"));',
            'queueRobotReport(F("HOME_RFID_ARRIVAL"));',
        ):
            self.assertIn(token, home_body)

        # An actuator STOP ACK failure cannot be reported as a successful HOME.
        self.assertIn("robotPhase = PHASE_TASK_COMPLETE;", home_body)
        self.assertIn('setCommandResult(F("I2C_ERROR"));', home_body)
        self.assertIn('queueRobotReport(F("ACTUATOR_STOP_ERROR"));', home_body)

    def test_unexpected_home_uid_uses_the_existing_route_mismatch_safe_stop(self) -> None:
        body = function_body(
            self.sensor, "void processRouteRfid(RouteStation scannedStation)"
        )
        mismatch_start = body.index("if (scannedStation != expectedStation)")
        home_start = body.index("if (scannedStation == STATION_HOME")
        mismatch_body = body[mismatch_start:home_start]

        # Except for the deliberately ignored tag still under the reader after
        # calibration, a HOME UID seen when another station is expected stops
        # both output boards and faults the route.
        self.assertLess(mismatch_start, home_start)
        for token in (
            "stopModuleController();",
            "taskActive = false;",
            "robotPhase = PHASE_TASK_COMPLETE;",
            'setCommandResult(F("FAILED"));',
            'queueRobotReport(F("ROUTE_UID_ERROR"));',
        ):
            self.assertIn(token, mismatch_body)

    def test_black_home_marker_remains_a_complete_fallback(self) -> None:
        body = function_body(self.sensor, "void applyMotorLinkState()")
        marker_start = body.index(
            "expectedStation == STATION_HOME && targetStation == STATION_HOME"
        )
        missed_start = body.index(
            "targetStation == STATION_HOME &&\n               "
            "routeHeading == HEADING_HOMEBOUND",
            marker_start,
        )
        marker_body = body[marker_start:missed_start]
        for token in (
            "confirmedStation = STATION_HOME;",
            "routeAtStation = true;",
            "taskActive = false;",
            "robotPhase = PHASE_IDLE;",
            'setCommandResult(F("COMPLETED"));',
            'queueRobotReport(F("HOME_ARRIVAL"));',
        ):
            self.assertIn(token, marker_body)
        self.assertNotIn("if (homeRfidConfigured()) {", marker_body)

        # Even a missed intermediate RFID still retains the established
        # homebound-only black-marker recovery and records the degraded result.
        missed_body = body[missed_start : body.index("UNEXPECTED_STOP_LINE", missed_start)]
        self.assertIn("confirmedStation = STATION_HOME;", missed_body)
        self.assertIn("robotPhase = PHASE_IDLE;", missed_body)
        self.assertIn('queueRobotReport(F("HOME_RFID_MISSED"));', missed_body)

    def test_registration_sketch_is_d8_to_d12_uid_read_only(self) -> None:
        for pattern in (
            r"RFID_SS_PIN\s*=\s*8",
            r"RFID_SCK_PIN\s*=\s*9",
            r"RFID_MOSI_PIN\s*=\s*10",
            r"RFID_MISO_PIN\s*=\s*11",
            r"RFID_RST_PIN\s*=\s*12",
        ):
            self.assertRegex(self.registration, pattern)

        for token in (
            "PICC_IsNewCardPresent()",
            "PICC_ReadCardSerial()",
            "printUidBytes();",
            'Serial.print(F("[HOME UID] "));',
            'Serial.print(F("[COPY',
            '#define ROBOT_HOME_UID \\"',
            "PICC_HaltA();",
            "PCD_StopCrypto1();",
        ):
            self.assertIn(token, self.registration)

        for forbidden in (
            "#include <Wire.h>",
            "#include <EEPROM.h>",
            "#include <SoftwareSerial.h>",
            "#include <AFMotor.h>",
            "Wire.begin(",
            "Wire.write(",
            "EEPROM.",
            "digitalWrite(",
            "analogWrite(",
            "MIFARE_Write(",
            "PICC_Write",
        ):
            self.assertNotIn(forbidden, self.registration)

        # The sketch prints a UID read live from the card.  It must not contain
        # a compiled-in card value or a credential-style config definition.
        self.assertNotRegex(
            self.registration,
            r"(?m)^\s*#define\s+ROBOT_HOME_UID\s+\"[0-9A-Fa-f ]+\"",
        )
        self.assertNotRegex(
            self.registration,
            r"(?i)\b(?:wifi_password|wifi_ssid|server_host)\b\s*=",
        )


if __name__ == "__main__":
    unittest.main()
