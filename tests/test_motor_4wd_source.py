"""Static safety contract for the production four-wheel MotorUno sketch."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "firmware"
    / "uno_line_tracker_motor_controller"
    / "uno_line_tracker_motor_controller.ino"
)


def function_body(source: str, name: str) -> str:
    """Return a simple Arduino function body using brace depth."""
    match = re.search(
        rf"\b(?:void|bool|byte)\s+{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{",
        source,
    )
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


class MotorFourWheelSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")

    def test_all_four_channels_are_assigned_to_matching_sides(self) -> None:
        expected = {
            "motorExistingLeft": 1,
            "motorExistingRight": 2,
            "motorN20RearLeft": 3,
            "motorN20RearRight": 4,
        }
        for name, channel in expected.items():
            self.assertRegex(
                self.source,
                rf"AF_DCMotor\s+{name}\s*\(\s*{channel}\s*\)\s*;",
            )

        left = function_body(self.source, "runLeftPair")
        right = function_body(self.source, "runRightPair")
        self.assertIn("motorExistingLeft.run(direction);", left)
        self.assertIn("motorN20RearLeft.run(direction);", left)
        self.assertIn("motorExistingRight.run(direction);", right)
        self.assertIn("motorN20RearRight.run(direction);", right)

    def test_existing_and_n20_axles_have_independent_pwm_constants(self) -> None:
        self.assertRegex(self.source, r"EXISTING_AXLE_SPEED\s*=\s*\d+")
        self.assertRegex(self.source, r"N20_REAR_AXLE_SPEED\s*=\s*\d+")
        self.assertIn("scaledTrackingSpeed(EXISTING_AXLE_SPEED)", self.source)
        self.assertIn("scaledTrackingSpeed(N20_REAR_AXLE_SPEED)", self.source)

    def test_outbound_is_four_wheel_forward_and_return_is_reverse(self) -> None:
        outbound = function_body(self.source, "driveOutbound")
        homebound = function_body(self.source, "driveHomebound")
        self.assertIn("runLeftPair(FORWARD);", outbound)
        self.assertIn("runRightPair(FORWARD);", outbound)
        self.assertIn("runLeftPair(BACKWARD);", homebound)
        self.assertIn("runRightPair(BACKWARD);", homebound)

        apply_command = function_body(self.source, "applyCommand")
        self.assertIn(
            "const bool nextHomebound = command == COMMAND_RETURN;",
            apply_command,
        )
        self.assertIn("headingHomebound = nextHomebound;", apply_command)
        self.assertIn("driveHomebound();", apply_command)
        self.assertIn("driveOutbound();", apply_command)
        for removed_turn_token in (
            "spinForReturn",
            "RETURN_TURN_MS",
            "returnTurnActive",
        ):
            self.assertNotIn(removed_turn_token, self.source)

    def test_direction_change_including_stop_boundary_uses_release_dead_time(self) -> None:
        self.assertRegex(
            self.source,
            r"DIRECTION_CHANGE_DEAD_TIME_MS\s*=\s*120",
        )
        apply_command = function_body(self.source, "applyCommand")
        self.assertIn("stopMotors();", apply_command)
        self.assertIn("directionChangeDeadTimeActive = true;", apply_command)
        self.assertIn("reversesLastDrive", apply_command)
        self.assertIn("releaseIntervalIncomplete", apply_command)
        self.assertIn("lastMotorReleaseAt", apply_command)
        executable_apply = re.sub(r"//.*", "", apply_command)
        self.assertNotIn("delay(", executable_apply)

        service = function_body(self.source, "serviceDirectionChangeDeadTime")
        self.assertIn("stopMotors();", service)
        self.assertIn("DIRECTION_CHANGE_DEAD_TIME_MS", service)
        executable_service = re.sub(r"//.*", "", service)
        self.assertNotIn("delay(", executable_service)
        loop = function_body(self.source, "loop")
        self.assertLess(
            loop.index("serviceDirectionChangeDeadTime()"),
            loop.index("followLine();"),
        )
        stop = function_body(self.source, "stopMotors")
        self.assertIn("lastMotorReleaseAt = millis();", stop)

    def test_line_correction_never_counter_rotates_or_releases_a_side(self) -> None:
        for name in ("correctTowardLeft", "correctTowardRight"):
            body = function_body(self.source, name)
            self.assertIn("runLeftPair(direction);", body)
            self.assertIn("runRightPair(direction);", body)
            self.assertNotIn("RELEASE", body)
        self.assertRegex(self.source, r"TRACKING_INNER_PERCENT\s*=\s*80")
        self.assertRegex(
            self.source,
            r"MIRROR_LINE_CORRECTION_WHEN_REVERSING\s*=\s*true",
        )

    def test_every_safety_stop_releases_all_four_channels(self) -> None:
        stop = function_body(self.source, "stopMotors")
        for name in (
            "motorExistingLeft",
            "motorExistingRight",
            "motorN20RearLeft",
            "motorN20RearRight",
        ):
            self.assertIn(f"{name}.run(RELEASE);", stop)

        # Existing protocol safety behavior remains present.
        for token in (
            "CONTROL_WATCHDOG_MS = 2000",
            "HOME_MARKER_CLEAR_TIMEOUT_MS = 2000",
            "byteCount != 2",
            "publishAppliedCommand(command, sequence)",
            "BENCH_RFID_ONLY_MODE = false",
        ):
            self.assertIn(token, self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
