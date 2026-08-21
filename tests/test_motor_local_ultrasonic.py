"""MotorUno 후방 HC-SR04 로컬 안전 정지 계약 테스트."""

from __future__ import annotations

import re
import unittest
from dataclasses import dataclass
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "firmware"
    / "uno_line_tracker_motor_controller"
    / "uno_line_tracker_motor_controller.ino"
)


def function_body(source: str, name: str) -> str:
    """중괄호 깊이를 이용해 간단한 Arduino 함수 본문을 반환한다."""
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


@dataclass
class LocalUltrasonicModel:
    """펌웨어의 후진 전용 래치/히스테리시스를 작게 모델링한다."""

    reverse: bool = False
    sensor_pause: bool = False
    local_pause: bool = False
    clear_streak: int = 0

    @property
    def moving(self) -> bool:
        return self.reverse and not self.sensor_pause and not self.local_pause

    def sensor_resume(self) -> None:
        self.sensor_pause = False

    def sample(self, kind: str, distance_cm: int | None = None) -> None:
        if not self.reverse:
            return
        if kind == "stuck_high":
            self.local_pause = True
            self.clear_streak = 0
            return
        if kind in {"no_echo", "out_of_range"}:
            self.clear_streak = 0
            return
        if kind != "valid" or distance_cm is None:
            raise ValueError("valid sample requires a distance")
        if distance_cm < 15:
            self.local_pause = True
            self.clear_streak = 0
            return
        if not self.local_pause:
            self.clear_streak = 0
            return
        if distance_cm < 18:
            self.clear_streak = 0
            return
        self.clear_streak += 1
        if self.clear_streak >= 3:
            self.local_pause = False
            self.clear_streak = 0


class MotorLocalUltrasonicBehaviorTests(unittest.TestCase):
    def test_near_obstacle_only_pauses_reverse(self) -> None:
        model = LocalUltrasonicModel(reverse=False)
        model.sample("valid", 5)
        self.assertFalse(model.local_pause)

        model.reverse = True
        model.sample("valid", 14)
        self.assertTrue(model.local_pause)
        self.assertFalse(model.moving)

    def test_three_consecutive_clear_values_are_required(self) -> None:
        model = LocalUltrasonicModel(reverse=True)
        model.sample("valid", 10)
        model.sample("valid", 18)
        model.sample("valid", 30)
        self.assertTrue(model.local_pause)
        model.sample("valid", 100)
        self.assertFalse(model.local_pause)
        self.assertTrue(model.moving)

    def test_hysteresis_or_invalid_sample_breaks_clear_streak(self) -> None:
        model = LocalUltrasonicModel(reverse=True)
        model.sample("valid", 10)
        model.sample("valid", 30)
        model.sample("no_echo")
        model.sample("valid", 30)
        model.sample("valid", 30)
        self.assertTrue(model.local_pause)
        model.sample("valid", 17)
        self.assertEqual(model.clear_streak, 0)

    def test_stuck_high_is_fatal_but_no_echo_is_diagnostic(self) -> None:
        no_echo = LocalUltrasonicModel(reverse=True)
        no_echo.sample("no_echo")
        self.assertFalse(no_echo.local_pause)

        stuck = LocalUltrasonicModel(reverse=True)
        stuck.sample("stuck_high")
        self.assertTrue(stuck.local_pause)

    def test_sensor_resume_cannot_clear_motor_local_latch(self) -> None:
        model = LocalUltrasonicModel(reverse=True, sensor_pause=True)
        model.sample("valid", 8)
        model.sensor_resume()
        self.assertFalse(model.sensor_pause)
        self.assertTrue(model.local_pause)
        self.assertFalse(model.moving)


class MotorLocalUltrasonicSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")

    def test_pins_thresholds_and_interrupt_capture_are_explicit(self) -> None:
        for pattern in (
            r"ULTRASONIC_ECHO_PIN\s*=\s*2",
            r"ULTRASONIC_TRIG_PIN\s*=\s*A1",
            r"ULTRASONIC_STOP_CM\s*=\s*15",
            r"ULTRASONIC_CLEAR_CM\s*=\s*18",
            r"ULTRASONIC_CLEAR_STREAK_REQUIRED\s*=\s*3",
        ):
            self.assertRegex(self.source, pattern)
        self.assertIn(
            "attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN)",
            self.source,
        )
        self.assertIn("captureUltrasonicEcho, CHANGE", self.source)

    def test_echo_isr_only_captures_edge_timestamps(self) -> None:
        body = function_body(self.source, "captureUltrasonicEcho")
        self.assertIn("micros()", body)
        self.assertIn("ultrasonicEchoPulseReady = true;", body)
        for forbidden in (
            "Serial",
            "Wire",
            "stopMotors",
            ".run(",
            "delay(",
            "delayMicroseconds",
        ):
            self.assertNotIn(forbidden, body)

    def test_local_latch_is_reverse_only_and_handles_failure_classes(self) -> None:
        reverse_guard = function_body(self.source, "reverseUltrasonicControlActive")
        self.assertIn("activeCommand == COMMAND_RETURN", reverse_guard)
        self.assertIn("headingHomebound", reverse_guard)

        sample = function_body(self.source, "applyUltrasonicSample")
        self.assertIn("ULTRASONIC_SAMPLE_STUCK_HIGH", sample)
        self.assertIn("ULTRASONIC_SAMPLE_NO_ECHO", sample)
        self.assertIn("ULTRASONIC_SAMPLE_OUT_OF_RANGE", sample)
        self.assertIn("distanceCm < ULTRASONIC_STOP_CM", sample)
        self.assertIn("distanceCm < ULTRASONIC_CLEAR_CM", sample)
        self.assertIn(
            "ultrasonicClearStreak < ULTRASONIC_CLEAR_STREAK_REQUIRED",
            sample,
        )

    def test_server_resume_does_not_clear_local_obstacle(self) -> None:
        apply_command = function_body(self.source, "applyCommand")
        resume_start = apply_command.index("if (command == COMMAND_RESUME)")
        movement_start = apply_command.index(
            "const bool nextHomebound = command == COMMAND_RETURN;",
            resume_start,
        )
        resume_branch = apply_command[resume_start:movement_start]
        self.assertIn("if (localObstaclePauseActive)", resume_branch)
        self.assertNotIn("localObstaclePauseActive = false", resume_branch)

    def test_local_pause_releases_all_motors_before_line_following(self) -> None:
        latch = function_body(self.source, "latchLocalObstacle")
        self.assertIn("stopMotors();", latch)
        self.assertIn("motorStatus = STATUS_OBSTACLE;", latch)

        loop = function_body(self.source, "loop")
        self.assertLess(loop.index("serviceRearUltrasonic();"), loop.index("followLine();"))
        self.assertIn("localObstaclePauseActive", loop)


if __name__ == "__main__":
    unittest.main(verbosity=2)
