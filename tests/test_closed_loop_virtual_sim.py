"""Regression gates for the bounded full-system virtual-clock simulator."""

from __future__ import annotations

import unittest
from pathlib import Path

try:  # ``python -m unittest tests...`` package import
    from .closed_loop_virtual_sim import (
        ClosedLoopMissionSim,
        run_all_nominal_scenarios,
    )
except ImportError:  # ``unittest discover -s tests`` top-level import
    from closed_loop_virtual_sim import ClosedLoopMissionSim, run_all_nominal_scenarios


ROOT = Path(__file__).resolve().parents[1]


class ClosedLoopVirtualClockTests(unittest.TestCase):
    def test_all_zone_and_action_pairs_reach_home_under_deadline(self) -> None:
        results = run_all_nominal_scenarios()
        self.assertEqual(
            {(result.zone, result.action) for result in results},
            {
                ("ZONE2", "HUMIDIFY"),
                ("ZONE2", "DEHUMIDIFY"),
                ("ZONE99", "HUMIDIFY"),
                ("ZONE99", "DEHUMIDIFY"),
            },
        )
        for result in results:
            with self.subTest(zone=result.zone, action=result.action):
                self.assertEqual(result.final_station, "HOME")
                self.assertEqual(result.final_phase, "IDLE")
                self.assertTrue(result.outputs_off)
                self.assertLessEqual(result.final_time_ms - 1_000_000, 15_000)
                self.assertLess(
                    result.task_revision,
                    result.hold_revision,
                    "completion must create a new ALL_STOP hold revision",
                )
                self.assertLess(
                    result.hold_revision,
                    result.return_revision,
                    "fresh normal data must create a new RETURN revision",
                )
                self.assertGreater(
                    result.normal_reading_ms,
                    result.module_completed_ms,
                    "return must be granted only by a post-completion sample",
                )
                self.assertGreater(
                    result.normal_reading_id, result.abnormal_reading_id
                )
                self.assertEqual(len(set(result.dispatched_revisions)), 3)
                self.assertEqual(len(result.dispatched_revisions), 3)

    def test_trace_proves_every_closed_loop_boundary_for_each_pair(self) -> None:
        for zone in ("ZONE2", "ZONE99"):
            for action in ("HUMIDIFY", "DEHUMIDIFY"):
                with self.subTest(zone=zone, action=action):
                    result = ClosedLoopMissionSim(zone, action).run()
                    boundaries = {
                        (entry.component, entry.event) for entry in result.trace
                    }
                    for expected in (
                        ("ZONE_SENSOR", "READING"),
                        ("PC_SERVER", "TASK"),
                        ("SENSOR_UNO", "TASK_DISPATCHED"),
                        ("SENSOR_UNO", "DUPLICATE_IGNORED"),
                        ("RC522", "TARGET"),
                        ("ACTUATOR_UNO", "MODULE_COMPLETE"),
                        ("SENSOR_SERVER_ACK", "MODULE_COMPLETE"),
                        ("PC_SERVER", "ALL_STOP"),
                        ("SENSOR_UNO", "ALL_STOP"),
                        ("PC_SERVER", "RETURN_HOME"),
                        ("SENSOR_UNO", "RETURN_STARTED"),
                        ("LINE_TRACKER", "HOME"),
                        ("SENSOR_SERVER_ACK", "HOME_ARRIVAL"),
                    ):
                        self.assertIn(expected, boundaries)

                    rfid_details = [
                        entry.detail
                        for entry in result.trace
                        if entry.component == "RC522"
                    ]
                    if zone == "ZONE2":
                        self.assertEqual(rfid_details, ["ZONE2"])
                    else:
                        self.assertEqual(
                            rfid_details, ["ZONE2", "ZONE99", "ZONE2"]
                        )


class SimulideStatusReadWorkaroundContractTests(unittest.TestCase):
    """Keep the R260501-only status adapter paired on both proxy sketches."""

    def test_sensor_and_actuator_share_six_single_byte_status_adapter(self) -> None:
        sensor = (
            ROOT
            / "simulide"
            / "firmware"
            / "sensor_uno_3uno_proxy"
            / "sensor_uno_3uno_proxy.ino"
        ).read_text(encoding="utf-8")
        actuator = (
            ROOT
            / "simulide"
            / "firmware"
            / "actuator_uno_i2c_proxy"
            / "actuator_uno_i2c_proxy.ino"
        ).read_text(encoding="utf-8")

        for source in (sensor, actuator):
            self.assertIn("STATUS_BYTE_SELECT_BASE = 0xF0", source)
            self.assertIn("STATUS_REPLY_SIZE = 6", source)

        self.assertIn("readSelectedStatus(ACTUATOR_ADDRESS", sensor)
        self.assertIn("sequence != lastActuatorSequence", sensor)
        self.assertIn("delay(2);", sensor)
        self.assertIn("delay(1);", sensor)

        self.assertIn("selectedStatusByte = value - STATUS_BYTE_SELECT_BASE", actuator)
        self.assertIn("Wire.write(statusReply[index])", actuator)
        self.assertIn("CONTROL_FRAME_MAGIC = 0xA5", actuator)
        self.assertIn("CONTROL_FRAME_SIZE = 4", actuator)
        self.assertIn("crc8Atm(frame, 3) != frame[3]", actuator)

    def test_proxy_boot_requires_stationary_home_sync_before_task(self) -> None:
        sensor = (
            ROOT
            / "simulide"
            / "firmware"
            / "sensor_uno_3uno_proxy"
            / "sensor_uno_3uno_proxy.ino"
        ).read_text(encoding="utf-8")
        motor = (
            ROOT
            / "simulide"
            / "firmware"
            / "motor_uno_i2c_proxy"
            / "motor_uno_i2c_proxy.ino"
        ).read_text(encoding="utf-8")

        self.assertIn("MOTOR_HOME_SYNC = 6", sensor)
        self.assertIn("bool routeCalibrated = false", sensor)
        self.assertIn("if (!requireHomeCalibration()) return;", sensor)
        self.assertIn("sendMotor(MOTOR_HOME_SYNC)", sensor)
        self.assertIn("mode = SAFE_STOP", sensor)

        self.assertIn("COMMAND_HOME_SYNC = 6", motor)
        self.assertIn("STATUS_CALIBRATION_REQUIRED = 7", motor)
        self.assertIn("bool calibrated = false", motor)
        self.assertIn("enterSafeStop(STATUS_CALIBRATION_REQUIRED)", motor)
        self.assertIn("digitalRead(LEFT_IR_PIN) != HIGH", motor)
        self.assertIn("digitalRead(RIGHT_IR_PIN) != HIGH", motor)
        # HOME_SYNC itself only checks the marker after forcing all motor LEDs off.
        home_sync = motor[motor.index("void applyHomeSync()") : motor.index("void applyCommand(")]
        safe_stop = motor[motor.index("void enterSafeStop(") : motor.index("void applyHomeSync()")]
        self.assertIn("outputsOff();", safe_stop)
        self.assertLess(
            home_sync.index("enterSafeStop(STATUS_CALIBRATION_REQUIRED)"),
            home_sync.index("digitalRead(LEFT_IR_PIN)"),
        )
        marker_failure = home_sync.index("marker absent")
        calibration_success = home_sync.index("calibrated = true")
        self.assertLess(marker_failure, calibration_success)

        calibrate = sensor[
            sensor.index("bool performHomeCalibration()") :
            sensor.index("void startTask(")
        ]
        self.assertLess(
            calibrate.index("routeCalibrated = false"),
            calibrate.index("sendMotor(MOTOR_HOME_SYNC)"),
        )
        self.assertLess(
            calibrate.index("sendMotor(MOTOR_HOME_SYNC)"),
            calibrate.index("routeCalibrated = true"),
        )


if __name__ == "__main__":
    unittest.main()
