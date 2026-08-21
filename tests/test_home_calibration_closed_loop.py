"""Offline closed-loop regression for the reboot-safe HOME calibration gate.

This test module intentionally opens no COM port, socket, or database.  It
connects the real server command/revision/ACK functions to the MotorUno
protocol model and a small boot-state model of SensorUno.  Physical facts the
software cannot create (the two IR levels at HOME and a board reboot) are
injected explicitly by each test.
"""

from __future__ import annotations

import unittest
from dataclasses import dataclass, field
from typing import Any
from unittest.mock import patch

try:  # ``python -m unittest tests...`` package import
    from .closed_loop_virtual_sim import SERVER
    from .test_robot_protocol_integration import (
        ActuatorCommand,
        ActuatorUnoSim,
        MotorCommand,
        MotorStatus,
        MotorUnoSim,
    )
except ImportError:  # direct script and ``unittest discover -s tests``
    from closed_loop_virtual_sim import SERVER
    from test_robot_protocol_integration import (
        ActuatorCommand,
        ActuatorUnoSim,
        MotorCommand,
        MotorStatus,
        MotorUnoSim,
    )


@dataclass(frozen=True)
class CommandOutcome:
    revision: int
    result: str
    event: str
    motor_ack: tuple[MotorStatus, int, int] | None


@dataclass
class BootSafeSensorSim:
    """Minimal SensorUno boot/calibration behavior around the real motor model.

    SensorUno deliberately owns its route certainty.  Therefore constructing a
    new instance never infers HOME from MotorUno memory, even if that MotorUno
    remained powered and calibrated.
    """

    motor: MotorUnoSim = field(
        default_factory=lambda: MotorUnoSim(
            calibrated=False, protocol_validated=False
        )
    )
    actuator: ActuatorUnoSim = field(default_factory=ActuatorUnoSim)
    route_calibrated: bool = False
    confirmed_station: str = "UNKNOWN"
    expected_station: str = "UNKNOWN"
    heading: str = "UNKNOWN"
    at_station: bool = False
    phase: str = "TASK_COMPLETE"
    result: str = "BOOT"
    event: str = "CALIBRATION_REQUIRED"
    last_revision: int = -1
    ack_revision: int = -1
    last_motor_ack: tuple[MotorStatus, int, int] | None = None
    left_ir_high: bool = False
    right_ir_high: bool = False

    def set_home_ir(self, left_high: bool, right_high: bool) -> None:
        self.left_ir_high = bool(left_high)
        self.right_ir_high = bool(right_high)
        self.motor.home_marker_present = self.left_ir_high and self.right_ir_high

    def apply_wire_command(
        self, payload: dict[str, Any], now_ms: int
    ) -> CommandOutcome:
        revision = int(payload["revision"])
        command = str(payload["command"]).upper()
        if revision == self.last_revision:
            return self._outcome("IGNORED", "DUPLICATE_REVISION")

        self.last_revision = revision
        self.ack_revision = revision
        if command == "CALIBRATE_HOME":
            return self._calibrate_home(now_ms)
        if command in {"TASK", "RETURN_HOME", "MOTOR_FWD"}:
            if not self.route_calibrated:
                return self._calibration_required(now_ms)
            return self._start_route_command(command, payload, now_ms)
        if command in {"ALL_STOP", "MOTOR_STOP"}:
            self._safe_stop(now_ms, invalidate_route=False)
            self.phase = "IDLE"
            self.result = "COMPLETED"
            self.event = "STOP_CONFIRMED"
            return self._outcome(self.result, self.event)
        raise AssertionError(f"unsupported test command: {command}")

    def _calibrate_home(self, now_ms: int) -> CommandOutcome:
        self.actuator.command(ActuatorCommand.STOP, now_ms)
        protocol_sequence = self.motor.command(MotorCommand.PROTOCOL_SYNC, now_ms)
        protocol_ack = self.motor.reply()
        protocol_ready = protocol_ack == (
            MotorStatus.PROTOCOL_REQUIRED,
            MotorCommand.PROTOCOL_SYNC,
            protocol_sequence,
        )
        sequence = self.motor.command(MotorCommand.HOME_SYNC, now_ms)
        self.last_motor_ack = self.motor.reply()
        exact_ack = self.last_motor_ack[1:] == (
            MotorCommand.HOME_SYNC,
            sequence,
        )
        succeeded = (
            protocol_ready
            and exact_ack
            and self.last_motor_ack[0] == MotorStatus.IDLE
        )
        if not succeeded:
            # HOME_SYNC is a verification command, never a search movement.
            # Its rejected frame is still exactly ACKed with status 7 while
            # the physical motor output remains stopped.
            self._invalidate_route()
            self.phase = "TASK_COMPLETE"
            self.result = "FAILED"
            self.event = "HOME_CALIBRATION_FAILED"
            return self._outcome(self.result, self.event)

        self.route_calibrated = True
        self.confirmed_station = "HOME"
        self.expected_station = "ZONE2"
        self.heading = "OUTBOUND"
        self.at_station = True
        self.phase = "IDLE"
        self.result = "COMPLETED"
        self.event = "HOME_CALIBRATED"
        return self._outcome(self.result, self.event)

    def _calibration_required(self, now_ms: int) -> CommandOutcome:
        self._safe_stop(now_ms, invalidate_route=True)
        self.phase = "TASK_COMPLETE"
        self.result = "FAILED"
        self.event = "CALIBRATION_REQUIRED"
        return self._outcome(self.result, self.event)

    def _start_route_command(
        self, command: str, payload: dict[str, Any], now_ms: int
    ) -> CommandOutcome:
        if command == "RETURN_HOME" and self.at_station:
            if self.confirmed_station == "HOME":
                self.phase = "IDLE"
                self.result = "COMPLETED"
                self.event = "HOME_ALREADY"
                return self._outcome(self.result, self.event)

        motor_command = (
            MotorCommand.RETURN
            if command == "RETURN_HOME" and self.heading != "HOMEBOUND"
            else MotorCommand.OUTBOUND
        )
        sequence = self.motor.command(motor_command, now_ms)
        self.last_motor_ack = self.motor.reply()
        exact_ack = self.last_motor_ack[1:] == (motor_command, sequence)
        if not exact_ack or self.last_motor_ack[0] != MotorStatus.RUNNING:
            # This is how an otherwise-calibrated SensorUno discovers that
            # MotorUno alone rebooted while the coordinator was idle.
            self._safe_stop(now_ms, invalidate_route=True)
            self.phase = "TASK_COMPLETE"
            self.result = "FAILED"
            self.event = "CALIBRATION_REQUIRED"
            return self._outcome(self.result, self.event)

        if command == "MOTOR_FWD":
            # Manual forward intentionally destroys automatic route certainty.
            self._invalidate_route()
            self.phase = "MOVING"
            self.result = "EXECUTING"
            self.event = "MANUAL_MOTOR_FWD"
            return self._outcome(self.result, self.event)

        if command == "TASK":
            target = str(payload.get("target_zone", "UNKNOWN")).upper()
            self.expected_station = "ZONE2"
            self.at_station = False
            self.phase = "MOVING"
            self.result = "EXECUTING"
            self.event = "DISPATCHED"
            # ZONE99 still first expects the intermediate ZONE2 RFID.
            if target not in {"ZONE2", "ZONE99"}:
                raise AssertionError(f"invalid target in test: {target}")
        else:
            self.heading = "HOMEBOUND"
            self.expected_station = (
                "HOME" if self.confirmed_station == "ZONE2" else "ZONE2"
            )
            self.at_station = False
            self.phase = "RETURNING"
            self.result = "EXECUTING"
            self.event = "RETURN_STARTED"
        return self._outcome(self.result, self.event)

    def finish_operation_at(self, station: str, now_ms: int) -> None:
        """Place the modeled robot in a known, safely stopped post-task state."""

        self.motor.command(MotorCommand.STOP, now_ms)
        self.actuator.command(ActuatorCommand.STOP, now_ms)
        self.confirmed_station = station
        self.expected_station = "ZONE99" if station == "ZONE2" else "ZONE2"
        self.at_station = True
        self.phase = "TASK_COMPLETE"
        self.result = "COMPLETED"
        self.event = "MODULE_COMPLETE"

    def motor_only_reboot(self) -> None:
        self.motor = MotorUnoSim(calibrated=False, protocol_validated=False)
        self.last_motor_ack = self.motor.reply()

    def service_motor_link(self, now_ms: int) -> bool:
        """Model the 400 ms status poll made while SensorUno is moving."""

        status, _command, _sequence = self.motor.reply()
        if status not in (
            MotorStatus.CALIBRATION_REQUIRED,
            MotorStatus.PROTOCOL_REQUIRED,
        ):
            return True
        self._safe_stop(now_ms, invalidate_route=True)
        self.phase = "TASK_COMPLETE"
        self.result = "FAILED"
        self.event = "CALIBRATION_REQUIRED"
        return False

    def _safe_stop(self, now_ms: int, *, invalidate_route: bool) -> None:
        self.motor.command(MotorCommand.STOP, now_ms)
        self.actuator.command(ActuatorCommand.STOP, now_ms)
        if invalidate_route:
            self._invalidate_route()

    def _invalidate_route(self) -> None:
        self.route_calibrated = False
        self.confirmed_station = "UNKNOWN"
        self.expected_station = "UNKNOWN"
        self.heading = "UNKNOWN"
        self.at_station = False

    def _outcome(self, result: str, event: str) -> CommandOutcome:
        return CommandOutcome(
            revision=self.ack_revision,
            result=result,
            event=event,
            motor_ack=self.last_motor_ack,
        )


class HomeCalibrationClosedLoopTests(unittest.TestCase):
    def setUp(self) -> None:
        with SERVER.MANUAL_CONTROL_LOCK:
            self._manual_backup = dict(SERVER.MANUAL_CONTROL)
            SERVER.MANUAL_CONTROL.update(
                enabled=False,
                revision=200,
                command="MOTOR_STOP",
                target_zone="HOME",
                action="NONE",
                updated_at=200,
            )
        with SERVER.ROBOT_NETWORK_LOCK:
            self._network_backup = dict(SERVER.ROBOT_NETWORK_STATE)
            SERVER.ROBOT_NETWORK_STATE.update(
                phase="UNKNOWN",
                event="WAITING",
                zone="HOME",
                action="NONE",
                reported_at=None,
                last_seen=None,
                ip=None,
                delivered_revision=None,
                delivered_at=None,
                ack_revision=None,
                ack_result=None,
                acknowledged_at=None,
                completed_auto_revision=None,
                completed_auto_at=None,
                completed_auto_zone=None,
                completed_auto_action=None,
            )

    def tearDown(self) -> None:
        with SERVER.MANUAL_CONTROL_LOCK:
            SERVER.MANUAL_CONTROL.clear()
            SERVER.MANUAL_CONTROL.update(self._manual_backup)
        with SERVER.ROBOT_NETWORK_LOCK:
            SERVER.ROBOT_NETWORK_STATE.clear()
            SERVER.ROBOT_NETWORK_STATE.update(self._network_backup)

    def test_boot_uncalibrated_blocks_every_route_command(self) -> None:
        sensor = BootSafeSensorSim()
        commands = (
            (1, "TASK", "ZONE2", "HUMIDIFY"),
            (2, "RETURN_HOME", "HOME", "NONE"),
            (3, "MOTOR_FWD", "HOME", "NONE"),
        )
        for revision, command, zone, action in commands:
            with self.subTest(command=command):
                outcome = sensor.apply_wire_command(
                    {
                        "revision": revision,
                        "command": command,
                        "target_zone": zone,
                        "action": action,
                    },
                    revision * 10,
                )
                self.assertEqual(outcome.result, "FAILED")
                self.assertEqual(outcome.event, "CALIBRATION_REQUIRED")
                self.assertEqual(outcome.revision, revision)
                self.assertFalse(sensor.route_calibrated)
                self.assertEqual(sensor.confirmed_station, "UNKNOWN")
                self.assertEqual(sensor.expected_station, "UNKNOWN")
                self.assertFalse(sensor.at_station)
                self.assertEqual(sensor.motor.active, MotorCommand.STOP)
                self.assertFalse(sensor.actuator.humidifier_on)
                self.assertFalse(sensor.actuator.peltier_on)
                self.assertFalse(sensor.actuator.fan_on)

    def test_home_sync_requires_both_ir_high_and_exactly_acks_each_frame(self) -> None:
        # A new MotorUno must stay released until the 4WD command semantics are
        # explicitly synchronized; a legacy SensorUno cannot open calibration.
        unsynchronized = MotorUnoSim(
            calibrated=False, protocol_validated=False, home_marker_present=True
        )
        sequence = unsynchronized.command(MotorCommand.HOME_SYNC, 0)
        self.assertEqual(
            unsynchronized.reply(),
            (
                MotorStatus.PROTOCOL_REQUIRED,
                MotorCommand.HOME_SYNC,
                sequence,
            ),
        )
        self.assertEqual(unsynchronized.active, MotorCommand.STOP)

        sensor = BootSafeSensorSim()
        for revision, levels in enumerate(
            ((False, False), (True, False), (False, True)), start=10
        ):
            with self.subTest(left=levels[0], right=levels[1]):
                sensor.set_home_ir(*levels)
                outcome = sensor.apply_wire_command(
                    {
                        "revision": revision,
                        "command": "CALIBRATE_HOME",
                        "target_zone": "HOME",
                        "action": "NONE",
                    },
                    revision * 10,
                )
                self.assertEqual(outcome.result, "FAILED")
                self.assertEqual(outcome.event, "HOME_CALIBRATION_FAILED")
                self.assertIsNotNone(outcome.motor_ack)
                status, applied, sequence = outcome.motor_ack  # type: ignore[misc]
                self.assertEqual(status, MotorStatus.CALIBRATION_REQUIRED)
                self.assertEqual(applied, MotorCommand.HOME_SYNC)
                self.assertGreaterEqual(sequence, 0)
                self.assertEqual(sensor.motor.active, MotorCommand.STOP)
                self.assertFalse(sensor.route_calibrated)

        sensor.set_home_ir(True, True)
        outcome = sensor.apply_wire_command(
            {
                "revision": 20,
                "command": "CALIBRATE_HOME",
                "target_zone": "HOME",
                "action": "NONE",
            },
            200,
        )
        self.assertEqual(outcome.result, "COMPLETED")
        self.assertEqual(outcome.event, "HOME_CALIBRATED")
        self.assertEqual(
            outcome.motor_ack,
            (
                MotorStatus.IDLE,
                MotorCommand.HOME_SYNC,
                sensor.motor.applied_sequence,
            ),
        )
        self.assertEqual(sensor.motor.active, MotorCommand.STOP)
        self.assertFalse(sensor.motor.line_following_started)
        self.assertTrue(sensor.route_calibrated)
        self.assertEqual(sensor.confirmed_station, "HOME")
        self.assertEqual(sensor.expected_station, "ZONE2")
        self.assertEqual(sensor.heading, "OUTBOUND")
        self.assertTrue(sensor.at_station)

        # A later MotorUno reset returns the observed boot tuple from the
        # Current boot contract: status 8, STOP/seq0, protocol/calibration locked.
        sensor.motor_only_reboot()
        self.assertEqual(
            sensor.motor.reply(),
            (MotorStatus.PROTOCOL_REQUIRED, MotorCommand.STOP, 0),
        )
        self.assertFalse(sensor.motor.calibrated)
        self.assertEqual(sensor.motor.active, MotorCommand.STOP)

    def test_real_server_calibrate_ack_auto_transition_then_new_task_revision(self) -> None:
        sensor = BootSafeSensorSim()
        sensor.set_home_ir(True, True)
        clock = 1_000
        global_revision = 200

        def fake_now() -> int:
            nonlocal clock
            clock += 1
            return clock

        def fake_persist(
            desired: dict[str, Any], timestamp: int, *, enter_auto: bool = False
        ) -> dict[str, Any]:
            nonlocal global_revision
            global_revision = SERVER.next_command_revision(
                global_revision,
                int(desired.get("revision") or 0),
                timestamp=timestamp,
            )
            # ``enter_auto`` changes DB watermarks in production; the command
            # state itself remains the normalized snapshot returned here.
            self.assertEqual(enter_auto, not bool(desired["enabled"]))
            return {
                **desired,
                "revision": global_revision,
                "updated_at": timestamp,
            }

        with (
            patch.object(SERVER, "persist_manual_transition", side_effect=fake_persist),
            patch.object(SERVER, "now", side_effect=fake_now),
            patch.object(SERVER, "touch_device", return_value=None),
            patch.object(SERVER, "record_event", return_value=None),
        ):
            manual = SERVER.update_manual_control(
                {
                    "mode": "MANUAL",
                    "command": "CALIBRATE_HOME",
                    # The real server must normalize these unrelated values.
                    "target_zone": "ZONE99",
                    "action": "DEHUMIDIFY",
                }
            )
            self.assertEqual(manual["command"], "CALIBRATE_HOME")
            self.assertEqual(manual["target_zone"], "HOME")
            self.assertEqual(manual["action"], "NONE")

            effective_manual = {**manual, "source": "MANUAL"}
            with patch.object(
                SERVER, "robot_command_snapshot", return_value=effective_manual
            ):
                wire = SERVER.robot_command()
            self.assertEqual(
                set(wire),
                {"revision", "command", "target_zone", "action"},
            )

            SERVER.mark_command_delivered(manual["revision"], fake_now())
            outcome = sensor.apply_wire_command(wire, now_ms=1_100)
            self.assertEqual(outcome.result, "COMPLETED")
            self.assertEqual(outcome.event, "HOME_CALIBRATED")

            with patch.object(
                SERVER, "robot_command_snapshot", return_value=effective_manual
            ):
                ack = SERVER.report_robot_network(
                    {
                        "phase": ["IDLE"],
                        "event": [outcome.event],
                        "zone": ["HOME"],
                        "action": ["NONE"],
                        "ack_revision": [str(outcome.revision)],
                        "result": [outcome.result],
                    },
                    "192.0.2.55",
                )
            self.assertTrue(ack["ack_accepted"])
            self.assertEqual(ack["ack_revision"], manual["revision"])

            auto = SERVER.update_manual_control({"mode": "AUTO"})
            self.assertFalse(auto["enabled"])
            self.assertGreater(auto["revision"], manual["revision"])

            task_revision = SERVER.next_command_revision(
                auto["revision"], timestamp=fake_now()
            )
            effective_auto = {
                "revision": task_revision,
                "command": "TASK",
                "target_zone": "ZONE99",
                "action": "HUMIDIFY",
                "source": "AUTO",
            }
            with patch.object(
                SERVER, "robot_command_snapshot", return_value=effective_auto
            ):
                task_wire = SERVER.robot_command()
            task_outcome = sensor.apply_wire_command(task_wire, now_ms=1_200)

        self.assertGreater(task_revision, auto["revision"])
        self.assertEqual(task_outcome.revision, task_revision)
        self.assertEqual(task_outcome.result, "EXECUTING")
        self.assertEqual(task_outcome.event, "DISPATCHED")
        self.assertEqual(sensor.phase, "MOVING")
        self.assertEqual(sensor.expected_station, "ZONE2")
        self.assertEqual(sensor.motor.status, MotorStatus.RUNNING)

    def test_motor_only_reboot_invalidates_route_during_and_after_operation(self) -> None:
        # During movement, the normal 400 ms status poll immediately sees 7.
        during = BootSafeSensorSim()
        during.set_home_ir(True, True)
        during.apply_wire_command(
            {
                "revision": 1,
                "command": "CALIBRATE_HOME",
                "target_zone": "HOME",
                "action": "NONE",
            },
            0,
        )
        during.apply_wire_command(
            {
                "revision": 2,
                "command": "TASK",
                "target_zone": "ZONE2",
                "action": "DEHUMIDIFY",
            },
            100,
        )
        self.assertEqual(during.phase, "MOVING")
        during.motor_only_reboot()
        self.assertEqual(during.motor.status, MotorStatus.PROTOCOL_REQUIRED)
        self.assertFalse(during.service_motor_link(500))
        self.assertFalse(during.route_calibrated)
        self.assertEqual(during.confirmed_station, "UNKNOWN")
        self.assertEqual(during.motor.active, MotorCommand.STOP)

        # If MotorUno reboots while SensorUno is stopped after a module run,
        # status 8 is discovered on the next RETURN start and the stale route
        # is invalidated instead of being reused.
        after = BootSafeSensorSim()
        after.set_home_ir(True, True)
        after.apply_wire_command(
            {
                "revision": 10,
                "command": "CALIBRATE_HOME",
                "target_zone": "HOME",
                "action": "NONE",
            },
            0,
        )
        after.apply_wire_command(
            {
                "revision": 11,
                "command": "TASK",
                "target_zone": "ZONE2",
                "action": "HUMIDIFY",
            },
            100,
        )
        after.finish_operation_at("ZONE2", 6_000)
        self.assertTrue(after.route_calibrated)
        after.motor_only_reboot()
        outcome = after.apply_wire_command(
            {
                "revision": 12,
                "command": "RETURN_HOME",
                "target_zone": "HOME",
                "action": "NONE",
            },
            6_100,
        )
        self.assertEqual(outcome.result, "FAILED")
        self.assertEqual(outcome.event, "CALIBRATION_REQUIRED")
        self.assertFalse(after.route_calibrated)
        self.assertEqual(after.confirmed_station, "UNKNOWN")
        self.assertEqual(after.expected_station, "UNKNOWN")
        self.assertEqual(after.motor.status, MotorStatus.PROTOCOL_REQUIRED)
        self.assertEqual(after.motor.active, MotorCommand.STOP)

    def test_sensor_reboot_never_assumes_home_from_a_still_calibrated_motor(self) -> None:
        first_sensor = BootSafeSensorSim()
        first_sensor.set_home_ir(True, True)
        first_sensor.apply_wire_command(
            {
                "revision": 1,
                "command": "CALIBRATE_HOME",
                "target_zone": "HOME",
                "action": "NONE",
            },
            0,
        )
        self.assertTrue(first_sensor.motor.calibrated)

        # SensorUno alone reboots; MotorUno has not forgotten its calibration.
        rebooted_sensor = BootSafeSensorSim(motor=first_sensor.motor)
        self.assertTrue(rebooted_sensor.motor.calibrated)
        self.assertFalse(rebooted_sensor.route_calibrated)
        self.assertEqual(rebooted_sensor.confirmed_station, "UNKNOWN")
        self.assertEqual(rebooted_sensor.expected_station, "UNKNOWN")
        self.assertFalse(rebooted_sensor.at_station)

        blocked = rebooted_sensor.apply_wire_command(
            {
                "revision": 2,
                "command": "TASK",
                "target_zone": "ZONE2",
                "action": "HUMIDIFY",
            },
            100,
        )
        self.assertEqual(blocked.event, "CALIBRATION_REQUIRED")
        self.assertEqual(rebooted_sensor.motor.active, MotorCommand.STOP)
        self.assertFalse(rebooted_sensor.route_calibrated)

        rebooted_sensor.set_home_ir(True, True)
        calibrated = rebooted_sensor.apply_wire_command(
            {
                "revision": 3,
                "command": "CALIBRATE_HOME",
                "target_zone": "HOME",
                "action": "NONE",
            },
            200,
        )
        self.assertEqual(calibrated.event, "HOME_CALIBRATED")
        self.assertTrue(rebooted_sensor.route_calibrated)
        self.assertEqual(rebooted_sensor.confirmed_station, "HOME")


if __name__ == "__main__":
    unittest.main()
