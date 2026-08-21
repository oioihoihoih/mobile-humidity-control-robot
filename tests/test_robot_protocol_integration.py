"""Offline integration regression for the three-Uno robot protocol.

This is an executable specification of the small I2C/HTTP state machines used
by the real sketches.  It deliberately opens no COM port, socket, or database,
so it is safe to run on a development PC while the physical robot is powered.

Run from the repository root with::

    python -m unittest discover -s tests -v

The final source-contract test also checks that the important pin assignments,
I2C values, server address, watchdog, and Wi-Fi backoff in the sketches have not
silently drifted away from this model.
"""

from __future__ import annotations

import re
import unittest
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SENSOR_SOURCE = ROOT / "firmware/uno_robot_esp01_rfid_relay/uno_robot_esp01_rfid_relay.ino"
MOTOR_SOURCE = ROOT / "firmware/uno_line_tracker_motor_controller/uno_line_tracker_motor_controller.ino"
ACTUATOR_SOURCE = ROOT / "firmware/uno_humidity_module_controller/uno_humidity_module_controller.ino"
NETWORK_SOURCE = ROOT / "firmware/uno_robot_esp01_rfid_relay/robot_network_config.example.h"
SENSOR_DIAGNOSTIC_SOURCE = ROOT / "firmware/uno_sensor_pin_diagnostic/uno_sensor_pin_diagnostic.ino"
ZONE2_DRIVE_DIAGNOSTIC_SOURCE = ROOT / "firmware/uno_zone2_rfid_drive_diagnostic/uno_zone2_rfid_drive_diagnostic.ino"


class MotorCommand(IntEnum):
    STOP = 0
    OUTBOUND = 0x11
    RETURN = 0x12
    PAUSE = 3
    RESUME = 4
    KEEPALIVE = 5
    HOME_SYNC = 6
    PROTOCOL_SYNC = 7


class MotorStatus(IntEnum):
    IDLE = 0
    RUNNING = 1
    OBSTACLE = 2
    STOP_LINE = 3
    WATCHDOG = 4
    INVALID = 5
    UNEXPECTED_MARKER = 6
    CALIBRATION_REQUIRED = 7
    PROTOCOL_REQUIRED = 8


class ActuatorCommand(IntEnum):
    STOP = 0
    HUMIDIFY = 1
    DEHUMIDIFY = 2


class ActuatorStatus(IntEnum):
    IDLE = 0
    RUNNING = 1
    DONE = 2
    ERROR = 3


class DisplayState(IntEnum):
    IDLE = 0
    MOVING = 1
    HUMIDIFY = 2
    DEHUMIDIFY = 3
    DONE = 4
    RETURNING = 5
    ERROR = 6


DISPLAY_FRAME_MAGIC = 0xD1
DISPLAY_FRAME_SIZE = 10
ACTUATOR_CONTROL_MAGIC = 0xA5
ACTUATOR_CONTROL_FRAME_SIZE = 4
ACTUATOR_STATUS_REPLY_SIZE = 6
DISPLAY_VALID = 0x01
LCD_READY = 0x02
LCD_ERROR = 0x04
DISPLAY_STALE = 0x08


def crc8_atm(values: bytes | tuple[int, ...] | list[int]) -> int:
    crc = 0
    for value in values:
        crc ^= int(value) & 0xFF
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def display_frame(
    sequence: int,
    state: DisplayState,
    zone_code: int,
    temperature_tenths: int,
    humidity_tenths: int,
    flags: int = 1,
) -> tuple[int, ...]:
    payload = [
        DISPLAY_FRAME_MAGIC,
        sequence & 0xFF,
        int(state),
        zone_code & 0xFF,
        temperature_tenths & 0xFF,
        (temperature_tenths >> 8) & 0xFF,
        humidity_tenths & 0xFF,
        (humidity_tenths >> 8) & 0xFF,
        flags & 0xFF,
    ]
    return tuple(payload + [crc8_atm(payload)])


def actuator_control_frame(
    sequence: int, command: ActuatorCommand
) -> tuple[int, int, int, int]:
    payload = (ACTUATOR_CONTROL_MAGIC, sequence & 0xFF, int(command))
    return payload + (crc8_atm(payload),)


class UltrasonicSample(IntEnum):
    UNKNOWN = 0
    VALID = 1
    NO_ECHO = 2
    STUCK_HIGH = 3
    OUT_OF_RANGE = 4


class RouteStation(IntEnum):
    HOME = 0
    ZONE2 = 1
    ZONE99 = 2


class RouteHeading(IntEnum):
    HOMEBOUND = -1
    OUTBOUND = 1


@dataclass
class LinearRouteSim:
    """HOME(0) -> ZONE2(1) -> ZONE99(2) route coordinator model."""

    confirmed: RouteStation = RouteStation.HOME
    expected: RouteStation = RouteStation.ZONE2
    target: RouteStation = RouteStation.HOME
    heading: RouteHeading = RouteHeading.OUTBOUND
    at_station: bool = True
    moving: bool = False
    waiting_rfid: bool = False
    completed: bool = False
    failed: bool = False
    reverse_last_start: bool = False
    last_rfid: RouteStation | None = None
    last_rfid_ms: int = -1_200
    direction_guard_started_ms: int = 0
    direction_guard_active: bool = False
    direction_clear_seen: bool = False
    route_known: bool = True

    def task(self, destination: RouteStation, now_ms: int = 0) -> None:
        self.target = destination
        self.completed = False
        if self.waiting_rfid and not self.at_station:
            return
        if self.at_station and self.confirmed == destination:
            self.completed = True
            return
        self._travel(destination, now_ms)

    def return_home(self, now_ms: int = 0) -> None:
        self.target = RouteStation.HOME
        if self.at_station and self.confirmed == RouteStation.HOME:
            self.completed = True
            self.moving = False
            return
        if self.moving and self.heading == RouteHeading.HOMEBOUND:
            return
        self._travel(RouteStation.HOME, now_ms)

    def _travel(self, destination: RouteStation, now_ms: int = 0) -> None:
        self.reverse_last_start = False
        previous_heading = self.heading
        if self.at_station:
            desired = (
                RouteHeading.OUTBOUND
                if destination > self.confirmed
                else RouteHeading.HOMEBOUND
            )
            self.heading = desired
            self.expected = RouteStation(self.confirmed + self.heading)
        else:
            ahead = (
                destination >= self.expected
                if self.heading == RouteHeading.OUTBOUND
                else destination <= self.expected
            )
            if not ahead:
                self.heading = (
                    RouteHeading.HOMEBOUND
                    if self.heading == RouteHeading.OUTBOUND
                    else RouteHeading.OUTBOUND
                )
                self.expected = self.confirmed
        # The chassis never turns around. HOMEBOUND means all four wheels run
        # in reverse while the robot keeps the same physical orientation.
        self.reverse_last_start = self.heading == RouteHeading.HOMEBOUND
        if self.heading != previous_heading:
            self.direction_guard_started_ms = now_ms
            self.direction_guard_active = True
            self.direction_clear_seen = False
        self.at_station = False
        self.waiting_rfid = False
        self.moving = True

    def stop_line(self) -> str:
        self.moving = False
        if self.expected == RouteStation.HOME and self.target == RouteStation.HOME:
            self.confirmed = RouteStation.HOME
            self.at_station = True
            self.completed = True
            return "HOME"
        if self.target == RouteStation.HOME and self.heading == RouteHeading.HOMEBOUND:
            # The physical HOME marker is direction-qualified, so it safely
            # recovers position even when the intermediate ZONE2 UID was missed.
            self.confirmed = RouteStation.HOME
            self.expected = RouteStation.HOME
            self.at_station = True
            self.failed = True
            return "HOME_RFID_MISSED"
        self.failed = True
        return "UNEXPECTED_STOP_LINE"

    def rfid(self, station: RouteStation, now_ms: int = 0) -> str:
        if not (self.moving or self.waiting_rfid):
            return "IGNORED"
        if self.direction_guard_active:
            if not self.direction_clear_seen or now_ms - self.direction_guard_started_ms < 850:
                return "DIRECTION_GUARD"
            self.direction_guard_active = False
        if self.last_rfid == station and station != self.expected:
            return "REPEAT_IGNORED"
        self.last_rfid = station
        self.last_rfid_ms = now_ms
        self.moving = False
        if station != self.expected:
            self.confirmed = station
            self.at_station = True
            self.waiting_rfid = False
            self.failed = True
            return "ROUTE_UID_ERROR"
        self.confirmed = station
        self.at_station = True
        self.waiting_rfid = False
        if station == self.target:
            self.completed = True
            return "TARGET"
        self._travel(self.target, now_ms)
        return "PASS"

    def rfid_clear(self, now_ms: int) -> None:
        if self.direction_guard_active:
            self.direction_clear_seen = True

    def obstacle_pause_during_direction_change(self, now_ms: int) -> None:
        if self.direction_guard_active:
            self.direction_guard_started_ms = now_ms
            self.direction_clear_seen = False

    def movement_timeout(self) -> None:
        self.moving = False
        self.failed = True
        self.route_known = False

    def return_timeout(self) -> None:
        self.moving = False
        self.failed = True
        self.route_known = False

    def manual_forward(self) -> None:
        # Manual FWD deliberately ignores route RFID, so any displacement
        # destroys the coordinator's station/segment certainty.
        self.moving = True
        self.route_known = False


@dataclass
class MotorUnoSim:
    """Behavioral model of the I2C 0x08 MotorUno slave."""

    status: MotorStatus = MotorStatus.IDLE
    active: MotorCommand = MotorCommand.STOP
    last_control_ms: int = 0
    stop_line_latched: bool = False
    paused: bool = False
    line_following_started: bool = False
    departure_clearing: bool = False
    stop_line_detection_armed: bool = False
    departure_started_ms: int = 0
    heading_homebound: bool = False
    both_high_started_ms: int | None = None
    applied_command: int = int(MotorCommand.STOP)
    applied_sequence: int = 0
    next_sequence: int = 0
    pending_frame: tuple[int, int] | None = None
    keepalive_pending: bool = False
    # Most historical route tests start from an already synchronized HOME.
    # Reboot/calibration tests explicitly construct calibrated=False.
    calibrated: bool = True
    protocol_validated: bool = True
    home_marker_present: bool = False

    def __post_init__(self) -> None:
        if not self.protocol_validated:
            self._safe_stop(MotorStatus.PROTOCOL_REQUIRED)
        elif not self.calibrated:
            self._safe_stop(MotorStatus.CALIBRATION_REQUIRED)

    def command(self, raw: int, now_ms: int) -> int:
        """Send and synchronously process one frame, as existing tests expect."""
        if raw == MotorCommand.KEEPALIVE:
            sequence = self.applied_sequence
        else:
            self.next_sequence = (self.next_sequence + 1) & 0xFF
            sequence = self.next_sequence
        self.receive_frame((int(raw), sequence))
        self.process_inbox(now_ms)
        return sequence

    def receive_frame(self, frame: tuple[int, ...]) -> None:
        """ISR-side mailbox model: only exact two-byte frames are accepted."""
        if not frame:
            return  # address-only I2C probe
        if len(frame) != 2:
            self.pending_frame = (0xFF, self.applied_sequence)
            return
        raw, sequence = frame
        if raw == MotorCommand.KEEPALIVE:
            self.keepalive_pending = True
        else:
            self.pending_frame = (raw, sequence)

    def process_inbox(self, now_ms: int) -> None:
        pending = self.pending_frame
        keepalive = self.keepalive_pending
        self.pending_frame = None
        self.keepalive_pending = False

        if pending is not None:
            raw, sequence = pending
            if (raw, sequence) == (self.applied_command, self.applied_sequence):
                if raw in set(MotorCommand):
                    self.last_control_ms = now_ms
            else:
                self._apply_command(raw, now_ms)
                self.applied_command = raw
                self.applied_sequence = sequence
        if keepalive:
            self.last_control_ms = now_ms

    def reply(self) -> tuple[MotorStatus, int, int]:
        return self.status, self.applied_command, self.applied_sequence

    def _apply_command(self, raw: int, now_ms: int) -> None:
        if raw not in set(MotorCommand):
            self._safe_stop(MotorStatus.INVALID)
            return
        command = MotorCommand(raw)
        self.last_control_ms = now_ms
        if command == MotorCommand.KEEPALIVE:
            if not self.protocol_validated:
                self._safe_stop(MotorStatus.PROTOCOL_REQUIRED)
            elif not self.calibrated:
                self._safe_stop(MotorStatus.CALIBRATION_REQUIRED)
            return
        if command == MotorCommand.PROTOCOL_SYNC:
            self.protocol_validated = True
            self.calibrated = False
            self._safe_stop(MotorStatus.PROTOCOL_REQUIRED)
            return
        if not self.protocol_validated:
            self._safe_stop(MotorStatus.PROTOCOL_REQUIRED)
            return
        if command == MotorCommand.HOME_SYNC:
            self.calibrated = False
            self._safe_stop(MotorStatus.CALIBRATION_REQUIRED)
            if self.home_marker_present:
                self.heading_homebound = False
                self.calibrated = True
                self.status = MotorStatus.IDLE
            return
        if command == MotorCommand.STOP:
            self._safe_stop(
                MotorStatus.IDLE
                if self.calibrated
                else MotorStatus.CALIBRATION_REQUIRED
            )
            return
        if not self.calibrated:
            self._safe_stop(MotorStatus.CALIBRATION_REQUIRED)
            return
        if command == MotorCommand.PAUSE:
            if self.active != MotorCommand.STOP and not self.stop_line_latched:
                self.paused = True
                self.status = MotorStatus.OBSTACLE
            return
        if command == MotorCommand.RESUME:
            if self.paused and self.active != MotorCommand.STOP and not self.stop_line_latched:
                self.paused = False
                self.status = MotorStatus.RUNNING
            return

        self.active = command
        self.paused = False
        self.stop_line_latched = False
        self._reset_departure()
        self.heading_homebound = command == MotorCommand.RETURN
        self.status = MotorStatus.IDLE if command == MotorCommand.STOP else MotorStatus.RUNNING

    def observe_line(self, left_high: bool, right_high: bool, now_ms: int = 0) -> None:
        if self.active == MotorCommand.STOP or self.paused:
            return

        both_high = left_high and right_high
        if not self.line_following_started:
            self.line_following_started = True
            if both_high and not self.heading_homebound:
                self.departure_clearing = True
                self.departure_started_ms = now_ms
                self.status = MotorStatus.RUNNING
                return
            self.stop_line_detection_armed = True

        if self.departure_clearing:
            if not both_high:
                self.departure_clearing = False
                self.stop_line_detection_armed = True
            elif now_ms - self.departure_started_ms >= 2_000:
                self._safe_stop(MotorStatus.UNEXPECTED_MARKER)
                return
            else:
                self.status = MotorStatus.RUNNING
                return

        if both_high and self.stop_line_detection_armed:
            if self.both_high_started_ms is None:
                self.both_high_started_ms = now_ms
            elif now_ms - self.both_high_started_ms >= 300:
                if self.heading_homebound:
                    self.stop_line_latched = True
                    self.status = MotorStatus.STOP_LINE
                else:
                    self._safe_stop(MotorStatus.UNEXPECTED_MARKER)
        elif not self.stop_line_latched:
            self.both_high_started_ms = None
            self.status = MotorStatus.RUNNING

    def tick(self, now_ms: int) -> None:
        if self.active != MotorCommand.STOP and now_ms - self.last_control_ms > 2_000:
            self._safe_stop(MotorStatus.WATCHDOG)

    def _safe_stop(self, status: MotorStatus) -> None:
        self.active = MotorCommand.STOP
        self.paused = False
        self.stop_line_latched = False
        self.both_high_started_ms = None
        self._reset_departure()
        self.status = status

    def _reset_departure(self) -> None:
        self.line_following_started = False
        self.departure_clearing = False
        self.stop_line_detection_armed = False
        self.departure_started_ms = 0
        self.both_high_started_ms = None


@dataclass
class ActuatorUnoSim:
    """Behavioral model of the I2C 0x09 ActuatorUno slave."""

    status: ActuatorStatus = ActuatorStatus.IDLE
    active: ActuatorCommand = ActuatorCommand.STOP
    started_ms: int = 0
    humidifier_on: bool = False
    peltier_on: bool = False
    fan_on: bool = False
    stage: str = "NONE"
    stage_started_ms: int = 0
    control_pending: tuple[int, ...] | None = None
    applied_sequence: int = 0
    display_pending: tuple[int, ...] | None = None
    last_display_sequence: int = 0
    display_status_flags: int = DISPLAY_STALE
    display_state: DisplayState = DisplayState.IDLE
    display_zone_code: int = 0
    temperature_tenths: int = 0
    humidity_tenths: int = 0
    last_display_ms: int = 0
    lcd_ready: bool = True
    lcd_error: bool = False
    rejected_display_frames: int = 0

    def receive_i2c(self, frame: tuple[int, ...]) -> None:
        """ISR mailbox model: exact 4B control and 10B display coexist."""
        if not frame:
            return
        if len(frame) == ACTUATOR_CONTROL_FRAME_SIZE:
            self.control_pending = frame
        elif len(frame) == DISPLAY_FRAME_SIZE:
            self.display_pending = frame
        else:
            # Ambiguous/malformed control traffic fails safe in the main loop.
            self.control_pending = (0xFF,)

    def process_i2c(self, now_ms: int) -> None:
        # Relay control is always applied before display parsing.
        control = self.control_pending
        self.control_pending = None
        if control is not None:
            if (
                len(control) != ACTUATOR_CONTROL_FRAME_SIZE
                or control[0] != ACTUATOR_CONTROL_MAGIC
                or crc8_atm(control[:3]) != control[3]
            ):
                self._protocol_error()
            else:
                self.command(control[2], now_ms, sequence=control[1])

        frame = self.display_pending
        self.display_pending = None
        if frame is None:
            return
        if (
            frame[0] != DISPLAY_FRAME_MAGIC
            or crc8_atm(frame[:-1]) != frame[-1]
            or frame[2] not in set(DisplayState)
            or frame[3] not in (0, 2, 99, 0xFF)
            or (frame[6] | (frame[7] << 8)) > 1_000
        ):
            self.rejected_display_frames += 1
            return

        self.last_display_sequence = frame[1]
        self.display_state = DisplayState(frame[2])
        self.display_zone_code = frame[3]
        raw_temperature = frame[4] | (frame[5] << 8)
        self.temperature_tenths = (
            raw_temperature - 0x10000 if raw_temperature & 0x8000 else raw_temperature
        )
        self.humidity_tenths = frame[6] | (frame[7] << 8)
        self.last_display_ms = now_ms
        self.display_status_flags = DISPLAY_VALID
        if self.lcd_ready:
            self.display_status_flags |= LCD_READY
        if self.lcd_error:
            self.display_status_flags |= LCD_ERROR

    def display_tick(self, now_ms: int) -> None:
        if self.display_status_flags & DISPLAY_VALID and now_ms - self.last_display_ms >= 30_000:
            self.display_status_flags |= DISPLAY_STALE

    def reply(self) -> tuple[int, int, int, int, int, int]:
        first_five = (
            int(self.status),
            int(self.active),
            self.applied_sequence,
            self.last_display_sequence,
            self.display_status_flags,
        )
        return first_five + (crc8_atm(first_five),)

    def command(self, raw: int, now_ms: int, *, sequence: int | None = None) -> None:
        if sequence is not None:
            sequence &= 0xFF
            if sequence == self.applied_sequence:
                if raw == self.active:
                    return
                self._protocol_error()
                return
            self.applied_sequence = sequence
        if (
            raw in (ActuatorCommand.HUMIDIFY, ActuatorCommand.DEHUMIDIFY)
            and raw == self.active
            and self.status == ActuatorStatus.RUNNING
        ):
            return
        self._outputs_off()
        if raw == ActuatorCommand.STOP:
            self.active = ActuatorCommand.STOP
            self.status = ActuatorStatus.IDLE
            self.stage = "NONE"
        elif raw == ActuatorCommand.HUMIDIFY:
            self.active = ActuatorCommand.HUMIDIFY
            self.status = ActuatorStatus.RUNNING
            self.started_ms = now_ms
            self.stage = "NONE"
            self.humidifier_on = True
        elif raw == ActuatorCommand.DEHUMIDIFY:
            self.active = ActuatorCommand.DEHUMIDIFY
            self.status = ActuatorStatus.RUNNING
            self.fan_on = True
            self.stage = "FAN_PRESTART"
            self.stage_started_ms = now_ms
        else:
            self.active = ActuatorCommand.STOP
            self.status = ActuatorStatus.ERROR
            self.stage = "NONE"

    def _protocol_error(self) -> None:
        self._outputs_off()
        self.active = ActuatorCommand.STOP
        self.status = ActuatorStatus.ERROR
        self.stage = "NONE"

    def tick(self, now_ms: int) -> None:
        if self.status != ActuatorStatus.RUNNING:
            return
        if self.active == ActuatorCommand.DEHUMIDIFY:
            if self.stage == "FAN_PRESTART" and now_ms - self.stage_started_ms >= 500:
                self.peltier_on = True
                self.stage = "PELTIER_RUNNING"
                self.started_ms = now_ms
            elif self.stage == "PELTIER_RUNNING" and now_ms - self.started_ms >= 5_000:
                self.peltier_on = False
                self.stage = "FAN_COOLDOWN"
                self.stage_started_ms = now_ms
            elif self.stage == "FAN_COOLDOWN" and now_ms - self.stage_started_ms >= 2_000:
                self._outputs_off()
                self.stage = "NONE"
                self.status = ActuatorStatus.DONE
        elif now_ms - self.started_ms >= 5_000:
            self._outputs_off()
            # active intentionally remains the completed command, matching the sketch.
            self.status = ActuatorStatus.DONE

    def _outputs_off(self) -> None:
        self.humidifier_on = False
        self.peltier_on = False
        self.fan_on = False


@dataclass
class TwoEventQueue:
    """The SensorUno's SRAM-conscious oldest+latest event queue."""

    pending: str | None = None
    deferred: str | None = None

    def put(self, event: str) -> None:
        if self.pending is None:
            self.pending = event
        elif event not in (self.pending, self.deferred):
            self.deferred = event

    def sent(self) -> None:
        self.pending = self.deferred
        self.deferred = None


@dataclass
class SensorUnoSim:
    """Coordinator model with a rear-facing, best-effort HC-SR04."""

    motor: MotorUnoSim = field(default_factory=MotorUnoSim)
    actuator: ActuatorUnoSim = field(default_factory=ActuatorUnoSim)
    phase: str = "IDLE"
    target_zone: str = "HOME"
    action: str = "NONE"
    task_active: bool = False
    last_revision: int = -1
    ack_revision: int = -1
    result: str = "BOOT"
    events: TwoEventQueue = field(default_factory=TwoEventQueue)
    expected_actuator: ActuatorCommand = ActuatorCommand.STOP
    actuator_sequence_counter: int = 0
    expected_actuator_sequence: int = 0
    actuator_running_seen: bool = False
    last_valid_ultrasonic_ms: int = 0
    ultrasonic_failure_score: int = 0
    ultrasonic_valid_streak: int = 0
    last_ultrasonic_sample: UltrasonicSample = UltrasonicSample.UNKNOWN
    last_ultrasonic_distance_cm: int | None = None
    last_ultrasonic_sample_ms: int = 0
    rfid_ready: bool = True
    rfid_version: int = 0x92

    UID_ZONE = {
        "AA BB CC DD": "ZONE2",
        "11 22 33 44": "ZONE99",
    }

    def command(self, revision: int, command: str, zone: str, action: str, now_ms: int) -> bool:
        """Returns True only when a new revision is executed."""
        if revision == self.last_revision:
            return False

        self.ack_revision = revision
        self.result = "EXECUTING"
        if command == "TASK":
            self.task_active = True
            self.target_zone = zone
            self.action = action
            self._send_actuator(ActuatorCommand.STOP, now_ms)
            # Re-read VersionReg for every automatic departure; a module that
            # was healthy at boot may have lost power or a jumper afterward.
            self.rfid_ready = self.rfid_version not in (0x00, 0xFF)
            if not self.rfid_ready:
                self._fault_stop("RFID_NOT_READY", now_ms)
            else:
                self.motor.command(MotorCommand.OUTBOUND, now_ms)
                self.phase = "MOVING"
                self.events.put("DISPATCHED")
        elif command == "ALL_STOP":
            self._all_stop(now_ms)
            self.phase = "IDLE"
            self.result = "COMPLETED"
            self.events.put("MANUAL_ALL_STOP")
        elif command == "RETURN_HOME":
            should_return = self.task_active or self.phase not in ("IDLE", "RETURNING")
            self.target_zone = "HOME"
            self.action = "NONE"
            self.task_active = False
            if should_return:
                self._send_actuator(ActuatorCommand.STOP, now_ms)
                rear_blocked = (
                    self.last_ultrasonic_sample == UltrasonicSample.STUCK_HIGH
                    or (
                        self.last_ultrasonic_sample == UltrasonicSample.VALID
                        and self.last_ultrasonic_distance_cm is not None
                        and self.last_ultrasonic_distance_cm < 15
                    )
                )
                if rear_blocked:
                    self._fault_stop("REVERSE_START_BLOCKED", now_ms)
                else:
                    self.motor.command(MotorCommand.RETURN, now_ms)
                    self.phase = "RETURNING"
                    self.events.put("RETURN_STARTED")
            else:
                self.phase = "IDLE"
                self.result = "COMPLETED"
                self.events.put("HOME_ALREADY")
        else:
            raise ValueError(f"model does not implement command {command}")

        self.last_revision = revision
        return True

    def process_motor_status(self, now_ms: int) -> None:
        if self.motor.status in (
            MotorStatus.WATCHDOG,
            MotorStatus.INVALID,
            MotorStatus.UNEXPECTED_MARKER,
            MotorStatus.IDLE,
        ):
            if self.phase in ("MOVING", "RETURNING"):
                event = (
                    "MOTOR_MARKER_ERROR"
                    if self.motor.status == MotorStatus.UNEXPECTED_MARKER
                    else "MOTOR_I2C_ERROR"
                )
                self._fault_stop(event, now_ms)
            return
        if self.motor.status != MotorStatus.STOP_LINE:
            return
        self.motor.command(MotorCommand.STOP, now_ms)
        if self.phase == "RETURNING":
            self.phase = "IDLE"
            self.result = "COMPLETED"
            self.events.put("HOME_ARRIVAL")
        elif self.phase == "MOVING" and self.task_active:
            self._fault_stop("UNEXPECTED_STOP_LINE", now_ms)

    def obstacle_distance(self, cm: int | None, now_ms: int) -> None:
        if cm is None:
            sample = UltrasonicSample.NO_ECHO
        elif 2 <= cm <= 400:
            sample = UltrasonicSample.VALID
        else:
            sample = UltrasonicSample.OUT_OF_RANGE
        self.ultrasonic_sample(sample, cm, now_ms)

    def ultrasonic_sample(
        self, sample: UltrasonicSample, cm: int | None, now_ms: int
    ) -> None:
        valid = sample == UltrasonicSample.VALID and cm is not None
        self.last_ultrasonic_sample = sample
        self.last_ultrasonic_distance_cm = cm if valid else None
        self.last_ultrasonic_sample_ms = now_ms
        if valid:
            if now_ms - self.last_valid_ultrasonic_ms > 1_000:
                self.ultrasonic_valid_streak = 0
            self.last_valid_ultrasonic_ms = now_ms
            self.ultrasonic_valid_streak = min(255, self.ultrasonic_valid_streak + 1)
            self.ultrasonic_failure_score = max(0, self.ultrasonic_failure_score - 1)
        else:
            self.ultrasonic_valid_streak = 0
            self.ultrasonic_failure_score = min(3, self.ultrasonic_failure_score + 1)

        # The sensor is mounted on the rear/N20 side. Outbound forward motion
        # is monitor-only; only an actual reverse/homebound run is controlled.
        if self.phase not in ("MOVING", "RETURNING") or not self.motor.heading_homebound:
            return
        pause_required = (
            sample == UltrasonicSample.STUCK_HIGH or (valid and cm < 15)
        )
        if pause_required and not self.motor.paused:
            self.motor.command(MotorCommand.PAUSE, now_ms)
        if pause_required or (valid and self.motor.paused and cm < 18):
            # Clear readings must begin after the last near-obstacle sample.
            self.ultrasonic_valid_streak = 0
        elif (
            valid
            and cm >= 18
            and self.motor.paused
            and self.ultrasonic_valid_streak >= 3
        ):
            self.motor.command(MotorCommand.RESUME, now_ms)

    def keepalive(self, now_ms: int) -> None:
        if self.phase in ("MOVING", "RETURNING"):
            self.motor.command(MotorCommand.KEEPALIVE, now_ms)

    def rfid(self, uid: str, now_ms: int) -> bool:
        zone = self.UID_ZONE.get(uid)
        if (
            self.phase in ("MOVING", "WAITING_RFID")
            and zone is not None
            and zone != self.target_zone
        ):
            self._fault_stop("ROUTE_UID_ERROR", now_ms)
            return False
        if not (
            self.task_active
            and zone == self.target_zone
            and self.phase in ("MOVING", "WAITING_RFID")
        ):
            return False
        self.motor.command(MotorCommand.STOP, now_ms)
        self.events.put("RFID_ARRIVAL")
        return self._start_module(now_ms)

    def _start_module(self, now_ms: int) -> bool:
        if self.action == "NONE":
            self._send_actuator(ActuatorCommand.STOP, now_ms)
            self.phase = "TASK_COMPLETE"
            self.result = "COMPLETED"
            self.events.put("MOVE_ONLY_COMPLETE")
            return True
        actions = {
            "HUMIDIFY": ActuatorCommand.HUMIDIFY,
            "DEHUMIDIFY": ActuatorCommand.DEHUMIDIFY,
        }
        if self.action not in actions:
            self.phase = "TASK_COMPLETE"
            self.result = "INVALID_ACTION"
            self.events.put("INVALID_ACTION")
            return False
        self.expected_actuator = actions[self.action]
        self.actuator_running_seen = False
        self.expected_actuator_sequence = self._send_actuator(
            self.expected_actuator, now_ms
        )
        if (
            self.actuator.status != ActuatorStatus.RUNNING
            or self.actuator.active != self.expected_actuator
            or self.actuator.applied_sequence != self.expected_actuator_sequence
        ):
            self._fault_stop("ACTUATOR_START_ERROR", now_ms)
            return False
        self.actuator_running_seen = True
        self.phase = "MODULE_RUNNING"
        return True

    def poll_actuator(self, now_ms: int) -> None:
        self.actuator.tick(now_ms)
        if self.phase != "MODULE_RUNNING":
            return
        if self.actuator.applied_sequence != self.expected_actuator_sequence:
            self._fault_stop("ACTUATOR_STATUS_ERROR", now_ms)
            return
        if self.actuator.status == ActuatorStatus.RUNNING:
            if self.actuator.active != self.expected_actuator:
                self._fault_stop("ACTUATOR_STATUS_ERROR", now_ms)
            else:
                self.actuator_running_seen = True
        elif (
            self.actuator.status == ActuatorStatus.DONE
            and self.actuator.active == self.expected_actuator
            and self.actuator_running_seen
        ):
            self._send_actuator(ActuatorCommand.STOP, now_ms)
            self.phase = "TASK_COMPLETE"
            self.result = "COMPLETED"
            self.events.put("MODULE_COMPLETE")
        else:
            self._fault_stop("ACTUATOR_STATUS_ERROR", now_ms)

    def server_failure(self, now_ms: int) -> None:
        if self.phase not in ("IDLE", "TASK_COMPLETE"):
            self._all_stop(now_ms)
            self.phase = "TASK_COMPLETE"
            self.result = "SERVER_OFFLINE"
            self.events.put("SERVER_OFFLINE_STOP")

    def _fault_stop(self, event: str, now_ms: int) -> None:
        self._all_stop(now_ms)
        self.phase = "TASK_COMPLETE"
        self.result = "FAILED"
        self.events.put(event)

    def _all_stop(self, now_ms: int) -> None:
        self.motor.command(MotorCommand.STOP, now_ms)
        self._send_actuator(ActuatorCommand.STOP, now_ms)
        self.task_active = False

    def _send_actuator(self, command: ActuatorCommand, now_ms: int) -> int:
        self.actuator_sequence_counter = (self.actuator_sequence_counter + 1) & 0xFF
        self.actuator.command(
            command, now_ms, sequence=self.actuator_sequence_counter
        )
        return self.actuator_sequence_counter


@dataclass
class ServerSim:
    """Minimal command delivery/ACK revision contract from server.py."""

    revision: int = 0
    payload: tuple[str, str, str] = ("RETURN_HOME", "HOME", "NONE")
    delivered_revision: int | None = None
    ack_revision: int | None = None
    ack_result: str | None = None

    def automatic_command(self, command: str, zone: str, action: str) -> int:
        desired = (command, zone, action)
        if desired != self.payload:
            self.revision += 1
            self.payload = desired
        return self.revision

    def deliver(self) -> tuple[int, str, str, str]:
        self.delivered_revision = self.revision
        return (self.revision, *self.payload)

    def ack(self, revision: int, result: str) -> bool:
        if revision != self.delivered_revision or revision != self.revision:
            return False
        if result not in {"EXECUTING", "COMPLETED", "FAILED", "INVALID_ACTION"}:
            return False
        self.ack_revision = revision
        self.ack_result = result
        return True


@dataclass
class SameRevisionRecoverySim:
    """Single-shot recovery contract after a transient server-loss stop.

    A stopped robot must first publish the failure event.  It may then resume
    the unchanged server command exactly once, but only while its route
    position and physical heading are still known.
    """

    command: str
    phase: str
    route_known: bool = True
    failure_report_pending: bool = False
    retry_allowed: bool = False
    restart_count: int = 0

    def server_loss(self) -> None:
        self.phase = "TASK_COMPLETE"
        self.failure_report_pending = True
        self.retry_allowed = self.route_known

    def failure_report_sent(self) -> None:
        self.failure_report_pending = False

    def poll_same_revision(self) -> bool:
        if not self.retry_allowed or self.failure_report_pending:
            return False

        # Consume before attempting the restart so a failed start cannot loop
        # forever on every subsequent poll of the same server revision.
        self.retry_allowed = False
        if self.command == "TASK":
            self.phase = "MOVING"
        elif self.command == "RETURN_HOME":
            self.phase = "RETURNING"
        else:
            return False
        self.restart_count += 1
        return True


@dataclass
class StatusReportDeliverySim:
    """An HTTP 200 is not delivery proof without the matching revision ACK."""

    acknowledged_revision: int
    events: TwoEventQueue = field(default_factory=TwoEventQueue)

    def accept_http_response(self, server_ack_revision: int | None) -> bool:
        if server_ack_revision != self.acknowledged_revision:
            return False
        self.events.sent()
        return True


@dataclass
class EspHttpResponseCollectorSim:
    """Streaming contract for SensorUno's small ESP-01 response buffer.

    ESP-AT can prepend ``SEND OK``/``+IPD`` framing and append ``CLOSED``.
    SensorUno therefore scans the header as a stream, discards it when the
    JSON object starts, and only accepts a complete object.  A SoftwareSerial
    overflow makes the transcript untrustworthy even if the surviving bytes
    happen to contain a plausible HTTP status or JSON fragment.
    """

    buffer_size: int = 128
    payload: str = ""
    http_response_started: bool = False
    http_ok: bool = False
    body_started: bool = False
    body_complete: bool = False
    overflowed: bool = False
    storage_truncated: bool = False

    def collect(
        self, transcript: str, *, software_serial_overflow: bool = False
    ) -> bool:
        self.payload = ""
        self.http_response_started = False
        self.http_ok = False
        self.body_started = False
        self.body_complete = False
        self.overflowed = software_serial_overflow
        self.storage_truncated = False
        header_tail = ""

        if self.overflowed:
            return False

        for character in transcript:
            if not self.body_started:
                # A rolling window prevents a long header from consuming the
                # 128-byte JSON buffer.  It is also sufficient for either
                # supported HTTP status-line spelling.
                header_tail = (header_tail + character)[-32:]
                if "HTTP/1." in header_tail:
                    self.http_response_started = True
                if "HTTP/1.0 200" in header_tail or "HTTP/1.1 200" in header_tail:
                    self.http_ok = True
                if character != "{" or not self.http_response_started:
                    continue
                self.body_started = True

            if self.body_complete:
                # ESP-AT's trailing CRLF/CLOSED belongs to framing, not JSON.
                continue
            if len(self.payload) < self.buffer_size - 1:
                self.payload += character
            else:
                # The application buffer may retain only the response prefix;
                # this is distinct from SoftwareSerial's ISR-ring overflow.
                # Status ACK puts ack_revision in that retained prefix.
                self.storage_truncated = True
            if character == "}":
                self.body_complete = True

        return (
            self.http_ok
            and self.body_started
            and self.body_complete
            and not self.overflowed
        )

    def extract_json_long(self, key: str) -> int | None:
        match = re.search(rf'"{re.escape(key)}":\s*(\d+)', self.payload)
        return int(match.group(1)) if match else None


@dataclass
class FailedStopRecoverySim:
    """Local 500ms STOP retry latch independent of server revisions."""

    phase: str = "MOVING"
    result: str = "EXECUTING"
    route_known: bool = True
    stop_retry_mode: bool = False
    motor_confirmed: bool = False
    actuator_confirmed: bool = False
    last_retry_ms: int = 0
    motor_attempts: int = 0
    actuator_attempts: int = 0
    events: TwoEventQueue = field(default_factory=TwoEventQueue)

    def begin_stop(self, motor_ack: bool, actuator_ack: bool, now_ms: int) -> None:
        self.motor_attempts += 1
        self.actuator_attempts += 1
        self.motor_confirmed = motor_ack
        self.actuator_confirmed = actuator_ack
        if self.motor_confirmed and self.actuator_confirmed:
            self._finish()
            return

        # A missing MotorUno STOP acknowledgement makes physical displacement
        # and heading uncertain.  An Actuator-only STOP failure does not move
        # the chassis, so its already-confirmed route position remains usable.
        if not self.motor_confirmed:
            self.route_known = False
        self.phase = "TASK_COMPLETE"
        self.result = "FAILED"
        self.stop_retry_mode = True
        self.last_retry_ms = now_ms

    def retry(
        self, now_ms: int, *, motor_ack: bool = False, actuator_ack: bool = False
    ) -> bool:
        if not self.stop_retry_mode or now_ms - self.last_retry_ms < 500:
            return False
        self.last_retry_ms = now_ms
        if not self.motor_confirmed:
            self.motor_attempts += 1
            self.motor_confirmed = motor_ack
        if not self.actuator_confirmed:
            self.actuator_attempts += 1
            self.actuator_confirmed = actuator_ack
        if self.motor_confirmed and self.actuator_confirmed:
            self._finish()
            return True
        return False

    def accepts_route_command(self) -> bool:
        return not self.stop_retry_mode

    def _finish(self) -> None:
        self.stop_retry_mode = False
        self.phase = "IDLE"
        self.result = "COMPLETED"
        self.events.put("STOP_CONFIRMED")


@dataclass
class ThreeUnoMissionCoordinatorSim:
    """One coordinator drives the route, MotorUno, and ActuatorUno models.

    Unlike tests that call the three models independently, RFID events enter
    through this object and it owns every STOP/continue/module transition.  It
    is intentionally small, but preserves the production straight-line route
    rule: HOME--ZONE2--ZONE99, with ZONE2 being an intermediate stop-and-resume
    on either leg.
    """

    route: LinearRouteSim = field(default_factory=LinearRouteSim)
    motor: MotorUnoSim = field(default_factory=MotorUnoSim)
    actuator: ActuatorUnoSim = field(default_factory=ActuatorUnoSim)
    phase: str = "IDLE"
    action: ActuatorCommand = ActuatorCommand.STOP
    task_active: bool = False
    module_cycles: int = 0
    last_module_completed_ms: int | None = None

    def task(self, target: RouteStation, action: ActuatorCommand, now_ms: int) -> None:
        self.action = action
        self.task_active = True
        self.route.task(target, now_ms)
        self._start_route_motor(now_ms)
        self.phase = "MOVING"

    def rfid(self, station: RouteStation, now_ms: int) -> str:
        outcome = self.route.rfid(station, now_ms)
        if outcome in ("DIRECTION_GUARD", "REPEAT_IGNORED"):
            return outcome

        self.motor.command(MotorCommand.STOP, now_ms)
        if outcome == "PASS":
            # The coordinator stops to confirm the UID, then reissues the
            # direction command: forward outbound or reverse homebound.
            self._start_route_motor(now_ms + 10)
            self.phase = "RETURNING" if self.route.target == RouteStation.HOME else "MOVING"
        elif outcome == "TARGET":
            self.actuator.command(self.action, now_ms)
            self.phase = "MODULE_RUNNING"
        else:
            self.task_active = False
            self.phase = "TASK_COMPLETE"
        return outcome

    def finish_module(self, now_ms: int) -> None:
        self.actuator.tick(now_ms)
        if self.actuator.status != ActuatorStatus.DONE:
            return
        self.actuator.command(ActuatorCommand.STOP, now_ms)
        self.module_cycles += 1
        self.last_module_completed_ms = now_ms
        self.phase = "TASK_COMPLETE"

    def apply_post_completion_reading(
        self, reading_ms: int, *, still_abnormal: bool
    ) -> str:
        """Model the server's fresh-reading revision grant after one burst."""
        if not (
            self.task_active
            and self.phase == "TASK_COMPLETE"
            and self.route.at_station
            and self.route.confirmed == self.route.target
            and self.last_module_completed_ms is not None
            and reading_ms > self.last_module_completed_ms
        ):
            return "WAIT_FRESH_READING"
        if not still_abnormal:
            self.return_home(reading_ms)
            return "RETURN_HOME"
        self.actuator.command(self.action, reading_ms)
        self.phase = "MODULE_RUNNING"
        return "REPEAT_GRANTED"

    def return_home(self, now_ms: int) -> None:
        self.task_active = False
        self.actuator.command(ActuatorCommand.STOP, now_ms)
        self.route.return_home(now_ms)
        self._start_route_motor(now_ms)
        self.phase = "RETURNING" if self.route.moving else "IDLE"

    def home_marker(self, first_high_ms: int, confirmed_ms: int) -> str:
        self.motor.observe_line(True, True, first_high_ms)
        self.motor.observe_line(True, True, confirmed_ms)
        if self.motor.status != MotorStatus.STOP_LINE:
            return "NO_MARKER"
        outcome = self.route.stop_line()
        self.motor.command(MotorCommand.STOP, confirmed_ms)
        self.phase = "IDLE" if outcome == "HOME" else "TASK_COMPLETE"
        return outcome

    def _start_route_motor(self, now_ms: int) -> None:
        command = (
            MotorCommand.RETURN
            if self.route.heading == RouteHeading.HOMEBOUND
            else MotorCommand.OUTBOUND
        )
        self.motor.command(command, now_ms)


@dataclass
class SoftwareLcdStepperSim:
    """ActuatorUno's bounded D5/D4 LCD render scheduler.

    The real sketch probes 0x27 before 0x3F and performs at most one cursor or
    character transaction per ``serviceLcd`` call.  This model deliberately
    has no relay fields: display failure cannot own or reset actuator timing.
    """

    acknowledged_addresses: frozenset[int] = frozenset({0x27})
    address: int = 0
    ready: bool = False
    error: bool = False
    pending: bool = False
    row: int = 0
    column: int = 0
    needs_cursor: bool = True
    lines: tuple[str, str] = (" " * 16, " " * 16)
    operations: list[tuple[str, int, int | str]] = field(default_factory=list)

    def probe(self) -> int:
        self.address = next(
            (candidate for candidate in (0x27, 0x3F)
             if candidate in self.acknowledged_addresses),
            0,
        )
        self.ready = self.address != 0
        self.error = not self.ready
        return self.address

    def schedule(self, lines: tuple[str, str]) -> None:
        self.lines = tuple(line[:16].ljust(16) for line in lines)
        self.pending = True
        self.row = 0
        self.column = 0
        self.needs_cursor = True

    def step(self, write_ok: bool = True) -> tuple[str, int, int | str] | None:
        if not self.ready or not self.pending:
            return None
        if not write_ok:
            self.ready = False
            self.error = True
            self.address = 0
            self.pending = False
            return None
        if self.needs_cursor:
            operation: tuple[str, int, int | str] = (
                "cursor",
                self.row,
                0x80 if self.row == 0 else 0xC0,
            )
            self.needs_cursor = False
        else:
            operation = ("character", self.row, self.lines[self.row][self.column])
            self.column += 1
            if self.column == 16:
                if self.row == 0:
                    self.row = 1
                    self.column = 0
                    self.needs_cursor = True
                else:
                    self.pending = False
        self.operations.append(operation)
        return operation


class SoftwareLcdStepperModelTests(unittest.TestCase):
    def test_primary_fallback_and_missing_lcd_addresses(self) -> None:
        cases = (
            (frozenset({0x27}), 0x27),
            (frozenset({0x3F}), 0x3F),
            (frozenset({0x27, 0x3F}), 0x27),
            (frozenset(), 0),
        )
        for acknowledged, expected in cases:
            with self.subTest(acknowledged=acknowledged):
                lcd = SoftwareLcdStepperSim(acknowledged)
                self.assertEqual(lcd.probe(), expected)
                self.assertEqual(lcd.ready, expected != 0)
                self.assertEqual(lcd.error, expected == 0)

    def test_full_render_is_split_into_one_bounded_operation_per_call(self) -> None:
        lcd = SoftwareLcdStepperSim()
        self.assertEqual(lcd.probe(), 0x27)
        lcd.schedule(("T25.0C H55.0%", "DEHUM ZONE99"))

        calls = 0
        while lcd.pending:
            before = len(lcd.operations)
            self.assertIsNotNone(lcd.step())
            self.assertEqual(len(lcd.operations), before + 1)
            calls += 1

        # Two cursor writes plus exactly 32 character writes.
        self.assertEqual(calls, 34)
        self.assertEqual(lcd.operations[0], ("cursor", 0, 0x80))
        self.assertEqual(lcd.operations[17], ("cursor", 1, 0xC0))
        self.assertEqual(
            [operation[0] for operation in lcd.operations].count("character"),
            32,
        )

    def test_write_failure_only_disables_the_lcd_state_machine(self) -> None:
        lcd = SoftwareLcdStepperSim()
        lcd.probe()
        lcd.schedule(("DHT22 ERROR", "IDLE HOME"))
        self.assertIsNone(lcd.step(write_ok=False))
        self.assertFalse(lcd.ready)
        self.assertTrue(lcd.error)
        self.assertFalse(lcd.pending)


class IntegrationProtocolTests(unittest.TestCase):
    def new_robot(self) -> SensorUnoSim:
        robot = SensorUnoSim()
        robot.events = TwoEventQueue()
        return robot

    def dispatch(self, robot: SensorUnoSim, revision: int, action: str) -> None:
        self.assertTrue(robot.command(revision, "TASK", "ZONE2", action, 0))
        self.assertEqual(robot.phase, "MOVING")
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)

    def arrive_zone2(self, robot: SensorUnoSim, now_ms: int = 1_000) -> None:
        # ZONE2/ZONE99 have no line marker: the cart is still moving when the
        # RC522 sees the station tag. Each test presents that UID next.
        robot.motor.observe_line(False, False, now_ms)
        robot.process_motor_status(now_ms)
        self.assertEqual(robot.phase, "MOVING")

    def test_motor_reboot_requires_stationary_home_marker_sync_before_motion(self) -> None:
        motor = MotorUnoSim(calibrated=False)
        self.assertEqual(motor.status, MotorStatus.CALIBRATION_REQUIRED)

        rejected_sequence = motor.command(MotorCommand.OUTBOUND, 0)
        self.assertEqual(motor.active, MotorCommand.STOP)
        self.assertEqual(motor.status, MotorStatus.CALIBRATION_REQUIRED)
        self.assertEqual(
            motor.reply()[1:], (MotorCommand.OUTBOUND, rejected_sequence)
        )

        motor.home_marker_present = False
        failed_sync_sequence = motor.command(MotorCommand.HOME_SYNC, 10)
        self.assertFalse(motor.calibrated)
        self.assertEqual(motor.active, MotorCommand.STOP)
        self.assertEqual(motor.status, MotorStatus.CALIBRATION_REQUIRED)
        self.assertEqual(
            motor.reply()[1:], (MotorCommand.HOME_SYNC, failed_sync_sequence)
        )

        motor.home_marker_present = True
        sync_sequence = motor.command(MotorCommand.HOME_SYNC, 20)
        self.assertTrue(motor.calibrated)
        self.assertFalse(motor.heading_homebound)
        self.assertEqual(motor.status, MotorStatus.IDLE)
        self.assertEqual(motor.reply()[1:], (MotorCommand.HOME_SYNC, sync_sequence))

        motor.command(MotorCommand.OUTBOUND, 30)
        self.assertEqual(motor.status, MotorStatus.RUNNING)

    def test_legacy_direction_values_are_invalid_even_after_v2_calibration(self) -> None:
        motor = MotorUnoSim(calibrated=True, protocol_validated=True)
        for sequence, legacy_value in enumerate((1, 2), start=40):
            motor.receive_frame((legacy_value, sequence))
            motor.process_inbox(sequence)
            self.assertEqual(motor.active, MotorCommand.STOP)
            self.assertEqual(motor.status, MotorStatus.INVALID)
            self.assertEqual(motor.reply()[1:], (legacy_value, sequence))

    def test_stale_motor_status_cannot_ack_a_new_sequence(self) -> None:
        motor = MotorUnoSim()
        motor.receive_frame((MotorCommand.OUTBOUND, 7))
        motor.process_inbox(0)
        self.assertEqual(
            motor.reply(), (MotorStatus.RUNNING, MotorCommand.OUTBOUND, 7)
        )

        # onReceive has accepted RETURN, but loop has not applied it yet. The
        # stale RUNNING byte alone must not acknowledge sequence 8.
        motor.receive_frame((MotorCommand.RETURN, 8))
        stale = motor.reply()
        self.assertEqual(stale[0], MotorStatus.RUNNING)
        self.assertNotEqual(stale[1:], (MotorCommand.RETURN, 8))

        motor.process_inbox(10)
        self.assertEqual(
            motor.reply(), (MotorStatus.RUNNING, MotorCommand.RETURN, 8)
        )

    def test_keepalive_does_not_overwrite_a_pending_motor_command(self) -> None:
        motor = MotorUnoSim()
        motor.receive_frame((MotorCommand.OUTBOUND, 33))
        motor.receive_frame((MotorCommand.KEEPALIVE, 0))
        self.assertEqual(motor.pending_frame, (MotorCommand.OUTBOUND, 33))
        self.assertTrue(motor.keepalive_pending)

        motor.process_inbox(500)
        self.assertEqual(motor.active, MotorCommand.OUTBOUND)
        self.assertEqual(
            motor.reply(), (MotorStatus.RUNNING, MotorCommand.OUTBOUND, 33)
        )
        self.assertEqual(motor.last_control_ms, 500)

    def test_duplicate_motor_sequence_is_idempotent(self) -> None:
        motor = MotorUnoSim()
        motor.receive_frame((MotorCommand.RETURN, 41))
        motor.process_inbox(100)
        self.assertTrue(motor.heading_homebound)

        # A retry refreshes link liveness and must remain reverse/homebound.
        motor.receive_frame((MotorCommand.RETURN, 41))
        motor.process_inbox(300)
        self.assertTrue(motor.heading_homebound)
        self.assertEqual(motor.last_control_ms, 300)
        self.assertEqual(motor.reply()[1:], (MotorCommand.RETURN, 41))

    def test_motor_sequence_wrap_from_255_to_zero_is_valid(self) -> None:
        motor = MotorUnoSim()
        motor.receive_frame((MotorCommand.OUTBOUND, 255))
        motor.process_inbox(0)
        self.assertEqual(motor.reply()[1:], (MotorCommand.OUTBOUND, 255))

        motor.receive_frame((MotorCommand.RETURN, 0))
        motor.process_inbox(100)
        self.assertEqual(motor.reply()[1:], (MotorCommand.RETURN, 0))

    def test_partial_motor_frame_forces_safe_stop(self) -> None:
        motor = MotorUnoSim()
        motor.command(MotorCommand.OUTBOUND, 0)
        motor.receive_frame((MotorCommand.RETURN,))
        motor.process_inbox(20)
        self.assertEqual(motor.status, MotorStatus.INVALID)
        self.assertEqual(motor.active, MotorCommand.STOP)
        self.assertEqual(motor.applied_command, 0xFF)

        # An address-only probe has no payload and is not a malformed command.
        previous = motor.reply()
        motor.receive_frame(())
        motor.process_inbox(30)
        self.assertEqual(motor.reply(), previous)

    def test_motor_reboot_reply_cannot_match_pre_reboot_ack(self) -> None:
        motor = MotorUnoSim()
        motor.receive_frame((MotorCommand.OUTBOUND, 77))
        motor.process_inbox(0)
        expected = motor.reply()[1:]
        self.assertEqual(expected, (MotorCommand.OUTBOUND, 77))

        rebooted = MotorUnoSim()
        self.assertEqual(rebooted.reply(), (MotorStatus.IDLE, MotorCommand.STOP, 0))
        self.assertNotEqual(rebooted.reply()[1:], expected)

    def test_linear_route_passes_zone2_before_zone99(self) -> None:
        route = LinearRouteSim()
        route.task(RouteStation.ZONE99)
        self.assertFalse(route.reverse_last_start)
        self.assertEqual(route.expected, RouteStation.ZONE2)
        self.assertEqual(route.rfid(RouteStation.ZONE2), "PASS")
        self.assertTrue(route.moving)
        self.assertEqual(route.expected, RouteStation.ZONE99)
        self.assertEqual(route.rfid(RouteStation.ZONE99), "TARGET")

    def test_same_rfid_is_ignored_briefly_after_intermediate_restart(self) -> None:
        route = LinearRouteSim()
        route.task(RouteStation.ZONE99)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_000), "PASS")
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_100), "REPEAT_IGNORED")
        self.assertTrue(route.moving)
        self.assertEqual(route.expected, RouteStation.ZONE99)
        self.assertEqual(route.rfid(RouteStation.ZONE99, 1_500), "TARGET")

    def test_same_intermediate_rfid_is_ignored_after_a_long_track_pause(self) -> None:
        route = LinearRouteSim()
        route.task(RouteStation.ZONE99)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_000), "PASS")

        # A weak battery, wheel slip, or an obstacle can leave the reader over
        # the station for longer than any fixed debounce interval.  Because the
        # just-passed station is behind the robot, it remains harmless until a
        # reroute explicitly makes that station expected again.
        self.assertEqual(route.rfid(RouteStation.ZONE2, 10_000), "REPEAT_IGNORED")
        self.assertTrue(route.moving)
        self.assertFalse(route.failed)
        self.assertEqual(route.expected, RouteStation.ZONE99)
        self.assertEqual(route.rfid(RouteStation.ZONE99, 11_000), "TARGET")

    def test_immediate_reverse_reroute_can_reconfirm_the_same_expected_station(self) -> None:
        route = LinearRouteSim()
        route.task(RouteStation.ZONE99)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_000), "PASS")

        # The server reroutes HOME while the cart has only just left ZONE2.
        # expected becomes ZONE2 again. There is no 180-degree turn interval,
        # so a legitimate reverse-direction crossing is accepted immediately.
        route.return_home(1_050)
        self.assertEqual(route.expected, RouteStation.ZONE2)
        self.assertTrue(route.reverse_last_start)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_100), "DIRECTION_GUARD")
        route.rfid_clear(1_150)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_899), "DIRECTION_GUARD")
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_900), "PASS")
        self.assertEqual(route.expected, RouteStation.HOME)

    def test_obstacle_pause_restarts_the_direction_change_clear_guard(self) -> None:
        route = LinearRouteSim(
            confirmed=RouteStation.ZONE99,
            expected=RouteStation.ZONE99,
            target=RouteStation.ZONE99,
            heading=RouteHeading.OUTBOUND,
        )
        route.return_home(1_000)
        self.assertTrue(route.reverse_last_start)
        route.rfid_clear(1_050)
        route.obstacle_pause_during_direction_change(1_150)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_999), "DIRECTION_GUARD")
        route.rfid_clear(2_000)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 2_001), "PASS")
        self.assertTrue(route.reverse_last_start)

    def test_zone99_return_passes_zone2_then_accepts_home_stop_line(self) -> None:
        route = LinearRouteSim(
            confirmed=RouteStation.ZONE99,
            expected=RouteStation.ZONE99,
            target=RouteStation.ZONE99,
            heading=RouteHeading.OUTBOUND,
        )
        route.return_home()
        self.assertTrue(route.reverse_last_start)
        self.assertEqual(route.expected, RouteStation.ZONE2)
        route.rfid_clear(100)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 850), "PASS")
        self.assertEqual(route.expected, RouteStation.HOME)
        self.assertEqual(route.stop_line(), "HOME")
        self.assertTrue(route.completed)
        self.assertEqual(route.confirmed, RouteStation.HOME)

    def test_full_zone99_module_repeat_then_normal_return_home(self) -> None:
        """Exercise the complete physical order across all three Uno models.

        The older SensorUnoSim only models a direct ZONE2 trip.  This scenario
        deliberately composes the route, motor, and actuator models so a test
        cannot accidentally skip the intermediate ZONE2 RFID on either leg.
        """
        route = LinearRouteSim()
        motor = MotorUnoSim()
        actuator = ActuatorUnoSim()

        # HOME -> ZONE2(pass) -> ZONE99(target), following one continuous line.
        route.task(RouteStation.ZONE99, 0)
        motor.command(MotorCommand.OUTBOUND, 0)
        motor.observe_line(True, True, 0)       # leave the wide HOME marker
        motor.observe_line(False, False, 200)  # continuous line acquired
        self.assertEqual(route.rfid(RouteStation.ZONE2, 1_000), "PASS")
        motor.command(MotorCommand.STOP, 1_000)
        motor.command(MotorCommand.OUTBOUND, 1_010)
        self.assertFalse(motor.heading_homebound)

        self.assertEqual(route.rfid(RouteStation.ZONE99, 3_000), "TARGET")
        motor.command(MotorCommand.STOP, 3_000)
        actuator.command(ActuatorCommand.HUMIDIFY, 3_000)
        self.assertTrue(actuator.humidifier_on)

        # The zone is still abnormal after one five-second burst, so the same
        # action is repeated in place without requiring another RFID scan.
        actuator.tick(8_000)
        self.assertEqual(actuator.status, ActuatorStatus.DONE)
        self.assertFalse(actuator.humidifier_on)
        actuator.command(ActuatorCommand.STOP, 8_010)
        actuator.command(ActuatorCommand.HUMIDIFY, 8_020)
        actuator.tick(13_020)
        self.assertEqual(actuator.status, ActuatorStatus.DONE)
        actuator.command(ActuatorCommand.STOP, 13_030)

        # A normal server reading changes the command to RETURN_HOME. RETURN is
        # physical reverse; after STOP at intermediate ZONE2 it must be reissued
        # as RETURN (not OUTBOUND) until the HOME-only marker is reached.
        route.return_home(13_100)
        motor.command(MotorCommand.RETURN, 13_100)
        self.assertTrue(route.reverse_last_start)
        self.assertTrue(motor.heading_homebound)
        route.rfid_clear(13_300)
        self.assertEqual(route.rfid(RouteStation.ZONE2, 13_950), "PASS")
        motor.command(MotorCommand.STOP, 13_950)
        motor.command(MotorCommand.RETURN, 13_960)
        self.assertTrue(motor.heading_homebound)

        motor.observe_line(False, False, 14_000)
        motor.observe_line(True, True, 15_000)
        motor.observe_line(True, True, 15_300)
        self.assertEqual(motor.status, MotorStatus.STOP_LINE)
        self.assertEqual(route.stop_line(), "HOME")
        motor.command(MotorCommand.STOP, 15_300)

        self.assertEqual(route.confirmed, RouteStation.HOME)
        self.assertTrue(route.at_station)
        self.assertEqual(motor.status, MotorStatus.IDLE)
        self.assertEqual(actuator.status, ActuatorStatus.IDLE)
        self.assertFalse(
            any(
                (
                    actuator.humidifier_on,
                    actuator.peltier_on,
                    actuator.fan_on,
                )
            )
        )

    def test_one_coordinator_runs_the_complete_zone99_round_trip(self) -> None:
        """RFID input alone drives all three Uno models through the full route."""
        car = ThreeUnoMissionCoordinatorSim()

        car.task(RouteStation.ZONE99, ActuatorCommand.HUMIDIFY, 0)
        self.assertEqual(car.phase, "MOVING")
        self.assertEqual(car.route.expected, RouteStation.ZONE2)
        self.assertEqual(car.rfid(RouteStation.ZONE2, 1_000), "PASS")
        self.assertEqual(car.phase, "MOVING")
        self.assertEqual(car.route.expected, RouteStation.ZONE99)
        self.assertEqual(car.motor.active, MotorCommand.OUTBOUND)

        self.assertEqual(car.rfid(RouteStation.ZONE99, 3_000), "TARGET")
        self.assertEqual(car.phase, "MODULE_RUNNING")
        self.assertTrue(car.actuator.humidifier_on)
        car.finish_module(8_000)
        self.assertEqual(car.module_cycles, 1)
        self.assertEqual(car.phase, "TASK_COMPLETE")
        self.assertEqual(car.actuator.status, ActuatorStatus.IDLE)

        # A reading sampled before/at completion cannot trigger another burst.
        self.assertEqual(
            car.apply_post_completion_reading(8_000, still_abnormal=True),
            "WAIT_FRESH_READING",
        )
        self.assertEqual(car.module_cycles, 1)

        # Only a new post-completion abnormal reading grants one more burst;
        # the route remains stopped at the confirmed RFID station.
        self.assertEqual(
            car.apply_post_completion_reading(8_020, still_abnormal=True),
            "REPEAT_GRANTED",
        )
        car.finish_module(13_020)
        self.assertEqual(car.module_cycles, 2)
        self.assertEqual(car.route.confirmed, RouteStation.ZONE99)
        self.assertEqual(car.motor.status, MotorStatus.IDLE)

        self.assertEqual(
            car.apply_post_completion_reading(13_100, still_abnormal=False),
            "RETURN_HOME",
        )
        self.assertEqual(car.phase, "RETURNING")
        self.assertTrue(car.motor.heading_homebound)
        self.assertEqual(car.route.expected, RouteStation.ZONE2)
        car.route.rfid_clear(13_300)
        self.assertEqual(car.rfid(RouteStation.ZONE2, 13_950), "PASS")
        self.assertEqual(car.phase, "RETURNING")
        self.assertEqual(car.route.expected, RouteStation.HOME)
        self.assertTrue(car.motor.heading_homebound)

        self.assertEqual(car.home_marker(15_000, 15_300), "HOME")
        self.assertEqual(car.phase, "IDLE")
        self.assertEqual(car.route.confirmed, RouteStation.HOME)
        self.assertTrue(car.route.at_station)
        self.assertEqual(car.motor.status, MotorStatus.IDLE)
        self.assertEqual(car.actuator.status, ActuatorStatus.IDLE)
        self.assertFalse(
            any(
                (
                    car.actuator.humidifier_on,
                    car.actuator.peltier_on,
                    car.actuator.fan_on,
                )
            )
        )

    def test_fresh_reading_closed_loop_covers_both_zones_and_actions(self) -> None:
        for target in (RouteStation.ZONE2, RouteStation.ZONE99):
            for action in (
                ActuatorCommand.HUMIDIFY,
                ActuatorCommand.DEHUMIDIFY,
            ):
                with self.subTest(target=target.name, action=action.name):
                    car = ThreeUnoMissionCoordinatorSim()
                    car.task(target, action, 0)
                    self.assertEqual(car.rfid(RouteStation.ZONE2, 1_000),
                                     "TARGET" if target == RouteStation.ZONE2 else "PASS")
                    arrival_ms = 1_000
                    if target == RouteStation.ZONE99:
                        self.assertEqual(car.rfid(RouteStation.ZONE99, 3_000), "TARGET")
                        arrival_ms = 3_000

                    if action == ActuatorCommand.HUMIDIFY:
                        self.assertTrue(car.actuator.humidifier_on)
                        completed_ms = arrival_ms + 5_000
                        car.finish_module(completed_ms)
                    else:
                        self.assertTrue(car.actuator.fan_on)
                        self.assertFalse(car.actuator.peltier_on)
                        car.finish_module(arrival_ms + 500)
                        self.assertTrue(car.actuator.peltier_on)
                        car.finish_module(arrival_ms + 5_500)
                        self.assertTrue(car.actuator.fan_on)
                        self.assertFalse(car.actuator.peltier_on)
                        completed_ms = arrival_ms + 7_500
                        car.finish_module(completed_ms)

                    self.assertEqual(car.phase, "TASK_COMPLETE")
                    self.assertEqual(car.module_cycles, 1)
                    self.assertEqual(
                        car.apply_post_completion_reading(
                            completed_ms, still_abnormal=True
                        ),
                        "WAIT_FRESH_READING",
                    )
                    self.assertEqual(
                        car.apply_post_completion_reading(
                            completed_ms + 1, still_abnormal=False
                        ),
                        "RETURN_HOME",
                    )
                    self.assertEqual(car.phase, "RETURNING")

                    home_base = completed_ms + 1_000
                    if target == RouteStation.ZONE99:
                        car.route.rfid_clear(completed_ms + 250)
                        self.assertEqual(
                            car.rfid(RouteStation.ZONE2, home_base), "PASS"
                        )
                    self.assertEqual(
                        car.home_marker(home_base + 1_000, home_base + 1_300),
                        "HOME",
                    )
                    self.assertEqual(car.phase, "IDLE")
                    self.assertFalse(car.actuator.humidifier_on)
                    self.assertFalse(car.actuator.peltier_on)
                    self.assertFalse(car.actuator.fan_on)

    def test_home_marker_recovers_position_when_zone2_rfid_was_missed(self) -> None:
        route = LinearRouteSim(
            confirmed=RouteStation.ZONE99,
            expected=RouteStation.ZONE2,
            target=RouteStation.HOME,
            heading=RouteHeading.HOMEBOUND,
            at_station=False,
            moving=True,
        )
        self.assertEqual(route.stop_line(), "HOME_RFID_MISSED")
        self.assertTrue(route.failed)
        self.assertTrue(route.at_station)
        self.assertEqual(route.confirmed, RouteStation.HOME)
        route.return_home()
        self.assertFalse(route.moving)
        self.assertTrue(route.completed)

    def test_route_timeouts_and_manual_forward_invalidate_position(self) -> None:
        outbound = LinearRouteSim()
        outbound.task(RouteStation.ZONE99)
        outbound.movement_timeout()
        self.assertTrue(outbound.failed)
        self.assertFalse(outbound.route_known)

        returning = LinearRouteSim(
            confirmed=RouteStation.ZONE99,
            expected=RouteStation.ZONE2,
            target=RouteStation.HOME,
            heading=RouteHeading.HOMEBOUND,
            at_station=False,
            moving=True,
        )
        returning.return_timeout()
        self.assertTrue(returning.failed)
        self.assertFalse(returning.route_known)

        manual = LinearRouteSim()
        manual.manual_forward()
        self.assertTrue(manual.moving)
        self.assertFalse(manual.route_known)

    def test_return_reroute_continues_reverse_or_switches_back_to_forward(self) -> None:
        ahead = LinearRouteSim(
            confirmed=RouteStation.ZONE99,
            expected=RouteStation.ZONE2,
            target=RouteStation.HOME,
            heading=RouteHeading.HOMEBOUND,
            at_station=False,
            moving=True,
        )
        ahead.task(RouteStation.ZONE2)
        self.assertTrue(ahead.reverse_last_start)
        self.assertEqual(ahead.heading, RouteHeading.HOMEBOUND)

        behind = LinearRouteSim(
            confirmed=RouteStation.ZONE99,
            expected=RouteStation.ZONE2,
            target=RouteStation.HOME,
            heading=RouteHeading.HOMEBOUND,
            at_station=False,
            moving=True,
        )
        behind.task(RouteStation.ZONE99)
        self.assertFalse(behind.reverse_last_start)
        self.assertEqual(behind.heading, RouteHeading.OUTBOUND)
        self.assertEqual(behind.expected, RouteStation.ZONE99)

    def test_idle_return_is_completed_without_motor_motion(self) -> None:
        route = LinearRouteSim()
        route.return_home()
        self.assertTrue(route.completed)
        self.assertFalse(route.moving)

    def test_out_of_order_route_uid_stops_and_preserves_known_station(self) -> None:
        route = LinearRouteSim()
        route.task(RouteStation.ZONE99)
        self.assertEqual(route.rfid(RouteStation.ZONE99), "ROUTE_UID_ERROR")
        self.assertTrue(route.failed)
        self.assertFalse(route.moving)
        self.assertEqual(route.confirmed, RouteStation.ZONE99)

    def test_outbound_wide_marker_is_not_mistaken_for_a_zone(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 43, "NONE")

        # The robot starts on HOME's marker: it must continue straight rather
        # than immediately announcing arrival.
        robot.motor.observe_line(True, True, 0)
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)
        self.assertTrue(robot.motor.departure_clearing)
        self.assertFalse(robot.motor.stop_line_detection_armed)

        # One sensor leaving the marker arms normal stop-line detection.
        robot.motor.observe_line(True, False, 350)
        self.assertFalse(robot.motor.departure_clearing)
        self.assertTrue(robot.motor.stop_line_detection_armed)

        # ZONEs are RFID-only. A sustained outbound wide marker is a safe fault,
        # never a WAITING_RFID arrival.
        robot.motor.observe_line(True, True, 900)
        robot.motor.observe_line(True, True, 1_200)
        self.assertEqual(robot.motor.status, MotorStatus.UNEXPECTED_MARKER)
        robot.process_motor_status(1_200)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertIn("MOTOR_MARKER_ERROR", (robot.events.pending, robot.events.deferred))

    def test_home_marker_clear_timeout_is_a_distinct_safe_stop(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 44, "NONE")
        robot.motor.observe_line(True, True, 0)
        robot.motor.observe_line(True, True, 1_999)
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)
        robot.motor.observe_line(True, True, 2_000)
        self.assertEqual(robot.motor.status, MotorStatus.UNEXPECTED_MARKER)
        self.assertEqual(robot.motor.active, MotorCommand.STOP)

        robot.process_motor_status(2_000)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")
        self.assertFalse(robot.task_active)
        self.assertIn(
            "MOTOR_MARKER_ERROR",
            (robot.events.pending, robot.events.deferred),
        )

    def test_outbound_marker_clear_timeout_starts_without_a_turn_delay(self) -> None:
        motor = MotorUnoSim()
        motor.command(MotorCommand.OUTBOUND, 100)
        motor.observe_line(True, True, 100)
        self.assertTrue(motor.departure_clearing)
        motor.observe_line(True, True, 2_099)
        self.assertEqual(motor.status, MotorStatus.RUNNING)
        motor.observe_line(True, True, 2_100)
        self.assertEqual(motor.status, MotorStatus.UNEXPECTED_MARKER)

    def test_homebound_restart_on_home_marker_stays_stopped(self) -> None:
        motor = MotorUnoSim(heading_homebound=True)
        motor.command(MotorCommand.RETURN, 0)
        motor.observe_line(True, True, 0)
        self.assertFalse(motor.departure_clearing)
        self.assertTrue(motor.stop_line_detection_armed)

        motor.observe_line(True, True, 300)
        self.assertEqual(motor.status, MotorStatus.STOP_LINE)
        self.assertTrue(motor.stop_line_latched)

    def test_pause_after_home_marker_cannot_hide_stop_line(self) -> None:
        motor = MotorUnoSim(heading_homebound=True)
        motor.command(MotorCommand.RETURN, 0)
        motor.observe_line(True, True, 0)
        motor.observe_line(True, True, 300)
        self.assertEqual(motor.status, MotorStatus.STOP_LINE)

        # A late obstacle PAUSE can be queued before SensorUno polls status.
        # HOME must remain visible and must never turn into OBSTACLE forever.
        motor.command(MotorCommand.PAUSE, 301)
        self.assertEqual(motor.status, MotorStatus.STOP_LINE)
        self.assertTrue(motor.stop_line_latched)
        self.assertFalse(motor.paused)

    def test_stop_invalid_and_watchdog_reset_departure_state(self) -> None:
        for stop_kind in ("STOP", "INVALID", "WATCHDOG"):
            with self.subTest(stop_kind=stop_kind):
                motor = MotorUnoSim()
                motor.command(MotorCommand.OUTBOUND, 0)
                motor.observe_line(True, True, 0)
                self.assertTrue(motor.departure_clearing)

                if stop_kind == "STOP":
                    motor.command(MotorCommand.STOP, 100)
                    expected = MotorStatus.IDLE
                elif stop_kind == "INVALID":
                    motor.command(99, 100)
                    expected = MotorStatus.INVALID
                else:
                    motor.tick(2_001)
                    expected = MotorStatus.WATCHDOG

                self.assertEqual(motor.status, expected)
                self.assertEqual(motor.active, MotorCommand.STOP)
                self.assertFalse(motor.line_following_started)
                self.assertFalse(motor.departure_clearing)
                self.assertFalse(motor.stop_line_detection_armed)

    def test_server_revision_is_stable_and_ack_requires_current_delivery(self) -> None:
        server = ServerSim()
        revision = server.automatic_command("TASK", "ZONE2", "HUMIDIFY")
        self.assertEqual(server.automatic_command("TASK", "ZONE2", "HUMIDIFY"), revision)
        self.assertFalse(server.ack(revision, "EXECUTING"), "not delivered yet")
        self.assertEqual(server.deliver()[0], revision)
        self.assertFalse(server.ack(revision - 1, "COMPLETED"))
        self.assertFalse(server.ack(revision, "MADE_UP"))
        self.assertTrue(server.ack(revision, "EXECUTING"))

    def test_http_200_without_matching_revision_does_not_drop_robot_event(self) -> None:
        report = StatusReportDeliverySim(acknowledged_revision=27)
        report.events.put("RFID_ARRIVAL")

        # The server uses JSON null when it rejects the status ACK.  Neither a
        # rejected nor a stale numeric ACK may remove the queued event.
        self.assertFalse(report.accept_http_response(None))
        self.assertEqual(report.events.pending, "RFID_ARRIVAL")
        self.assertFalse(report.accept_http_response(26))
        self.assertEqual(report.events.pending, "RFID_ARRIVAL")

        self.assertTrue(report.accept_http_response(27))
        self.assertIsNone(report.events.pending)

    def test_task_none_moves_waits_for_correct_rfid_then_completes_with_outputs_off(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 1, "NONE")
        self.arrive_zone2(robot)
        self.assertTrue(robot.rfid("AA BB CC DD", 1_200))
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "COMPLETED")
        self.assertEqual(robot.actuator.status, ActuatorStatus.IDLE)
        self.assertFalse(any((robot.actuator.humidifier_on, robot.actuator.peltier_on, robot.actuator.fan_on)))

    def test_rc522_is_revalidated_after_boot_before_each_task_departure(self) -> None:
        robot = self.new_robot()
        self.assertTrue(robot.rfid_ready, "boot-time RC522 probe was healthy")
        robot.rfid_version = 0x00  # power/jumper lost after setup

        self.assertTrue(robot.command(402, "TASK", "ZONE2", "NONE", 0))
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")
        self.assertEqual(robot.motor.status, MotorStatus.IDLE)
        self.assertFalse(robot.rfid_ready)
        self.assertEqual(robot.events.pending, "RFID_NOT_READY")

    def test_out_of_order_registered_rfid_is_a_safe_stop(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 11, "HUMIDIFY")
        self.arrive_zone2(robot)
        self.assertFalse(robot.rfid("11 22 33 44", 1_100))
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")
        self.assertEqual(robot.motor.status, MotorStatus.IDLE)
        self.assertIn("ROUTE_UID_ERROR", (robot.events.pending, robot.events.deferred))

    def test_humidify_requires_matching_running_then_done(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 2, "HUMIDIFY")
        self.arrive_zone2(robot)
        self.assertTrue(robot.rfid("AA BB CC DD", 1_000))
        self.assertTrue(robot.actuator.humidifier_on)
        self.assertFalse(robot.actuator.peltier_on)
        robot.poll_actuator(6_000)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "COMPLETED")
        self.assertEqual(robot.actuator.status, ActuatorStatus.IDLE)

    def test_dehumidify_runs_peltier_and_fan_and_rejects_mismatched_echo(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 3, "DEHUMIDIFY")
        self.arrive_zone2(robot)
        self.assertTrue(robot.rfid("AA BB CC DD", 500))
        self.assertFalse(robot.actuator.peltier_on)
        self.assertTrue(robot.actuator.fan_on)
        robot.actuator.tick(1_000)
        self.assertTrue(robot.actuator.peltier_on)
        robot.actuator.active = ActuatorCommand.HUMIDIFY  # stale/wrong I2C echo
        robot.poll_actuator(1_100)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")
        self.assertFalse(robot.actuator.peltier_on)
        self.assertFalse(robot.actuator.fan_on)

    def test_done_echo_must_still_match_the_command_that_was_started(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 31, "HUMIDIFY")
        self.arrive_zone2(robot)
        robot.rfid("AA BB CC DD", 500)
        self.assertTrue(robot.actuator_running_seen)
        robot.actuator.status = ActuatorStatus.DONE
        robot.actuator.active = ActuatorCommand.DEHUMIDIFY
        robot.poll_actuator(600)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")

    def test_old_actuator_running_sequence_is_not_a_new_task_ack(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 32, "HUMIDIFY")
        self.arrive_zone2(robot)
        robot.rfid("AA BB CC DD", 500)
        expected = robot.expected_actuator_sequence

        robot.actuator.status = ActuatorStatus.RUNNING
        robot.actuator.active = ActuatorCommand.HUMIDIFY
        robot.actuator.applied_sequence = (expected - 1) & 0xFF
        robot.poll_actuator(600)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")

    def test_old_actuator_done_sequence_cannot_complete_a_new_task(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 33, "DEHUMIDIFY")
        self.arrive_zone2(robot)
        robot.rfid("AA BB CC DD", 500)
        expected = robot.expected_actuator_sequence

        robot.actuator.status = ActuatorStatus.DONE
        robot.actuator.active = ActuatorCommand.DEHUMIDIFY
        robot.actuator.applied_sequence = (expected - 1) & 0xFF
        robot.poll_actuator(600)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")

    def test_rear_ultrasonic_pauses_only_reverse_and_resumes_after_three_clear(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 4, "NONE")
        self.assertTrue(robot.command(5, "RETURN_HOME", "HOME", "NONE", 100))
        self.assertEqual(robot.phase, "RETURNING")
        self.assertTrue(robot.motor.heading_homebound)
        robot.obstacle_distance(10, 200)
        self.assertEqual(robot.motor.status, MotorStatus.OBSTACLE)
        robot.obstacle_distance(16, 300)  # hysteresis band: remain paused
        self.assertEqual(robot.motor.status, MotorStatus.OBSTACLE)
        robot.obstacle_distance(20, 400)
        robot.obstacle_distance(20, 500)
        self.assertEqual(robot.motor.status, MotorStatus.OBSTACLE)
        robot.obstacle_distance(20, 600)
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)
        robot.keepalive(1_900)
        robot.motor.tick(3_800)
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)
        robot.motor.tick(3_901)
        self.assertEqual(robot.motor.status, MotorStatus.WATCHDOG)
        robot.process_motor_status(3_901)
        self.assertEqual(robot.result, "FAILED")

    def test_rear_ultrasonic_is_monitor_only_during_forward_motion(self) -> None:
        robot = SensorUnoSim(events=TwoEventQueue())
        # A close object behind the robot must not pause an outbound/forward run.
        self.assertTrue(robot.command(401, "TASK", "ZONE2", "NONE", 0))
        self.assertEqual(robot.phase, "MOVING")
        robot.obstacle_distance(5, 150)
        self.assertFalse(robot.motor.paused)
        for now_ms in (300, 450, 600):
            robot.obstacle_distance(None, now_ms)
        self.assertEqual(robot.phase, "MOVING")
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)

    def test_no_echo_is_nonfatal_and_does_not_gate_reverse_departure(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 40, "NONE")
        for now_ms in (100, 250, 400):
            robot.obstacle_distance(None, now_ms)
        self.assertTrue(robot.command(41, "RETURN_HOME", "HOME", "NONE", 550))
        self.assertEqual(robot.phase, "RETURNING")
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)
        for now_ms in (700, 850, 1_000):
            robot.obstacle_distance(None, now_ms)
        self.assertEqual(robot.phase, "RETURNING")
        self.assertEqual(robot.motor.status, MotorStatus.RUNNING)

    def test_stuck_high_blocks_reverse_start_and_pauses_an_active_reverse(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 45, "NONE")
        robot.ultrasonic_sample(UltrasonicSample.STUCK_HIGH, None, 50)
        self.assertTrue(robot.command(46, "RETURN_HOME", "HOME", "NONE", 100))
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "FAILED")
        self.assertEqual(robot.motor.active, MotorCommand.STOP)

        robot = self.new_robot()
        self.dispatch(robot, 47, "NONE")
        self.assertTrue(robot.command(48, "RETURN_HOME", "HOME", "NONE", 100))
        robot.ultrasonic_sample(UltrasonicSample.STUCK_HIGH, None, 150)
        self.assertEqual(robot.phase, "RETURNING")
        self.assertEqual(robot.motor.status, MotorStatus.OBSTACLE)

    def test_invalid_echo_while_obstacle_paused_requires_new_clear_streak(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 46, "NONE")
        self.assertTrue(robot.command(47, "RETURN_HOME", "HOME", "NONE", 100))
        robot.obstacle_distance(10, 150)
        self.assertTrue(robot.motor.paused)
        robot.obstacle_distance(30, 300)
        robot.obstacle_distance(None, 450)  # resets the clear streak, but is nonfatal
        robot.obstacle_distance(30, 600)
        robot.obstacle_distance(30, 750)
        self.assertTrue(robot.motor.paused)
        robot.obstacle_distance(30, 900)
        self.assertFalse(robot.motor.paused)

    def test_all_stop_turns_off_every_output_and_cancels_task(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 5, "DEHUMIDIFY")
        self.arrive_zone2(robot)
        robot.rfid("AA BB CC DD", 100)
        self.assertFalse(robot.actuator.peltier_on)
        self.assertTrue(robot.actuator.fan_on)
        self.assertTrue(robot.command(6, "ALL_STOP", "HOME", "NONE", 200))
        self.assertEqual(robot.phase, "IDLE")
        self.assertFalse(robot.task_active)
        self.assertEqual(robot.motor.status, MotorStatus.IDLE)
        self.assertEqual(robot.actuator.status, ActuatorStatus.IDLE)
        self.assertFalse(any((robot.actuator.humidifier_on, robot.actuator.peltier_on, robot.actuator.fan_on)))

    def test_dehumidifier_fan_prestart_duplicate_guard_and_cooldown(self) -> None:
        actuator = ActuatorUnoSim()
        actuator.command(ActuatorCommand.DEHUMIDIFY, 0)
        self.assertTrue(actuator.fan_on)
        self.assertFalse(actuator.peltier_on)

        actuator.tick(499)
        self.assertFalse(actuator.peltier_on)
        actuator.tick(500)
        self.assertTrue(actuator.peltier_on)

        # 같은 RUNNING 명령 재전송은 5초 타이머를 다시 시작하지 않는다.
        actuator.command(ActuatorCommand.DEHUMIDIFY, 4_900)
        actuator.tick(5_500)
        self.assertFalse(actuator.peltier_on)
        self.assertTrue(actuator.fan_on)
        self.assertEqual(actuator.status, ActuatorStatus.RUNNING)

        actuator.tick(7_499)
        self.assertTrue(actuator.fan_on)
        actuator.tick(7_500)
        self.assertEqual(actuator.status, ActuatorStatus.DONE)
        self.assertFalse(actuator.fan_on)

    def test_actuator_control_frame_crc_sequence_ack_and_idempotent_replay(self) -> None:
        actuator = ActuatorUnoSim()
        frame = actuator_control_frame(0x2A, ActuatorCommand.HUMIDIFY)
        self.assertEqual(frame, (0xA5, 0x2A, 0x01, 0xA3))

        actuator.receive_i2c(frame)
        actuator.process_i2c(100)
        self.assertEqual(actuator.status, ActuatorStatus.RUNNING)
        self.assertEqual(actuator.applied_sequence, 0x2A)
        reply = actuator.reply()
        self.assertEqual(reply[:3], (ActuatorStatus.RUNNING, ActuatorCommand.HUMIDIFY, 0x2A))
        self.assertEqual(reply[5], crc8_atm(reply[:5]))

        actuator.tick(5_100)
        self.assertEqual(actuator.status, ActuatorStatus.DONE)
        actuator.receive_i2c(frame)  # lost ACK retry must not restart a completed burst
        actuator.process_i2c(5_200)
        self.assertEqual(actuator.status, ActuatorStatus.DONE)
        self.assertFalse(actuator.humidifier_on)

    def test_bad_control_crc_and_same_sequence_command_conflict_fail_safe(self) -> None:
        actuator = ActuatorUnoSim()
        actuator.receive_i2c(actuator_control_frame(7, ActuatorCommand.DEHUMIDIFY))
        actuator.process_i2c(0)
        self.assertTrue(actuator.fan_on)

        bad = list(actuator_control_frame(8, ActuatorCommand.HUMIDIFY))
        bad[-1] ^= 0x01
        actuator.receive_i2c(tuple(bad))
        actuator.process_i2c(10)
        self.assertEqual(actuator.status, ActuatorStatus.ERROR)
        self.assertEqual(actuator.applied_sequence, 7)
        self.assertFalse(any((actuator.humidifier_on, actuator.peltier_on, actuator.fan_on)))

        conflict = ActuatorUnoSim()
        conflict.receive_i2c(actuator_control_frame(9, ActuatorCommand.HUMIDIFY))
        conflict.process_i2c(0)
        conflict.receive_i2c(actuator_control_frame(9, ActuatorCommand.STOP))
        conflict.process_i2c(1)
        self.assertEqual(conflict.status, ActuatorStatus.ERROR)
        self.assertEqual(conflict.applied_sequence, 9)

    def test_display_frame_updates_lcd_data_without_operating_relays(self) -> None:
        actuator = ActuatorUnoSim()
        frame = display_frame(7, DisplayState.DEHUMIDIFY, 99, 259, 584, 0x07)
        actuator.receive_i2c(frame)
        actuator.process_i2c(100)

        self.assertEqual(actuator.last_display_sequence, 7)
        self.assertEqual(actuator.display_state, DisplayState.DEHUMIDIFY)
        self.assertEqual(actuator.display_zone_code, 99)
        self.assertEqual(actuator.temperature_tenths, 259)
        self.assertEqual(actuator.humidity_tenths, 584)
        self.assertEqual(actuator.status, ActuatorStatus.IDLE)
        self.assertFalse(any((actuator.humidifier_on, actuator.peltier_on, actuator.fan_on)))
        reply = actuator.reply()
        self.assertEqual(reply[3], 7)
        self.assertEqual(reply[5], crc8_atm(reply[:5]))

    def test_display_crc_and_little_endian_frame_have_fixed_golden_values(self) -> None:
        # CRC-8/ATM's standard check value prevents a matching-but-wrong CRC
        # implementation in both the frame builder and parser from going green.
        self.assertEqual(crc8_atm(b"123456789"), 0xF4)
        frame = display_frame(1, DisplayState.IDLE, 0, 250, 550, 1)
        self.assertEqual(
            frame,
            (0xD1, 0x01, 0x00, 0x00, 0xFA, 0x00, 0x26, 0x02, 0x01, 0x06),
        )

    def test_bad_display_is_isolated_but_ambiguous_frame_fails_safe(self) -> None:
        actuator = ActuatorUnoSim()
        actuator.command(ActuatorCommand.DEHUMIDIFY, 0)
        bad = list(display_frame(3, DisplayState.DEHUMIDIFY, 2, 250, 600))
        bad[5] ^= 0x01
        actuator.receive_i2c(tuple(bad))
        actuator.process_i2c(100)
        self.assertEqual(actuator.rejected_display_frames, 1)
        self.assertEqual(actuator.status, ActuatorStatus.RUNNING)
        self.assertTrue(actuator.fan_on)

        actuator.receive_i2c((DISPLAY_FRAME_MAGIC, 1, 2))
        actuator.process_i2c(200)
        self.assertEqual(actuator.status, ActuatorStatus.ERROR)
        self.assertFalse(any((actuator.humidifier_on, actuator.peltier_on, actuator.fan_on)))

    def test_invalid_display_payload_is_rejected_without_resetting_the_task(self) -> None:
        invalid_frames = []
        bad_state = list(display_frame(1, DisplayState.IDLE, 2, 250, 600))
        bad_state[2] = 7
        bad_state[-1] = crc8_atm(bad_state[:-1])
        invalid_frames.append(tuple(bad_state))

        bad_zone = list(display_frame(2, DisplayState.MOVING, 2, 250, 600))
        bad_zone[3] = 3
        bad_zone[-1] = crc8_atm(bad_zone[:-1])
        invalid_frames.append(tuple(bad_zone))

        bad_humidity = list(display_frame(3, DisplayState.HUMIDIFY, 99, 250, 1_001))
        invalid_frames.append(tuple(bad_humidity))

        for frame in invalid_frames:
            with self.subTest(frame=frame):
                actuator = ActuatorUnoSim()
                actuator.command(ActuatorCommand.HUMIDIFY, 0)
                actuator.receive_i2c(frame)
                actuator.process_i2c(1_000)
                self.assertEqual(actuator.rejected_display_frames, 1)
                self.assertEqual(actuator.status, ActuatorStatus.RUNNING)
                self.assertTrue(actuator.humidifier_on)
                actuator.tick(5_000)
                self.assertEqual(actuator.status, ActuatorStatus.DONE)
                self.assertFalse(actuator.humidifier_on)

    def test_empty_probe_is_noop_and_every_ambiguous_length_fails_safe(self) -> None:
        probe = ActuatorUnoSim()
        probe.receive_i2c(())
        probe.process_i2c(0)
        self.assertEqual(probe.status, ActuatorStatus.IDLE)

        valid = display_frame(9, DisplayState.DEHUMIDIFY, 99, 250, 600)
        malformed_frames = [valid[:length] for length in range(2, 10)]
        malformed_frames.append(valid + (0x00,))
        for frame in malformed_frames:
            with self.subTest(length=len(frame)):
                actuator = ActuatorUnoSim()
                actuator.command(ActuatorCommand.DEHUMIDIFY, 0)
                actuator.receive_i2c(frame)
                actuator.process_i2c(100)
                self.assertEqual(actuator.status, ActuatorStatus.ERROR)
                self.assertFalse(
                    any((actuator.humidifier_on, actuator.peltier_on, actuator.fan_on))
                )

    def test_command_and_display_mailboxes_do_not_overwrite_each_other(self) -> None:
        actuator = ActuatorUnoSim()
        actuator.command(ActuatorCommand.HUMIDIFY, 0)
        actuator.receive_i2c(display_frame(4, DisplayState.DONE, 2, -35, 1_000))
        actuator.receive_i2c(actuator_control_frame(1, ActuatorCommand.STOP))
        actuator.process_i2c(50)
        self.assertEqual(actuator.status, ActuatorStatus.IDLE)
        self.assertEqual(actuator.temperature_tenths, -35)
        self.assertEqual(actuator.humidity_tenths, 1_000)
        self.assertEqual(actuator.last_display_sequence, 4)

    def test_lcd_failure_and_stale_display_never_extend_actuator_timers(self) -> None:
        actuator = ActuatorUnoSim(lcd_ready=False, lcd_error=True)
        actuator.command(ActuatorCommand.DEHUMIDIFY, 0)
        actuator.receive_i2c(display_frame(1, DisplayState.DEHUMIDIFY, 99, 250, 850))
        actuator.process_i2c(0)
        actuator.tick(500)
        self.assertTrue(actuator.peltier_on)
        actuator.tick(5_500)
        self.assertFalse(actuator.peltier_on)
        self.assertTrue(actuator.fan_on)
        actuator.tick(7_500)
        self.assertEqual(actuator.status, ActuatorStatus.DONE)
        self.assertFalse(actuator.fan_on)
        actuator.display_tick(29_999)
        self.assertFalse(actuator.display_status_flags & DISPLAY_STALE)
        actuator.display_tick(30_000)
        self.assertTrue(actuator.display_status_flags & DISPLAY_STALE)

    def test_failed_stop_ack_retries_locally_until_both_boards_confirm(self) -> None:
        stop = FailedStopRecoverySim()
        stop.begin_stop(motor_ack=False, actuator_ack=True, now_ms=0)

        self.assertTrue(stop.stop_retry_mode)
        self.assertEqual(stop.phase, "TASK_COMPLETE")
        self.assertEqual(stop.result, "FAILED")
        self.assertFalse(stop.route_known)
        self.assertFalse(stop.accepts_route_command())
        self.assertIsNone(stop.events.pending)

        self.assertFalse(stop.retry(499, motor_ack=True))
        self.assertEqual(stop.motor_attempts, 1)
        self.assertEqual(stop.actuator_attempts, 1)

        self.assertTrue(stop.retry(500, motor_ack=True))
        self.assertEqual(stop.motor_attempts, 2)
        # The already-confirmed ActuatorUno is not needlessly commanded again.
        self.assertEqual(stop.actuator_attempts, 1)
        self.assertFalse(stop.stop_retry_mode)
        self.assertEqual(stop.phase, "IDLE")
        self.assertEqual(stop.result, "COMPLETED")
        self.assertEqual(stop.events.pending, "STOP_CONFIRMED")
        self.assertTrue(stop.accepts_route_command())

    def test_failed_stop_retries_each_unconfirmed_board_independently(self) -> None:
        stop = FailedStopRecoverySim()
        stop.begin_stop(motor_ack=False, actuator_ack=False, now_ms=0)
        self.assertFalse(stop.retry(500, motor_ack=True, actuator_ack=False))
        self.assertTrue(stop.motor_confirmed)
        self.assertFalse(stop.actuator_confirmed)
        self.assertFalse(stop.retry(999, actuator_ack=True))
        self.assertTrue(stop.retry(1_000, actuator_ack=True))
        self.assertEqual(stop.motor_attempts, 2)
        self.assertEqual(stop.actuator_attempts, 3)
        self.assertEqual(stop.events.pending, "STOP_CONFIRMED")

        actuator_only = FailedStopRecoverySim()
        actuator_only.begin_stop(motor_ack=True, actuator_ack=False, now_ms=0)
        self.assertTrue(
            actuator_only.route_known,
            "an Actuator-only STOP failure cannot change chassis position/heading",
        )
        self.assertTrue(actuator_only.retry(500, actuator_ack=True))

    def test_server_failure_during_task_is_a_safe_stop(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 7, "HUMIDIFY")
        robot.server_failure(300)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        self.assertEqual(robot.result, "SERVER_OFFLINE")
        self.assertFalse(robot.task_active)
        self.assertEqual(robot.motor.status, MotorStatus.IDLE)
        self.assertEqual(robot.actuator.status, ActuatorStatus.IDLE)

    def test_server_loss_same_revision_retries_once_after_failure_report(self) -> None:
        for command, active_phase, resumed_phase in (
            ("TASK", "MOVING", "MOVING"),
            ("RETURN_HOME", "RETURNING", "RETURNING"),
        ):
            with self.subTest(command=command):
                recovery = SameRevisionRecoverySim(command, active_phase)
                recovery.server_loss()

                # Keep the robot stopped until the server has received the
                # reason for the stop.
                self.assertFalse(recovery.poll_same_revision())
                self.assertEqual(recovery.phase, "TASK_COMPLETE")

                recovery.failure_report_sent()
                self.assertTrue(recovery.poll_same_revision())
                self.assertEqual(recovery.phase, resumed_phase)
                self.assertEqual(recovery.restart_count, 1)

                # The allowance is consumed before starting, so another poll
                # of the unchanged revision cannot create a restart loop.
                self.assertFalse(recovery.poll_same_revision())
                self.assertEqual(recovery.restart_count, 1)

    def test_server_loss_with_unknown_route_never_retries_same_revision(self) -> None:
        recovery = SameRevisionRecoverySim(
            command="TASK", phase="MOVING", route_known=False
        )
        recovery.server_loss()
        recovery.failure_report_sent()
        self.assertFalse(recovery.poll_same_revision())
        self.assertEqual(recovery.phase, "TASK_COMPLETE")
        self.assertEqual(recovery.restart_count, 0)

    def test_event_queue_keeps_oldest_then_latest_and_promotes_after_send(self) -> None:
        queue = TwoEventQueue()
        queue.put("RFID_ARRIVAL")
        queue.put("MODULE_RUNNING")
        queue.put("MODULE_COMPLETE")
        self.assertEqual(queue.pending, "RFID_ARRIVAL")
        self.assertEqual(queue.deferred, "MODULE_COMPLETE")
        queue.sent()
        self.assertEqual(queue.pending, "MODULE_COMPLETE")
        self.assertIsNone(queue.deferred)
        queue.sent()
        self.assertIsNone(queue.pending)

    def test_complete_task_then_return_home_on_new_revision(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 8, "HUMIDIFY")
        self.arrive_zone2(robot)
        robot.rfid("AA BB CC DD", 1_000)
        robot.poll_actuator(6_000)
        self.assertEqual(robot.phase, "TASK_COMPLETE")
        for now_ms in (6_050, 6_075, 6_100):
            robot.obstacle_distance(100, now_ms)
        self.assertTrue(robot.command(9, "RETURN_HOME", "HOME", "NONE", 6_100))
        self.assertEqual(robot.phase, "RETURNING")
        # RETURN drives straight backward; after leaving the zone, find HOME.
        robot.motor.observe_line(True, True, 6_800)
        robot.motor.observe_line(False, False, 6_900)
        robot.motor.observe_line(True, True, 7_000)
        robot.motor.observe_line(True, True, 7_300)
        robot.process_motor_status(7_300)
        self.assertEqual(robot.phase, "IDLE")
        self.assertEqual(robot.result, "COMPLETED")
        self.assertIn("HOME_ARRIVAL", (robot.events.pending, robot.events.deferred))

    def test_same_revision_is_not_dispatched_twice(self) -> None:
        robot = self.new_robot()
        self.dispatch(robot, 10, "HUMIDIFY")
        self.assertFalse(robot.command(10, "TASK", "ZONE99", "DEHUMIDIFY", 100))
        self.assertEqual(robot.target_zone, "ZONE2")
        self.assertEqual(robot.action, "HUMIDIFY")


class EspHttpResponseCollectorTests(unittest.TestCase):
    BODY = (
        '{"revision":1787186401,"command":"ALL_STOP",'
        '"target_zone":"HOME","action":"NONE"}'
    )

    @classmethod
    def setUpClass(cls) -> None:
        # This reproduces the observed response shape: a header too large for
        # espBuffer followed by the 81-byte command JSON that does fit.
        header_prefix = (
            "HTTP/1.0 200 OK\r\n"
            "Server: humidity-test\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: 81\r\n"
            "Connection: close\r\n"
            "X-Pad: "
        )
        header_suffix = "\r\n\r\n"
        pad_length = 160 - len(header_prefix) - len(header_suffix)
        cls.header = header_prefix + ("H" * pad_length) + header_suffix

        cls.ipd_prefix = "SEND OK\r\n\r\n+IPD,241:"
        cls.closed_suffix = "\r\nCLOSED\r\n"
        cls.transcript = cls.ipd_prefix + cls.header + cls.BODY + cls.closed_suffix

        assert pad_length >= 0
        assert len(cls.header.encode("ascii")) == 160
        assert len(cls.BODY.encode("ascii")) == 81
        assert len((cls.header + cls.BODY).encode("ascii")) == 241

    def test_long_header_exact_body_and_esp_at_framing_parse_revision(self) -> None:
        collector = EspHttpResponseCollectorSim()

        self.assertTrue(collector.collect(self.transcript))
        self.assertTrue(collector.http_ok)
        self.assertTrue(collector.body_started)
        self.assertTrue(collector.body_complete)
        self.assertFalse(collector.overflowed)
        self.assertEqual(collector.payload, self.BODY)
        self.assertEqual(collector.extract_json_long("revision"), 1787186401)

    def test_header_only_truncated_body_and_missing_opener_are_rejected(self) -> None:
        malformed = {
            "header only": self.ipd_prefix + self.header + self.closed_suffix,
            "truncated body": self.ipd_prefix + self.header + self.BODY[:-1],
            "missing opener": self.ipd_prefix + self.header + self.BODY[1:],
        }
        for label, transcript in malformed.items():
            with self.subTest(label=label):
                collector = EspHttpResponseCollectorSim()
                self.assertFalse(collector.collect(transcript))

    def test_software_serial_overflow_invalidates_an_otherwise_valid_response(self) -> None:
        collector = EspHttpResponseCollectorSim()

        self.assertFalse(
            collector.collect(self.transcript, software_serial_overflow=True)
        )
        self.assertTrue(collector.overflowed)
        self.assertIsNone(collector.extract_json_long("revision"))

    def test_long_status_body_can_complete_after_storage_prefix_is_full(self) -> None:
        body = (
            '{"accepted":true,"ack_revision":1787186401,'
            '"phase":"TASK_COMPLETE","event":"MODULE_COMPLETE",'
            '"result":"COMPLETED","ack_accepted":true,'
            '"ack_rejection":null}'
        )
        transcript = self.ipd_prefix + self.header + body + self.closed_suffix
        collector = EspHttpResponseCollectorSim()

        self.assertTrue(collector.collect(transcript))
        self.assertTrue(collector.storage_truncated)
        self.assertTrue(collector.body_complete)
        self.assertEqual(collector.extract_json_long("ack_revision"), 1787186401)


class FirmwareSourceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.sensor = SENSOR_SOURCE.read_text(encoding="utf-8")
        cls.motor = MOTOR_SOURCE.read_text(encoding="utf-8")
        cls.actuator = ACTUATOR_SOURCE.read_text(encoding="utf-8")
        cls.network = NETWORK_SOURCE.read_text(encoding="utf-8")
        cls.sensor_diagnostic = SENSOR_DIAGNOSTIC_SOURCE.read_text(encoding="utf-8")
        cls.zone2_drive_diagnostic = ZONE2_DRIVE_DIAGNOSTIC_SOURCE.read_text(
            encoding="utf-8"
        )

    def assert_source(self, source: str, pattern: str) -> None:
        self.assertRegex(source, pattern)

    def test_current_sensor_pin_map_and_public_site_configuration(self) -> None:
        for pattern in (
            r"UNO_ESP_RX_PIN\s*=\s*6",
            r"UNO_ESP_TX_PIN\s*=\s*5",
            r"RFID_SS_PIN\s*=\s*8",
            r"RFID_SCK_PIN\s*=\s*9",
            r"RFID_MOSI_PIN\s*=\s*10",
            r"RFID_MISO_PIN\s*=\s*11",
            r"RFID_RST_PIN\s*=\s*12",
        ):
            self.assert_source(self.sensor, pattern)
        for removed_sensor_peripheral in (
            "#include <DHT.h>",
            "DHT_PIN",
            "ULTRASONIC_ECHO_PIN",
            "ULTRASONIC_TRIG_PIN",
        ):
            self.assertNotIn(removed_sensor_peripheral, self.sensor)
        self.assert_source(self.actuator, r"DHT_PIN\s*=\s*2")
        self.assert_source(self.actuator, r"DHT_TYPE\s*=\s*DHT22")
        self.assert_source(self.motor, r"ULTRASONIC_ECHO_PIN\s*=\s*2")
        self.assert_source(self.motor, r"ULTRASONIC_TRIG_PIN\s*=\s*A1")
        self.assertIn('#define ROBOT_SERVER_HOST "192.0.2.10"', self.network)
        self.assertIn('#define ROBOT_ZONE2_UID "AA BB CC DD"', self.network)
        self.assertIn('#define ROBOT_ZONE99_UID "11 22 33 44"', self.network)
        self.assertIn(
            "const char RFID_ZONE2_UID[] PROGMEM = ROBOT_ZONE2_UID;",
            self.sensor,
        )
        self.assertIn(
            "const char RFID_ZONE99_UID[] PROGMEM = ROBOT_ZONE99_UID;",
            self.sensor,
        )

    def test_motor_operational_build_does_not_bypass_line_tracking(self) -> None:
        self.assert_source(
            self.motor,
            r"constexpr\s+bool\s+BENCH_RFID_ONLY_MODE\s*=\s*false",
        )
        self.assert_source(
            self.motor,
            r"constexpr\s+bool\s+LINE_BLACK_IS_HIGH\s*=\s*(?:true|false)",
        )
        self.assertIn("bool isBlack(byte pin)", self.motor)

    def test_home_calibration_interlock_blocks_boot_assumptions_and_motion(self) -> None:
        for source in (self.sensor, self.motor):
            self.assert_source(source, r"(?:MOTOR_)?COMMAND_OUTBOUND\s*=\s*0x11")
            self.assert_source(
                source, r"(?:MOTOR_)?COMMAND_(?:RETURN|REVERSE_HOME)\s*=\s*0x12"
            )
            self.assert_source(source, r"(?:MOTOR_)?COMMAND_HOME_SYNC\s*=\s*6")
            self.assert_source(source, r"(?:MOTOR_)?COMMAND_PROTOCOL_SYNC\s*=\s*7")
            self.assert_source(
                source, r"(?:MOTOR_)?STATUS_CALIBRATION_REQUIRED\s*=\s*7"
            )
            self.assert_source(
                source, r"(?:MOTOR_)?STATUS_PROTOCOL_REQUIRED\s*=\s*8"
            )

        self.assert_source(
            self.zone2_drive_diagnostic, r"MOTOR_FORWARD\s*=\s*0x11"
        )
        self.assert_source(
            self.zone2_drive_diagnostic, r"MOTOR_PROTOCOL_SYNC\s*=\s*7"
        )
        diagnostic_setup = self.zone2_drive_diagnostic[
            self.zone2_drive_diagnostic.index("void setup()") :
        ]
        self.assertLess(
            diagnostic_setup.index('stopMotorConfirmed(F("diagnostic boot"))'),
            diagnostic_setup.index("rfid.PCD_Init()"),
        )
        self.assertLess(
            diagnostic_setup.index("stopActuatorBestEffort()"),
            diagnostic_setup.index("rfid.PCD_Init()"),
        )
        self.assertIn("ACTUATOR_CONTROL_MAGIC = 0xA5", self.zone2_drive_diagnostic)
        self.assertNotIn("Serial.println(uidText)", self.zone2_drive_diagnostic)

        self.assert_source(self.sensor, r"bool\s+routeCalibrated\s*=\s*false")
        self.assert_source(
            self.sensor, r"confirmedStation\s*=\s*STATION_UNKNOWN"
        )
        self.assert_source(
            self.sensor, r"expectedStation\s*=\s*STATION_UNKNOWN"
        )
        self.assertIn("bool requireHomeCalibration()", self.sensor)
        self.assertIn("if (!requireHomeCalibration()) return false;", self.sensor)
        self.assertIn('PSTR("CALIBRATE_HOME")', self.sensor)
        self.assertIn(
            "sendMotorCommandChecked(MOTOR_COMMAND_PROTOCOL_SYNC)", self.sensor
        )
        self.assertIn("sendMotorCommandChecked(MOTOR_COMMAND_HOME_SYNC)", self.sensor)
        self.assertIn("status == MOTOR_STATUS_IDLE;", self.sensor)
        self.assertIn("command == 'C' || command == 'c'", self.sensor)
        self.assertNotIn("Serial.println(uidText)", self.sensor)
        self.assertIn("[RFID] ARRIVAL CONFIRMED station=", self.sensor)

        calibration_start = self.sensor.index("bool performHomeCalibration()")
        calibration_end = self.sensor.index(
            "bool startPlaceholderMovement()", calibration_start
        )
        calibration_body = self.sensor[calibration_start:calibration_end]
        protocol_sync = calibration_body.index(
            "sendMotorCommandChecked(MOTOR_COMMAND_PROTOCOL_SYNC)"
        )
        motor_stop = calibration_body.index("stopMotorController()")
        actuator_stop = calibration_body.index("stopModuleController()")
        home_sync = calibration_body.index(
            "sendMotorCommandChecked(MOTOR_COMMAND_HOME_SYNC)"
        )
        self.assertLess(protocol_sync, motor_stop)
        self.assertLess(motor_stop, actuator_stop)
        self.assertLess(actuator_stop, home_sync)
        self.assertIn("motorStopped && moduleStopped", calibration_body)

        self.assert_source(self.motor, r"bool\s+calibrated\s*=\s*false")
        sync_start = self.motor.index("void applyHomeSync()")
        sync_end = self.motor.index("void applyCommand(byte command)", sync_start)
        sync_body = self.motor[sync_start:sync_end]
        self.assertIn("stopMotors();", self.motor)
        self.assertIn("!isBlack(LEFT_IR_PIN)", sync_body)
        self.assertIn("!isBlack(RIGHT_IR_PIN)", sync_body)
        self.assertIn("headingHomebound = false;", sync_body)
        self.assertIn("calibrated = true;", sync_body)

        apply_start = sync_end
        apply_end = self.motor.index("void followLine()", apply_start)
        apply_body = self.motor[apply_start:apply_end]
        self.assertIn("if (!calibrated)", apply_body)
        self.assertIn("if (!protocolValidated)", apply_body)
        self.assertIn("isValidControlCommand(command)", apply_body)
        self.assertIn("STATUS_CALIBRATION_REQUIRED", apply_body)
        self.assertIn("STATUS_PROTOCOL_REQUIRED", apply_body)

    def test_same_revision_never_restarts_a_completed_module_without_server_grant(self) -> None:
        same_start = self.sensor.index("if (nextRevision == lastCommandRevision)")
        same_end = self.sensor.index('Serial.print(F("[COMMAND] new revision="))', same_start)
        same_revision_body = self.sensor[same_start:same_end]
        self.assertNotIn("same violation persists", same_revision_body)
        self.assertNotIn("startPlaceholderModule(false)", same_revision_body)
        self.assertIn("retrySameRevisionAllowed", same_revision_body)

    def test_http_requests_are_bounded_and_fit_the_reduced_uno_buffer(self) -> None:
        self.assertRegex(self.sensor, r"char\s+requestBuffer\[180\]")
        self.assertIn("GET /api/robot/command HTTP/1.0\\r\\n\\r\\n", self.sensor)
        self.assertIn("result=%s HTTP/1.0\\r\\n\\r\\n", self.sensor)
        self.assertGreaterEqual(
            self.sensor.count(
                "requestLength >= static_cast<int>(sizeof(requestBuffer))"
            ),
            2,
        )
        # Maximum sizes of the actual fixed C buffers: phase19, event23,
        # zone7, action15, signed-long10, result15, plus request syntax/NUL.
        worst = (
            "GET /api/robot/status?phase=" + "P" * 19
            + "&event=" + "E" * 23
            + "&zone=" + "Z" * 7
            + "&action=" + "A" * 15
            + "&ack_revision=2147483647&result=" + "R" * 15
            + " HTTP/1.0\r\n\r\n"
        )
        self.assertLess(len(worst), 180)

    def test_http_collector_requires_complete_json_and_rejects_uart_overflow(self) -> None:
        collect_start = self.sensor.index("bool collectHttpResponse()")
        collect_end = self.sensor.index("bool fetchCommandResponse()", collect_start)
        body = self.sensor[collect_start:collect_end]

        # HTTP 200 only proves that a response began.  The command parser may
        # run only after both JSON delimiters survived SoftwareSerial.
        self.assertRegex(body, r"bool\s+bodyComplete\s*=\s*false\s*;")
        self.assertRegex(body, r"bool\s+headersComplete\s*=\s*false\s*;")
        self.assertRegex(
            body,
            r"else\s+if\s*\(\s*c\s*==\s*'\{'\s*\)\s*\{"
            r"[\s\S]{0,300}?bodyStarted\s*=\s*true\s*;",
        )
        self.assertRegex(body, r"c\s*==\s*'\}'")
        self.assertRegex(body, r"bodyComplete\s*=\s*true\s*;")

        # SoftwareSerial's 64-byte ISR ring buffer has an explicit sticky
        # overflow flag.  A response assembled after dropped bytes must fail,
        # even when its remaining suffix happens to look like valid JSON.
        self.assertIn("esp8266.overflow()", body)
        self.assertRegex(
            body,
            r"if\s*\(\s*esp8266\.overflow\(\)\s*\)\s*\{?"
            r"[\s\S]{0,300}?return\s+false\s*;",
        )

        returns = re.findall(r"return\s+([^;]+);", body)
        self.assertTrue(returns)
        final_return = returns[-1]
        for required_proof in ("httpOk", "bodyStarted", "bodyComplete"):
            self.assertIn(required_proof, final_return)

    def test_every_raw_stop_failure_arms_the_local_retry_latch(self) -> None:
        motor_start = self.sensor.index("bool stopMotorController()")
        motor_end = self.sensor.index("bool startMotorController", motor_start)
        motor_body = self.sensor[motor_start:motor_end]
        self.assertIn(
            "armStopRetry(STOP_RETRY_KEEP_FAULT, false, true)", motor_body
        )

        actuator_start = self.sensor.index("bool stopModuleController()")
        actuator_end = self.sensor.index("void armStopRetry", actuator_start)
        actuator_body = self.sensor[actuator_start:actuator_end]
        self.assertIn(
            "armStopRetry(STOP_RETRY_KEEP_FAULT, true, false)", actuator_body
        )

        retry_start = actuator_end
        retry_end = self.sensor.index("void serviceStopRetry", retry_start)
        retry_body = self.sensor[retry_start:retry_end]
        self.assertIn("stopMotorRetryNeeded || !motorStopped", retry_body)
        self.assertIn("stopActuatorRetryNeeded || !moduleStopped", retry_body)
        self.assertIn("mode != STOP_RETRY_KEEP_FAULT", retry_body)
        self.assertIn("if (!alreadyRetrying) lastStopRetryAt", retry_body)

    def test_ambiguous_actuator_start_is_followed_by_confirmed_stop(self) -> None:
        start = self.sensor.index("bool startPlaceholderModule(bool reportArrival)")
        end = self.sensor.index("byte pollActuatorTask()", start)
        body = self.sensor[start:end]

        send_failure = re.search(
            r"if\s*\(\s*!sendActuatorFrame\(actuatorCommand, actuatorSequence\)\s*\)\s*"
            r"\{(?P<body>[\s\S]*?)return false;",
            body,
        )
        self.assertIsNotNone(send_failure)
        self.assertIn("stopModuleController();", send_failure.group("body"))

        running_failure = re.search(
            r"if\s*\(\s*!waitForActuatorCommand\(actuatorCommand, actuatorSequence,\s*"
            r"ACTUATOR_STATUS_RUNNING\)\s*\)\s*"
            r"\{(?P<body>[\s\S]*?)return false;",
            body,
        )
        self.assertIsNotNone(running_failure)
        self.assertIn("stopModuleController();", running_failure.group("body"))

    def test_i2c_protocol_values_and_actuator_outputs_match_all_three_sketches(self) -> None:
        self.assert_source(self.sensor, r"MOTOR_UNO_ADDRESS\s*=\s*0x08")
        self.assert_source(self.sensor, r"ACTUATOR_UNO_ADDRESS\s*=\s*0x09")
        self.assert_source(self.motor, r"I2C_ADDRESS\s*=\s*0x08")
        self.assert_source(self.actuator, r"I2C_ADDRESS\s*=\s*0x09")
        self.assert_source(self.actuator, r"HUMIDIFIER_RELAY_PIN\s*=\s*A0")
        self.assert_source(self.actuator, r"PELTIER_RELAY_PIN\s*=\s*A1")
        self.assert_source(self.actuator, r"COOLING_FAN_RELAY_PIN\s*=\s*7")

    def test_actuator_dispatches_command_and_display_frames_without_collision(self) -> None:
        for source in (self.sensor, self.actuator):
            self.assert_source(source, r"DISPLAY_FRAME_MAGIC\s*=\s*0xD1")
            self.assert_source(source, r"DISPLAY_FRAME_SIZE\s*=\s*10")

        self.assert_source(self.sensor, r"ACTUATOR_CONTROL_MAGIC\s*=\s*0xA5")
        self.assert_source(self.sensor, r"ACTUATOR_CONTROL_FRAME_SIZE\s*=\s*4")
        self.assert_source(self.sensor, r"ACTUATOR_STATUS_REPLY_SIZE\s*=\s*6")
        self.assert_source(self.actuator, r"CONTROL_FRAME_MAGIC\s*=\s*0xA5")
        self.assert_source(self.actuator, r"CONTROL_FRAME_SIZE\s*=\s*4")
        self.assert_source(self.actuator, r"STATUS_REPLY_SIZE\s*=\s*6")

        send_start = self.sensor.index("bool sendActuatorFrame(")
        send_end = self.sensor.index("bool readActuatorState(", send_start)
        send_body = self.sensor[send_start:send_end]
        for token in (
            "sequence = ++actuatorSequenceCounter;",
            "ACTUATOR_CONTROL_MAGIC, sequence, command, 0",
            "frame[3] = crc8Atm(frame, 3);",
            "Wire.write(frame, sizeof(frame));",
        ):
            self.assertIn(token, send_body)

        for token in (
            "byteCount != CONTROL_FRAME_SIZE && byteCount != DISPLAY_FRAME_SIZE",
            "controlMailbox[0] = 0xFF",
            "volatile byte controlMailbox[CONTROL_FRAME_SIZE]",
            "volatile byte displayMailbox[DISPLAY_FRAME_SIZE]",
            "volatile bool displayMailboxPending",
            "volatile bool commandPending",
            "FAN_PRESTART_MS = 500",
            "FAN_COOLDOWN_MS = 2000",
            "DEHUM_STAGE_FAN_PRESTART",
            "DEHUM_STAGE_PELTIER_RUNNING",
            "DEHUM_STAGE_FAN_COOLDOWN",
            "crc8Atm(frame, 3) != frame[3]",
            "applyCommand(frame[2], frame[1])",
        ):
            self.assertIn(token, self.actuator)

        receive_match = re.search(
            r"void receiveI2cCommand\([^\{]+\{(?P<body>.*?)\n\}\n\n"
            r"// .*?6.*?\nvoid sendI2cStatus",
            self.actuator,
            re.DOTALL,
        )
        self.assertIsNotNone(receive_match)
        receive_body = re.sub(r"//.*", "", receive_match.group("body"))
        for forbidden in (
            "Serial",
            "millis",
            "micros",
            "crc8Atm",
            "serviceLcd",
            "writeRelay",
            "stopAllOutputs",
            "applyCommand",
        ):
            self.assertNotIn(forbidden, receive_body)
        self.assertIn("displayMailboxPending = true;", receive_body)
        self.assertIn("commandPending = true;", receive_body)

        start = self.actuator.index("void startDehumidifier(byte sequence)")
        end = self.actuator.index("void applyCommand", start)
        start_body = self.actuator[start:end]
        self.assertIn("writeRelay(COOLING_FAN_RELAY_PIN, true);", start_body)
        self.assertNotIn("writeRelay(PELTIER_RELAY_PIN, true);", start_body)

    def test_motor_sequence_ack_protocol_and_isr_contract(self) -> None:
        for token in (
            "requestedSequence",
            "appliedCommand",
            "appliedSequence",
            "byteCount != 2",
            "requestedCommand = 0xFF",
            "command == appliedCommand && sequence == appliedSequence",
            "publishAppliedCommand(command, sequence)",
        ):
            self.assertIn(token, self.motor)

        receive_match = re.search(
            r"void receiveI2cCommand\([^\{]+\{(?P<body>.*?)\n\}\n\nvoid sendI2cStatus",
            self.motor,
            re.DOTALL,
        )
        request_match = re.search(
            r"void sendI2cStatus\([^\{]*\{(?P<body>.*?)\n\}\n\nbool isValidControlCommand",
            self.motor,
            re.DOTALL,
        )
        self.assertIsNotNone(receive_match)
        self.assertIsNotNone(request_match)
        for callback in (receive_match.group("body"), request_match.group("body")):
            executable = re.sub(r"//.*", "", callback)
            for forbidden in ("Serial", "millis", "applyCommand", "motorLeft", "motorRight"):
                self.assertNotIn(forbidden, executable)

        self.assertIn("Wire.write(statusSnapshot);", request_match.group("body"))
        self.assertIn("Wire.write(commandSnapshot);", request_match.group("body"))
        self.assertIn("Wire.write(sequenceSnapshot);", request_match.group("body"))
        self.assertRegex(
            self.sensor,
            r"sendMotorFrame\([^\{]+\{[\s\S]*?Wire\.write\(command\);"
            r"[\s\S]*?Wire\.write\(sequence\);",
        )
        self.assertRegex(
            self.sensor,
            r"requestFrom\(static_cast<int>\(MOTOR_UNO_ADDRESS\),\s*3\)",
        )
        self.assertIn("appliedCommand != command || appliedSequence != sequence", self.sensor)
        self.assertIn("appliedCommand != acknowledgedMotorCommand", self.sensor)
        self.assertIn("appliedSequence != acknowledgedMotorSequence", self.sensor)

    def test_motor_watchdog_and_wifi_backoff_are_present(self) -> None:
        self.assert_source(self.motor, r"CONTROL_WATCHDOG_MS\s*=\s*2000")
        self.assert_source(self.motor, r"HOME_MARKER_CLEAR_TIMEOUT_MS\s*=\s*2000")
        self.assert_source(self.motor, r"STATUS_UNEXPECTED_MARKER\s*=\s*6")
        self.assert_source(self.sensor, r"MOTOR_STATUS_UNEXPECTED_MARKER\s*=\s*6")
        self.assertIn("updateHomeMarkerClearSafety();", self.motor)
        self.assertIn("faultStatus == MOTOR_STATUS_UNEXPECTED_MARKER", self.sensor)
        self.assert_source(self.sensor, r"MOTOR_KEEPALIVE_MS\s*=\s*400")
        self.assert_source(self.sensor, r"WIFI_RECONNECT_INTERVAL_MS\s*=\s*15000")
        self.assert_source(self.motor, r"ULTRASONIC_MIN_CM\s*=\s*2")
        self.assert_source(self.motor, r"ULTRASONIC_MAX_CM\s*=\s*400")
        self.assert_source(self.motor, r"ULTRASONIC_CLEAR_STREAK_REQUIRED\s*=\s*3")
        self.assertIn("attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO_PIN)", self.motor)
        self.assertIn("captureUltrasonicEcho", self.motor)
        self.assertIn("ULTRASONIC_SAMPLE_STUCK_HIGH", self.motor)
        self.assertIn(
            'F("[BOOT] STUCK_HIGH stops reverse; NO_ECHO is diagnostic")',
            self.motor,
        )
        self.assertNotIn("pulseIn(ULTRASONIC_ECHO_PIN", self.motor)
        self.assertIn("millis() - lastWifiReconnectAttemptAt >= WIFI_RECONNECT_INTERVAL_MS", self.sensor)

    def test_motor_pause_preserves_an_already_latched_home_marker(self) -> None:
        apply_start = self.motor.index("void applyCommand(byte command)")
        apply_end = self.motor.index("void followLine()", apply_start)
        apply_body = self.motor[apply_start:apply_end]
        pause_start = apply_body.index("if (command == COMMAND_PAUSE)")
        resume_start = apply_body.index("if (command == COMMAND_RESUME)", pause_start)
        pause_body = apply_body[pause_start:resume_start]
        self.assertIn(
            "homeMarkerLatched",
            pause_body,
            "PAUSE must not overwrite STATUS_STOP_LINE after HOME was latched",
        )

    def test_rear_ultrasonic_is_local_to_motor_and_controls_only_reverse(self) -> None:
        self.assert_source(self.sensor, r"USB_SERIAL_BAUD\s*=\s*115200")
        self.assertIn("Serial.begin(USB_SERIAL_BAUD);", self.sensor)
        self.assert_source(self.sensor, r"#define\s+VERBOSE_OPERATION_LOGS\s+0")
        self.assertIn("if (command == '?')", self.sensor)
        self.assertIn('Serial.print(F("[DIAG] P="));', self.sensor)
        for core_error in (
            '[I2C] transmit failed address=0x',
            '[WIFI] ESP-01 AT communication failed',
            '[WIFI] ESP send busy -> quiet backoff',
            '[RFID] card UID read failed',
        ):
            self.assertIn(core_error, self.sensor)
        self.assertNotIn("ultrasonicReadyForMovement", self.sensor)
        self.assertNotIn("rejectMovementForUltrasonic", self.sensor)
        self.assertNotIn("void updateObstacleSensor()", self.sensor)
        self.assertNotIn("#include <DHT.h>", self.sensor)
        self.assertIn("localObstaclePauseActive", self.motor)
        self.assertIn("bool reverseUltrasonicControlActive()", self.motor)
        self.assertIn("activeCommand == COMMAND_RETURN && headingHomebound", self.motor)
        self.assertIn("distanceCm < ULTRASONIC_STOP_CM", self.motor)
        self.assertIn("distanceCm < ULTRASONIC_CLEAR_CM", self.motor)
        self.assertIn("ULTRASONIC_CLEAR_STREAK_REQUIRED", self.motor)
        self.assertIn("ULTRASONIC_SAMPLE_STUCK_HIGH", self.motor)
        self.assertIn("ULTRASONIC_SAMPLE_NO_ECHO", self.motor)
        self.assertIn("obstaclePauseActive = status == MOTOR_STATUS_OBSTACLE;", self.sensor)

        return_start = self.sensor.index("bool startPlaceholderReturn()")
        return_end = self.sensor.index("bool startPlaceholderModule(", return_start)
        return_body = self.sensor[return_start:return_end]
        self.assertNotIn("rearSampleFresh", return_body)
        self.assertNotIn("rearStartBlocked", return_body)
        self.assertIn("return startRouteTravel(STATION_HOME);", return_body)

    def test_failed_tcp_stages_are_closed_before_the_next_poll(self) -> None:
        close_start = self.sensor.index("bool closeTcpAfterFailure()")
        close_end = self.sensor.index("bool connectWifi()", close_start)
        close_body = self.sensor[close_start:close_end]
        self.assertIn('sendAt("AT+CIPCLOSE", "OK", 1500, "ERROR")', close_body)
        self.assertIn("if (!espBusySeen)", close_body)

        fetch_start = self.sensor.index("bool fetchCommandResponse()")
        fetch_end = self.sensor.index(
            "\nbool reportRobotStatus(bool heartbeatOnly) {", fetch_start
        )
        self.assertGreaterEqual(
            self.sensor[fetch_start:fetch_end].count("closeTcpAfterFailure();"), 3
        )

        report_start = fetch_end
        report_end = self.sensor.index("\nbool extractJsonText", report_start)
        self.assertGreaterEqual(
            self.sensor[report_start:report_end].count("closeTcpAfterFailure();"), 3
        )

    def test_esp_busy_uses_quiet_backoff_instead_of_a_reset_command_storm(self) -> None:
        wait_start = self.sensor.index("bool waitFor(")
        wait_end = self.sensor.index("bool sendAt(", wait_start)
        wait_body = self.sensor[wait_start:wait_end]
        self.assertIn('strstr(espBuffer, "busy s")', wait_body)
        self.assertIn('strstr(espBuffer, "busy p")', wait_body)
        self.assertIn("espBusySeen = true;", wait_body)

        connect_start = self.sensor.index("bool connectWifi()")
        connect_end = self.sensor.index("bool collectHttpResponse()", connect_start)
        connect_body = self.sensor[connect_start:connect_end]
        first_probe = connect_body.index('sendAt("AT", "OK", 2500)')
        busy_guard = connect_body.index("if (!atReady && espBusySeen)")
        escape = connect_body.index('esp8266.print(F("+++"))')
        mode_reset = connect_body.index('sendAt("AT+CIPMODE=0"')
        self.assertLess(first_probe, busy_guard)
        self.assertLess(busy_guard, escape)
        self.assertLess(escape, mode_reset)
        executable_connect = re.sub(r"//.*", "", connect_body)
        self.assertNotIn('F("AT+RST', executable_connect)
        self.assertNotIn('sendAt("AT+RST', executable_connect)
        self.assertIn("if (!closeTcpAfterFailure()) return false;", connect_body)
        self.assertIn('sendAt("ATE0", "OK", 2000) && espBusySeen', connect_body)
        self.assertIn(
            'sendAt("AT+CIPMODE=0", "OK", 2000, "no change") && espBusySeen',
            connect_body,
        )
        self.assert_source(self.sensor, r"WIFI_RECONNECT_INTERVAL_MS\s*=\s*15000")

        collect_start = self.sensor.index("bool collectHttpResponse()")
        collect_end = self.sensor.index("bool fetchCommandResponse()", collect_start)
        collect_body = self.sensor[collect_start:collect_end]
        self.assertIn('strstr(espBuffer, "busy s")', collect_body)
        self.assertIn('strstr(espBuffer, "busy p")', collect_body)
        self.assertIn("wifiReady = false;", collect_body)
        self.assertIn("lastWifiReconnectAttemptAt = millis();", collect_body)

    def test_only_outbound_motion_may_clear_a_starting_home_marker(self) -> None:
        start = self.motor.index("void followLine()")
        end = self.motor.index("void updateHomeMarkerClearSafety()", start)
        body = self.motor[start:end]
        self.assertIn(
            "const bool leftBlack = leftValue == "
            "(LINE_BLACK_IS_HIGH ? HIGH : LOW);",
            body,
        )
        self.assertIn(
            "const bool rightBlack = rightValue == "
            "(LINE_BLACK_IS_HIGH ? HIGH : LOW);",
            body,
        )
        self.assertRegex(
            body,
            r"if\s*\(\s*!headingHomebound\s*&&\s*"
            r"leftBlack\s*&&\s*rightBlack\s*\)",
        )

    def test_linear_route_return_and_heartbeat_contracts_are_present(self) -> None:
        for token in (
            "STATION_HOME = 0",
            "STATION_ZONE2 = 1",
            "STATION_ZONE99 = 2",
            "confirmedStation",
            "expectedStation",
            "routeHeading",
            "targetAheadOnCurrentSegment",
            "ROUTE_UID_ERROR",
            'PSTR("RETURN_HOME")',
            "HOME_ALREADY",
            "RETURN_CONTINUING",
            "INVALID_COMMAND",
            "HEARTBEAT_INTERVAL_MS = 9000",
            "reportRobotStatus(true)",
        ):
            self.assertIn(token, self.sensor)
        self.assertNotIn('setCommandResult(F("SERVER_OFFLINE"))', self.sensor)
        self.assertRegex(
            self.sensor,
            r"MOTOR_FWD[\s\S]*?startMotorController\(HEADING_OUTBOUND\)",
        )
        self.assertNotIn("obstacleDistanceCm < 0 ||", self.sensor)
        self.assertIn("appliedCommand == expectedActuatorCommand", self.sensor)
        self.assertIn("appliedSequence != expectedActuatorSequence", self.sensor)

    def test_route_start_failure_and_missed_home_uid_latch_safe_positions(self) -> None:
        start = self.sensor.index("bool startRouteTravel(RouteStation destination)")
        end = self.sensor.index("void serviceMotorLink()", start)
        route_body = self.sensor[start:end]
        self.assertIn("RouteHeading nextHeading = routeHeading;", route_body)
        self.assertIn("RouteStation nextExpected = expectedStation;", route_body)
        failed_start = route_body.index("if (!startMotorController(nextHeading))")
        committed_heading = route_body.index("routeHeading = nextHeading;")
        self.assertLess(failed_start, committed_heading)
        failure_body = route_body[failed_start:committed_heading]
        for statement in (
            "stopMotorController();",
            "stopModuleController();",
            "latchRouteUnknown();",
        ):
            self.assertIn(statement, failure_body)

        unknown_start = self.sensor.index("void latchRouteUnknown()")
        unknown_end = self.sensor.index("bool stopMotorController()", unknown_start)
        unknown_body = self.sensor[unknown_start:unknown_end]
        for statement in (
            "confirmedStation = STATION_UNKNOWN;",
            "expectedStation = STATION_UNKNOWN;",
            "routeAtStation = false;",
        ):
            self.assertIn(statement, unknown_body)

        stop_start = self.sensor.index("bool stopMotorController()")
        stop_end = self.sensor.index("bool startMotorController", stop_start)
        stop_body = self.sensor[stop_start:stop_end]
        self.assertIn("obstaclePauseActive", stop_body)
        self.assertNotIn("latchRouteUnknown();", stop_body)

        fault_start = self.sensor.index("void applyMotorLinkState()")
        fault_end = self.sensor.index("void stopSafelyForServerLoss()", fault_start)
        fault_body = self.sensor[fault_start:fault_end]
        self.assertRegex(
            fault_body,
            r"if \(motorLinkFaultPending\)[\s\S]*?"
            r"stopMotorController\(\);[\s\S]*?"
            r"stopModuleController\(\);[\s\S]*?latchRouteUnknown\(\);",
        )

        self.assertIn('queueRobotReport(F("HOME_RFID_MISSED"))', self.sensor)
        self.assertRegex(
            self.sensor,
            r"targetStation == STATION_HOME[\s\S]*?"
            r"routeHeading == HEADING_HOMEBOUND[\s\S]*?"
            r"confirmedStation = STATION_HOME;[\s\S]*?"
            r"routeAtStation = true;[\s\S]*?robotPhase = PHASE_IDLE;",
        )

        wait_body = re.search(
            r"bool waitFor\([^\{]+\{(?P<body>.*?)\n\}", self.sensor, re.DOTALL
        )
        self.assertIsNotNone(wait_body)
        body = wait_body.group("body")
        self.assertIn("applyMotorLinkState();", body)
        self.assertIn("updatePlaceholderStateMachine();", body)
        self.assertIn("checkRfidArrival();", body)
        wait_services = [
            body.index("applyMotorLinkState();"),
            body.index("updatePlaceholderStateMachine();"),
            body.index("checkRfidArrival();"),
        ]
        self.assertEqual(wait_services, sorted(wait_services))
        # DHT/HC now run on their owning peripheral boards, never in an ESP wait.
        self.assertNotIn("updateDhtSensor();", body)
        self.assertNotIn("updateObstacleSensor();", body)
        self.assertNotIn("updateSensorLcd();", body)

        http_body = re.search(
            r"bool collectHttpResponse\(\)\s*\{(?P<body>.*?)\n\}",
            self.sensor,
            re.DOTALL,
        )
        self.assertIsNotNone(http_body)
        http_loop = http_body.group("body")
        for service in (
            "applyMotorLinkState();",
            "updatePlaceholderStateMachine();",
            "checkRfidArrival();",
        ):
            self.assertIn(service, http_loop)
        http_services = [
            http_loop.index("applyMotorLinkState();"),
            http_loop.index("updatePlaceholderStateMachine();"),
            http_loop.index("checkRfidArrival();"),
        ]
        self.assertEqual(http_services, sorted(http_services))
        self.assert_source(self.sensor, r"RFID_SCAN_INTERVAL_MS\s*=\s*40")
        self.assert_source(self.sensor, r"RFID_DIRECTION_SETTLE_MS\s*=\s*850")
        self.assert_source(self.sensor, r"espBuffer\[128\]")
        self.assertIn("if (!routeMoving || manualForwardActive) return;", self.sensor)
        process_start = self.sensor.index(
            "\nvoid processRouteRfid(RouteStation scannedStation) {"
        )
        process_end = self.sensor.index("\nvoid checkRfidArrival()", process_start)
        process_body = self.sensor[process_start:process_end]
        rfid_stop_failure_start = process_body.index("if (!stopMotorController())")
        rfid_stop_failure_end = process_body.index(
            "confirmedStation = scannedStation", rfid_stop_failure_start
        )
        rfid_stop_failure = process_body[
            rfid_stop_failure_start:rfid_stop_failure_end
        ]
        self.assertIn(
            "latchRouteUnknown();",
            rfid_stop_failure,
            "a missed Motor STOP ACK can let the cart move past the scanned RFID",
        )
        repeat_start = process_body.index(
            "if (scannedStation == lastAcceptedRfidStation"
        )
        repeat_end = process_body.index(
            "lastAcceptedRfidStation = scannedStation", repeat_start
        )
        repeat_guard = process_body[repeat_start:repeat_end]
        self.assertIn("scannedStation != expectedStation", repeat_guard)
        self.assertNotIn("lastAcceptedRfidAt", repeat_guard)
        self.assertNotIn("RFID_REPEAT_GUARD_MS", repeat_guard)
        self.assertIn("rfidDirectionGuardStartedAt = millis();", self.sensor)
        self.assertIn("rfidDirectionClearSeen = true;", self.sensor)

    def test_rfid_direction_guard_cannot_be_bypassed_by_test_injection(self) -> None:
        """All RFID entry points share the same tag-clear interlock.

        Both the web RFID_TEST command and the USB ``T`` command call
        processRouteRfid() directly.  Keeping the guard in that central function
        prevents either diagnostic path from accepting the old tag immediately
        after forward/reverse direction changes.
        """
        # Skip the forward declaration near the top of the sketch and inspect
        # the actual function definition.
        start = self.sensor.index(
            "\nvoid processRouteRfid(RouteStation scannedStation) {"
        )
        end = self.sensor.index("void checkRfidArrival()", start)
        body = self.sensor[start:end]
        guard = body.index("rfidDirectionGuardActive")
        stop = body.index("stopMotorController()")
        self.assertLess(guard, stop)
        self.assertIn("!rfidDirectionClearSeen", body[guard:stop])

    def test_failed_rc522_is_an_automatic_departure_interlock(self) -> None:
        """A robot that cannot identify stations must never start a TASK route."""
        self.assertRegex(self.sensor, r"bool\s+rfidReady\s*=\s*false\s*;")

        setup_start = self.sensor.index("void setup()")
        setup_end = self.sensor.index("void loop()", setup_start)
        setup_body = self.sensor[setup_start:setup_end]
        self.assertIn("rfidReady", setup_body)
        self.assertRegex(
            setup_body,
            r"rfidVersion\s*!=\s*0x00[\s\S]*?rfidVersion\s*!=\s*0xFF",
        )

        # Boot-time health can go stale after a loose power or SPI jumper.  A
        # helper must therefore perform a fresh VersionReg read, update the
        # global readiness latch, and return the result.
        health_start = self.sensor.index("bool refreshRfidHealth()")
        health_end = self.sensor.index("\n}", health_start)
        health_body = self.sensor[health_start:health_end]
        self.assertIn("SoftwareMFRC522::VersionReg", health_body)
        self.assertRegex(
            health_body,
            r"version\s*!=\s*0x00[\s\S]*?version\s*!=\s*0xFF",
        )
        self.assertIn("rfidReady = readyNow;", health_body)
        self.assertIn("return rfidReady;", health_body)

        # Both kinds of automatic physical departure (outbound TASK and
        # RETURN_HOME) must revalidate RC522 immediately before startRouteTravel.
        movement_start = self.sensor.index("bool startPlaceholderMovement()")
        movement_end = self.sensor.index("bool startPlaceholderReturn()", movement_start)
        return_end = self.sensor.index("bool startPlaceholderModule", movement_end)
        for route_body in (
            self.sensor[movement_start:movement_end],
            self.sensor[movement_end:return_end],
        ):
            with self.subTest(route_function=route_body.splitlines()[0]):
                probe = route_body.index("if (!refreshRfidHealth())")
                start = route_body.index("startRouteTravel(")
                self.assertLess(probe, start)
                self.assertIn('queueRobotReport(F("RFID_NOT_READY"))', route_body)
        self.assertGreaterEqual(
            self.sensor.count("SoftwareMFRC522::VersionReg"), 2
        )

    def test_route_timeouts_and_manual_forward_latch_unknown_position(self) -> None:
        state_start = self.sensor.index("void updatePlaceholderStateMachine()")
        state_end = self.sensor.index("void checkUsbTestCommands()", state_start)
        state_body = self.sensor[state_start:state_end]

        move_start = state_body.index("robotPhase == PHASE_MOVING")
        module_start = state_body.index("robotPhase == PHASE_MODULE_RUNNING")
        move_timeout = state_body[move_start:module_start]
        self.assertIn('queueRobotReport(F("RFID_NOT_FOUND"))', move_timeout)
        self.assertIn("latchRouteUnknown();", move_timeout)

        return_start = state_body.index("robotPhase == PHASE_RETURNING")
        return_timeout = state_body[return_start:]
        self.assertIn('queueRobotReport(F("RETURN_TIMEOUT"))', return_timeout)
        self.assertIn("latchRouteUnknown();", return_timeout)

        manual_start = self.sensor.index(
            'if (!strcmp_P(command, PSTR("MOTOR_FWD")))'
        )
        manual_end = self.sensor.index(
            '} else if (!strcmp_P(command, PSTR("MOTOR_RETURN")))', manual_start
        )
        manual_body = self.sensor[manual_start:manual_end]
        self.assertIn("latchRouteUnknown();", manual_body)
        self.assertLess(
            manual_body.index("latchRouteUnknown();"),
            manual_body.index("startMotorController(HEADING_OUTBOUND)"),
        )

    def test_failed_stop_ack_uses_a_local_independent_retry_latch(self) -> None:
        self.assertRegex(
            self.sensor,
            r"StopRetryMode\s+stopRetryMode\s*=\s*STOP_RETRY_NONE\s*;",
        )
        self.assertRegex(
            self.sensor, r"STOP_RETRY_INTERVAL_MS\s*=\s*500"
        )

        # Skip the forward declaration and inspect the implementation.
        arm_start = self.sensor.index(
            "\nvoid armStopRetry(StopRetryMode mode, bool motorStopped, "
            "bool moduleStopped) {"
        )
        arm_end = self.sensor.index("\n}", arm_start)
        arm_body = self.sensor[arm_start:arm_end]
        self.assertIn("robotPhase = PHASE_TASK_COMPLETE;", arm_body)
        self.assertIn('setCommandResult(F("FAILED"));', arm_body)

        # Skip its prototype near the top of the sketch.
        retry_start = self.sensor.index("\nvoid serviceStopRetry() {")
        retry_end = self.sensor.index("\n}", retry_start)
        retry_body = self.sensor[retry_start:retry_end]
        self.assertRegex(
            retry_body,
            r"if\s*\(\s*stopMotorRetryNeeded\s*&&\s*stopMotorController\(\)\s*\)",
        )
        self.assertRegex(
            retry_body,
            r"if\s*\(\s*stopActuatorRetryNeeded\s*&&\s*stopModuleController\(\)\s*\)",
        )
        both_pending = retry_body.index(
            "if (stopMotorRetryNeeded || stopActuatorRetryNeeded) return;"
        )
        complete = retry_body.index('queueRobotReport(F("STOP_CONFIRMED"))')
        self.assertLess(both_pending, complete)
        self.assertIn("robotPhase = PHASE_IDLE;", retry_body)
        self.assertIn('setCommandResult(F("COMPLETED"));', retry_body)
        self.assertIn("if (stopMotorRetryNeeded) latchRouteUnknown();", retry_body)

        loop_start = self.sensor.index("void loop()")
        loop_body = self.sensor[loop_start:]
        self.assertIn("serviceStopRetry();", loop_body)

        # While the local latch is active, route commands must not be parsed or
        # acknowledged.  Either pollServerCommand itself gates them, or loop()
        # suppresses network polling until both STOP acknowledgements arrive.
        poll_start = self.sensor.index("bool pollServerCommand()")
        poll_end = self.sensor.index("void makeUidText()", poll_start)
        poll_body = self.sensor[poll_start:poll_end]
        poll_guard = poll_body.find("stopRetryMode")
        revision_dispatch = poll_body.find("nextRevision == lastCommandRevision")
        loop_guard = loop_body.find("if (stopRetryMode)")
        loop_poll = loop_body.find("pollServerCommand()")
        self.assertTrue(
            (0 <= poll_guard < revision_dispatch)
            or (0 <= loop_guard < loop_poll),
            "STOP retry must gate TASK/RETURN before revision consumption",
        )

    def test_stop_retry_is_not_starved_by_blocking_esp_network_waits(self) -> None:
        """A nominal 500ms local retry must not sit behind a 7-30s AT wait."""
        loop_start = self.sensor.index("void loop()")
        loop_body = self.sensor[loop_start:]
        network_poll = loop_body.index("pollServerCommand()")

        # Either loop() must avoid entering network code while the latch is
        # active, or every blocking ESP wait path must service the latch.  The
        # former is preferable because it cannot dispatch a newly fetched route
        # in the same call stack that only just confirmed STOP.
        loop_prefix = loop_body[:network_poll]
        loop_suppresses_poll = bool(
            re.search(
                r"stopRetryMode\s*==\s*STOP_RETRY_NONE[\s\S]*?pollServerCommand",
                loop_body[: network_poll + len("pollServerCommand")],
            )
            or re.search(
                r"stopRetryMode\s*!=\s*STOP_RETRY_NONE[\s\S]*?"
                r"(?:return|continue)",
                loop_prefix,
            )
        )

        wait_start = self.sensor.index("bool waitFor(")
        wait_end = self.sensor.index("bool sendAt(", wait_start)
        collect_start = self.sensor.index("bool collectHttpResponse()")
        collect_end = self.sensor.index("bool fetchCommandResponse()", collect_start)
        waits_service_retry = all(
            "serviceStopRetry();" in body
            for body in (
                self.sensor[wait_start:wait_end],
                self.sensor[collect_start:collect_end],
            )
        )
        self.assertTrue(
            loop_suppresses_poll or waits_service_retry,
            "STOP retry can otherwise be delayed by long CIPSTART/CIPSEND/HTTP waits",
        )
        # loop() 진입 전 latch만 막는 것으로는 부족하다. ESP 대기 도중
        # Motor/RFID 상태 처리에서 latch가 새로 생기는 경합도 있으므로 두
        # blocking wait 모두 즉시 retry를 서비스하고 탈출해야 한다.
        for wait_body in (
            self.sensor[wait_start:wait_end],
            self.sensor[collect_start:collect_end],
        ):
            guard = wait_body.index(
                "if (stopRetryMode != STOP_RETRY_NONE) {"
            )
            retry = wait_body.index("serviceStopRetry();", guard)
            early_exit = wait_body.index("return false;", retry)
            self.assertLess(guard, retry)
            self.assertLess(retry, early_exit)

    def test_same_revision_server_loss_recovery_is_single_shot_and_route_safe(self) -> None:
        self.assertRegex(
            self.sensor, r"bool\s+retrySameRevisionAllowed\s*=\s*false\s*;"
        )

        stop_start = self.sensor.index("void stopSafelyForServerLoss()")
        stop_end = self.sensor.index("bool waitForActuatorCommand", stop_start)
        stop_body = self.sensor[stop_start:stop_end]
        self.assertRegex(
            stop_body,
            r"retrySameRevisionAllowed\s*=\s*"
            r"motorStopped\s*&&\s*moduleStopped\s*&&\s*"
            r"validRouteStation\(confirmedStation\)\s*&&\s*"
            r"validRouteStation\(expectedStation\)",
        )

        same_start = self.sensor.index("if (nextRevision == lastCommandRevision)")
        same_end = self.sensor.index(
            'Serial.print(F("[COMMAND] new revision="))', same_start
        )
        same_body = self.sensor[same_start:same_end]
        self.assertIn("retrySameRevisionAllowed", same_body)
        self.assertIn("!robotReportPending", same_body)
        self.assertIn("!deferredEvent[0]", same_body)
        self.assertIn('!strcmp_P(command, PSTR("TASK"))', same_body)
        self.assertIn('!strcmp_P(command, PSTR("RETURN_HOME"))', same_body)

        consume = same_body.index("retrySameRevisionAllowed = false;")
        task_restart = same_body.index("startPlaceholderMovement()")
        return_restart = same_body.index("startPlaceholderReturn()")
        self.assertLess(consume, task_restart)
        self.assertLess(consume, return_restart)

        new_revision_body = self.sensor[same_end : self.sensor.index(
            'if (!strcmp_P(command, PSTR("MOTOR_FWD")))', same_end
        )]
        self.assertIn("retrySameRevisionAllowed = false;", new_revision_body)

    def test_status_event_is_finished_only_after_matching_server_ack_revision(self) -> None:
        start = self.sensor.index("\nbool reportRobotStatus(bool heartbeatOnly) {")
        end = self.sensor.index("\nbool extractJsonText", start)
        body = self.sensor[start:end]

        parse_match = re.search(
            r'extractJsonLong\("ack_revision",\s*(\w+)\)', body
        )
        self.assertIsNotNone(parse_match)
        server_revision = parse_match.group(1)
        self.assertRegex(
            body,
            rf"{re.escape(server_revision)}\s*!=\s*acknowledgedRevision",
        )
        self.assertLess(parse_match.start(), body.index("finishRobotReport()"))

    def test_sensor_telemetry_frame_and_actuator_crc_reply_contracts(self) -> None:
        # SensorUno must no longer contain an LCD driver or probe an LCD address
        # on the shared A4/A5 control bus.
        for removed_sensor_lcd_symbol in (
            "LCD_ADDRESS",
            "lcdExpanderWrite",
            "initializeLcd",
            "updateSensorLcd",
            "lcdLineCache",
            "LiquidCrystal_I2C",
        ):
            self.assertNotIn(removed_sensor_lcd_symbol, self.sensor)

        for source in (self.sensor, self.actuator):
            self.assert_source(source, r"DISPLAY_FRAME_MAGIC\s*=\s*0xD1")
            self.assert_source(source, r"DISPLAY_FRAME_SIZE\s*=\s*10")
            self.assert_source(source, r"crc\s*=\s*\(crc\s*&\s*0x80\)")
            self.assertIn("^ 0x07", source)
            for name, value in (
                ("DISPLAY_STATE_IDLE", 0),
                ("DISPLAY_STATE_MOVING", 1),
                ("DISPLAY_STATE_HUMIDIFY", 2),
                ("DISPLAY_STATE_DEHUMIDIFY", 3),
                ("DISPLAY_STATE_DONE", 4),
                ("DISPLAY_STATE_RETURNING", 5),
                ("DISPLAY_STATE_ERROR", 6),
            ):
                self.assert_source(source, rf"{name}\s*=\s*{value}")

        build_start = self.sensor.index("void buildDisplayPayload(byte* payload)")
        build_end = self.sensor.index("bool sendDisplayTelemetryFrame()", build_start)
        build_body = self.sensor[build_start:build_end]
        for token in (
            "payload[0] = currentDisplayState();",
            "payload[1] = currentDisplayZoneCode();",
            "payload[2] = 0;",
            "payload[3] = 0;",
            "payload[4] = 0;",
            "payload[5] = 0;",
            "payload[6] = flags;",
        ):
            self.assertIn(token, build_body)
        self.assertNotIn("sensorTemperature", build_body)
        self.assertNotIn("sensorHumidity", build_body)
        self.assertIn("DHT dht(DHT_PIN, DHT_TYPE);", self.actuator)

        send_start = self.sensor.index("bool sendDisplayTelemetryFrame()")
        send_end = self.sensor.index("void serviceDisplayTelemetry()", send_start)
        send_body = self.sensor[send_start:send_end]
        for token in (
            "frame[0] = DISPLAY_FRAME_MAGIC;",
            "frame[1] = displaySequence;",
            "memcpy(frame + 2, displayPayloadCache, sizeof(displayPayloadCache));",
            "frame[9] = crc8Atm(frame, 9);",
            "Wire.beginTransmission(ACTUATOR_UNO_ADDRESS);",
            "Wire.write(frame, sizeof(frame));",
        ):
            self.assertIn(token, send_body)
        self.assert_source(self.sensor, r"displayPayloadCache\s*\[7\]")
        self.assert_source(self.sensor, r"DISPLAY_HEARTBEAT_MS\s*=\s*2000")
        self.assert_source(self.sensor, r"DISPLAY_RETRY_MS\s*=\s*500")

        read_start = self.sensor.index("bool readActuatorState(")
        read_end = self.sensor.index("bool i2cDevicePresent", read_start)
        read_body = self.sensor[read_start:read_end]
        self.assertRegex(
            read_body,
            r"requestFrom\([\s\S]*?ACTUATOR_UNO_ADDRESS[\s\S]*?ACTUATOR_STATUS_REPLY_SIZE\)",
        )
        for token in (
            "byte response[ACTUATOR_STATUS_REPLY_SIZE];",
            "crc8Atm(response, 5) != response[5]",
            "appliedSequence = response[2];",
            "lastDisplayAckSequence = response[3];",
            "lastDisplayStatusFlags = response[4];",
            "lastDisplayAckSequence == displaySequence",
        ):
            self.assertIn(token, read_body)

        for token in (
            "volatile byte statusReply[STATUS_REPLY_SIZE]",
            "nextReply[5] = crc8Atm(nextReply, 5);",
            "Wire.write(replySnapshot, sizeof(replySnapshot));",
        ):
            self.assertIn(token, self.actuator)

        request_start = self.actuator.index("void sendI2cStatus()")
        request_end = self.actuator.index("void serviceWireTimeout", request_start)
        request_body = re.sub(r"//.*", "", self.actuator[request_start:request_end])
        for forbidden in (
            "Serial",
            "millis",
            "micros",
            "crc8Atm",
            "serviceLcd",
            "writeRelay",
            "applyCommand",
        ):
            self.assertNotIn(forbidden, request_body)

        # Every blocking master transaction on SensorUno restores/checks the
        # hardware Wire clock timeout; the D5/D4 LCD bus is not part of Wire.
        wire_calls = re.findall(
            r"Wire\.(?:endTransmission\(\)|requestFrom\([^;]*?\));",
            self.sensor,
            re.DOTALL,
        )
        protected_calls = re.findall(
            r"Wire\.(?:endTransmission\(\)|requestFrom\([^;]*?\));"
            r"\s*restoreI2cClockAfterTimeout\(\);",
            self.sensor,
            re.DOTALL,
        )
        self.assertGreaterEqual(len(wire_calls), 4)
        self.assertEqual(len(protected_calls), len(wire_calls))

    def test_actuator_lcd_uses_bounded_open_drain_d5_d4_software_i2c(self) -> None:
        for pattern in (
            r"LCD_SOFT_SDA_PIN\s*=\s*5",
            r"LCD_SOFT_SCL_PIN\s*=\s*4",
            r"LCD_PRIMARY_ADDRESS\s*=\s*0x27",
            r"LCD_FALLBACK_ADDRESS\s*=\s*0x3F",
            r"SOFT_I2C_CLOCK_TIMEOUT_US\s*=\s*1000",
            r"LCD_RETRY_MS\s*=\s*5000",
        ):
            self.assert_source(self.actuator, pattern)
        self.assertNotIn("LiquidCrystal_I2C", self.actuator)
        self.assertNotRegex(
            self.actuator,
            r"digitalWrite\(LCD_SOFT_(?:SDA|SCL)_PIN,\s*HIGH\)",
        )

        release_start = self.actuator.index("void releaseSoftI2cLine")
        release_end = self.actuator.index("void pullSoftI2cLineLow", release_start)
        release_body = self.actuator[release_start:release_end]
        self.assertIn("pinMode(pin, INPUT_PULLUP);", release_body)
        self.assertNotIn("digitalWrite(pin, HIGH)", release_body)

        pull_start = release_end
        pull_end = self.actuator.index("bool waitForSoftSclHigh", pull_start)
        pull_body = self.actuator[pull_start:pull_end]
        self.assertLess(
            pull_body.index("digitalWrite(pin, LOW);"),
            pull_body.index("pinMode(pin, OUTPUT);"),
        )
        self.assertNotIn("digitalWrite(pin, HIGH)", pull_body)

        lcd_bus_start = release_start
        lcd_bus_end = self.actuator.index("void serviceDisplayMailbox", lcd_bus_start)
        lcd_bus_body = self.actuator[lcd_bus_start:lcd_bus_end]
        self.assertNotIn("Wire.", lcd_bus_body)
        self.assertIn("micros() - startedAt >= SOFT_I2C_CLOCK_TIMEOUT_US", lcd_bus_body)
        self.assertIn("pulse < 9", lcd_bus_body)
        self.assertGreaterEqual(lcd_bus_body.count("address << 1"), 2)

        lcd_service_start = self.actuator.index("void serviceLcd()")
        lcd_service_end = self.actuator.index(
            "void serviceDisplayMailbox", lcd_service_start
        )
        lcd_service = self.actuator[lcd_service_start:lcd_service_end]
        primary_probe = lcd_service.index("softI2cProbe(LCD_PRIMARY_ADDRESS)")
        fallback_probe = lcd_service.index("softI2cProbe(LCD_FALLBACK_ADDRESS)")
        self.assertLess(primary_probe, fallback_probe)
        # Rendering is cooperative: no loop can emit a whole row in one call.
        executable = re.sub(r"//.*", "", lcd_service)
        self.assertNotRegex(executable, r"\b(?:for|while)\s*\(")
        self.assertIn("++lcdRenderColumn;", lcd_service)
        self.assertIn("if (lcdRenderColumn < 16) return;", lcd_service)

    def test_lcd_and_display_faults_are_isolated_from_relay_timers(self) -> None:
        mark_start = self.actuator.index("void markLcdFailure")
        mark_end = self.actuator.index("void finishLcdInitialization", mark_start)
        mark_body = self.actuator[mark_start:mark_end]
        for forbidden in (
            "stopAllOutputs",
            "writeRelay",
            "publishActuatorState",
            "taskStartedAt",
            "stageStartedAt",
            "dehumidifyStage",
        ):
            self.assertNotIn(forbidden, mark_body)
        for token in (
            "lcdReady = false;",
            "lcdError = true;",
            "lcdInitState = LCD_INIT_RETRY_WAIT;",
            "rebuildDisplayFlags();",
        ):
            self.assertIn(token, mark_body)

        display_start = self.actuator.index("void serviceDisplayMailbox()")
        display_end = self.actuator.index("void serviceDisplayStaleness", display_start)
        display_body = self.actuator[display_start:display_end]
        for forbidden in (
            "stopAllOutputs",
            "writeRelay",
            "applyCommand",
            "taskStartedAt",
            "stageStartedAt",
            "dehumidifyStage",
        ):
            self.assertNotIn(forbidden, display_body)
        for token in (
            "frame[0] != DISPLAY_FRAME_MAGIC",
            "calculatedCrc != frame[DISPLAY_FRAME_SIZE - 1]",
            "nextState > DISPLAY_STATE_ERROR",
            "displayPayloadErrorCount",
        ):
            self.assertIn(token, display_body)
        self.assertLess(
            display_body.index("frame[0] != DISPLAY_FRAME_MAGIC"),
            display_body.index("lastDisplaySeq = frame[1];"),
        )
        for token in (
            "currentDisplayState = nextState;",
            "currentZoneCode = nextZone;",
            "currentInputFlags = frame[8];",
        ):
            self.assertIn(token, display_body)
        for removed_remote_dht_field in (
            "nextTemperatureTenths",
            "nextHumidityTenths",
            "currentTemperatureTenths",
            "currentHumidityTenths",
        ):
            self.assertNotIn(removed_remote_dht_field, display_body)
        local_dht_start = self.actuator.index("void serviceLocalDht()")
        local_dht_end = self.actuator.index("void serviceDisplayStaleness", local_dht_start)
        local_dht_body = self.actuator[local_dht_start:local_dht_end]
        self.assertIn("dht.readHumidity()", local_dht_body)
        self.assertIn("dht.readTemperature()", local_dht_body)
        self.assertIn("localHumidityTenths", local_dht_body)
        self.assertIn("localTemperatureTenths", local_dht_body)

        self.assert_source(self.actuator, r"DISPLAY_STALE_MS\s*=\s*30000")
        loop_start = self.actuator.index("void loop()")
        loop_body = self.actuator[loop_start:]
        ordered_services = [
            loop_body.index("serviceWireTimeout();"),
            loop_body.index("serviceCommandMailbox();"),
            loop_body.index("serviceActuatorTask();"),
            loop_body.index("serviceDisplayMailbox();"),
            loop_body.index("serviceDisplayStaleness();"),
            loop_body.index("serviceLcd();"),
        ]
        self.assertEqual(ordered_services, sorted(ordered_services))

        sensor_display_start = self.sensor.index("void serviceDisplayTelemetry()")
        sensor_display_end = self.sensor.index(
            "void copyFlashText", sensor_display_start
        )
        sensor_display_body = self.sensor[sensor_display_start:sensor_display_end]
        for forbidden in (
            "stopMotorController",
            "stopModuleController",
            "armStopRetry",
            "latchRouteUnknown",
        ):
            self.assertNotIn(forbidden, sensor_display_body)


if __name__ == "__main__":
    unittest.main(verbosity=2)
