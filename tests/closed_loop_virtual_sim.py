"""Deterministic, offline closed-loop simulator for the three-Uno robot.

This is deliberately test-only.  It never opens a COM port, socket, MySQL
connection, or the remote server.  The simulator joins the production server's
humidity/revision/ACK functions to the behavioral Sensor/Motor/Actuator models
from :mod:`test_robot_protocol_integration` and supplies only the physical
environment events that software cannot create itself (RFID stations, a
no-card observation after direction reversal, and the HOME stop marker).

The virtual clock advances in 100 ms quanta, so one complete mission takes a
few milliseconds of host time while preserving the firmware's 0.5 s fan
prestart, 5 s module run, 2 s fan cooldown, 0.85 s RFID direction-settle guard,
and 0.3 s HOME marker confirmation timings.
"""

from __future__ import annotations

import importlib.util
import sys
from contextlib import ExitStack
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from unittest.mock import patch

try:  # ``python -m unittest tests...`` package import
    from .test_robot_protocol_integration import (
        ActuatorCommand,
        MotorCommand,
        RouteStation,
        ThreeUnoMissionCoordinatorSim,
    )
except ImportError:  # direct script and ``unittest discover -s tests``
    from test_robot_protocol_integration import (
        ActuatorCommand,
        MotorCommand,
        RouteStation,
        ThreeUnoMissionCoordinatorSim,
    )


ROOT = Path(__file__).resolve().parents[1]
SERVER_PATH = ROOT / "server" / "server.py"


def _load_server() -> Any:
    """Load server.py once under an isolated name for offline simulation."""

    module_name = "closed_loop_server_under_test"
    existing = sys.modules.get(module_name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(module_name, SERVER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SERVER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


SERVER = _load_server()


@dataclass
class VirtualClock:
    """Millisecond clock whose server view is whole Unix-like seconds."""

    milliseconds: int = 1_000_000

    @property
    def seconds(self) -> int:
        return self.milliseconds // 1_000

    def advance(self, delta_ms: int) -> int:
        if delta_ms < 0:
            raise ValueError("virtual time cannot move backwards")
        self.milliseconds += delta_ms
        return self.milliseconds


class OfflineCommandDatabase:
    """Small SQL-shaped state store for the production mission functions.

    Only statements issued by ``robot_command_snapshot``,
    ``recompute_mission`` and ``complete_auto_task`` are accepted.  An
    unexpected query fails the test instead of silently creating a weaker
    simulation.
    """

    def __init__(self, timestamp: int) -> None:
        self.zones: dict[str, dict[str, Any]] = {
            zone_id: {
                "zone_id": zone_id,
                "temperature": 25.0,
                "humidity": 70.0,
                "updated_at": timestamp,
                "latest_reading_id": 1,
                "acted_through_reading_id": 0,
                "violation_mode": None,
                "violation_started_at": None,
            }
            for zone_id in SERVER.DEFAULT_ZONE_IDS
        }
        self.mission: dict[str, Any] = {
            "id": 1,
            "revision": 900,
            "command": "RETURN_HOME",
            "target_zone": "HOME",
            "action": "NONE",
            "reason": "initial normal readings",
            "updated_at": timestamp,
        }
        self.manual: dict[str, Any] = {
            "id": 1,
            "enabled": False,
            "revision": 800,
            "command": "ALL_STOP",
            "target_zone": "HOME",
            "action": "NONE",
            "updated_at": timestamp,
        }
        self.commits = 0
        self.rollbacks = 0

    def connection(self) -> Any:
        database = self

        class Connection:
            def cursor(self) -> "OfflineCommandCursor":
                return OfflineCommandCursor(database)

            def commit(self) -> None:
                database.commits += 1

            def rollback(self) -> None:
                database.rollbacks += 1

            def close(self) -> None:
                return None

        return Connection()


class OfflineCommandCursor:
    def __init__(self, database: OfflineCommandDatabase) -> None:
        self.database = database
        self.last_sql = ""
        self.rowcount = 0

    def __enter__(self) -> "OfflineCommandCursor":
        return self

    def __exit__(self, *_args: object) -> bool:
        return False

    def execute(self, sql: str, params: Any = None) -> None:
        normalized = " ".join(sql.split())
        self.last_sql = normalized.lower()
        self.rowcount = 0

        if self.last_sql.startswith(
            "update zone_status set acted_through_reading_id"
        ):
            if "where zone_id = %s" in self.last_sql:
                zone_ids = (params[0],)
            elif "where zone_id in" in self.last_sql:
                zone_ids = tuple(params)
            else:
                raise AssertionError(f"unsupported watermark update: {normalized}")
            for zone_id in zone_ids:
                row = self.database.zones.get(str(zone_id))
                if row is None:
                    continue
                previous = int(row.get("acted_through_reading_id") or 0)
                latest = int(row.get("latest_reading_id") or 0)
                row["acted_through_reading_id"] = max(previous, latest)
                if latest > previous:
                    self.rowcount += 1
            return

        if self.last_sql.startswith("update mission set revision"):
            if len(params) == 7:
                revision, command, zone, action, reason, updated_at, _ = params
            elif len(params) == 4:
                revision, reason, updated_at, expected_revision = params
                if int(self.database.mission["revision"]) != int(expected_revision):
                    return
                command, zone, action = "ALL_STOP", "HOME", "NONE"
            else:
                raise AssertionError((normalized, params))
            self.database.mission.update(
                revision=int(revision),
                command=str(command),
                target_zone=str(zone),
                action=str(action),
                reason=str(reason),
                updated_at=int(updated_at),
            )
            self.rowcount = 1
            return

        if self.last_sql.startswith("update mission set reason"):
            reason, updated_at, _ = params
            self.database.mission.update(
                reason=str(reason), updated_at=int(updated_at)
            )
            self.rowcount = 1
            return

        # Every remaining supported statement is a SELECT.  fetchone/fetchall
        # interprets the remembered normalized text and returns a copy.
        if self.last_sql.startswith("select "):
            return
        raise AssertionError(f"unexpected SQL in closed-loop simulator: {normalized}")

    def fetchall(self) -> list[dict[str, Any]]:
        if "from zone_status" in self.last_sql:
            return [dict(row) for row in self.database.zones.values()]
        raise AssertionError(f"unexpected fetchall: {self.last_sql}")

    def fetchone(self) -> dict[str, Any]:
        if "from mission" in self.last_sql:
            return dict(self.database.mission)
        if "from manual_control" in self.last_sql:
            return dict(self.database.manual)
        raise AssertionError(f"unexpected fetchone: {self.last_sql}")


@dataclass(frozen=True)
class TraceEntry:
    time_ms: int
    component: str
    event: str
    revision: int | None = None
    detail: str = ""


@dataclass(frozen=True)
class ClosedLoopResult:
    zone: str
    action: str
    task_revision: int
    hold_revision: int
    return_revision: int
    abnormal_reading_id: int
    normal_reading_id: int
    module_completed_ms: int
    normal_reading_ms: int
    final_time_ms: int
    final_station: str
    final_phase: str
    outputs_off: bool
    dispatched_revisions: tuple[int, ...]
    trace: tuple[TraceEntry, ...]


@dataclass
class ClosedLoopMissionSim:
    """Production-server-driven mission with deterministic physical events."""

    zone: str
    action: str
    clock: VirtualClock = field(default_factory=VirtualClock)
    car: ThreeUnoMissionCoordinatorSim = field(
        default_factory=ThreeUnoMissionCoordinatorSim
    )
    trace: list[TraceEntry] = field(default_factory=list)
    last_sensor_revision: int = -1
    dispatched_revisions: list[int] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.zone not in SERVER.DEFAULT_ZONE_IDS:
            raise ValueError(f"unsupported target zone: {self.zone}")
        if self.action not in {"HUMIDIFY", "DEHUMIDIFY"}:
            raise ValueError(f"unsupported action: {self.action}")
        self.database = OfflineCommandDatabase(self.clock.seconds)

    @property
    def target_station(self) -> RouteStation:
        return RouteStation.ZONE2 if self.zone == "ZONE2" else RouteStation.ZONE99

    @property
    def actuator_command(self) -> ActuatorCommand:
        return (
            ActuatorCommand.HUMIDIFY
            if self.action == "HUMIDIFY"
            else ActuatorCommand.DEHUMIDIFY
        )

    def _record(
        self,
        component: str,
        event: str,
        *,
        revision: int | None = None,
        detail: str = "",
    ) -> None:
        self.trace.append(
            TraceEntry(
                self.clock.milliseconds,
                component,
                event,
                revision,
                detail,
            )
        )

    def _reset_server_state(self) -> None:
        with SERVER.SETTINGS_LOCK:
            SERVER.THRESHOLDS.clear()
            SERVER.THRESHOLDS.update(SERVER.DEFAULT_THRESHOLDS)
        with SERVER.MANUAL_CONTROL_LOCK:
            SERVER.MANUAL_CONTROL.update(self.database.manual)
        with SERVER.ROBOT_NETWORK_LOCK:
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

    def _runtime(self) -> ExitStack:
        stack = ExitStack()
        stack.enter_context(
            patch.object(SERVER, "connect", side_effect=self.database.connection)
        )
        stack.enter_context(
            patch.object(SERVER, "now", side_effect=lambda: self.clock.seconds)
        )
        stack.enter_context(patch.object(SERVER, "touch_device", return_value=None))
        stack.enter_context(patch.object(SERVER, "record_event", return_value=None))
        return stack

    def _reading(self, zone: str, humidity: float) -> int:
        row = self.database.zones[zone]
        previous_mode = row.get("violation_mode")
        condition = SERVER.classify_humidity(humidity, previous_mode)
        next_id = int(row.get("latest_reading_id") or 0) + 1
        mode = None if condition["action"] == "NONE" else condition["action"]
        if mode is None:
            started_at = None
        elif mode == previous_mode:
            started_at = row.get("violation_started_at") or self.clock.seconds
        else:
            started_at = self.clock.seconds
        row.update(
            temperature=25.0,
            humidity=float(humidity),
            updated_at=self.clock.seconds,
            latest_reading_id=next_id,
            violation_mode=mode,
            violation_started_at=started_at,
        )
        self._record(
            "ZONE_SENSOR",
            "READING",
            detail=(
                f"{zone} H={humidity:.1f}% id={next_id} "
                f"state={condition['state']}"
            ),
        )
        return next_id

    def _robot_zone(self) -> str:
        return self.car.route.confirmed.name

    def _report(self, result: str, event: str) -> dict[str, Any]:
        response = SERVER.report_robot_network(
            {
                "phase": [self.car.phase],
                "event": [event],
                "zone": [self._robot_zone()],
                "action": [self.action if self.car.task_active else "NONE"],
                "ack_revision": [str(self.last_sensor_revision)],
                "result": [result],
            },
            "192.0.2.44",
        )
        if not response["ack_accepted"]:
            raise AssertionError(response["ack_rejection"])
        self._record(
            "SENSOR_SERVER_ACK",
            event,
            revision=self.last_sensor_revision,
            detail=result,
        )
        return response

    def _dispatch(self, command: dict[str, Any]) -> str:
        revision = int(command["revision"])
        if revision == self.last_sensor_revision:
            self._record("SENSOR_UNO", "DUPLICATE_IGNORED", revision=revision)
            return "DUPLICATE"

        self.last_sensor_revision = revision
        self.dispatched_revisions.append(revision)
        name = str(command["command"])
        if name == "TASK":
            target = (
                RouteStation.ZONE2
                if command["target_zone"] == "ZONE2"
                else RouteStation.ZONE99
            )
            action = (
                ActuatorCommand.HUMIDIFY
                if command["action"] == "HUMIDIFY"
                else ActuatorCommand.DEHUMIDIFY
            )
            self.car.task(target, action, self.clock.milliseconds)
            self._record(
                "SENSOR_UNO",
                "TASK_DISPATCHED",
                revision=revision,
                detail=f"{target.name}/{action.name}",
            )
            self._report("EXECUTING", "DISPATCHED")
        elif name == "ALL_STOP":
            self.car.motor.command(MotorCommand.STOP, self.clock.milliseconds)
            self.car.actuator.command(ActuatorCommand.STOP, self.clock.milliseconds)
            self.car.task_active = False
            self.car.phase = "IDLE"
            self._record("SENSOR_UNO", "ALL_STOP", revision=revision)
            self._report("COMPLETED", "STOP_CONFIRMED")
        elif name == "RETURN_HOME":
            self.car.return_home(self.clock.milliseconds)
            self._record("SENSOR_UNO", "RETURN_STARTED", revision=revision)
            if self.car.phase == "IDLE":
                self._report("COMPLETED", "HOME_ALREADY")
            else:
                self._report("EXECUTING", "RETURN_STARTED")
        else:
            raise AssertionError(f"unexpected automatic command: {command}")
        return name

    def _poll_server(self) -> tuple[dict[str, Any], str]:
        command = SERVER.robot_command_snapshot()
        SERVER.mark_command_delivered(int(command["revision"]), self.clock.seconds)
        self._record(
            "PC_SERVER",
            str(command["command"]),
            revision=int(command["revision"]),
            detail=f"{command['target_zone']}/{command['action']}",
        )
        return command, self._dispatch(command)

    def _advance_module_until_complete(self, timeout_ms: int = 8_000) -> int:
        deadline = self.clock.milliseconds + timeout_ms
        while self.clock.milliseconds < deadline:
            self.clock.advance(100)
            self.car.finish_module(self.clock.milliseconds)
            if self.car.phase == "TASK_COMPLETE":
                self._record(
                    "ACTUATOR_UNO",
                    "MODULE_COMPLETE",
                    detail=self.action,
                )
                return self.clock.milliseconds
        raise AssertionError(f"{self.action} did not finish before virtual deadline")

    def _rfid(self, station: RouteStation, travel_ms: int = 1_000) -> str:
        self.clock.advance(travel_ms)
        outcome = self.car.rfid(station, self.clock.milliseconds)
        self._record(
            "RC522",
            outcome,
            detail=station.name,
        )
        return outcome

    def _rfid_clear_after_reverse(self) -> None:
        """Model the RC522 seeing open track after forward changes to reverse."""

        self.clock.advance(250)
        self.car.route.rfid_clear(self.clock.milliseconds)
        self._record("RFID_FIELD", "NO_CARD", detail="direction guard clear")

    def _home_marker(self) -> str:
        self.clock.advance(1_000)
        first_high = self.clock.milliseconds
        self.clock.advance(300)
        outcome = self.car.home_marker(first_high, self.clock.milliseconds)
        self._record("LINE_TRACKER", outcome, detail="HOME wide marker")
        return outcome

    def run(self) -> ClosedLoopResult:
        self._reset_server_state()
        with self._runtime():
            abnormal_humidity = 45.0 if self.action == "HUMIDIFY" else 90.0
            abnormal_id = self._reading(self.zone, abnormal_humidity)

            task, dispatched = self._poll_server()
            if dispatched != "TASK":
                raise AssertionError(task)
            task_revision = int(task["revision"])
            if task["target_zone"] != self.zone or task["action"] != self.action:
                raise AssertionError(task)

            # A repeated HTTP poll must not restart a mission on the same rev.
            repeated, repeat_result = self._poll_server()
            if int(repeated["revision"]) != task_revision or repeat_result != "DUPLICATE":
                raise AssertionError((repeated, repeat_result))

            first = self._rfid(RouteStation.ZONE2)
            expected_first = "TARGET" if self.zone == "ZONE2" else "PASS"
            if first != expected_first:
                raise AssertionError((first, expected_first))
            if self.zone == "ZONE99":
                if self._rfid(RouteStation.ZONE99) != "TARGET":
                    raise AssertionError("ZONE99 target RFID was not accepted")

            if self.car.phase != "MODULE_RUNNING":
                raise AssertionError(self.car.phase)
            if self.action == "HUMIDIFY":
                if not self.car.actuator.humidifier_on:
                    raise AssertionError("humidifier relay never started")
            elif not self.car.actuator.fan_on:
                raise AssertionError("dehumidifier fan never prestarted")

            completed_ms = self._advance_module_until_complete()
            self._report("COMPLETED", "MODULE_COMPLETE")

            # complete_auto_task must atomically consume the abnormal sample
            # and install the distinct ALL_STOP/fresh-reading hold revision.
            hold, hold_dispatch = self._poll_server()
            if hold_dispatch != "ALL_STOP" or hold["command"] != "ALL_STOP":
                raise AssertionError(hold)
            hold_revision = int(hold["revision"])
            if (
                int(self.database.zones[self.zone]["acted_through_reading_id"])
                != abnormal_id
            ):
                raise AssertionError("completed reading watermark was not consumed")

            # A sample timestamped at/before module completion is intentionally
            # not injected.  The one-second step proves this is a fresh reading.
            self.clock.advance(1_000)
            normal_reading_ms = self.clock.milliseconds
            normal_id = self._reading(self.zone, 70.0)
            returning, return_dispatch = self._poll_server()
            if return_dispatch != "RETURN_HOME":
                raise AssertionError(returning)
            return_revision = int(returning["revision"])

            if self.zone == "ZONE99":
                self._rfid_clear_after_reverse()
                if self._rfid(RouteStation.ZONE2, travel_ms=750) != "PASS":
                    raise AssertionError("homebound ZONE2 RFID was not accepted")
            if self._home_marker() != "HOME":
                raise AssertionError("HOME line marker did not finish return")
            self._report("COMPLETED", "HOME_ARRIVAL")

            outputs_off = not any(
                (
                    self.car.actuator.humidifier_on,
                    self.car.actuator.peltier_on,
                    self.car.actuator.fan_on,
                )
            )
            return ClosedLoopResult(
                zone=self.zone,
                action=self.action,
                task_revision=task_revision,
                hold_revision=hold_revision,
                return_revision=return_revision,
                abnormal_reading_id=abnormal_id,
                normal_reading_id=normal_id,
                module_completed_ms=completed_ms,
                normal_reading_ms=normal_reading_ms,
                final_time_ms=self.clock.milliseconds,
                final_station=self.car.route.confirmed.name,
                final_phase=self.car.phase,
                outputs_off=outputs_off,
                dispatched_revisions=tuple(self.dispatched_revisions),
                trace=tuple(self.trace),
            )


def run_all_nominal_scenarios() -> tuple[ClosedLoopResult, ...]:
    """Run ZONE2/ZONE99 crossed with HUMIDIFY/DEHUMIDIFY."""

    return tuple(
        ClosedLoopMissionSim(zone, action).run()
        for zone in SERVER.DEFAULT_ZONE_IDS
        for action in ("HUMIDIFY", "DEHUMIDIFY")
    )


if __name__ == "__main__":
    for result in run_all_nominal_scenarios():
        elapsed = result.final_time_ms - 1_000_000
        print(
            f"PASS {result.zone:6s} {result.action:10s} "
            f"rev={result.task_revision}->{result.hold_revision}->"
            f"{result.return_revision} elapsed={elapsed}ms "
            f"events={len(result.trace)}"
        )
