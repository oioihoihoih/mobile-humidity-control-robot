"""Offline regression tests for server.py.

These tests replace every database/network side effect with small in-memory
fakes. They are safe to run while the real robot server and MySQL database are
stopped or in use elsewhere.
"""

from __future__ import annotations

import importlib.util
import io
import unittest
from pathlib import Path
from unittest.mock import patch


SERVER_PATH = Path(__file__).with_name("server.py")
SPEC = importlib.util.spec_from_file_location("robot_server_under_test", SERVER_PATH)
assert SPEC is not None and SPEC.loader is not None
server = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(server)


def fake_manual_transition(
    desired: dict,
    timestamp: int,
    *,
    enter_auto: bool = False,
    audit_event: tuple[str, str, str] | None = None,
    audit_snapshot: bool = False,
) -> dict:
    """Unit tests must never open the developer's real MySQL instance."""
    del enter_auto, audit_event, audit_snapshot
    return {
        **desired,
        "revision": server.next_command_revision(
            int(desired.get("revision") or 0),
            timestamp=timestamp,
        ),
        "updated_at": timestamp,
    }


class MissionCursor:
    def __init__(
        self,
        zones: list[dict],
        mission: dict,
        manual_revision: int = 0,
    ):
        self.zones = zones
        self.mission = dict(mission)
        self.manual = {
            "id": 1,
            "enabled": False,
            "revision": manual_revision,
            "command": "ALL_STOP",
            "target_zone": "HOME",
            "action": "NONE",
            "updated_at": 0,
        }
        self.last_sql = ""
        self.calls: list[tuple[str, object]] = []
        self.updates: list[str] = []

    def execute(self, sql: str, params=None) -> None:
        normalized = " ".join(sql.split())
        self.last_sql = normalized.lower()
        self.calls.append((normalized, params))
        if self.last_sql.startswith("update mission set revision"):
            revision, command, zone, action, reason, updated_at, _ = params
            self.mission.update(
                revision=revision,
                command=command,
                target_zone=zone,
                action=action,
                reason=reason,
                updated_at=updated_at,
            )
            self.updates.append("command")
        elif self.last_sql.startswith("update mission set reason"):
            reason, updated_at, _ = params
            self.mission.update(reason=reason, updated_at=updated_at)
            self.updates.append("reason")

    def fetchall(self) -> list[dict]:
        if "from zone_status" in self.last_sql:
            return [dict(row) for row in self.zones]
        raise AssertionError(f"unexpected fetchall: {self.last_sql}")

    def fetchone(self) -> dict:
        if "from mission" in self.last_sql:
            return dict(self.mission)
        if "from manual_control" in self.last_sql:
            return dict(self.manual)
        raise AssertionError(f"unexpected fetchone: {self.last_sql}")


class CommandDatabase:
    """AUTO/MANUAL 명령 트랜잭션만 재현하는 완전 오프라인 DB fake."""

    def __init__(
        self,
        zones: list[dict],
        mission: dict,
        manual: dict,
        *,
        fail_event_insert: bool = False,
    ):
        self.zones = {row["zone_id"]: dict(row) for row in zones}
        self.mission = dict(mission)
        self.manual = dict(manual)
        self.events: list[dict] = []
        self.fail_event_insert = fail_event_insert
        self.commits = 0
        self.rollbacks = 0

    def connection(self):
        database = self
        original_zones = {key: dict(value) for key, value in database.zones.items()}
        original_mission = dict(database.mission)
        original_manual = dict(database.manual)
        original_events = list(database.events)

        class Connection:
            def cursor(self):
                return CommandCursor(database)

            def commit(self):
                database.commits += 1

            def rollback(self):
                database.rollbacks += 1
                database.zones.clear()
                database.zones.update(
                    {key: dict(value) for key, value in original_zones.items()}
                )
                database.mission.clear()
                database.mission.update(original_mission)
                database.manual.clear()
                database.manual.update(original_manual)
                database.events[:] = original_events

            def close(self):
                pass

        return Connection()


class CommandCursor:
    def __init__(self, database: CommandDatabase):
        self.database = database
        self.last_sql = ""
        self.rowcount = 0

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def execute(self, sql: str, params=None) -> None:
        normalized = " ".join(sql.split())
        self.last_sql = normalized.lower()
        self.rowcount = 0

        if self.last_sql.startswith("update zone_status set acted_through_reading_id"):
            if "where zone_id = %s" in self.last_sql:
                zone_ids = (params[0],)
            else:
                zone_ids = tuple(params)
            for zone_id in zone_ids:
                zone = self.database.zones.get(zone_id)
                if zone is None:
                    continue
                previous = int(zone.get("acted_through_reading_id") or 0)
                updated = max(
                    previous,
                    int(zone.get("latest_reading_id") or 0),
                )
                zone["acted_through_reading_id"] = updated
                if updated != previous:
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
            elif len(params) == 3:
                revision, reason, updated_at = params
                command, zone, action = "ALL_STOP", "HOME", "NONE"
            else:
                raise AssertionError((normalized, params))
            self.database.mission.update(
                revision=int(revision),
                command=command,
                target_zone=zone,
                action=action,
                reason=reason,
                updated_at=int(updated_at),
            )
            self.rowcount = 1
            return

        if self.last_sql.startswith("update mission set reason"):
            reason, updated_at, _ = params
            self.database.mission.update(reason=reason, updated_at=updated_at)
            self.rowcount = 1
            return

        if self.last_sql.startswith("insert into manual_control"):
            enabled, revision, command, zone, action, updated_at = params
            self.database.manual.update(
                enabled=bool(enabled),
                revision=int(revision),
                command=command,
                target_zone=zone,
                action=action,
                updated_at=int(updated_at),
            )
            self.rowcount = 1
            return

        if self.last_sql.startswith("insert into event_log"):
            if self.database.fail_event_insert:
                raise server.pymysql.OperationalError(2006, "audit insert failed")
            received_at, source, event_type, message, data_json = params
            self.database.events.append(
                {
                    "received_at": received_at,
                    "source": source,
                    "event_type": event_type,
                    "message": message,
                    "data": server.json.loads(data_json),
                }
            )
            self.rowcount = 1

    def fetchall(self) -> list[dict]:
        if "from zone_status" in self.last_sql:
            return [dict(row) for row in self.database.zones.values()]
        raise AssertionError(f"unexpected fetchall: {self.last_sql}")

    def fetchone(self) -> dict:
        if "from mission" in self.last_sql:
            return dict(self.database.mission)
        if "from manual_control" in self.last_sql:
            return dict(self.database.manual)
        raise AssertionError(f"unexpected fetchone: {self.last_sql}")


class ServerLogicTests(unittest.TestCase):
    def setUp(self) -> None:
        with server.MANUAL_CONTROL_LOCK:
            server.MANUAL_CONTROL.update(
                enabled=False,
                revision=1_000_000,
                command="MOTOR_STOP",
                target_zone="HOME",
                action="NONE",
                updated_at=None,
            )
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(
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
        with server.SETTINGS_LOCK:
            server.THRESHOLDS.clear()
            server.THRESHOLDS.update(server.DEFAULT_THRESHOLDS)
        with server.SERIAL_LOCK:
            server.SERIAL_STATE.clear()
            server.SERIAL_STATE.update(server.SERIAL_STATE_DEFAULTS)
            server.SERIAL_LINES.clear()

    def test_manual_task_accepts_only_active_zones(self) -> None:
        with (
            patch.object(server, "persist_manual_transition", side_effect=fake_manual_transition),
            patch.object(server, "record_event"),
            patch.object(server, "now", return_value=1_700_000_000),
        ):
            with self.assertRaisesRegex(ValueError, "ZONE2, ZONE99"):
                server.update_manual_control(
                    {
                        "mode": "MANUAL",
                        "command": "TASK",
                        "target_zone": "ZONE1",
                        "action": "HUMIDIFY",
                    }
                )
            result = server.update_manual_control(
                {
                    "mode": "MANUAL",
                    "command": "TASK",
                    "target_zone": "ZONE2",
                    "action": "HUMIDIFY",
                }
            )
        self.assertEqual(result["target_zone"], "ZONE2")

    def test_calibrate_home_is_distinct_four_field_manual_command_with_global_revision(self) -> None:
        """HOME 보정은 TASK/AUTO가 아니며 기존 DB·wire 계약을 그대로 쓴다."""
        database = CommandDatabase(
            [],
            {
                "id": 1,
                "revision": 1_700_000_100,
                "command": "ALL_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "reason": "safe hold",
                "updated_at": 1_700_000_100,
            },
            {
                "id": 1,
                "enabled": False,
                "revision": 1_700_000_050,
                "command": "MOTOR_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "updated_at": 1_700_000_050,
            },
        )
        with (
            patch.object(server, "connect", side_effect=database.connection),
            patch.object(server, "record_event"),
            patch.object(server, "now", return_value=1_700_000_000),
        ):
            control = server.update_manual_control(
                {
                    "mode": "MANUAL",
                    "command": "CALIBRATE_HOME",
                    # 입력이 섞여도 보정 명령의 고정 계약으로 정규화한다.
                    "target_zone": "ZONE99",
                    "action": "DEHUMIDIFY",
                }
            )

        self.assertTrue(control["enabled"])
        self.assertEqual(control["command"], "CALIBRATE_HOME")
        self.assertEqual(control["target_zone"], "HOME")
        self.assertEqual(control["action"], "NONE")
        self.assertEqual(control["revision"], 1_700_000_101)
        self.assertEqual(database.manual["revision"], control["revision"])
        self.assertEqual(database.mission["revision"], 1_700_000_100)

        with patch.object(
            server,
            "robot_command_snapshot",
            return_value={**control, "source": "MANUAL"},
        ):
            wire = server.robot_command()
        self.assertEqual(
            set(wire),
            {"revision", "command", "target_zone", "action"},
        )
        self.assertEqual(wire["command"], "CALIBRATE_HOME")

        server.mark_command_delivered(control["revision"], timestamp=1_700_000_102)
        no_side_effect = lambda *args, **kwargs: None
        with (
            patch.object(
                server,
                "robot_command_snapshot",
                return_value={**wire, "source": "MANUAL"},
            ),
            patch.object(server, "touch_device", side_effect=no_side_effect),
            patch.object(server, "record_event", side_effect=no_side_effect),
            patch.object(server, "now", return_value=1_700_000_103),
        ):
            ack = server.report_robot_network(
                {
                    "phase": ["IDLE"],
                    "event": ["HOME_CALIBRATED"],
                    "zone": ["HOME"],
                    "action": ["NONE"],
                    "ack_revision": [str(control["revision"])],
                    "result": ["COMPLETED"],
                },
                "192.0.2.20",
            )
        self.assertTrue(ack["ack_accepted"])
        self.assertEqual(ack["ack_revision"], control["revision"])

    def test_manual_command_and_audit_event_commit_atomically(self) -> None:
        database = CommandDatabase(
            [],
            {
                "id": 1,
                "revision": 1_700_000_100,
                "command": "ALL_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "reason": "safe hold",
                "updated_at": 1_700_000_100,
            },
            {
                "id": 1,
                "enabled": False,
                "revision": 1_700_000_050,
                "command": "MOTOR_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "updated_at": 1_700_000_050,
            },
        )
        with (
            patch.object(server, "connect", side_effect=database.connection),
            patch.object(server, "now", return_value=1_700_000_200),
        ):
            control = server.update_manual_control(
                {"mode": "MANUAL", "command": "MOTOR_STOP"}
            )

        self.assertEqual(database.commits, 1)
        self.assertEqual(database.rollbacks, 0)
        self.assertEqual(database.manual["revision"], control["revision"])
        self.assertEqual(len(database.events), 1)
        self.assertEqual(database.events[0]["event_type"], "MANUAL_COMMAND")
        self.assertEqual(database.events[0]["data"]["revision"], control["revision"])

    def test_manual_command_rolls_back_when_audit_event_insert_fails(self) -> None:
        original_manual = {
            "id": 1,
            "enabled": False,
            "revision": 1_700_000_050,
            "command": "MOTOR_STOP",
            "target_zone": "HOME",
            "action": "NONE",
            "updated_at": 1_700_000_050,
        }
        database = CommandDatabase(
            [],
            {
                "id": 1,
                "revision": 1_700_000_100,
                "command": "ALL_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "reason": "safe hold",
                "updated_at": 1_700_000_100,
            },
            original_manual,
            fail_event_insert=True,
        )
        memory_before = server.manual_control_snapshot()

        with (
            patch.object(server, "connect", side_effect=database.connection),
            patch.object(server, "now", return_value=1_700_000_200),
        ):
            with self.assertRaises(server.pymysql.OperationalError):
                server.update_manual_control(
                    {"mode": "MANUAL", "command": "MOTOR_STOP"}
                )

        self.assertEqual(database.commits, 0)
        self.assertEqual(database.rollbacks, 1)
        self.assertEqual(database.manual, original_manual)
        self.assertEqual(database.events, [])
        self.assertEqual(server.manual_control_snapshot(), memory_before)

    def test_calibrate_home_obeys_pending_ack_and_all_stop_latch(self) -> None:
        with server.MANUAL_CONTROL_LOCK:
            server.MANUAL_CONTROL.update(
                enabled=True,
                revision=123,
                command="MOTOR_STOP",
                target_zone="HOME",
                action="NONE",
                updated_at=100,
            )
        with self.assertRaisesRegex(ValueError, "not been acknowledged"):
            server.update_manual_control(
                {"mode": "MANUAL", "command": "CALIBRATE_HOME"}
            )

        # ALL_STOP을 이미 ACK했더라도 latch는 명시적인 AUTO 복귀 확인 전까지
        # HOME 보정 명령을 포함한 다른 수동 명령으로 우회할 수 없다.
        with server.MANUAL_CONTROL_LOCK:
            server.MANUAL_CONTROL.update(command="ALL_STOP")
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(
                ack_revision=123,
                ack_result="COMPLETED",
            )
        with self.assertRaisesRegex(ValueError, "ALL_STOP is latched"):
            server.update_manual_control(
                {"mode": "MANUAL", "command": "CALIBRATE_HOME"}
            )

    def test_unacknowledged_manual_command_is_not_overwritten_except_all_stop(self) -> None:
        with server.MANUAL_CONTROL_LOCK:
            server.MANUAL_CONTROL.update(
                enabled=True,
                revision=1_700_000_000,
                command="MOTOR_FWD",
                target_zone="HOME",
                action="NONE",
                updated_at=1_700_000_000,
            )
        persisted: list[dict] = []
        with (
            patch.object(
                server,
                "persist_manual_transition",
                side_effect=lambda desired, timestamp, **kwargs: (
                    persisted.append(
                        fake_manual_transition(desired, timestamp, **kwargs)
                    )
                    or persisted[-1]
                ),
            ),
            patch.object(server, "record_event"),
            patch.object(server, "now", return_value=1_700_000_001),
        ):
            with self.assertRaisesRegex(ValueError, "not been acknowledged"):
                server.update_manual_control({"mode": "MANUAL", "command": "MOTOR_RETURN"})
            emergency = server.update_manual_control({"mode": "MANUAL", "command": "ALL_STOP"})
            repeated = server.update_manual_control({"mode": "MANUAL", "command": "ALL_STOP"})
        self.assertEqual(emergency["command"], "ALL_STOP")
        self.assertEqual(repeated["revision"], emergency["revision"])
        self.assertEqual(len(persisted), 1)

    def test_auto_mode_waits_for_current_manual_ack(self) -> None:
        with server.MANUAL_CONTROL_LOCK:
            server.MANUAL_CONTROL.update(
                enabled=True,
                revision=123,
                command="MOTOR_STOP",
                target_zone="HOME",
                action="NONE",
                updated_at=100,
            )
        with (
            patch.object(server, "persist_manual_transition", side_effect=fake_manual_transition),
            patch.object(server, "record_event"),
            patch.object(server, "now", return_value=200),
        ):
            with self.assertRaisesRegex(ValueError, "not been acknowledged"):
                server.update_manual_control({"mode": "AUTO"})
            with server.ROBOT_NETWORK_LOCK:
                server.ROBOT_NETWORK_STATE.update(ack_revision=123, ack_result="COMPLETED")
            result = server.update_manual_control({"mode": "AUTO"})
        self.assertFalse(result["enabled"])

    def test_ack_requires_delivered_and_effective_revision(self) -> None:
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(delivered_revision=42, delivered_at=90)
        no_side_effect = lambda *args, **kwargs: None
        with (
            patch.object(
                server,
                "robot_command_snapshot",
                return_value={"revision": 42, "source": "AUTO"},
            ),
            patch.object(server, "touch_device", side_effect=no_side_effect),
            patch.object(server, "record_event", side_effect=no_side_effect),
            patch.object(server, "now", return_value=100),
        ):
            accepted = server.report_robot_network(
                {
                    "phase": ["MOVING"],
                    "event": ["HEARTBEAT"],
                    "zone": ["ZONE2"],
                    "action": ["HUMIDIFY"],
                    "ack_revision": ["42"],
                    "result": ["EXECUTING"],
                },
                "192.0.2.10",
            )
            self.assertTrue(accepted["ack_accepted"])
            with server.ROBOT_NETWORK_LOCK:
                server.ROBOT_NETWORK_STATE.update(ack_revision=None, ack_result=None)
            rejected = server.report_robot_network(
                {
                    "phase": ["MOVING"],
                    "event": ["HEARTBEAT"],
                    "zone": ["ZONE2"],
                    "action": ["HUMIDIFY"],
                    "ack_revision": ["41"],
                    "result": ["COMPLETED"],
                },
                "192.0.2.10",
            )
        self.assertFalse(rejected["ack_accepted"])
        self.assertIsNone(server.ROBOT_NETWORK_STATE["ack_revision"])
        self.assertEqual(server.ROBOT_NETWORK_STATE["last_seen"], 100)

    def test_ack_result_allowlist_and_display_states(self) -> None:
        no_side_effect = lambda *args, **kwargs: None
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(delivered_revision=7)
        with (
            patch.object(
                server,
                "robot_command_snapshot",
                return_value={"revision": 7, "source": "AUTO"},
            ),
            patch.object(server, "touch_device", side_effect=no_side_effect),
            patch.object(server, "record_event", side_effect=no_side_effect),
        ):
            with self.assertRaisesRegex(ValueError, "unsupported robot command result"):
                server.report_robot_network(
                    {"ack_revision": ["7"], "result": ["MADE_UP"]}, "192.0.2.10"
                )

        command = {
            "revision": 7,
            "command": "TASK",
            "target_zone": "ZONE2",
            "action": "HUMIDIFY",
            "updated_at": 1,
        }
        for result, expected in (
            ("EXECUTING", "ACK_EXECUTING"),
            ("COMPLETED", "ACK_COMPLETED"),
            ("FAILED", "ACK_FAILED"),
            ("INVALID_ACTION", "ACK_FAILED"),
        ):
            state = server.command_delivery_snapshot(
                command,
                {
                    "delivered_revision": 7,
                    "ack_revision": 7,
                    "ack_result": result,
                    "online": True,
                },
            )
            self.assertEqual(state["state"], expected)

    def test_completed_manual_task_transitions_once_to_same_zone_task_none(self) -> None:
        """A manual module burst must not repeat forever on the same revision."""
        with server.MANUAL_CONTROL_LOCK:
            server.MANUAL_CONTROL.update(
                enabled=True,
                revision=123,
                command="TASK",
                target_zone="ZONE99",
                action="DEHUMIDIFY",
                updated_at=100,
            )
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(delivered_revision=123, delivered_at=100)

        persisted: list[dict] = []
        no_side_effect = lambda *args, **kwargs: None

        def current_manual_command() -> dict:
            snapshot = server.manual_control_snapshot()
            return {
                key: snapshot[key]
                for key in ("revision", "command", "target_zone", "action")
            } | {"source": "MANUAL"}

        def capture_manual_transition(desired, timestamp, **kwargs):
            snapshot = fake_manual_transition(desired, timestamp, **kwargs)
            persisted.append(dict(snapshot))
            return snapshot

        with (
            patch.object(server, "robot_command_snapshot", side_effect=current_manual_command),
            patch.object(
                server,
                "persist_manual_transition",
                side_effect=capture_manual_transition,
            ),
            patch.object(server, "touch_device", side_effect=no_side_effect),
            patch.object(server, "record_event", side_effect=no_side_effect),
            patch.object(server, "now", return_value=200),
        ):
            executing = server.report_robot_network(
                {
                    "phase": ["MODULE_RUNNING"],
                    "event": ["RFID_ARRIVAL"],
                    "zone": ["ZONE99"],
                    "action": ["DEHUMIDIFY"],
                    "ack_revision": ["123"],
                    "result": ["EXECUTING"],
                },
                "192.0.2.10",
            )
            self.assertTrue(executing["ack_accepted"])
            self.assertEqual(server.MANUAL_CONTROL["action"], "DEHUMIDIFY")

            completed = server.report_robot_network(
                {
                    "phase": ["TASK_COMPLETE"],
                    "event": ["MODULE_COMPLETE"],
                    "zone": ["ZONE99"],
                    "action": ["DEHUMIDIFY"],
                    "ack_revision": ["123"],
                    "result": ["COMPLETED"],
                },
                "192.0.2.10",
            )
            self.assertTrue(completed["ack_accepted"])
            terminated = server.manual_control_snapshot()
            self.assertTrue(terminated["enabled"])
            self.assertEqual(terminated["command"], "TASK")
            self.assertEqual(terminated["target_zone"], "ZONE99")
            self.assertEqual(terminated["action"], "NONE")
            self.assertGreater(terminated["revision"], 123)
            termination_revision = terminated["revision"]

            # A lost HTTP response may make the robot replay the already
            # accepted completion.  It is ACKed idempotently but cannot create
            # another termination revision.
            duplicate = server.report_robot_network(
                {
                    "phase": ["TASK_COMPLETE"],
                    "event": ["MODULE_COMPLETE"],
                    "zone": ["ZONE99"],
                    "action": ["DEHUMIDIFY"],
                    "ack_revision": ["123"],
                    "result": ["COMPLETED"],
                },
                "192.0.2.10",
            )
            self.assertTrue(duplicate["ack_accepted"])
            self.assertEqual(duplicate["ack_revision"], 123)

            # ACKing the termination TASK/NONE is terminal as well.
            with server.ROBOT_NETWORK_LOCK:
                server.ROBOT_NETWORK_STATE["delivered_revision"] = termination_revision
            stopped = server.report_robot_network(
                {
                    "phase": ["TASK_COMPLETE"],
                    "event": ["MOVE_ONLY_COMPLETE"],
                    "zone": ["ZONE99"],
                    "action": ["NONE"],
                    "ack_revision": [str(termination_revision)],
                    "result": ["COMPLETED"],
                },
                "192.0.2.10",
            )
            self.assertTrue(stopped["ack_accepted"])

        self.assertEqual(server.MANUAL_CONTROL["revision"], termination_revision)
        self.assertEqual(len(persisted), 1)

    def test_manual_task_handoff_requires_an_accepted_completed_ack(self) -> None:
        no_side_effect = lambda *args, **kwargs: None
        for result in ("EXECUTING", "FAILED"):
            with self.subTest(result=result):
                self.setUp()
                with server.MANUAL_CONTROL_LOCK:
                    server.MANUAL_CONTROL.update(
                        enabled=True,
                        revision=321,
                        command="TASK",
                        target_zone="ZONE2",
                        action="HUMIDIFY",
                        updated_at=100,
                    )
                with server.ROBOT_NETWORK_LOCK:
                    server.ROBOT_NETWORK_STATE["delivered_revision"] = 321
                with (
                    patch.object(
                        server,
                        "robot_command_snapshot",
                        return_value={"revision": 321, "source": "MANUAL"},
                    ),
                    patch.object(
                        server,
                        "persist_manual_transition",
                        side_effect=fake_manual_transition,
                    ),
                    patch.object(server, "touch_device", side_effect=no_side_effect),
                    patch.object(server, "record_event", side_effect=no_side_effect),
                    patch.object(server, "now", return_value=200),
                ):
                    response = server.report_robot_network(
                        {
                            "phase": ["TASK_COMPLETE"],
                            "event": ["STATUS"],
                            "zone": ["ZONE2"],
                            "action": ["HUMIDIFY"],
                            "ack_revision": ["321"],
                            "result": [result],
                        },
                        "192.0.2.10",
                    )
                self.assertTrue(response["ack_accepted"])
                self.assertEqual(server.MANUAL_CONTROL["revision"], 321)
                self.assertEqual(server.MANUAL_CONTROL["action"], "HUMIDIFY")

    def test_command_delivery_does_not_spoof_online_status(self) -> None:
        server.mark_command_delivered(55, timestamp=100)
        state = server.robot_network_snapshot(timestamp=100)
        self.assertEqual(state["delivered_revision"], 55)
        self.assertFalse(state["online"])
        self.assertIsNone(state["last_seen"])
        self.assertIsNone(state["ip"])

    def test_robot_heartbeat_updates_online_state_without_event_log_spam(self) -> None:
        touches: list[tuple] = []
        events: list[tuple] = []
        with (
            patch.object(server, "now", return_value=100),
            patch.object(server, "touch_device", side_effect=lambda *args, **kwargs: touches.append((args, kwargs))),
            patch.object(server, "record_event", side_effect=lambda *args, **kwargs: events.append((args, kwargs))),
        ):
            result = server.report_robot_network(
                {
                    "phase": ["IDLE"],
                    "event": ["HEARTBEAT"],
                    "zone": ["HOME"],
                    "action": ["NONE"],
                },
                "192.0.2.81",
            )
        self.assertTrue(result["accepted"])
        self.assertEqual(server.ROBOT_NETWORK_STATE["last_seen"], 100)
        self.assertEqual(server.ROBOT_NETWORK_STATE["ip"], "192.0.2.81")
        self.assertEqual(len(touches), 1)
        self.assertEqual(events, [])
        self.assertTrue(server.robot_network_snapshot(timestamp=110)["online"])

    def test_usb_connection_alone_is_not_gateway_online(self) -> None:
        self.assertFalse(server.SERIAL_STATE_DEFAULTS["enabled"])
        self.assertEqual(server.SERIAL_STATE_DEFAULTS["configured_port"], "")
        with server.SERIAL_LOCK:
            server.SERIAL_STATE.update(
                enabled=True,
                connected=True,
                active_port="COM5",
                esp_ready=False,
                wifi_ready=False,
                server_reachable=False,
            )
        state = server.serial_snapshot(timestamp=100)
        self.assertTrue(state["serial_connected"])
        self.assertFalse(state["network_ready"])
        self.assertFalse(state["online"])
        self.assertFalse(state["fully_ready"])
        self.assertEqual(state["health"], "SERIAL_ONLY")

    def test_gateway_health_requires_a_fresh_server_heartbeat(self) -> None:
        with server.SERIAL_LOCK:
            server.SERIAL_STATE.update(enabled=True, connected=True, active_port="COM5")
        bridge = server.SerialBridge()
        with (
            patch.object(server, "now", return_value=100),
            patch.object(server, "touch_device"),
            patch.object(server, "record_event"),
        ):
            bridge._append_line("[ESP RX] OK")
            bridge._append_line("[GATEWAY] WIFI OK")
        wifi_only = server.serial_snapshot(timestamp=100)
        self.assertTrue(wifi_only["esp_ready"])
        self.assertTrue(wifi_only["wifi_ready"])
        self.assertFalse(wifi_only["network_ready"])
        self.assertEqual(wifi_only["health"], "DEGRADED")

        with (
            patch.object(server, "now", return_value=101),
            patch.object(server, "touch_device"),
            patch.object(server, "record_event"),
        ):
            bridge._append_line("[GATEWAY] HEARTBEAT OK")
        ready = server.serial_snapshot(timestamp=101)
        self.assertTrue(ready["network_ready"])
        self.assertTrue(ready["fully_ready"])
        self.assertEqual(ready["health"], "READY")

        stale = server.serial_snapshot(
            timestamp=101 + server.GATEWAY_HEARTBEAT_STALE_SECONDS + 1
        )
        self.assertFalse(stale["network_ready"])
        self.assertEqual(stale["health"], "DEGRADED")

        with (
            patch.object(server, "now", return_value=140),
            patch.object(server, "touch_device"),
            patch.object(server, "record_event"),
        ):
            bridge._append_line("[ESP RX] (no response)")
        failed = server.serial_snapshot(timestamp=140)
        self.assertFalse(failed["esp_ready"])
        self.assertFalse(failed["wifi_ready"])
        self.assertFalse(failed["network_ready"])
        self.assertEqual(failed["health"], "SERIAL_ONLY")
        self.assertEqual(failed["gateway_error"], "ESP-01 AT no response")

    def test_http_gateway_heartbeat_can_be_network_only(self) -> None:
        server.mark_gateway_heartbeat("192.0.2.55", timestamp=200)
        state = server.serial_snapshot(timestamp=200)
        self.assertFalse(state["serial_connected"])
        self.assertTrue(state["network_ready"])
        self.assertTrue(state["online"])
        self.assertFalse(state["fully_ready"])
        self.assertEqual(state["health"], "NETWORK_ONLY")
        self.assertEqual(state["gateway_ip"], "192.0.2.55")

    def test_gateway_devices_do_not_inherit_false_online_from_failure_logs(self) -> None:
        devices = [
            {"device_id": "USB_SERVER_GATEWAY", "online": True, "status": "ONLINE", "details": {}},
            {"device_id": "SERVER_GATEWAY_WIFI", "online": True, "status": "ONLINE", "details": {}},
            {"device_id": "ROBOT_WIFI", "online": True, "status": "ONLINE", "details": {}},
        ]
        serial_state = {
            "serial_connected": True,
            "esp_ready": False,
            "wifi_ready": False,
            "network_ready": False,
            "fully_ready": False,
            "health": "SERIAL_ONLY",
        }
        server.apply_gateway_health_to_devices(devices, serial_state)
        self.assertFalse(devices[0]["online"])
        self.assertEqual(devices[0]["status"], "SERIAL_ONLY")
        self.assertFalse(devices[1]["online"])
        self.assertTrue(devices[2]["online"])

    def test_repeated_at_retry_lines_are_coalesced_but_counted(self) -> None:
        bridge = server.SerialBridge()
        with (
            patch.object(server, "now", side_effect=(100, 101, 102, 140)),
            patch.object(server, "touch_device"),
            patch.object(server, "record_event"),
        ):
            bridge._append_line("[ESP RX] (no response)")
            bridge._append_line("[ESP RX] (no response)")
            bridge._append_line("[ESP RX] (no response)")
            bridge._append_line("[ESP RX] (no response)")
        state = server.serial_snapshot(timestamp=140)
        self.assertEqual(state["sequence"], 4)
        self.assertEqual(len(state["lines"]), 2)
        self.assertEqual(state["lines"][0]["repeat_count"], 3)
        self.assertEqual(state["lines"][1]["repeat_count"], 1)

    def test_revision_is_stable_and_ties_are_deterministic(self) -> None:
        zones = [
            {
                "zone_id": "ZONE99",
                "temperature": 25,
                "humidity": 50,
                "updated_at": 999,
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 900,
            },
            {
                "zone_id": "ZONE2",
                "temperature": 25,
                "humidity": 50,
                "updated_at": 999,
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 900,
            },
        ]
        mission = {
            "id": 1,
            "revision": 7,
            "command": "TASK",
            "target_zone": "ZONE2",
            "action": "HUMIDIFY",
            "reason": "old display reason",
            "updated_at": 900,
        }
        cursor = MissionCursor(zones, mission)
        with patch.object(server, "now", return_value=1000):
            first = server.recompute_mission(cursor)
        self.assertEqual(first["target_zone"], "ZONE2")
        self.assertEqual(first["revision"], 7)
        self.assertEqual(cursor.updates, [])
        zone_query = next(call for call in cursor.calls if "FROM zone_status" in call[0])
        self.assertEqual(tuple(zone_query[1]), ("ZONE2", "ZONE99"))

    def test_completed_auto_task_rearms_only_after_a_new_target_reading(self) -> None:
        zones = [
            {
                "zone_id": "ZONE2",
                "temperature": 25,
                "humidity": 50,
                "updated_at": 1_000,
                "latest_reading_id": 10,
                "acted_through_reading_id": 10,
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 900,
            },
            {
                "zone_id": "ZONE99",
                "temperature": 25,
                "humidity": 70,
                "updated_at": 1_000,
                "latest_reading_id": 20,
                "acted_through_reading_id": 0,
                "violation_mode": "NONE",
                "violation_started_at": None,
            },
        ]
        mission = {
            "id": 1,
            "revision": 8,
            "command": "ALL_STOP",
            "target_zone": "HOME",
            "action": "NONE",
            "reason": "waiting for a newer zone reading",
            "updated_at": 990,
        }

        # 완료 시 소비한 reading id는 서버 메모리와 무관하게 다음 burst가 아니다.
        cursor = MissionCursor(zones, mission)
        with patch.object(server, "now", return_value=1_010):
            unchanged = server.recompute_mission(cursor)
        self.assertEqual(unchanged["revision"], 8)
        self.assertEqual(unchanged["command"], "ALL_STOP")
        self.assertEqual(cursor.updates, [])

        # 새 reading_log.id가 여전히 저습도일 때 정확히 한 번만 재가동한다.
        zones[0] = {
            **zones[0],
            "updated_at": 1_001,
            "latest_reading_id": 11,
        }
        cursor = MissionCursor(zones, mission)
        with patch.object(server, "now", return_value=1_010):
            rearmed = server.recompute_mission(cursor)
            repeated_get = server.recompute_mission(cursor)
        self.assertGreater(rearmed["revision"], 8)
        self.assertEqual(repeated_get["revision"], rearmed["revision"])
        self.assertEqual(rearmed["command"], "TASK")
        self.assertEqual(cursor.updates.count("command"), 1)

    def test_auto_completion_is_persistent_idempotent_and_restart_safe(self) -> None:
        zones = [
            {
                "zone_id": "ZONE2",
                "temperature": 25,
                "humidity": 50,
                "updated_at": 990,
                "latest_reading_id": 55,
                "acted_through_reading_id": 54,
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 900,
            },
            {
                "zone_id": "ZONE99",
                "temperature": 25,
                "humidity": 70,
                "updated_at": 990,
                "latest_reading_id": 22,
                "acted_through_reading_id": 0,
                "violation_mode": None,
                "violation_started_at": None,
            },
        ]
        database = CommandDatabase(
            zones,
            {
                "id": 1,
                "revision": 100,
                "command": "TASK",
                "target_zone": "ZONE2",
                "action": "HUMIDIFY",
                "reason": "low humidity",
                "updated_at": 900,
            },
            {
                "id": 1,
                "enabled": False,
                "revision": 90,
                "command": "ALL_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "updated_at": 800,
            },
        )
        effective = {
            "revision": 100,
            "command": "TASK",
            "target_zone": "ZONE2",
            "action": "HUMIDIFY",
            "source": "AUTO",
        }
        with patch.object(server, "connect", side_effect=database.connection):
            completed = server.complete_auto_task(effective, 100, 1_000)
            duplicate = server.complete_auto_task(effective, 100, 1_001)

        self.assertIsNotNone(completed)
        self.assertIsNone(duplicate)
        self.assertEqual(database.zones["ZONE2"]["acted_through_reading_id"], 55)
        self.assertEqual(database.mission["command"], "ALL_STOP")
        self.assertEqual(database.mission["target_zone"], "HOME")
        self.assertEqual(database.mission["action"], "NONE")
        self.assertEqual(database.mission["revision"], 1_000)
        self.assertEqual(database.commits, 1)

        # 서버 메모리를 모두 잃은 재시작을 가정해도 DB watermark만으로 HOLD한다.
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(
                completed_auto_revision=None,
                completed_auto_at=None,
                completed_auto_zone=None,
                completed_auto_action=None,
            )
        with patch.object(server, "now", return_value=1_002):
            after_restart = server.recompute_mission(CommandCursor(database))
        self.assertEqual(after_restart["command"], "ALL_STOP")
        self.assertEqual(after_restart["revision"], 1_000)

        database.zones["ZONE2"].update(
            latest_reading_id=56,
            updated_at=1_003,
        )
        with patch.object(server, "now", return_value=1_003):
            next_burst = server.recompute_mission(CommandCursor(database))
        self.assertEqual(next_burst["command"], "TASK")
        self.assertEqual(next_burst["revision"], 1_003)

    def test_completion_race_consumes_target_without_overwriting_newer_mission(self) -> None:
        database = CommandDatabase(
            [
                {
                    "zone_id": "ZONE2",
                    "temperature": 25,
                    "humidity": 50,
                    "updated_at": 1_000,
                    "latest_reading_id": 56,
                    "acted_through_reading_id": 54,
                    "violation_mode": "HUMIDIFY",
                    "violation_started_at": 900,
                },
                {
                    "zone_id": "ZONE99",
                    "temperature": 25,
                    "humidity": 40,
                    "updated_at": 1_000,
                    "latest_reading_id": 22,
                    "acted_through_reading_id": 0,
                    "violation_mode": "HUMIDIFY",
                    "violation_started_at": 800,
                },
            ],
            {
                "id": 1,
                "revision": 101,
                "command": "TASK",
                "target_zone": "ZONE99",
                "action": "HUMIDIFY",
                "reason": "newer priority winner",
                "updated_at": 1_000,
            },
            {
                "id": 1,
                "enabled": False,
                "revision": 90,
                "command": "ALL_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "updated_at": 800,
            },
        )
        stale_snapshot = {
            "revision": 100,
            "command": "TASK",
            "target_zone": "ZONE2",
            "action": "HUMIDIFY",
            "source": "AUTO",
        }
        with (
            patch.object(server, "connect", side_effect=database.connection),
            patch.object(server, "now", return_value=1_001),
        ):
            completion = server.complete_auto_task(stale_snapshot, 100, 1_001)
            duplicate = server.complete_auto_task(stale_snapshot, 100, 1_002)

        self.assertIsNotNone(completion)
        self.assertIsNone(duplicate)
        self.assertEqual(database.zones["ZONE2"]["acted_through_reading_id"], 56)
        self.assertEqual(database.mission["revision"], 101)
        self.assertEqual(database.mission["command"], "TASK")
        self.assertEqual(database.mission["target_zone"], "ZONE99")
        self.assertEqual(database.commits, 1)
        self.assertEqual(database.rollbacks, 1)

    def test_manual_to_auto_consumes_old_readings_and_uses_global_revisions(self) -> None:
        zones = [
            {
                "zone_id": "ZONE2",
                "temperature": 25,
                "humidity": 50,
                "updated_at": 1_690,
                "latest_reading_id": 10,
                "acted_through_reading_id": 0,
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 1_600,
            },
            {
                "zone_id": "ZONE99",
                "temperature": 25,
                "humidity": 70,
                "updated_at": 1_690,
                "latest_reading_id": 20,
                "acted_through_reading_id": 0,
                "violation_mode": None,
                "violation_started_at": None,
            },
        ]
        database = CommandDatabase(
            zones,
            {
                "id": 1,
                "revision": 1_500,
                "command": "TASK",
                "target_zone": "ZONE2",
                "action": "HUMIDIFY",
                "reason": "old AUTO task",
                "updated_at": 1_500,
            },
            {
                "id": 1,
                "enabled": True,
                "revision": 1_700,
                "command": "MOTOR_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "updated_at": 1_700,
            },
        )
        desired_auto = {**database.manual, "enabled": False}
        with patch.object(server, "connect", side_effect=database.connection):
            auto = server.persist_manual_transition(
                desired_auto,
                2_000,
                enter_auto=True,
            )
            snapshot = server.robot_command_snapshot()
            wire = server.robot_command()

        self.assertFalse(auto["enabled"])
        self.assertEqual(database.mission["revision"], 2_000)
        self.assertEqual(database.manual["revision"], 2_000)
        self.assertEqual(database.mission["command"], "ALL_STOP")
        self.assertEqual(snapshot["source"], "AUTO")
        self.assertEqual(snapshot["revision"], 2_000)
        self.assertEqual(set(wire), {"revision", "command", "target_zone", "action"})
        self.assertNotIn("source", wire)
        self.assertEqual(database.zones["ZONE2"]["acted_through_reading_id"], 10)
        self.assertEqual(database.zones["ZONE99"]["acted_through_reading_id"], 20)

        # 과거 AUTO rev 1500은 되살아나지 않고, 전환 후 새 측정만 새 TASK를 연다.
        database.zones["ZONE2"].update(
            latest_reading_id=11,
            updated_at=2_001,
        )
        with patch.object(server, "now", return_value=2_001):
            new_auto = server.recompute_mission(CommandCursor(database))
        self.assertEqual(new_auto["command"], "TASK")
        self.assertGreater(new_auto["revision"], 2_000)

        previous_auto_revision = new_auto["revision"]
        next_manual = {**database.manual, "enabled": True, "command": "MOTOR_STOP"}
        with patch.object(server, "connect", side_effect=database.connection):
            manual = server.persist_manual_transition(next_manual, 2_001)
        self.assertGreater(manual["revision"], previous_auto_revision)

    def test_revision_allocator_enforces_avr_signed_long_limit(self) -> None:
        self.assertEqual(
            server.next_command_revision(
                server.AVR_REVISION_MAX - 1,
                timestamp=0,
            ),
            server.AVR_REVISION_MAX,
        )
        with self.assertRaisesRegex(RuntimeError, "AVR signed long"):
            server.next_command_revision(server.AVR_REVISION_MAX, timestamp=0)
        with self.assertRaisesRegex(RuntimeError, "AVR signed long"):
            server.next_command_revision(0, timestamp=server.AVR_REVISION_MAX + 1)

    def test_auto_completion_barrier_is_first_ack_and_excludes_manual(self) -> None:
        command = {
            "revision": 42,
            "command": "TASK",
            "target_zone": "ZONE2",
            "action": "HUMIDIFY",
        }
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(delivered_revision=42, delivered_at=90)
        no_side_effect = lambda *args, **kwargs: None
        query = {
            "phase": ["TASK_COMPLETE"],
            "event": ["MODULE_COMPLETE"],
            "zone": ["ZONE2"],
            "action": ["HUMIDIFY"],
            "ack_revision": ["42"],
            "result": ["COMPLETED"],
        }
        with (
            patch.object(
                server,
                "robot_command_snapshot",
                return_value={**command, "source": "AUTO"},
            ),
            patch.object(
                server,
                "complete_auto_task",
                return_value={
                    "completed_zone": "ZONE2",
                    "completed_action": "HUMIDIFY",
                    "hold_revision": 43,
                },
            ),
            patch.object(server, "touch_device", side_effect=no_side_effect),
            patch.object(server, "record_event", side_effect=no_side_effect),
            patch.object(server, "now", return_value=100),
        ):
            server.report_robot_network(query, "192.0.2.10")
        with server.ROBOT_NETWORK_LOCK:
            self.assertEqual(server.ROBOT_NETWORK_STATE["completed_auto_at"], 100)

        # 동일 완료 ACK가 재전송되어도 기준 시각은 뒤로 밀리지 않는다.
        with (
            patch.object(
                server,
                "robot_command_snapshot",
                return_value={**command, "source": "AUTO"},
            ),
            patch.object(server, "complete_auto_task", return_value=None),
            patch.object(server, "touch_device", side_effect=no_side_effect),
            patch.object(server, "record_event", side_effect=no_side_effect),
            patch.object(server, "now", return_value=120),
        ):
            server.report_robot_network(query, "192.0.2.10")
        with server.ROBOT_NETWORK_LOCK:
            self.assertEqual(server.ROBOT_NETWORK_STATE["completed_auto_at"], 100)

        # 수동 TASK 완료는 기존 one-shot 처리만 사용하고 AUTO barrier를 만들지 않는다.
        with server.ROBOT_NETWORK_LOCK:
            server.ROBOT_NETWORK_STATE.update(
                delivered_revision=43,
                completed_auto_revision=None,
                completed_auto_at=None,
                completed_auto_zone=None,
                completed_auto_action=None,
            )
        with server.MANUAL_CONTROL_LOCK:
            server.MANUAL_CONTROL.update(
                enabled=True, revision=43, command="TASK",
                target_zone="ZONE2", action="HUMIDIFY", updated_at=90,
            )
        manual_command = {**command, "revision": 43}
        with (
            patch.object(
                server,
                "robot_command_snapshot",
                return_value={**manual_command, "source": "MANUAL"},
            ),
            patch.object(server, "touch_device", side_effect=no_side_effect),
            patch.object(server, "record_event", side_effect=no_side_effect),
            patch.object(server, "persist_manual_transition", side_effect=fake_manual_transition),
            patch.object(server, "now", return_value=130),
        ):
            server.report_robot_network(
                {**query, "ack_revision": ["43"]}, "192.0.2.10"
            )
        with server.ROBOT_NETWORK_LOCK:
            self.assertIsNone(server.ROBOT_NETWORK_STATE["completed_auto_revision"])

    def test_dashboard_distinguishes_waiting_for_a_fresh_reading(self) -> None:
        command = {
            "revision": 7,
            "command": "TASK",
            "target_zone": "ZONE2",
            "action": "HUMIDIFY",
            "source": "AUTO",
        }
        robot = {
            "ack_revision": 7,
            "ack_result": "COMPLETED",
            "completed_auto_revision": 7,
            "delivered_revision": 7,
            "online": True,
        }
        delivery = server.command_delivery_snapshot(command, robot)
        self.assertEqual(delivery["state"], "WAITING_READING")
        self.assertIn("새 구역 측정 대기", delivery["message"])

        # 완료 후 effective command는 새 ALL_STOP hold revision이므로, DB에서
        # 계산된 waiting count로도 동일 상태가 보여야 한다.
        hold = server.effective_command_snapshot(
            {
                "revision": 8,
                "command": "ALL_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "updated_at": 101,
                "waiting_for_new_reading_count": 1,
            },
            {
                "enabled": False,
                "revision": 1_000,
                "command": "MOTOR_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "updated_at": 90,
            },
        )
        hold_delivery = server.command_delivery_snapshot(
            hold,
            {
                "ack_revision": 7,
                "ack_result": "COMPLETED",
                "delivered_revision": 7,
                "online": True,
            },
        )
        self.assertEqual(hold_delivery["state"], "WAITING_READING")
        self.assertIn("자동 안전 정지", hold_delivery["message"])
        self.assertIn("1개 구역", hold_delivery["message"])

    def test_health_endpoint_exposes_build_without_database_access(self) -> None:
        handler = object.__new__(server.Handler)
        handler.path = "/api/health"
        captured: dict = {}

        def capture(status, payload) -> None:
            captured.update(status=status, payload=payload)

        handler.send_json = capture
        with patch.object(server, "now", return_value=1_700_000_000):
            handler.do_GET()
        self.assertEqual(captured["status"], server.HTTPStatus.OK)
        self.assertTrue(captured["payload"]["ok"])
        self.assertEqual(captured["payload"]["build"], server.SERVER_BUILD_ID)
        self.assertIn("fresh-reading-burst-gate", captured["payload"]["features"])
        self.assertIn(
            "persistent-reading-id-watermarks",
            captured["payload"]["features"],
        )
        self.assertIn("global-command-revisions", captured["payload"]["features"])
        self.assertIn("mobile-control-app", captured["payload"]["features"])
        self.assertIn("http-auth-rejection-close", captured["payload"]["features"])

    def test_readiness_endpoint_reports_database_state(self) -> None:
        handler = object.__new__(server.Handler)
        handler.path = "/api/ready"
        captured: dict = {}
        handler.send_json = lambda status, payload: captured.update(
            status=status,
            payload=payload,
        )

        with patch.object(
            server,
            "database_readiness",
            return_value=(False, "OperationalError"),
        ):
            handler.do_GET()

        self.assertEqual(
            captured["status"],
            server.HTTPStatus.SERVICE_UNAVAILABLE,
        )
        self.assertFalse(captured["payload"]["ok"])
        self.assertEqual(captured["payload"]["database"], "unavailable")

    def test_database_failures_return_sanitized_service_unavailable_json(self) -> None:
        failure = server.pymysql.OperationalError(1045, "private database detail")

        get_handler = object.__new__(server.Handler)
        get_handler.path = "/api/dashboard"
        get_response: dict = {}
        get_handler.send_json = lambda status, payload: get_response.update(
            status=status,
            payload=payload,
        )
        with patch.object(server, "dashboard", side_effect=failure):
            get_handler.do_GET()

        self.assertEqual(
            get_response["status"],
            server.HTTPStatus.SERVICE_UNAVAILABLE,
        )
        self.assertTrue(get_response["payload"]["retryable"])
        self.assertNotIn("private", str(get_response["payload"]))

        body = b'{"mode":"MANUAL","command":"MOTOR_STOP"}'
        post_handler = object.__new__(server.Handler)
        post_handler.path = "/api/control"
        post_handler.client_address = ("127.0.0.1", 12345)
        post_handler.headers = {"Content-Length": str(len(body))}
        post_handler.rfile = io.BytesIO(body)
        post_response: dict = {}
        post_handler.send_json = lambda status, payload: post_response.update(
            status=status,
            payload=payload,
        )
        with patch.object(server, "update_manual_control", side_effect=failure):
            post_handler.do_POST()

        self.assertEqual(
            post_response["status"],
            server.HTTPStatus.SERVICE_UNAVAILABLE,
        )
        self.assertEqual(post_response["payload"]["error"], "database unavailable")
        self.assertNotIn("private", str(post_response["payload"]))

    def test_remote_control_requires_the_configured_bearer_token(self) -> None:
        with patch.object(server, "CONTROL_API_TOKEN", "team-secret"):
            self.assertTrue(
                server.control_request_authorized("127.0.0.1", None)
            )
            self.assertFalse(
                server.control_request_authorized("192.0.2.10", None)
            )
            self.assertFalse(
                server.control_request_authorized(
                    "192.0.2.10",
                    "Bearer wrong",
                )
            )
            self.assertTrue(
                server.control_request_authorized(
                    "192.0.2.10",
                    "Bearer team-secret",
                )
            )

    def test_rejected_remote_control_closes_connection_with_unread_body(self) -> None:
        """A rejected POST body must not corrupt the next HTTP/1.1 request."""
        body = b'{"mode":"MANUAL","command":"ALL_STOP"}'
        handler = object.__new__(server.Handler)
        handler.path = "/api/control"
        handler.client_address = ("192.0.2.10", 12345)
        handler.headers = {"Content-Length": str(len(body))}
        handler.rfile = io.BytesIO(body)
        handler.close_connection = False
        captured: dict = {}
        handler.send_json = lambda status, payload: captured.update(
            status=status,
            payload=payload,
        )

        with patch.object(server, "CONTROL_API_TOKEN", "team-secret"):
            handler.do_POST()

        self.assertEqual(captured["status"], server.HTTPStatus.FORBIDDEN)
        self.assertTrue(handler.close_connection)
        self.assertEqual(handler.rfile.tell(), 0)

    def test_json_response_advertises_connection_close(self) -> None:
        handler = object.__new__(server.Handler)
        handler.close_connection = True
        handler.wfile = io.BytesIO()
        headers: dict[str, str] = {}
        handler.send_response = lambda status: None
        handler.send_header = lambda name, value: headers.update({name: value})
        handler.end_headers = lambda: None

        handler.send_json(server.HTTPStatus.FORBIDDEN, {"error": "forbidden"})

        self.assertEqual(headers["Connection"], "close")

    def test_legacy_port_does_not_expose_dashboard_or_control_routes(self) -> None:
        handler = object.__new__(server.LegacyHandler)
        handler.path = "/api/dashboard"
        captured: dict = {}
        handler.send_json = lambda status, payload: captured.update(
            status=status,
            payload=payload,
        )

        handler.do_GET()

        self.assertEqual(captured["status"], server.HTTPStatus.NOT_FOUND)

    def test_humidity_hysteresis_uses_the_previous_violation_mode(self) -> None:
        self.assertEqual(server.HUMIDITY_HYSTERESIS_PERCENT, 2.0)

        self.assertEqual(server.classify_humidity(59.9)["action"], "HUMIDIFY")
        self.assertTrue(server.classify_humidity(60.0)["normal"])
        recovering_low = server.classify_humidity(61.9, "HUMIDIFY")
        self.assertFalse(recovering_low["normal"])
        self.assertEqual(recovering_low["action"], "HUMIDIFY")
        self.assertTrue(server.classify_humidity(62.0, "HUMIDIFY")["normal"])

        self.assertEqual(server.classify_humidity(80.1)["action"], "DEHUMIDIFY")
        self.assertTrue(server.classify_humidity(80.0)["normal"])
        recovering_high = server.classify_humidity(78.1, "DEHUMIDIFY")
        self.assertFalse(recovering_high["normal"])
        self.assertEqual(recovering_high["action"], "DEHUMIDIFY")
        self.assertTrue(server.classify_humidity(78.0, "DEHUMIDIFY")["normal"])

        # A jump across the whole normal band changes corrective direction.
        self.assertEqual(
            server.classify_humidity(81.0, "HUMIDIFY")["action"],
            "DEHUMIDIFY",
        )

        zones = [
            {
                "zone_id": "ZONE2",
                "temperature": 25,
                "humidity": 61.9,
                "updated_at": 1_000,
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 900,
            },
            {
                "zone_id": "ZONE99",
                "temperature": 25,
                "humidity": 70,
                "updated_at": 1_000,
                "violation_mode": None,
                "violation_started_at": None,
            },
        ]
        mission = {
            "id": 1,
            "revision": 4,
            "command": "RETURN_HOME",
            "target_zone": "HOME",
            "action": "NONE",
            "reason": "old",
            "updated_at": 1,
        }
        with patch.object(server, "now", return_value=1_000):
            recovering = server.recompute_mission(MissionCursor(zones, mission))
            zones[0] = {**zones[0], "humidity": 62.0}
            released = server.recompute_mission(MissionCursor(zones, mission))
        self.assertEqual(recovering["command"], "TASK")
        self.assertEqual(recovering["action"], "HUMIDIFY")
        self.assertEqual(released["command"], "RETURN_HOME")

    def test_record_reading_preserves_only_a_fresh_previous_violation(self) -> None:
        def record_with_prior(prior: dict) -> tuple[dict, tuple]:
            updates: list[tuple] = []

            class Cursor:
                last_sql = ""
                lastrowid = 321

                def __enter__(self):
                    return self

                def __exit__(self, *_args):
                    return False

                def execute(self, sql, params=None):
                    self.last_sql = " ".join(sql.split()).lower()
                    if self.last_sql.startswith("update zone_status"):
                        updates.append(tuple(params))

                def fetchone(self):
                    if "select violation_mode" in self.last_sql:
                        return dict(prior)
                    raise AssertionError(self.last_sql)

            class Connection:
                def __init__(self):
                    self.cursor_object = Cursor()

                def cursor(self):
                    return self.cursor_object

                def commit(self):
                    pass

                def rollback(self):
                    pass

                def close(self):
                    pass

            with (
                patch.object(server, "connect", return_value=Connection()),
                patch.object(server, "now", return_value=1_000),
                patch.object(server, "touch_device_with_cursor"),
                patch.object(server, "recompute_mission", return_value={"command": "TASK"}),
            ):
                result = server.record_reading(
                    {"zone_id": "ZONE2", "temperature": 25, "humidity": 61.0}
                )
            self.assertEqual(len(updates), 1)
            return result, updates[0]

        fresh_result, fresh_update = record_with_prior(
            {
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 900,
                "updated_at": 980,
            }
        )
        self.assertEqual(fresh_result["action"], "HUMIDIFY")
        self.assertEqual(fresh_update[3], 321)
        self.assertEqual(fresh_update[4:6], ("HUMIDIFY", 900))

        stale_result, stale_update = record_with_prior(
            {
                "violation_mode": "HUMIDIFY",
                "violation_started_at": 100,
                "updated_at": 900,
            }
        )
        self.assertEqual(stale_result["action"], "NONE")
        self.assertEqual(stale_update[3], 321)
        self.assertEqual(stale_update[4:6], (None, None))

    def test_stale_zones_stop_the_robot_only_when_no_fresh_violation_exists(self) -> None:
        self.assertEqual(server.STALE_AFTER_SECONDS, 45)
        base_mission = {
            "id": 1,
            "revision": 10,
            "command": "RETURN_HOME",
            "target_zone": "HOME",
            "action": "NONE",
            "reason": "old",
            "updated_at": 1,
        }

        fresh_normal = {
            "zone_id": "ZONE2",
            "temperature": 25,
            "humidity": 70,
            "updated_at": 1_000,
            "violation_mode": None,
            "violation_started_at": None,
        }
        second_fresh_normal = {
            **fresh_normal,
            "zone_id": "ZONE99",
        }
        stale_normal = {
            **second_fresh_normal,
            "updated_at": 954,  # 46 seconds old: just beyond the default.
        }
        boundary_fresh = {
            **second_fresh_normal,
            "updated_at": 955,  # Exactly 45 seconds old remains fresh.
        }
        fresh_low = {
            **fresh_normal,
            "humidity": 50,
            "violation_mode": "HUMIDIFY",
            "violation_started_at": 900,
        }
        no_humidity = {
            **second_fresh_normal,
            "humidity": None,
        }

        with patch.object(server, "now", return_value=1_000):
            all_fresh = server.recompute_mission(
                MissionCursor([fresh_normal, second_fresh_normal], base_mission)
            )
            boundary = server.recompute_mission(
                MissionCursor([fresh_normal, boundary_fresh], base_mission)
            )
            mixed_stale = server.recompute_mission(
                MissionCursor([fresh_normal, stale_normal], base_mission)
            )
            all_stale = server.recompute_mission(
                MissionCursor(
                    [
                        {**fresh_normal, "updated_at": 954},
                        stale_normal,
                    ],
                    base_mission,
                )
            )
            missing_zone = server.recompute_mission(
                MissionCursor([fresh_normal], base_mission)
            )
            missing_value = server.recompute_mission(
                MissionCursor([fresh_normal, no_humidity], base_mission)
            )
            candidate_wins = server.recompute_mission(
                MissionCursor([fresh_low, stale_normal], base_mission)
            )

        self.assertEqual(all_fresh["command"], "RETURN_HOME")
        self.assertEqual(boundary["command"], "RETURN_HOME")
        for mission in (mixed_stale, all_stale, missing_zone, missing_value):
            self.assertEqual(mission["command"], "ALL_STOP")
            self.assertEqual(mission["target_zone"], "HOME")
            self.assertEqual(mission["action"], "NONE")
            self.assertEqual(mission["system_status"], "SENSOR_ERROR")
        self.assertEqual(candidate_wins["command"], "TASK")
        self.assertEqual(candidate_wins["target_zone"], "ZONE2")

    def test_threshold_memory_changes_only_after_database_commit(self) -> None:
        commits: list[float] = []

        class Cursor(MissionCursor):
            def __init__(self):
                super().__init__(
                    [],
                    {
                        "id": 1,
                        "revision": 1,
                        "command": "RETURN_HOME",
                        "target_zone": "HOME",
                        "action": "NONE",
                        "reason": "No fresh zone readings are available",
                        "updated_at": 1,
                    },
                )

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        class Connection:
            def __init__(self):
                self.cursor_object = Cursor()

            def cursor(self):
                return self.cursor_object

            def commit(self):
                commits.append(server.THRESHOLDS["humidity_min"])

            def rollback(self):
                pass

            def close(self):
                pass

        payload = {
            "temperature_min": 17,
            "temperature_max": 29,
            "humidity_min": 55,
            "humidity_max": 75,
        }
        connection = Connection()
        with (
            patch.object(server, "connect", return_value=connection),
            patch.object(server, "now", return_value=1000),
        ):
            result = server.update_thresholds(payload)
        self.assertEqual(commits, [server.DEFAULT_THRESHOLDS["humidity_min"]])
        self.assertEqual(result["humidity_min"], 55)
        statements = [sql.lower() for sql, _ in connection.cursor_object.calls]
        self.assertTrue(any("insert into system_settings" in sql for sql in statements))
        self.assertTrue(any("update mission" in sql for sql in statements))
        self.assertTrue(any("insert into event_log" in sql for sql in statements))

    def test_threshold_transaction_rolls_back_before_memory_publish(self) -> None:
        commits = 0
        rollbacks = 0

        class Cursor(MissionCursor):
            def __init__(self):
                super().__init__(
                    [],
                    {
                        "id": 1,
                        "revision": 1,
                        "command": "RETURN_HOME",
                        "target_zone": "HOME",
                        "action": "NONE",
                        "reason": "No fresh zone readings are available",
                        "updated_at": 1,
                    },
                )

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def execute(self, sql: str, params=None) -> None:
                super().execute(sql, params)
                if "insert into event_log" in self.last_sql:
                    raise server.pymysql.OperationalError(2006, "audit insert failed")

        class Connection:
            def __init__(self):
                self.cursor_object = Cursor()

            def cursor(self):
                return self.cursor_object

            def commit(self):
                nonlocal commits
                commits += 1

            def rollback(self):
                nonlocal rollbacks
                rollbacks += 1

            def close(self):
                pass

        payload = {
            "temperature_min": 17,
            "temperature_max": 29,
            "humidity_min": 55,
            "humidity_max": 75,
        }
        with (
            patch.object(server, "connect", return_value=Connection()),
            patch.object(server, "now", return_value=1000),
        ):
            with self.assertRaises(server.pymysql.OperationalError):
                server.update_thresholds(payload)

        self.assertEqual(commits, 0)
        self.assertEqual(rollbacks, 1)
        self.assertEqual(
            server.current_thresholds()["humidity_min"],
            server.DEFAULT_THRESHOLDS["humidity_min"],
        )

    def test_manual_control_schema_validation_reports_missing_columns(self) -> None:
        class Cursor:
            def __init__(self, columns):
                self.columns = columns

            def execute(self, _sql, _params):
                pass

            def fetchall(self):
                return [{"column_name": column} for column in self.columns]

        server.validate_manual_control_schema(Cursor(server.MANUAL_CONTROL_COLUMNS))
        with self.assertRaisesRegex(RuntimeError, "updated_at"):
            server.validate_manual_control_schema(
                Cursor(server.MANUAL_CONTROL_COLUMNS - {"updated_at"})
            )

    def test_zone_watermark_migration_is_idempotent_and_consumes_legacy_rows_once(self) -> None:
        class Cursor:
            def __init__(self):
                self.columns = {
                    "zone_id", "temperature", "humidity", "updated_at",
                    "violation_mode", "violation_started_at",
                }
                self.last_sql = ""
                self.calls: list[str] = []
                self.applied_migrations: set[str] = set()

            def execute(self, sql, _params=None):
                normalized = " ".join(sql.split()).lower()
                self.last_sql = normalized
                self.calls.append(normalized)
                if "add column latest_reading_id" in normalized:
                    self.columns.add("latest_reading_id")
                if "add column acted_through_reading_id" in normalized:
                    self.columns.add("acted_through_reading_id")
                if normalized.startswith("insert into schema_migrations"):
                    self.applied_migrations.add(_params[0])

            def fetchall(self):
                if "from information_schema.columns" in self.last_sql:
                    return [{"column_name": column} for column in self.columns]
                raise AssertionError(self.last_sql)

            def fetchone(self):
                if "from schema_migrations" in self.last_sql:
                    return (
                        {"migration_key": server.READING_WATERMARK_MIGRATION}
                        if server.READING_WATERMARK_MIGRATION
                        in self.applied_migrations
                        else None
                    )
                raise AssertionError(self.last_sql)

        cursor = Cursor()
        server.migrate_zone_status_reading_watermarks(cursor)
        server.migrate_zone_status_reading_watermarks(cursor)

        alter_calls = [sql for sql in cursor.calls if sql.startswith("alter table")]
        consume_calls = [
            sql for sql in cursor.calls
            if sql.startswith("update zone_status set acted_through_reading_id")
        ]
        latest_backfills = [
            sql for sql in cursor.calls
            if sql.startswith("update zone_status as zs")
        ]
        self.assertEqual(len(alter_calls), 2)
        self.assertEqual(len(consume_calls), 1)
        self.assertEqual(len(latest_backfills), 1)
        self.assertTrue(
            server.ZONE_STATUS_WATERMARK_COLUMNS.issubset(cursor.columns)
        )

    def test_static_dashboard_and_logic_pages_are_cross_linked(self) -> None:
        dashboard_html = server.DASHBOARD_PATH.read_text(encoding="utf-8")
        logic_html = server.SYSTEM_LOGIC_PATH.read_text(encoding="utf-8")

        self.assertIn('href="/logic"', dashboard_html)
        self.assertIn('href="/"', logic_html)
        self.assertIn("검은 선", logic_html)
        self.assertIn("RFID", logic_html)
        self.assertIn("습도", logic_html)
        self.assertIn('data-command="CALIBRATE_HOME"', dashboard_html)
        self.assertIn("재부팅 후 필수", dashboard_html)
        self.assertIn("CALIBRATE_HOME", logic_html)
        self.assertIn("AUTO 전환이나 구역 TASK", logic_html)

    def test_app_route_serves_the_phone_page_without_touching_the_database(self) -> None:
        """/app은 정적 파일만 읽는다. DB가 죽어도 화면 자체는 떠야 한다."""
        served: dict = {}

        for path in ("/app", "/app/"):
            with self.subTest(path=path):
                handler = object.__new__(server.Handler)
                handler.path = path
                handler.wfile = io.BytesIO()
                handler.send_response = lambda status: served.update(status=status)
                handler.send_header = lambda *args: None
                handler.end_headers = lambda: None

                with patch.object(server, "connect", side_effect=AssertionError("no DB")):
                    handler.do_GET()

                self.assertEqual(served["status"], server.HTTPStatus.OK)
                body = handler.wfile.getvalue().decode("utf-8")
                self.assertIn("<title>구르미</title>", body)
                self.assertIn("/api/dashboard", body)

    def test_phone_app_page_reuses_existing_apis_and_links_back(self) -> None:
        app_html = server.APP_PATH.read_text(encoding="utf-8")
        dashboard_html = server.DASHBOARD_PATH.read_text(encoding="utf-8")

        # 새 화면은 별도 API를 만들지 않고 기존 계약만 사용한다.
        self.assertIn("'/api/dashboard'", app_html)
        self.assertIn("'/api/control'", app_html)
        self.assertIn("'/api/settings'", app_html)
        self.assertIn("humidity-rover-control-token", app_html)

        # 수동 명령은 서버가 허용하는 이름만 보낸다.
        for command in ("TASK", "MOTOR_RETURN", "ALL_STOP", "CALIBRATE_HOME"):
            self.assertIn(command, app_html)

        # 관제 대시보드는 유지하고 서로 오갈 수 있어야 한다.
        self.assertIn('href="/"', app_html)
        self.assertIn('href="/logic"', app_html)
        self.assertIn('href="/app"', dashboard_html)

    def test_legacy_port_does_not_expose_the_phone_app_page(self) -> None:
        handler = object.__new__(server.LegacyHandler)
        handler.path = "/app"
        captured: dict = {}
        handler.send_json = lambda status, payload: captured.update(
            status=status,
            payload=payload,
        )

        handler.do_GET()

        self.assertEqual(captured["status"], server.HTTPStatus.NOT_FOUND)


if __name__ == "__main__":
    unittest.main(verbosity=2)
