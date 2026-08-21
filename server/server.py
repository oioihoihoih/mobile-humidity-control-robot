#!/usr/bin/env python3
"""Local controller for the mobile humidity robot MVP.

Run on the PC/notebook that is on the same Wi-Fi network as all ESP boards.
It accepts readings from the configured active zones, persists them to local MySQL,
decides the highest-priority violation, and exposes the latest robot command.
"""

from __future__ import annotations

import hmac
import ipaddress
import json
import os
import threading
import time
from collections import deque
from decimal import Decimal
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import pymysql
from pymysql.cursors import DictCursor

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # 서버는 pyserial 없이도 네트워크 관제를 계속 제공한다.
    serial = None
    list_ports = None

HOST = os.getenv("ROBOT_BIND_HOST", "127.0.0.1")
PORT = int(os.getenv("ROBOT_PORT", "8000"))
SERVER_BUILD_ID = "2026-08-21-repository-hardening-v1"
AVR_REVISION_MAX = 2_147_483_647
# 기존 ESP-01 매뉴얼과 호환하기 위한 보조 포트입니다.
# 새 코드는 8000/api/readings, 기존 센서 코드는 3000/api/humidity를 씁니다.
LEGACY_PORT = int(os.getenv("LEGACY_PORT", "3000"))
LEGACY_ENABLED = os.getenv("LEGACY_ENABLED", "0").strip().lower() in {
    "1", "true", "yes", "on"
}
CONTROL_API_TOKEN = os.getenv("CONTROL_API_TOKEN", "")
MAX_REQUEST_BODY_BYTES = int(os.getenv("MAX_REQUEST_BODY_BYTES", "16384"))
REQUEST_TIMEOUT_SECONDS = float(os.getenv("REQUEST_TIMEOUT_SECONDS", "10"))
REMOTE_CONTROL_PATHS = frozenset(
    {
        "/api/control",
        "/api/settings",
        "/api/serial/reconnect",
        "/api/serial/test-rfid",
    }
)
DEFAULT_THRESHOLDS = {
    "temperature_min": float(os.getenv("TEMPERATURE_MIN", "18.0")),
    "temperature_max": float(os.getenv("TEMPERATURE_MAX", "28.0")),
    "humidity_min": float(os.getenv("LOW_HUMIDITY", "60.0")),
    "humidity_max": float(os.getenv("HIGH_HUMIDITY", "80.0")),
}
THRESHOLDS = dict(DEFAULT_THRESHOLDS)
SETTINGS_LOCK = threading.Lock()
HUMIDITY_HYSTERESIS_PERCENT = float(
    os.getenv("HUMIDITY_HYSTERESIS_PERCENT", "2.0")
)
STALE_AFTER_SECONDS = int(os.getenv("STALE_AFTER_SECONDS", "45"))
# 현재 시연에서 임무 판단과 대시보드에 사용하는 고정 구역입니다.
# 과거 테스트로 DB에 남은 ZONE1 등은 이 목록에 없으면 자동 임무에서 제외합니다.
DEFAULT_ZONE_IDS = ("ZONE2", "ZONE99")
ACTIVE_ZONE_IDS = frozenset(DEFAULT_ZONE_IDS)
# MySQL은 전용 최소권한 계정으로 실행한다. DB 생성은 기본적으로 관리자
# 설정 단계에서 수행하며, 개발용 자동 생성은 명시적으로 opt-in한다.
MYSQL_HOST = os.getenv("MYSQL_HOST", "127.0.0.1")
MYSQL_PORT = int(os.getenv("MYSQL_PORT", "3306"))
MYSQL_USER = os.getenv("MYSQL_USER", "humibot")
MYSQL_PASSWORD = os.getenv("MYSQL_PASSWORD", "")
MYSQL_DATABASE = os.getenv("MYSQL_DATABASE", "mobile_humidity_robot")
MYSQL_AUTO_CREATE_DATABASE = os.getenv(
    "MYSQL_AUTO_CREATE_DATABASE", "0"
).strip().lower() in {"1", "true", "yes", "on"}
DB_LOCK = threading.Lock()
DASHBOARD_PATH = Path(__file__).with_name("dashboard.html")
SYSTEM_LOGIC_PATH = Path(__file__).with_name("system_logic.html")
# /app은 같은 /api/dashboard 데이터를 쓰는 휴대폰용 간이 화면이다. 관제
# 대시보드(/)는 그대로 두고 별도 경로로만 제공한다.
APP_PATH = Path(__file__).with_name("dashboard_app.html")
SERIAL_PORT = os.getenv("SERIAL_PORT", "").strip()
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "9600"))
SERIAL_ENABLED = os.getenv("SERIAL_ENABLED", "0").strip().lower() in {
    "1", "true", "yes", "on"
}
GATEWAY_HEARTBEAT_STALE_SECONDS = int(
    os.getenv("GATEWAY_HEARTBEAT_STALE_SECONDS", "30")
)
SERIAL_LINE_COALESCE_SECONDS = int(
    os.getenv("SERIAL_LINE_COALESCE_SECONDS", "30")
)
SERIAL_LINES: deque[dict[str, Any]] = deque(maxlen=240)
SERIAL_LOCK = threading.Lock()
SERIAL_STATE_DEFAULTS: dict[str, Any] = {
    "enabled": SERIAL_ENABLED,
    "configured_port": SERIAL_PORT,
    "baud": SERIAL_BAUD,
    "connected": False,
    "active_port": None,
    "error": None,
    "last_line_at": None,
    "sequence": 0,
    "robot_phase": "UNKNOWN",
    # connected는 COM 포트가 열렸다는 뜻일 뿐이다. ESP/Wi-Fi/서버 경로의
    # 정상 여부는 아래 필드와 heartbeat로 별도 판정한다.
    "esp_ready": False,
    "wifi_ready": False,
    "server_reachable": False,
    "last_esp_response_at": None,
    "last_wifi_ok_at": None,
    "last_gateway_heartbeat_at": None,
    "last_network_error_at": None,
    "gateway_error": None,
    "gateway_ip": None,
}
SERIAL_STATE: dict[str, Any] = dict(SERIAL_STATE_DEFAULTS)
ROBOT_NETWORK_LOCK = threading.Lock()
ROBOT_NETWORK_STATE: dict[str, Any] = {
    "phase": "UNKNOWN",
    "event": "WAITING",
    "zone": "HOME",
    "action": "NONE",
    "reported_at": None,
    "last_seen": None,
    "ip": None,
    # delivered_*는 서버가 명령 응답을 만든 시점, ack_*는 자동차가 실제로
    # 해당 revision을 처리했다고 다시 보고한 시점이다.
    "delivered_revision": None,
    "delivered_at": None,
    "ack_revision": None,
    "ack_result": None,
    "acknowledged_at": None,
    # 최근 자동 모듈 완료를 화면에 설명하기 위한 런타임 정보다. 안전 gate는
    # 이 RAM 값이 아니라 zone_status의 reading-id watermark에 영구 저장된다.
    # 아래 필드는 wire JSON에는 노출하지 않는다.
    "completed_auto_revision": None,
    "completed_auto_at": None,
    "completed_auto_zone": None,
    "completed_auto_action": None,
}

# 웹 대시보드에서 시험용으로 내리는 수동 명령이다. 자동 모드에서는
# 기존 습도 우선순위 임무를 그대로 사용하고, 수동 모드에서만 이를 덮어쓴다.
MANUAL_CONTROL_LOCK = threading.Lock()
MANUAL_CONTROL: dict[str, Any] = {
    "enabled": False,
    "revision": 1_000_000,
    "command": "MOTOR_STOP",
    "target_zone": "HOME",
    "action": "NONE",
    "updated_at": None,
}
MANUAL_COMMANDS = {
    "TASK",
    "CALIBRATE_HOME",
    "MOTOR_FWD",
    "MOTOR_RETURN",
    "MOTOR_STOP",
    "ALL_STOP",
    "ACT_HUMIDIFY",
    "ACT_DEHUMID",
    "ACT_STOP",
    "RFID_TEST",
    "I2C_CHECK",
}

# SensorUno가 실제로 보고하는 결과만 수용한다. 임의 문자열을 성공 ACK로
# 취급하면 INVALID_ACTION 같은 실행 실패가 UI에서 성공으로 보일 수 있다.
ACK_RESULT_STATES = {
    "ACK": "ACK_EXECUTING",  # result가 없던 이전 펌웨어와의 호환용
    "EXECUTING": "ACK_EXECUTING",
    "COMPLETED": "ACK_COMPLETED",
    "FAILED": "ACK_FAILED",
    "I2C_ERROR": "ACK_FAILED",
    "INVALID_ACTION": "ACK_FAILED",
    "ACT_START_ERROR": "ACK_FAILED",
    "IGNORED": "ACK_FAILED",
}

MANUAL_CONTROL_COLUMNS = {
    "id",
    "enabled",
    "revision",
    "command",
    "target_zone",
    "action",
    "updated_at",
}
ZONE_STATUS_WATERMARK_COLUMNS = {
    "latest_reading_id",
    "acted_through_reading_id",
}
READING_WATERMARK_MIGRATION = "2026-08-20-zone-reading-watermarks-v1"

if THRESHOLDS["temperature_min"] >= THRESHOLDS["temperature_max"]:
    raise RuntimeError("TEMPERATURE_MIN must be lower than TEMPERATURE_MAX")
if THRESHOLDS["humidity_min"] >= THRESHOLDS["humidity_max"]:
    raise RuntimeError("LOW_HUMIDITY must be lower than HIGH_HUMIDITY")
if HUMIDITY_HYSTERESIS_PERCENT < 0:
    raise RuntimeError("HUMIDITY_HYSTERESIS_PERCENT must not be negative")
if STALE_AFTER_SECONDS <= 0:
    raise RuntimeError("STALE_AFTER_SECONDS must be positive")
if not 1 <= PORT <= 65535 or not 1 <= LEGACY_PORT <= 65535:
    raise RuntimeError("ROBOT_PORT and LEGACY_PORT must be valid TCP ports")
if MAX_REQUEST_BODY_BYTES <= 0:
    raise RuntimeError("MAX_REQUEST_BODY_BYTES must be positive")
if REQUEST_TIMEOUT_SECONDS <= 0:
    raise RuntimeError("REQUEST_TIMEOUT_SECONDS must be positive")


def now() -> int:
    return int(time.time())


def json_default(value: Any) -> float:
    """MySQL DECIMAL 값을 API에서 일반 숫자로 반환한다."""
    if isinstance(value, Decimal):
        return float(value)
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


def is_loopback_address(address: str) -> bool:
    try:
        return ipaddress.ip_address(address).is_loopback
    except ValueError:
        return False


def control_request_authorized(client_ip: str, authorization: str | None) -> bool:
    """Keep dangerous UI controls local unless a shared token is configured."""
    if is_loopback_address(client_ip):
        return True
    if not CONTROL_API_TOKEN or not authorization:
        return False
    scheme, separator, supplied = authorization.partition(" ")
    return (
        bool(separator)
        and scheme.lower() == "bearer"
        and hmac.compare_digest(supplied, CONTROL_API_TOKEN)
    )


def database_readiness() -> tuple[bool, str | None]:
    """Check that the configured database accepts a trivial query."""
    connection = None
    try:
        connection = connect()
        with connection.cursor() as cursor:
            cursor.execute("SELECT 1")
            cursor.fetchone()
        return True, None
    except Exception as error:  # readiness must return JSON instead of crashing
        return False, type(error).__name__
    finally:
        if connection is not None:
            connection.close()


def connect(include_database: bool = True) -> pymysql.Connection:
    """MySQL에 연결한다. DB 생성 전에는 include_database=False를 사용한다."""
    settings: dict[str, Any] = {
        "host": MYSQL_HOST,
        "port": MYSQL_PORT,
        "user": MYSQL_USER,
        "password": MYSQL_PASSWORD,
        "charset": "utf8mb4",
        "cursorclass": DictCursor,
        "autocommit": False,
    }
    if include_database:
        settings["database"] = MYSQL_DATABASE
    return pymysql.connect(**settings)


def database_identifier() -> str:
    """환경 변수로 받은 DB 이름을 SQL 식별자로 안전하게 제한한다."""
    if not MYSQL_DATABASE.replace("_", "").isalnum():
        raise RuntimeError("MYSQL_DATABASE must contain only letters, numbers, and underscores")
    return f"`{MYSQL_DATABASE}`"


def validate_manual_control_schema(cursor: Any) -> None:
    """기존 DB의 수동 제어 테이블이 현재 서버와 호환되는지 확인한다.

    CREATE TABLE IF NOT EXISTS만으로는 과거에 일부 컬럼만 만들어진 테이블을
    고칠 수 없다. 자동 ALTER 대신 명확한 오류로 중단해 잘못된 안전 상태로
    서버가 뜨는 일을 막는다.
    """
    cursor.execute(
        """
        SELECT COLUMN_NAME AS column_name
        FROM information_schema.columns
        WHERE table_schema = %s AND table_name = 'manual_control'
        """,
        (MYSQL_DATABASE,),
    )
    columns = {str(row["column_name"]).lower() for row in cursor.fetchall()}
    missing = sorted(MANUAL_CONTROL_COLUMNS - columns)
    if missing:
        raise RuntimeError(
            "manual_control schema is incompatible; missing columns: "
            + ", ".join(missing)
        )


def migrate_zone_status_reading_watermarks(cursor: Any) -> None:
    """기존 DB에 측정 소비 워터마크를 중복 실행 가능하게 추가한다.

    ``updated_at``은 초 단위라 측정과 임무 완료가 같은 초에 일어나면 어느
    쪽이 나중인지 판별할 수 없다. AUTO 가동권은 단조 증가하는
    ``reading_log.id``로 판별한다. 기존 설치를 처음 마이그레이션할 때는
    보수적으로 현재 측정을 모두 소비 처리해, 배포/재시작 직후 과거 TASK가
    재실행되지 않게 한다. 다음 센서 측정부터 자동 판단이 다시 열린다.
    """
    cursor.execute(
        """
        SELECT COLUMN_NAME AS column_name
        FROM information_schema.columns
        WHERE table_schema = %s AND table_name = 'zone_status'
        """,
        (MYSQL_DATABASE,),
    )
    columns = {str(row["column_name"]).lower() for row in cursor.fetchall()}
    add_latest = "latest_reading_id" not in columns
    add_acted = "acted_through_reading_id" not in columns

    if add_latest:
        cursor.execute(
            "ALTER TABLE zone_status "
            "ADD COLUMN latest_reading_id BIGINT UNSIGNED NULL AFTER updated_at"
        )
    if add_acted:
        cursor.execute(
            "ALTER TABLE zone_status "
            "ADD COLUMN acted_through_reading_id BIGINT UNSIGNED NOT NULL "
            "DEFAULT 0 AFTER latest_reading_id"
        )

    cursor.execute(
        """
        CREATE TABLE IF NOT EXISTS schema_migrations (
            migration_key VARCHAR(96) NOT NULL PRIMARY KEY,
            applied_at BIGINT NOT NULL
        ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
        """
    )
    cursor.execute(
        "SELECT migration_key FROM schema_migrations WHERE migration_key = %s",
        (READING_WATERMARK_MIGRATION,),
    )
    already_applied = cursor.fetchone() is not None
    if not already_applied:
        # 중간 버전에서 latest 컬럼만 생긴 DB도 복구할 수 있도록 NULL만 채운다.
        cursor.execute(
            """
            UPDATE zone_status AS zs
            LEFT JOIN (
                SELECT zone_id, MAX(id) AS latest_id
                FROM reading_log GROUP BY zone_id
            ) AS rl ON rl.zone_id = zs.zone_id
            SET zs.latest_reading_id = rl.latest_id
            WHERE zs.latest_reading_id IS NULL AND rl.latest_id IS NOT NULL
            """
        )
        cursor.execute(
            """
            UPDATE zone_status
            SET acted_through_reading_id = GREATEST(
                acted_through_reading_id, COALESCE(latest_reading_id, 0)
            )
            """
        )
        cursor.execute(
            """
            INSERT INTO schema_migrations(migration_key, applied_at)
            VALUES (%s, %s)
            """,
            (READING_WATERMARK_MIGRATION, now()),
        )


def initialise_database() -> None:
    if MYSQL_AUTO_CREATE_DATABASE:
        bootstrap = connect(include_database=False)
        try:
            with bootstrap.cursor() as cursor:
                cursor.execute(
                    f"CREATE DATABASE IF NOT EXISTS {database_identifier()} "
                    "CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci"
                )
            bootstrap.commit()
        finally:
            bootstrap.close()

    connection = connect()
    try:
        with connection.cursor() as cursor:
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS reading_log (
                    id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
                    received_at BIGINT NOT NULL,
                    zone_id VARCHAR(16) NOT NULL,
                    temperature DECIMAL(5, 1) NOT NULL,
                    humidity DECIMAL(5, 1) NOT NULL,
                    INDEX idx_reading_log_zone_time (zone_id, received_at)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS zone_status (
                    zone_id VARCHAR(16) NOT NULL PRIMARY KEY,
                    temperature DECIMAL(5, 1) NULL,
                    humidity DECIMAL(5, 1) NULL,
                    updated_at BIGINT NULL,
                    latest_reading_id BIGINT UNSIGNED NULL,
                    acted_through_reading_id BIGINT UNSIGNED NOT NULL DEFAULT 0,
                    violation_mode VARCHAR(16) NULL,
                    violation_started_at BIGINT NULL
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            migrate_zone_status_reading_watermarks(cursor)
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS mission (
                    id TINYINT UNSIGNED NOT NULL PRIMARY KEY,
                    revision INT NOT NULL,
                    command VARCHAR(32) NOT NULL,
                    target_zone VARCHAR(16) NOT NULL,
                    action VARCHAR(16) NOT NULL,
                    reason TEXT NOT NULL,
                    updated_at BIGINT NOT NULL
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS manual_control (
                    id TINYINT UNSIGNED NOT NULL PRIMARY KEY,
                    enabled TINYINT(1) NOT NULL,
                    revision BIGINT UNSIGNED NOT NULL,
                    command VARCHAR(32) NOT NULL,
                    target_zone VARCHAR(16) NOT NULL,
                    action VARCHAR(16) NOT NULL,
                    updated_at BIGINT NULL
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            validate_manual_control_schema(cursor)
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS device_status (
                    device_id VARCHAR(32) NOT NULL PRIMARY KEY,
                    device_type VARCHAR(32) NOT NULL,
                    status VARCHAR(16) NOT NULL,
                    last_seen BIGINT NOT NULL,
                    details_json TEXT NOT NULL
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS event_log (
                    id BIGINT UNSIGNED NOT NULL AUTO_INCREMENT PRIMARY KEY,
                    received_at BIGINT NOT NULL,
                    source VARCHAR(32) NOT NULL,
                    event_type VARCHAR(32) NOT NULL,
                    message VARCHAR(255) NOT NULL,
                    data_json TEXT NOT NULL,
                    INDEX idx_event_log_time (received_at)
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS system_settings (
                    setting_key VARCHAR(32) NOT NULL PRIMARY KEY,
                    setting_value DECIMAL(7, 2) NOT NULL,
                    updated_at BIGINT NOT NULL
                ) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4
                """
            )
            # 초기 플레이스홀더 버전에서 이동 완료를 RFID 도착으로 잘못 분류한 기록을 정정한다.
            cursor.execute(
                """
                UPDATE event_log SET event_type = 'MOVEMENT'
                WHERE event_type = 'RFID_ARRIVAL'
                  AND message LIKE '[MOTOR PLACEHOLDER] ARRIVED NEAR%'
                """
            )
            for setting_key, setting_value in DEFAULT_THRESHOLDS.items():
                cursor.execute(
                    """
                    INSERT IGNORE INTO system_settings(setting_key, setting_value, updated_at)
                    VALUES (%s, %s, %s)
                    """,
                    (setting_key, setting_value, now()),
                )
            cursor.execute("SELECT setting_key, setting_value FROM system_settings")
            loaded_thresholds = {
                row["setting_key"]: float(row["setting_value"])
                for row in cursor.fetchall()
                if row["setting_key"] in DEFAULT_THRESHOLDS
            }
            with SETTINGS_LOCK:
                THRESHOLDS.update(loaded_thresholds)
            for zone_id in DEFAULT_ZONE_IDS:
                cursor.execute("INSERT IGNORE INTO zone_status(zone_id) VALUES (%s)", (zone_id,))
            cursor.execute(
                """
                INSERT IGNORE INTO mission
                  (id, revision, command, target_zone, action, reason, updated_at)
                VALUES (1, 0, 'RETURN_HOME', 'HOME', 'NONE', 'No readings yet', %s)
                """,
                (now(),),
            )
            # 최초 마이그레이션이나 빈 DB에서는 안전 정지를 latch한 상태로 시작한다.
            # 이후의 수동/자동 모드 상태와 revision은 이 행에 계속 영구 저장된다.
            seed_revision = next_command_revision(
                999_999,
                timestamp=now(),
            )
            cursor.execute(
                """
                INSERT IGNORE INTO manual_control
                  (id, enabled, revision, command, target_zone, action, updated_at)
                VALUES (1, 1, %s, 'ALL_STOP', 'HOME', 'NONE', %s)
                """,
                (seed_revision, now()),
            )
            cursor.execute("SELECT * FROM manual_control WHERE id = 1")
            persisted_manual = cursor.fetchone()
            if persisted_manual:
                if not 0 <= int(persisted_manual["revision"]) <= AVR_REVISION_MAX:
                    raise RuntimeError(
                        "persisted manual revision is outside the AVR signed long range"
                    )
                with MANUAL_CONTROL_LOCK:
                    MANUAL_CONTROL.update(
                        enabled=bool(persisted_manual["enabled"]),
                        revision=int(persisted_manual["revision"]),
                        command=persisted_manual["command"],
                        target_zone=persisted_manual["target_zone"],
                        action=persisted_manual["action"],
                        updated_at=persisted_manual["updated_at"],
                    )
            cursor.execute("SELECT revision FROM mission WHERE id = 1")
            persisted_mission = cursor.fetchone()
            if (
                not persisted_mission
                or not 0 <= int(persisted_mission["revision"]) <= AVR_REVISION_MAX
            ):
                raise RuntimeError(
                    "persisted AUTO mission revision is outside the AVR signed long range"
                )
        connection.commit()
    finally:
        connection.close()


def touch_device_with_cursor(
    cursor: Any,
    device_id: str,
    device_type: str,
    status: str = "ONLINE",
    details: dict[str, Any] | None = None,
) -> None:
    """센서·자동차·시리얼 장치의 마지막 접속 상태를 갱신한다."""
    cursor.execute(
        """
        INSERT INTO device_status(device_id, device_type, status, last_seen, details_json)
        VALUES (%s, %s, %s, %s, %s)
        ON DUPLICATE KEY UPDATE
          device_type=VALUES(device_type), status=VALUES(status),
          last_seen=VALUES(last_seen), details_json=VALUES(details_json)
        """,
        (
            device_id[:32],
            device_type[:32],
            status[:16],
            now(),
            json.dumps(details or {}, ensure_ascii=False, separators=(",", ":")),
        ),
    )


def touch_device(
    device_id: str,
    device_type: str,
    status: str = "ONLINE",
    details: dict[str, Any] | None = None,
) -> None:
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                touch_device_with_cursor(cursor, device_id, device_type, status, details)
            connection.commit()
        finally:
            connection.close()


def record_event_with_cursor(
    cursor: Any,
    source: str,
    event_type: str,
    message: str,
    data: dict[str, Any] | None = None,
    *,
    timestamp: int | None = None,
) -> None:
    """이미 열린 트랜잭션에 이벤트를 추가한다."""
    cursor.execute(
        """
        INSERT INTO event_log(received_at, source, event_type, message, data_json)
        VALUES (%s, %s, %s, %s, %s)
        """,
        (
            now() if timestamp is None else timestamp,
            source[:32],
            event_type[:32],
            message[:255],
            json.dumps(data or {}, ensure_ascii=False, separators=(",", ":")),
        ),
    )


def record_event(
    source: str,
    event_type: str,
    message: str,
    data: dict[str, Any] | None = None,
) -> None:
    """RFID·릴레이·연결 변화 등 중요한 이벤트를 DB에 보존한다."""
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                record_event_with_cursor(cursor, source, event_type, message, data)
            connection.commit()
        finally:
            connection.close()


def gateway_state_changes_for_serial_line(
    line: str, timestamp: int | None = None
) -> dict[str, Any]:
    """게이트웨이 로그 한 줄에서 확인 가능한 연결 상태만 추출한다.

    USB 포트가 열렸다는 사실과 ESP-01이 응답하는지, 공유기에 연결됐는지,
    실제 서버까지 heartbeat가 도달했는지는 서로 다른 상태다. 성공 근거가
    없는 상태를 추측해서 올리지 않고, 명시적인 성공/실패 로그만 반영한다.
    """
    observed_at = now() if timestamp is None else timestamp
    upper = line.strip().upper()
    changes: dict[str, Any] = {}

    if upper.startswith("[ESP RX]"):
        if "(NO RESPONSE)" in upper:
            changes.update(
                esp_ready=False,
                wifi_ready=False,
                server_reachable=False,
                last_network_error_at=observed_at,
                gateway_error="ESP-01 AT no response",
            )
        elif upper != "[ESP RX]" and "ERROR" not in upper and "FAIL" not in upper:
            changes.update(
                esp_ready=True,
                last_esp_response_at=observed_at,
            )

    if (
        "[GATEWAY] WI-FI CONNECTION START" in upper
        or "[GATEWAY] RECONNECTING WI-FI" in upper
    ):
        changes.update(wifi_ready=False, server_reachable=False)

    if "[GATEWAY] WIFI OK" in upper or "[BOOT] GATEWAY READY" in upper:
        changes.update(
            esp_ready=True,
            wifi_ready=True,
            server_reachable=False,
            last_esp_response_at=observed_at,
            last_wifi_ok_at=observed_at,
            gateway_error=None,
        )

    if (
        "[BOOT] WIFI ERROR" in upper
        or "AP CONNECTION FAILED" in upper
        or "[GATEWAY] NEXT CYCLE WILL RECONNECT" in upper
    ):
        changes.update(
            wifi_ready=False,
            server_reachable=False,
            last_network_error_at=observed_at,
            gateway_error=line.strip()[:160],
        )

    if "[GATEWAY] HEARTBEAT ERROR" in upper:
        changes.update(
            server_reachable=False,
            last_network_error_at=observed_at,
            gateway_error="Gateway heartbeat failed",
        )

    if "[GATEWAY] HEARTBEAT OK" in upper:
        changes.update(
            esp_ready=True,
            wifi_ready=True,
            server_reachable=True,
            last_esp_response_at=observed_at,
            last_wifi_ok_at=observed_at,
            last_gateway_heartbeat_at=observed_at,
            gateway_error=None,
        )
    return changes


def mark_gateway_heartbeat(
    client_ip: str | None = None, timestamp: int | None = None
) -> None:
    """ESP-01에서 HTTP heartbeat가 실제 도착한 사실을 기록한다."""
    observed_at = now() if timestamp is None else timestamp
    with SERIAL_LOCK:
        SERIAL_STATE.update(
            esp_ready=True,
            wifi_ready=True,
            server_reachable=True,
            last_esp_response_at=observed_at,
            last_wifi_ok_at=observed_at,
            last_gateway_heartbeat_at=observed_at,
            gateway_error=None,
            gateway_ip=client_ip,
        )


def serial_coalesce_key(line: str) -> str | None:
    """반복 AT 재시도 중 UI에서 한 항목으로 묶을 저정보량 로그를 고른다."""
    upper = line.strip().upper()
    if "[GATEWAY] AT READY CHECK" in upper:
        return "GATEWAY_AT_READY_CHECK"
    if upper == "[ESP TX] AT":
        return "ESP_TX_AT"
    if upper == "[ESP RX] (NO RESPONSE)":
        return "ESP_RX_NO_RESPONSE"
    if upper in {
        "[GATEWAY] WI-FI CONNECTION START",
        "[GATEWAY] RECONNECTING WI-FI",
    }:
        return upper
    return None


def serial_snapshot(timestamp: int | None = None) -> dict[str, Any]:
    current = now() if timestamp is None else timestamp
    with SERIAL_LOCK:
        state = dict(SERIAL_STATE)
        # 반복 로그 항목은 제자리에서 count를 올리므로 JSON 직렬화 중 변경되지
        # 않도록 각 dict도 복사한다.
        state["lines"] = [dict(entry) for entry in SERIAL_LINES]
    heartbeat_at = state.get("last_gateway_heartbeat_at")
    heartbeat_age = (
        max(0, current - int(heartbeat_at)) if heartbeat_at is not None else None
    )
    network_ready = bool(
        state.get("server_reachable")
        and heartbeat_age is not None
        and heartbeat_age <= GATEWAY_HEARTBEAT_STALE_SECONDS
    )
    serial_connected = bool(state.get("connected"))
    state.update(
        serial_connected=serial_connected,
        network_ready=network_ready,
        ready=network_ready,
        online=network_ready,
        fully_ready=serial_connected and network_ready,
        heartbeat_age_seconds=heartbeat_age,
    )
    if network_ready and serial_connected:
        state["health"] = "READY"
    elif network_ready:
        state["health"] = "NETWORK_ONLY"
    elif not state.get("enabled"):
        state["health"] = "DISABLED"
    elif serial_connected and state.get("wifi_ready"):
        state["health"] = "DEGRADED"
    elif serial_connected and state.get("esp_ready"):
        state["health"] = "ESP_READY"
    elif serial_connected:
        state["health"] = "SERIAL_ONLY"
    else:
        state["health"] = "OFFLINE"
    if list_ports is not None:
        state["available_ports"] = [port.device for port in list_ports.comports()]
    else:
        state["available_ports"] = []
    state["pyserial_available"] = serial is not None
    return state


def robot_network_snapshot(timestamp: int | None = None) -> dict[str, Any]:
    with ROBOT_NETWORK_LOCK:
        state = dict(ROBOT_NETWORK_STATE)
    current = now() if timestamp is None else timestamp
    state["online"] = bool(
        state["last_seen"] is not None
        and current - int(state["last_seen"]) <= 15
    )
    return state


def mark_command_delivered(revision: int, timestamp: int | None = None) -> None:
    """명령 응답을 제공한 사실만 기록한다.

    명령 URL을 브라우저가 열었다고 로봇이 온라인인 것은 아니다. 온라인과
    IP는 로봇의 별도 status 보고만 갱신하고, 여기서는 전달 시도만 남긴다.
    """
    delivered_at = now() if timestamp is None else timestamp
    with ROBOT_NETWORK_LOCK:
        ROBOT_NETWORK_STATE.update(
            delivered_revision=int(revision),
            delivered_at=delivered_at,
        )


def manual_control_snapshot() -> dict[str, Any]:
    with MANUAL_CONTROL_LOCK:
        return dict(MANUAL_CONTROL)


def manual_command_acknowledged(snapshot: dict[str, Any]) -> bool:
    """현재 수동 명령 revision을 자동차가 한 번이라도 처리했는지 반환한다."""
    with ROBOT_NETWORK_LOCK:
        ack_revision = ROBOT_NETWORK_STATE.get("ack_revision")
        ack_result = str(ROBOT_NETWORK_STATE.get("ack_result") or "").upper()
    return (
        ack_revision is not None
        and int(ack_revision) == int(snapshot["revision"])
        and ack_result in ACK_RESULT_STATES
    )


def next_command_revision(
    *current_revisions: int,
    timestamp: int | None = None,
) -> int:
    """AUTO와 MANUAL이 함께 쓰는 다음 AVR 안전 revision을 반환한다."""
    current_max = max((int(value) for value in current_revisions), default=0)
    revision = max(current_max + 1, now() if timestamp is None else int(timestamp))
    if revision > AVR_REVISION_MAX:
        raise RuntimeError("command revision exhausted the AVR signed long range")
    return revision


def next_global_revision_with_cursor(
    cursor: Any,
    timestamp: int,
    *known_revisions: int,
) -> int:
    """같은 DB 트랜잭션에서 AUTO/MANUAL 전체보다 큰 revision을 할당한다."""
    cursor.execute("SELECT revision FROM mission WHERE id = 1 FOR UPDATE")
    mission = cursor.fetchone()
    cursor.execute("SELECT revision FROM manual_control WHERE id = 1 FOR UPDATE")
    manual = cursor.fetchone()
    revisions = list(known_revisions)
    if mission:
        revisions.append(int(mission["revision"]))
    if manual:
        revisions.append(int(manual["revision"]))
    return next_command_revision(*revisions, timestamp=timestamp)


def write_manual_control_with_cursor(cursor: Any, snapshot: dict[str, Any]) -> None:
    cursor.execute(
        """
        INSERT INTO manual_control
          (id, enabled, revision, command, target_zone, action, updated_at)
        VALUES (1, %s, %s, %s, %s, %s, %s)
        ON DUPLICATE KEY UPDATE
          enabled=VALUES(enabled), revision=VALUES(revision),
          command=VALUES(command), target_zone=VALUES(target_zone),
          action=VALUES(action), updated_at=VALUES(updated_at)
        """,
        (
            1 if snapshot["enabled"] else 0,
            snapshot["revision"],
            snapshot["command"],
            snapshot["target_zone"],
            snapshot["action"],
            snapshot["updated_at"],
        ),
    )


def persist_manual_transition(
    desired: dict[str, Any],
    timestamp: int,
    *,
    enter_auto: bool = False,
) -> dict[str, Any]:
    """수동 명령과 감사 이벤트를 하나의 DB 트랜잭션으로 저장한다."""
    # 기존 테스트·내부 호출자가 사용하는 함수 시그니처는 유지하면서,
    # update_manual_control이 넣은 트랜잭션 전용 메타데이터만 여기서 소비한다.
    audit_event = desired.get("_audit_event")
    audit_snapshot = bool(desired.get("_audit_snapshot"))
    desired = {
        key: value
        for key, value in desired.items()
        if key not in {"_audit_event", "_audit_snapshot"}
    }
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                revision = next_global_revision_with_cursor(
                    cursor,
                    timestamp,
                    int(desired.get("revision") or 0),
                )
                snapshot = {
                    **desired,
                    "revision": revision,
                    "updated_at": timestamp,
                }
                if enter_auto:
                    # 전환 전에 존재하던 모든 측정은 새 AUTO 가동권으로 쓰지
                    # 않는다. 각 구역에서 전환 후 측정이 한 번 들어올 때까지
                    # 안전 정지를 유지한다.
                    zone_placeholders = ",".join(["%s"] * len(DEFAULT_ZONE_IDS))
                    cursor.execute(
                        f"""
                        UPDATE zone_status
                        SET acted_through_reading_id = GREATEST(
                            acted_through_reading_id,
                            COALESCE(latest_reading_id, 0)
                        )
                        WHERE zone_id IN ({zone_placeholders})
                        """,
                        DEFAULT_ZONE_IDS,
                    )
                    cursor.execute(
                        """
                        UPDATE mission
                        SET revision = %s, command = 'ALL_STOP',
                            target_zone = 'HOME', action = 'NONE',
                            reason = %s, updated_at = %s
                        WHERE id = 1
                        """,
                        (
                            revision,
                            "AUTO enabled; waiting for post-transition zone readings",
                            timestamp,
                        ),
                    )
                write_manual_control_with_cursor(cursor, snapshot)
                if audit_event is not None:
                    source, event_type, message = audit_event
                    record_event_with_cursor(
                        cursor,
                        source,
                        event_type,
                        message,
                        snapshot if audit_snapshot else None,
                        timestamp=timestamp,
                    )
            connection.commit()
            return snapshot
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()


def advance_completed_manual_task(
    ack_revision: int | None,
    ack_result: str | None,
    timestamp: int,
) -> dict[str, Any] | None:
    """완료된 수동 모듈 임무를 같은 구역의 1회성 TASK/NONE으로 넘긴다.

    자동 습도 임무와 달리 수동 HUMIDIFY/DEHUMIDIFY는 한 번만 실행해야 한다.
    새 revision이 없으면 SensorUno가 기존 TASK를 계속 반복하므로, 완료 ACK가
    실제 수용된 뒤에만 같은 구역 TASK/NONE을 새 명령으로 저장한다.
    """
    if ack_revision is None or str(ack_result or "").upper() != "COMPLETED":
        return None

    with MANUAL_CONTROL_LOCK:
        current = dict(MANUAL_CONTROL)
        if not (
            current["enabled"]
            and int(current["revision"]) == int(ack_revision)
            and current["command"] == "TASK"
            and current["action"] != "NONE"
        ):
            return None

        snapshot = {
            **current,
            "command": "TASK",
            "target_zone": current["target_zone"],
            "action": "NONE",
        }
        # DB commit이 성공한 뒤에만 메모리 명령을 바꾼다.
        snapshot = persist_manual_transition(snapshot, timestamp)
        MANUAL_CONTROL.update(snapshot)
        return dict(snapshot)


def update_manual_control(payload: dict[str, Any]) -> dict[str, Any]:
    """웹 시험 패널의 명령을 검증하고 다음 로봇 폴링까지 보관한다."""
    mode = str(payload.get("mode", "MANUAL")).strip().upper()
    if mode not in {"AUTO", "MANUAL"}:
        raise ValueError("mode must be AUTO or MANUAL")
    if mode == "AUTO":
        with MANUAL_CONTROL_LOCK:
            if not MANUAL_CONTROL["enabled"]:
                return dict(MANUAL_CONTROL)
            if MANUAL_CONTROL["enabled"] and not manual_command_acknowledged(MANUAL_CONTROL):
                raise ValueError(
                    "the current manual command has not been acknowledged by the robot"
                )
            if (
                MANUAL_CONTROL["enabled"]
                and MANUAL_CONTROL["command"] == "ALL_STOP"
                and payload.get("confirm_all_stop") is not True
            ):
                raise ValueError("ALL_STOP is latched; explicit confirmation is required")
            snapshot = {
                **MANUAL_CONTROL,
                "enabled": False,
                "command": "ALL_STOP",
                "target_zone": "HOME",
                "action": "NONE",
                "_audit_event": (
                    "DASHBOARD",
                    "AUTO_MODE",
                    "Robot control returned to automatic mode",
                ),
            }
            timestamp = now()
            snapshot = persist_manual_transition(
                snapshot,
                timestamp,
                enter_auto=True,
            )
            snapshot.pop("_audit_event", None)
            snapshot.pop("_audit_snapshot", None)
            MANUAL_CONTROL.update(snapshot)
        return snapshot

    command = str(payload.get("command", "")).strip().upper()
    if command not in MANUAL_COMMANDS:
        raise ValueError("unsupported manual robot command")

    target_zone = str(payload.get("target_zone", "HOME")).strip().upper()
    action = str(payload.get("action", "NONE")).strip().upper()
    if command == "TASK":
        if target_zone not in ACTIVE_ZONE_IDS:
            raise ValueError(
                f"manual task target must be one of: {', '.join(DEFAULT_ZONE_IDS)}"
            )
        if action not in {"HUMIDIFY", "DEHUMIDIFY", "NONE"}:
            raise ValueError("manual task action must be HUMIDIFY, DEHUMIDIFY or NONE")
    else:
        # CALIBRATE_HOME은 AUTO 전환이나 구역 TASK가 아니다. 차량을 HOME
        # 종점에 물리적으로 놓은 뒤 Sensor/Motor의 위치·방향 기준만 맞추는
        # 별도 수동 명령이며, 기존 4필드 계약에서는 HOME/NONE으로 표현한다.
        target_zone = "HOME"
        if command == "ACT_HUMIDIFY":
            action = "HUMIDIFY"
        elif command == "ACT_DEHUMID":
            action = "DEHUMIDIFY"
        else:
            action = "NONE"

    with MANUAL_CONTROL_LOCK:
        pending = (
            MANUAL_CONTROL["enabled"]
            and not manual_command_acknowledged(MANUAL_CONTROL)
        )
        if pending and command != "ALL_STOP":
            raise ValueError(
                "the current manual command has not been acknowledged by the robot"
            )
        if pending and MANUAL_CONTROL["command"] == "ALL_STOP" and command == "ALL_STOP":
            # 같은 미확인 긴급 정지를 반복 클릭해 revision을 계속 바꾸면 로봇이
            # ACK할 목표가 사라진다. 기존 latch를 그대로 반환한다.
            return dict(MANUAL_CONTROL)
        if (
            MANUAL_CONTROL["enabled"]
            and MANUAL_CONTROL["command"] == "ALL_STOP"
            and command != "ALL_STOP"
        ):
            raise ValueError("ALL_STOP is latched; return to AUTO with confirmation before another command")
        snapshot = {
            **MANUAL_CONTROL,
            "enabled": True,
            "command": command,
            "target_zone": target_zone,
            "action": action,
            "_audit_event": (
                "DASHBOARD",
                "MANUAL_COMMAND",
                f"Manual command {command}: {target_zone} / {action}",
            ),
            "_audit_snapshot": True,
        }
        timestamp = now()
        snapshot = persist_manual_transition(snapshot, timestamp)
        snapshot.pop("_audit_event", None)
        snapshot.pop("_audit_snapshot", None)
        MANUAL_CONTROL.update(snapshot)
    return snapshot


def complete_auto_task(
    effective_command: dict[str, Any],
    ack_revision: int,
    timestamp: int,
) -> dict[str, Any] | None:
    """AUTO 모듈 완료를 측정 소비 + 안전 정지 명령으로 원자 저장한다.

    ACK를 받기 전 target 구역의 가장 최신 ``reading_log.id``까지 소비한다.
    같은 트랜잭션에서 mission을 새 revision의 ALL_STOP으로 바꾸므로 서버나
    SensorUno가 재시작돼도 완료한 TASK가 다시 노출되지 않는다.
    """
    if not (
        effective_command.get("source") == "AUTO"
        and effective_command.get("command") == "TASK"
        and effective_command.get("action") in {"HUMIDIFY", "DEHUMIDIFY"}
        and int(effective_command.get("revision", -1)) == int(ack_revision)
    ):
        return None

    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                cursor.execute("SELECT * FROM mission WHERE id = 1 FOR UPDATE")
                mission = cursor.fetchone()
                cursor.execute("SELECT * FROM manual_control WHERE id = 1 FOR UPDATE")
                manual = cursor.fetchone()
                if not mission or not manual:
                    connection.rollback()
                    return None
                mission_still_matches_ack = bool(
                    int(mission["revision"]) == int(ack_revision)
                    and mission["command"] == "TASK"
                    and mission["target_zone"] == effective_command["target_zone"]
                    and mission["action"] == effective_command["action"]
                )

                cursor.execute(
                    """
                    UPDATE zone_status
                    SET acted_through_reading_id = GREATEST(
                        acted_through_reading_id,
                        COALESCE(latest_reading_id, 0)
                    )
                    WHERE zone_id = %s
                    """,
                    (effective_command["target_zone"],),
                )
                watermark_advanced = int(cursor.rowcount) > 0

                if mission_still_matches_ack and not bool(manual["enabled"]):
                    hold_revision = next_command_revision(
                        int(mission["revision"]),
                        int(manual["revision"]),
                        timestamp=timestamp,
                    )
                    hold_reason = (
                        f"Completed {mission['target_zone']} / {mission['action']}; "
                        "waiting for a newer zone reading"
                    )
                    cursor.execute(
                        """
                        UPDATE mission
                        SET revision = %s, command = 'ALL_STOP',
                            target_zone = 'HOME', action = 'NONE',
                            reason = %s, updated_at = %s
                        WHERE id = 1 AND revision = %s
                        """,
                        (hold_revision, hold_reason, timestamp, ack_revision),
                    )
                    if int(cursor.rowcount) != 1:
                        connection.rollback()
                        return None
                    completion_recorded = True
                else:
                    if not watermark_advanced:
                        connection.rollback()
                        return None
                    # ACK 검증과 이 트랜잭션 사이에 새 reading이 mission을
                    # 바꿨어도 완료 구역 watermark는 반드시 소비한다. 더 최신
                    # mission을 과거 rev로 덮지 않고, 소비 결과만 다시 계산한다.
                    adjusted_mission = recompute_mission(
                        cursor,
                        persist_reason=True,
                    )
                    hold_revision = int(adjusted_mission["revision"])
                    completion_recorded = watermark_advanced
            connection.commit()
            if not completion_recorded:
                return None
            return {
                "completed_revision": int(ack_revision),
                "completed_zone": effective_command["target_zone"],
                "completed_action": effective_command["action"],
                "hold_revision": hold_revision,
            }
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()


def report_robot_network(query: dict[str, list[str]], client_ip: str) -> dict[str, Any]:
    allowed_phases = {
        "UNKNOWN", "IDLE", "MOVING", "WAITING_RFID",
        "MODULE_RUNNING", "TASK_COMPLETE", "RETURNING",
    }

    def token(name: str, default: str, limit: int) -> str:
        value = str(query.get(name, [default])[0]).strip().upper()[:limit]
        if not value or any(not (c.isalnum() or c in "_-") for c in value):
            raise ValueError(f"invalid robot status field: {name}")
        return value

    phase = token("phase", "UNKNOWN", 24)
    if phase not in allowed_phases:
        raise ValueError("invalid robot phase")
    event = token("event", "HEARTBEAT", 32)
    zone = token("zone", "HOME", 16)
    action = token("action", "NONE", 20)
    ack_values = query.get("ack_revision") or query.get("revision")
    ack_revision: int | None = None
    ack_result: str | None = None
    if ack_values:
        raw_revision = str(ack_values[0]).strip()
        if not raw_revision.isdigit():
            raise ValueError("ack_revision must be a non-negative integer")
        ack_revision = int(raw_revision)
        if ack_revision > 2_147_483_647:
            raise ValueError("ack_revision exceeds the AVR signed long range")
        ack_result = token("result", "ACK", 24)
        if ack_result not in ACK_RESULT_STATES:
            raise ValueError("unsupported robot command result")

    # ACK는 이 서버가 가장 최근에 제공했고 지금도 유효한 명령에 대해서만
    # 인정한다. 이전 임무의 늦은 보고는 상태/heartbeat로는 받되 ACK로 저장하지
    # 않아 새 명령이 실행된 것처럼 보이지 않게 한다.
    effective_command = robot_command_snapshot() if ack_revision is not None else None
    effective_revision = (
        int(effective_command["revision"]) if effective_command is not None else None
    )

    timestamp = now()
    ack_accepted = False
    ack_rejection: str | None = None
    with ROBOT_NETWORK_LOCK:
        delivered_revision = ROBOT_NETWORK_STATE.get("delivered_revision")
        previous_ack_revision = ROBOT_NETWORK_STATE.get("ack_revision")
        previous_ack_result = str(
            ROBOT_NETWORK_STATE.get("ack_result") or ""
        ).upper()
        ROBOT_NETWORK_STATE.update(
            phase=phase, event=event, zone=zone, action=action,
            reported_at=timestamp, last_seen=timestamp, ip=client_ip,
        )
        if ack_revision is not None:
            current_ack_accepted = (
                delivered_revision is not None
                and int(delivered_revision) == ack_revision
                and effective_revision == ack_revision
            )
            replay_of_accepted_ack = (
                previous_ack_revision is not None
                and int(previous_ack_revision) == ack_revision
                and previous_ack_result == ack_result
            )
            ack_accepted = current_ack_accepted or replay_of_accepted_ack
            if current_ack_accepted:
                ROBOT_NETWORK_STATE.update(
                    ack_revision=ack_revision,
                    ack_result=ack_result,
                    acknowledged_at=timestamp,
                )
            else:
                ack_rejection = (
                    f"ack revision {ack_revision} does not match "
                    f"delivered={delivered_revision}, effective={effective_revision}"
                )

    details = {
        "ip": client_ip, "transport": "ESP-01 Wi-Fi",
        "phase": phase, "event": event, "zone": zone, "action": action,
        "reported_at": timestamp, "ack_revision": ack_revision,
        "ack_result": ack_result, "ack_accepted": ack_accepted,
        "ack_rejection": ack_rejection,
    }
    touch_device("ROBOT_WIFI", "ROBOT_NETWORK", details=details)
    if event != "HEARTBEAT":
        record_event(
            "ROBOT_WIFI", event,
            f"Robot {phase}: {zone} / {action}",
            details,
        )
    auto_completion: dict[str, Any] | None = None
    if (
        ack_accepted
        and effective_command is not None
        and effective_command.get("source") == "AUTO"
        and phase == "TASK_COMPLETE"
        and ack_result == "COMPLETED"
        and ack_revision is not None
    ):
        auto_completion = complete_auto_task(
            effective_command,
            ack_revision,
            timestamp,
        )
        if auto_completion is not None:
            with ROBOT_NETWORK_LOCK:
                ROBOT_NETWORK_STATE.update(
                    completed_auto_revision=ack_revision,
                    completed_auto_at=timestamp,
                    completed_auto_zone=auto_completion["completed_zone"],
                    completed_auto_action=auto_completion["completed_action"],
                )

    if ack_accepted:
        # ROBOT_NETWORK_LOCK을 해제한 뒤 MANUAL_CONTROL_LOCK을 잡는다. 반대
        # 순서를 사용하는 대시보드 수동 명령과 교착되지 않게 한다.
        advance_completed_manual_task(ack_revision, ack_result, timestamp)
    return {
        "accepted": True, "phase": phase, "event": event,
        "ack_revision": ack_revision if ack_accepted else None,
        "result": ack_result, "ack_accepted": ack_accepted,
        "ack_rejection": ack_rejection,
    }


def classify_serial_line(line: str) -> tuple[str, bool]:
    """시리얼 한 줄을 UI 분류와 DB 보존 여부로 변환한다."""
    upper = line.upper()
    if "[RFID] UID=" in upper:
        return "RFID_CARD", True
    if "[RFID TEST]" in upper:
        return "RFID_TEST", True
    if "[MOTOR PLACEHOLDER]" in upper:
        return "MOVEMENT", True
    if "[MODULE PLACEHOLDER]" in upper:
        return "MODULE", True
    if "[RFID]" in upper and "ARRIVAL CONFIRMED" in upper:
        return "RFID_ARRIVAL", True
    if "[RELAY]" in upper:
        return "RELAY", True
    if "WIFI OK" in upper or "WIFI ERROR" in upper or "AP CONNECTION" in upper:
        return "WIFI", True
    if "ERROR" in upper or "FAIL" in upper:
        return "ERROR", True
    if "[BOOT]" in upper:
        return "SYSTEM", True
    if "[COMMAND]" in upper or "[CAR]" in upper:
        return "ROBOT", False
    if "[HTTP" in upper or "[SERVER]" in upper or "[ESP" in upper:
        return "NETWORK", False
    if "[RFID]" in upper:
        return "RFID", False
    return "SERIAL", False


class SerialBridge(threading.Thread):
    """Arduino USB 시리얼을 읽어 관제 UI와 RFID 이벤트 DB로 전달한다."""

    def __init__(self) -> None:
        super().__init__(name="arduino-serial-bridge", daemon=True)
        self._connection: Any = None
        self._reconnect = threading.Event()
        self._last_device_touch = 0
        self._write_lock = threading.Lock()

    def request_reconnect(self, port: str | None = None) -> None:
        if port:
            with SERIAL_LOCK:
                SERIAL_STATE["configured_port"] = port
        self._reconnect.set()
        connection = self._connection
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def send_rfid_test(self) -> None:
        connection = self._connection
        if connection is None or not connection.is_open:
            raise RuntimeError("Arduino serial is not connected")
        with self._write_lock:
            connection.write(b"T")
            connection.flush()
        record_event("DASHBOARD", "RFID_TEST", "Placeholder RFID test command sent to Arduino")

    def _set_state(self, **values: Any) -> None:
        with SERIAL_LOCK:
            SERIAL_STATE.update(values)

    def _append_line(self, line: str) -> None:
        timestamp = now()
        event_type, persist = classify_serial_line(line)
        gateway_changes = gateway_state_changes_for_serial_line(line, timestamp)
        upper = line.upper()
        phase = None
        if "[MOTOR PLACEHOLDER] START ->" in upper:
            phase = "MOVING"
        elif "WAITING FOR ARRIVAL TAG" in upper:
            phase = "WAITING_RFID"
        elif "[MODULE PLACEHOLDER] START" in upper:
            phase = "MODULE_RUNNING"
        elif "[MODULE PLACEHOLDER] COMPLETE" in upper:
            phase = "TASK_COMPLETE"
        elif "[MOTOR PLACEHOLDER] RETURN START" in upper:
            phase = "RETURNING"
        elif (
            "HOME ARRIVAL COMPLETE" in upper
            or "[CAR] IDLE AT HOME" in upper
            or "IDLE/HOME STATE RETAINED" in upper
        ):
            phase = "IDLE"
        with SERIAL_LOCK:
            SERIAL_STATE["sequence"] += 1
            SERIAL_STATE["last_line_at"] = timestamp
            if gateway_changes:
                SERIAL_STATE.update(gateway_changes)
            if phase:
                SERIAL_STATE["robot_phase"] = phase
            entry = {
                "sequence": SERIAL_STATE["sequence"],
                "received_at": timestamp,
                "first_received_at": timestamp,
                "repeat_count": 1,
                "type": event_type,
                "message": line,
            }
            coalesce_key = serial_coalesce_key(line)
            repeated_entry = None
            if coalesce_key is not None:
                for candidate in reversed(SERIAL_LINES):
                    if serial_coalesce_key(str(candidate.get("message", ""))) != coalesce_key:
                        continue
                    if timestamp - int(candidate["received_at"]) <= SERIAL_LINE_COALESCE_SECONDS:
                        repeated_entry = candidate
                    break
            if repeated_entry is None:
                SERIAL_LINES.append(entry)
            else:
                SERIAL_LINES.remove(repeated_entry)
                repeated_entry.update(
                    sequence=SERIAL_STATE["sequence"],
                    received_at=timestamp,
                    repeat_count=int(repeated_entry.get("repeat_count", 1)) + 1,
                )
                SERIAL_LINES.append(repeated_entry)
        if persist:
            record_event("USB_SERVER_GATEWAY", event_type, line)
        if timestamp - self._last_device_touch >= 5:
            gateway = serial_snapshot(timestamp)
            touch_device(
                "USB_SERVER_GATEWAY", "SERVER_GATEWAY",
                status="ONLINE" if gateway["fully_ready"] else gateway["health"],
                details={
                    "port": gateway["active_port"],
                    "baud": gateway["baud"],
                    "serial_connected": gateway["serial_connected"],
                    "esp_ready": gateway["esp_ready"],
                    "wifi_ready": gateway["wifi_ready"],
                    "network_ready": gateway["network_ready"],
                },
            )
            self._last_device_touch = timestamp

    def run(self) -> None:
        if not SERIAL_ENABLED:
            self._set_state(error="USB serial bridge disabled (network-only server mode)")
            return
        if serial is None:
            self._set_state(error="pyserial is not installed")
            return

        while True:
            try:
                with SERIAL_LOCK:
                    port = str(SERIAL_STATE["configured_port"])
                    baud = int(SERIAL_STATE["baud"])
                self._reconnect.clear()
                self._set_state(error=None)
                self._connection = serial.Serial(port, baud, timeout=1)
                self._set_state(
                    connected=True,
                    active_port=port,
                    error=None,
                )
                gateway = serial_snapshot()
                touch_device(
                    "USB_SERVER_GATEWAY", "SERVER_GATEWAY",
                    status="ONLINE" if gateway["fully_ready"] else gateway["health"],
                    details={
                        "port": port,
                        "baud": baud,
                        "transport": "USB serial",
                        "serial_connected": True,
                        "esp_ready": gateway["esp_ready"],
                        "wifi_ready": gateway["wifi_ready"],
                        "network_ready": gateway["network_ready"],
                    },
                )
                record_event("SERVER", "SERIAL_CONNECTED", f"Arduino serial connected on {port}")

                while self._connection.is_open and not self._reconnect.is_set():
                    raw = self._connection.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    self._append_line(line)
            except Exception as error:
                self._set_state(connected=False, active_port=None, error=str(error))
            finally:
                connection = self._connection
                self._connection = None
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                self._set_state(connected=False, active_port=None)
            time.sleep(2)


SERIAL_BRIDGE = SerialBridge()


def current_thresholds() -> dict[str, float]:
    with SETTINGS_LOCK:
        values = dict(THRESHOLDS)
    # low/high 별칭은 기존 UI·클라이언트와의 호환성을 유지한다.
    return {
        **values,
        "low": values["humidity_min"],
        "high": values["humidity_max"],
    }


def update_thresholds(payload: dict[str, Any]) -> dict[str, float]:
    required = tuple(DEFAULT_THRESHOLDS)
    try:
        values = {key: float(payload[key]) for key in required}
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "temperature_min, temperature_max, humidity_min and humidity_max must be numeric"
        ) from error

    if not (-40 <= values["temperature_min"] < values["temperature_max"] <= 100):
        raise ValueError("temperature range must be within -40..100 and min must be below max")
    if not (0 <= values["humidity_min"] < values["humidity_max"] <= 100):
        raise ValueError("humidity range must be within 0..100 and min must be below max")

    timestamp = now()
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                for key, value in values.items():
                    cursor.execute(
                        """
                        INSERT INTO system_settings(setting_key, setting_value, updated_at)
                        VALUES (%s, %s, %s)
                        ON DUPLICATE KEY UPDATE
                          setting_value=VALUES(setting_value), updated_at=VALUES(updated_at)
                        """,
                        (key, value, timestamp),
                    )
                recompute_mission(
                    cursor,
                    persist_reason=True,
                    threshold_values=values,
                )
                record_event_with_cursor(
                    cursor,
                    "DASHBOARD",
                    "SETTINGS_UPDATED",
                    "Temperature and humidity thresholds updated",
                    values,
                    timestamp=timestamp,
                )
            # 설정·새 mission·감사 이벤트가 모두 저장된 뒤에만 런타임
            # 판정값을 공개한다. 어느 단계든 실패하면 전체를 rollback한다.
            connection.commit()
            with SETTINGS_LOCK:
                THRESHOLDS.update(values)
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()
    return current_thresholds()


def classify_temperature(
    temperature: float | None,
    threshold_values: dict[str, float] | None = None,
) -> dict[str, Any]:
    thresholds = current_thresholds() if threshold_values is None else threshold_values
    if temperature is None:
        return {"state": "WAITING", "normal": False, "message": "No temperature reading received"}
    value = float(temperature)
    if value < thresholds["temperature_min"]:
        return {
            "state": "TEMP_LOW", "normal": False,
            "message": f"Temperature {value:.1f}°C is below {thresholds['temperature_min']:.1f}°C",
        }
    if value > thresholds["temperature_max"]:
        return {
            "state": "TEMP_HIGH", "normal": False,
            "message": f"Temperature {value:.1f}°C is above {thresholds['temperature_max']:.1f}°C",
        }
    return {"state": "NORMAL", "normal": True, "message": "Temperature is within range"}


def classify_humidity(
    humidity: float | None,
    previous_violation_mode: str | None = None,
    threshold_values: dict[str, float] | None = None,
) -> dict[str, Any]:
    """습도를 판정하며 직전 위반 모드에는 2% 복귀 히스테리시스를 적용한다."""
    if humidity is None:
        return {
            "state": "WAITING",
            "normal": False,
            "action": "NONE",
            "excess": 0.0,
            "message": "No humidity reading received",
        }
    # MySQL DECIMAL은 Decimal로 반환되므로 판정 계산 전에 float로 통일한다.
    humidity_value = float(humidity)
    thresholds = current_thresholds() if threshold_values is None else threshold_values
    low_humidity = thresholds["humidity_min"]
    high_humidity = thresholds["humidity_max"]
    low_recovery = min(
        low_humidity + HUMIDITY_HYSTERESIS_PERCENT,
        high_humidity,
    )
    high_recovery = max(
        high_humidity - HUMIDITY_HYSTERESIS_PERCENT,
        low_humidity,
    )
    previous_mode = str(previous_violation_mode or "").strip().upper()
    if humidity_value < low_humidity:
        excess = low_humidity - humidity_value
        return {
            "state": "ERROR_LOW",
            "normal": False,
            "action": "HUMIDIFY",
            "excess": round(excess, 1),
            "message": f"Humidity {humidity_value:.1f}% is below {low_humidity:.1f}%",
        }
    if humidity_value > high_humidity:
        excess = humidity_value - high_humidity
        return {
            "state": "ERROR_HIGH",
            "normal": False,
            "action": "DEHUMIDIFY",
            "excess": round(excess, 1),
            "message": f"Humidity {humidity_value:.1f}% is above {high_humidity:.1f}%",
        }
    if (
        previous_mode == "HUMIDIFY"
        and humidity_value < low_recovery
    ):
        return {
            "state": "ERROR_LOW",
            "normal": False,
            "action": "HUMIDIFY",
            "excess": 0.0,
            "message": (
                f"Humidity {humidity_value:.1f}% is recovering; "
                f"humidifying remains active until "
                f"{low_recovery:.1f}%"
            ),
        }
    if (
        previous_mode == "DEHUMIDIFY"
        and humidity_value > high_recovery
    ):
        return {
            "state": "ERROR_HIGH",
            "normal": False,
            "action": "DEHUMIDIFY",
            "excess": 0.0,
            "message": (
                f"Humidity {humidity_value:.1f}% is recovering; "
                f"dehumidifying remains active until "
                f"{high_recovery:.1f}%"
            ),
        }
    return {
        "state": "NORMAL",
        "normal": True,
        "action": "NONE",
        "excess": 0.0,
        "message": f"Humidity {humidity_value:.1f}% is within the configured range",
    }


def violation_for(
    humidity: float | None,
    previous_violation_mode: str | None = None,
    threshold_values: dict[str, float] | None = None,
) -> tuple[str | None, float]:
    """기존 임무 계산 코드가 사용할 동작과 초과 폭을 반환한다."""
    condition = classify_humidity(
        humidity,
        previous_violation_mode,
        threshold_values,
    )
    action = condition["action"]
    return (None if action == "NONE" else action, condition["excess"])


def is_valid_zone_id(zone_id: str) -> bool:
    """ZONE1부터 ZONE99까지의 고정 센싱 구역을 허용한다."""
    if not zone_id.startswith("ZONE") or not zone_id[4:].isdigit():
        return False
    number = int(zone_id[4:])
    return 1 <= number <= 99


def recompute_mission(
    cursor: Any,
    *,
    persist_reason: bool = False,
    threshold_values: dict[str, float] | None = None,
) -> dict[str, Any]:
    timestamp = now()
    candidates: list[dict[str, Any]] = []
    fresh_zone_count = 0
    unavailable_zone_count = 0
    waiting_for_new_reading_count = 0

    zone_placeholders = ",".join(["%s"] * len(DEFAULT_ZONE_IDS))
    cursor.execute(
        f"SELECT * FROM zone_status WHERE zone_id IN ({zone_placeholders})",
        DEFAULT_ZONE_IDS,
    )
    statuses = {
        str(status["zone_id"]): status
        for status in cursor.fetchall()
    }
    for zone_id in DEFAULT_ZONE_IDS:
        status = statuses.get(zone_id)
        if status is None:
            unavailable_zone_count += 1
            continue
        is_stale = (
            status["updated_at"] is None
            or status["humidity"] is None
            or timestamp - status["updated_at"] > STALE_AFTER_SECONDS
        )
        if is_stale:
            unavailable_zone_count += 1
            continue
        fresh_zone_count += 1

        condition = classify_humidity(
            status["humidity"],
            status.get("violation_mode"),
            threshold_values,
        )
        # 실 DB에서는 migration 이후 항상 reading_log.id가 들어 있다. 이
        # fallback은 과거 테스트 fixture/수동 복구 행만 안전하게 처리한다.
        latest_reading_id = status.get("latest_reading_id")
        if latest_reading_id is None and status.get("updated_at") is not None:
            latest_reading_id = 1
        acted_through_reading_id = int(
            status.get("acted_through_reading_id") or 0
        )
        has_unconsumed_reading = bool(
            latest_reading_id is not None
            and int(latest_reading_id) > acted_through_reading_id
        )
        if not has_unconsumed_reading:
            waiting_for_new_reading_count += 1

        mode = condition["action"]
        excess = condition["excess"]
        if condition["normal"]:
            continue
        if not has_unconsumed_reading:
            continue

        started_at = status["violation_started_at"] or timestamp
        duration_seconds = timestamp - started_at
        # Large deviations win first.  Duration breaks ties between similar deviations.
        priority = excess * 100.0 + duration_seconds / 60.0
        candidates.append(
            {
                "zone_id": status["zone_id"],
                "state": condition["state"],
                "action": mode,
                "humidity": status["humidity"],
                "excess": round(excess, 1),
                "duration_seconds": duration_seconds,
                "priority": round(priority, 2),
            }
        )

    # 완전히 같은 우선순위에서는 구역 ID 오름차순으로 고정한다. DB 반환
    # 순서에 따라 winner가 바뀌어 revision이 반복 증가하는 것을 막는다.
    candidates.sort(key=lambda item: (-float(item["priority"]), item["zone_id"]))
    if candidates:
        winner = candidates[0]
        desired = {
            "command": "TASK",
            "target_zone": winner["zone_id"],
            "action": winner["action"],
            "reason": (
                f"priority={winner['priority']}; humidity={winner['humidity']}; "
                f"excess={winner['excess']}; duration={winner['duration_seconds']}s"
            ),
        }
    elif unavailable_zone_count or waiting_for_new_reading_count:
        desired = {
            "command": "ALL_STOP",
            "target_zone": "HOME",
            "action": "NONE",
            "reason": (
                "Waiting for a fresh, unconsumed reading from one or more "
                "zone sensors"
                if waiting_for_new_reading_count and not unavailable_zone_count
                else "No actionable humidity reading is available, but one or more "
                     "zone sensors are stale, missing, or awaiting a new reading"
            ),
        }
    else:
        desired = {
            "command": "RETURN_HOME",
            "target_zone": "HOME",
            "action": "NONE",
            "reason": "All fresh zone readings are within the configured range",
        }

    cursor.execute("SELECT * FROM mission WHERE id = 1")
    current = cursor.fetchone()
    # revision은 실제 동작이 달라지거나, 완료 뒤 새 측정이 다음 burst를 허용할
    # 때만 올린다. duration/priority가 든 표시용 reason은 시간이 흐를 때마다
    # 달라질 수 있으므로 dashboard GET만으로 새 명령처럼 보이면 안 된다.
    command_changed = any(
        current[field] != desired[field]
        for field in ("command", "target_zone", "action")
    )
    if command_changed:
        revision = next_global_revision_with_cursor(
            cursor,
            timestamp,
            int(current["revision"]),
        )
        cursor.execute(
            """
            UPDATE mission
            SET revision = %s, command = %s, target_zone = %s, action = %s, reason = %s, updated_at = %s
            WHERE id = %s
            """,
            (
                revision,
                desired["command"],
                desired["target_zone"],
                desired["action"],
                desired["reason"],
                timestamp,
                1,
            ),
        )
    elif persist_reason and current["reason"] != desired["reason"]:
        cursor.execute(
            "UPDATE mission SET reason = %s, updated_at = %s WHERE id = %s",
            (desired["reason"], timestamp, 1),
        )

    cursor.execute("SELECT * FROM mission WHERE id = 1")
    mission = cursor.fetchone()
    if candidates:
        system_status = "ERROR"
    elif unavailable_zone_count:
        system_status = "SENSOR_ERROR"
    elif waiting_for_new_reading_count:
        system_status = "WAITING"
    elif fresh_zone_count:
        system_status = "NORMAL"
    else:
        system_status = "WAITING"
    return {
        **mission,
        "system_status": system_status,
        "fresh_zone_count": fresh_zone_count,
        "unavailable_zone_count": unavailable_zone_count,
        "waiting_for_new_reading_count": waiting_for_new_reading_count,
        "candidates": candidates,
    }


def record_reading(payload: dict[str, Any]) -> dict[str, Any]:
    zone_id = str(payload.get("zone_id", "")).upper()
    if not is_valid_zone_id(zone_id):
        raise ValueError("zone_id must use the form ZONE1 through ZONE99")
    if zone_id not in ACTIVE_ZONE_IDS:
        raise ValueError(
            f"zone_id is not active; configured zones are {', '.join(DEFAULT_ZONE_IDS)}"
        )

    try:
        temperature = float(payload["temperature"])
        humidity = float(payload["humidity"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("temperature and humidity must be numeric") from error

    if not (-40 <= temperature <= 100 and 0 <= humidity <= 100):
        raise ValueError("reading is outside a plausible temperature/humidity range")

    timestamp = now()
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                # ZONE99처럼 처음 들어오는 구역도 자동으로 관제 목록에 추가한다.
                cursor.execute("INSERT IGNORE INTO zone_status(zone_id) VALUES (%s)", (zone_id,))
                cursor.execute(
                    "SELECT violation_mode, violation_started_at, updated_at "
                    "FROM zone_status WHERE zone_id = %s",
                    (zone_id,),
                )
                prior = cursor.fetchone()
                prior_is_fresh = bool(
                    prior
                    and prior.get("updated_at") is not None
                    and timestamp - int(prior["updated_at"]) <= STALE_AFTER_SECONDS
                )
                prior_mode = prior["violation_mode"] if prior_is_fresh else None
                condition = classify_humidity(
                    humidity,
                    prior_mode,
                )
                mode = None if condition["normal"] else condition["action"]
                started_at = None
                if mode:
                    started_at = (
                        prior["violation_started_at"]
                        if (
                            prior_is_fresh
                            and prior_mode == mode
                            and prior["violation_started_at"]
                        )
                        else timestamp
                    )

                cursor.execute(
                    "INSERT INTO reading_log(received_at, zone_id, temperature, humidity) VALUES (%s, %s, %s, %s)",
                    (timestamp, zone_id, temperature, humidity),
                )
                reading_id = int(cursor.lastrowid)
                cursor.execute(
                    """
                    UPDATE zone_status
                    SET temperature = %s, humidity = %s, updated_at = %s,
                        latest_reading_id = %s,
                        violation_mode = %s, violation_started_at = %s
                    WHERE zone_id = %s
                    """,
                    (
                        temperature,
                        humidity,
                        timestamp,
                        reading_id,
                        mode,
                        started_at,
                        zone_id,
                    ),
                )
                touch_device_with_cursor(
                    cursor,
                    zone_id,
                    "HUMIDITY_SENSOR",
                    details={
                        "temperature": temperature,
                        "humidity": humidity,
                        "ip": payload.get("_client_ip"),
                    },
                )
                mission = recompute_mission(cursor, persist_reason=True)
            connection.commit()
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()
    return {
        "zone_id": zone_id,
        "temperature": temperature,
        "humidity": humidity,
        "state": condition["state"],
        "normal": condition["normal"],
        "action": condition["action"],
        "message": condition["message"],
        "mission": mission,
    }


def apply_gateway_health_to_devices(
    devices: list[dict[str, Any]], serial_state: dict[str, Any]
) -> None:
    """DB의 최근 로그 시간이 게이트웨이 준비 상태를 가장하지 않게 보정한다.

    실패 로그도 시리얼 데이터이므로 예전 코드는 이를 받을 때마다 장치를
    ONLINE으로 표시했다. USB 장치는 USB와 네트워크가 모두 준비돼야 하고,
    Wi-Fi 장치는 실제 HTTP heartbeat가 신선할 때만 온라인이다.
    """
    for device in devices:
        device_id = str(device.get("device_id", ""))
        if device_id == "USB_SERVER_GATEWAY":
            device["online"] = bool(serial_state.get("fully_ready"))
            device["status"] = (
                "ONLINE" if device["online"] else str(serial_state.get("health", "OFFLINE"))
            )
            device.setdefault("details", {}).update(
                serial_connected=bool(serial_state.get("serial_connected")),
                esp_ready=bool(serial_state.get("esp_ready")),
                wifi_ready=bool(serial_state.get("wifi_ready")),
                network_ready=bool(serial_state.get("network_ready")),
            )
        elif device_id == "SERVER_GATEWAY_WIFI":
            device["online"] = bool(serial_state.get("network_ready"))
            device["status"] = "ONLINE" if device["online"] else "OFFLINE"


def dashboard() -> dict[str, Any]:
    timestamp = now()
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                mission = recompute_mission(cursor)
                cursor.execute("SELECT * FROM manual_control WHERE id = 1")
                manual = cursor.fetchone()
                if not manual:
                    raise RuntimeError("manual_control row id=1 is missing")
                manual = {
                    **manual,
                    "enabled": bool(manual["enabled"]),
                    "revision": int(manual["revision"]),
                }
                zone_placeholders = ",".join(["%s"] * len(DEFAULT_ZONE_IDS))
                cursor.execute(
                    f"SELECT * FROM zone_status "
                    f"WHERE zone_id IN ({zone_placeholders}) ORDER BY zone_id",
                    DEFAULT_ZONE_IDS,
                )
                zones = cursor.fetchall()
                temperature_alert_count = 0
                for status in zones:
                    status["stale"] = (
                        status["updated_at"] is None
                        or status["humidity"] is None
                        or timestamp - status["updated_at"] > STALE_AFTER_SECONDS
                    )
                    if status["stale"]:
                        condition = {
                            "state": "STALE" if status["updated_at"] is not None else "WAITING",
                            "normal": False,
                            "action": "NONE",
                            "excess": 0.0,
                            "message": "Sensor data is stale" if status["updated_at"] is not None else "No reading received",
                        }
                        status["humidity_state"] = condition["state"]
                        status["temperature_state"] = condition["state"]
                        status["temperature_normal"] = False
                    else:
                        humidity_condition = classify_humidity(
                            status["humidity"], status.get("violation_mode")
                        )
                        temperature_condition = classify_temperature(status["temperature"])
                        condition = dict(humidity_condition)
                        status["humidity_state"] = humidity_condition["state"]
                        status["temperature_state"] = temperature_condition["state"]
                        status["temperature_normal"] = temperature_condition["normal"]
                        if not temperature_condition["normal"]:
                            temperature_alert_count += 1
                            if humidity_condition["normal"]:
                                condition.update(
                                    {
                                        "state": temperature_condition["state"],
                                        "normal": False,
                                        "message": temperature_condition["message"],
                                    }
                                )
                    status.update(condition)
                mission["temperature_alert_count"] = temperature_alert_count
                if not mission["candidates"] and temperature_alert_count:
                    mission["system_status"] = "WARNING"
                cursor.execute(
                    f"""
                    SELECT received_at, zone_id, temperature, humidity
                    FROM (
                      SELECT id, received_at, zone_id, temperature, humidity
                      FROM reading_log
                      WHERE zone_id IN ({zone_placeholders})
                      ORDER BY id DESC LIMIT 120
                    ) recent
                    ORDER BY received_at, id
                    """,
                    DEFAULT_ZONE_IDS,
                )
                history = cursor.fetchall()
                cursor.execute(
                    """
                    SELECT id, received_at, source, event_type, message, data_json
                    FROM event_log ORDER BY id DESC LIMIT 30
                    """
                )
                events = cursor.fetchall()
                for event in events:
                    try:
                        event["data"] = json.loads(event.pop("data_json") or "{}")
                    except json.JSONDecodeError:
                        event["data"] = {}
                cursor.execute(
                    f"SELECT * FROM device_status "
                    f"WHERE device_type <> 'HUMIDITY_SENSOR' "
                    f"OR device_id IN ({zone_placeholders}) ORDER BY device_id",
                    DEFAULT_ZONE_IDS,
                )
                devices = cursor.fetchall()
                for device in devices:
                    try:
                        device["details"] = json.loads(device.pop("details_json") or "{}")
                    except json.JSONDecodeError:
                        device["details"] = {}
                    device["online"] = timestamp - device["last_seen"] <= 15
                    device["status"] = "ONLINE" if device["online"] else "OFFLINE"
            connection.commit()
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()
    robot_state = robot_network_snapshot(timestamp)
    effective = effective_command_snapshot(mission, manual)
    serial_state = serial_snapshot(timestamp)
    apply_gateway_health_to_devices(devices, serial_state)
    return {
        "thresholds": current_thresholds(),
        "zones": zones,
        "mission": mission,
        "history": history,
        "events": events,
        "devices": devices,
        "serial": serial_state,
        "robot_network": robot_state,
        "manual_control": manual,
        "effective_command": effective,
        "command_delivery": command_delivery_snapshot(effective, robot_state),
        "server_time": timestamp,
    }


def effective_command_snapshot(
    mission: dict[str, Any], manual: dict[str, Any]
) -> dict[str, Any]:
    source = manual if manual["enabled"] else mission
    snapshot = {
        key: source[key]
        for key in ("revision", "command", "target_zone", "action", "updated_at")
    } | {"source": "MANUAL" if manual["enabled"] else "AUTO"}
    snapshot["waiting_for_new_reading_count"] = (
        0
        if manual["enabled"]
        else int(mission.get("waiting_for_new_reading_count") or 0)
    )
    return snapshot


def command_delivery_snapshot(
    command: dict[str, Any], robot_state: dict[str, Any]
) -> dict[str, Any]:
    revision = int(command["revision"])
    ack_revision = robot_state.get("ack_revision")
    delivered_revision = robot_state.get("delivered_revision")
    result = str(robot_state.get("ack_result") or "").upper()

    persisted_waiting_for_reading = bool(
        command.get("source") == "AUTO"
        and command.get("command") == "ALL_STOP"
        and int(command.get("waiting_for_new_reading_count") or 0) > 0
    )

    if persisted_waiting_for_reading:
        state = "WAITING_READING"
        message = (
            "자동 안전 정지 · 새 구역 측정 대기 · "
            f"{int(command['waiting_for_new_reading_count'])}개 구역 · rev {revision}"
        )
    elif ack_revision is not None and int(ack_revision) == revision:
        state = ACK_RESULT_STATES.get(result, "ACK_FAILED")
        waiting_fresh_reading = (
            state == "ACK_COMPLETED"
            and command.get("source") == "AUTO"
            and command.get("command") == "TASK"
            and command.get("action") in {"HUMIDIFY", "DEHUMIDIFY"}
            and robot_state.get("completed_auto_revision") == revision
        )
        if waiting_fresh_reading:
            state = "WAITING_READING"
            message = f"모듈 1회 완료 · 새 구역 측정 대기 · rev {revision}"
        elif state == "ACK_COMPLETED":
            message = f"로봇 실행 완료 · rev {revision} · {result}"
        elif state == "ACK_EXECUTING":
            message = f"로봇 명령 수신/실행 중 · rev {revision} · {result or 'ACK'}"
        else:
            message = f"로봇 실행 실패 확인 · rev {revision} · {result or 'UNKNOWN'}"
    elif delivered_revision is not None and int(delivered_revision) == revision:
        state = "DELIVERED"
        message = f"로봇에 전송됨 · rev {revision} · 실행 확인 대기"
    else:
        state = "REGISTERED"
        message = f"서버에 등록됨 · rev {revision} · 로봇 폴링 대기"

    return {
        "state": state,
        "message": message,
        "revision": revision,
        "delivered_revision": delivered_revision,
        "delivered_at": robot_state.get("delivered_at"),
        "ack_revision": ack_revision,
        "ack_result": robot_state.get("ack_result"),
        "acknowledged_at": robot_state.get("acknowledged_at"),
        "robot_online": bool(robot_state.get("online")),
    }


def mission_snapshot() -> dict[str, Any]:
    """로봇 폴링용으로 history/events 없이 최신 자동 임무만 계산한다."""
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                mission = recompute_mission(cursor)
            connection.commit()
            return mission
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()


def robot_command_snapshot() -> dict[str, Any]:
    """DB의 한 트랜잭션에서 현재 유효 명령과 출처를 함께 확정한다.

    과거 구현은 mission DB 조회와 MANUAL_CONTROL 메모리 조회 사이에 모드가
    바뀌면 서로 다른 시점의 값을 조합할 수 있었다. mission 계산과 영구
    manual 행 선택을 같은 DB_LOCK/트랜잭션에서 처리해 출처 전환 race를
    제거한다.
    """
    with DB_LOCK:
        connection = connect()
        try:
            with connection.cursor() as cursor:
                mission = recompute_mission(cursor)
                cursor.execute("SELECT * FROM manual_control WHERE id = 1")
                manual = cursor.fetchone()
                if not manual:
                    raise RuntimeError("manual_control row id=1 is missing")
                source_name = "MANUAL" if bool(manual["enabled"]) else "AUTO"
                source = manual if source_name == "MANUAL" else mission
                command = {
                    key: source[key]
                    for key in ("revision", "command", "target_zone", "action")
                } | {"source": source_name}
            connection.commit()
            return command
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()


def robot_command() -> dict[str, Any]:
    # wire API는 기존 SensorUno 파서와 호환되는 정확히 네 필드만 유지한다.
    snapshot = robot_command_snapshot()
    return {
        key: snapshot[key]
        for key in ("revision", "command", "target_zone", "action")
    }


DASHBOARD_HTML = """<!doctype html>
<html lang='ko'><meta charset='utf-8'><title>이동형 제습/가습 로봇 관제</title>
<style>
body{font-family:system-ui,sans-serif;max-width:900px;margin:32px auto;padding:0 16px;background:#f8fafc;color:#0f172a}
h1{margin-bottom:6px}.card{background:#fff;border:1px solid #e2e8f0;border-radius:12px;padding:18px;margin:14px 0}
table{width:100%;border-collapse:collapse}th,td{padding:10px;border-bottom:1px solid #e2e8f0;text-align:left}.bad{color:#dc2626;font-weight:700}.ok{color:#16a34a;font-weight:700}.stale{color:#a16207;font-weight:700}code{background:#eef2ff;padding:2px 5px;border-radius:4px}
</style><body><h1>이동형 제습/가습 로봇 관제</h1><div id='app'>불러오는 중…</div>
<script>
function stamp(v){return v?new Date(v*1000).toLocaleTimeString():'수신 대기'}
async function refresh(){
 const d=await fetch('/api/dashboard').then(r=>r.json());
 const labels={NORMAL:'정상',ERROR_LOW:'저습도 이상',ERROR_HIGH:'고습도 이상',STALE:'데이터 끊김',WAITING:'수신 대기'};
 const m=d.mission; const rows=d.zones.map(z=>{let css=z.state==='NORMAL'?'ok':(z.state==='STALE'||z.state==='WAITING')?'stale':'bad';return `<tr><td>${z.zone_id}</td><td>${z.temperature??'-'} °C</td><td>${z.humidity??'-'} %</td><td class='${css}'>${labels[z.state]||z.state}</td><td>${stamp(z.updated_at)}</td></tr>`}).join('');
 document.querySelector('#app').innerHTML=`<div class='card'><b>전체 상태:</b> <code>${m.system_status}</code><br><b>자동차 명령:</b> <code>${m.command}</code> → <b>${m.target_zone}</b> / ${m.action}<br><small>rev ${m.revision} · ${m.reason}</small></div><div class='card'><b>임계값:</b> ${d.thresholds.low}% 미만 가습 · ${d.thresholds.high}% 초과 제습<table><thead><tr><th>구역</th><th>온도</th><th>습도</th><th>상태</th><th>마지막 수신</th></tr></thead><tbody>${rows}</tbody></table></div>`;
}
refresh();setInterval(refresh,2000);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    # ESP-01 자동차 코드와 일반 브라우저가 동일한 HTTP/1.1 응답을 받게 한다.
    protocol_version = "HTTP/1.1"

    def setup(self) -> None:
        super().setup()
        self.connection.settimeout(REQUEST_TIMEOUT_SECONDS)

    def send_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        # Compact JSON also keeps the tiny manual parser in robot_controller.ino simple.
        data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), default=json_default
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_database_unavailable(self) -> None:
        """DB 내부 정보는 숨기고 클라이언트에 재시도 가능한 실패를 알린다."""
        self.send_json(
            HTTPStatus.SERVICE_UNAVAILABLE,
            {"error": "database unavailable", "retryable": True},
        )

    def send_html_file(self, path: Path) -> None:
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/":
            self.send_html_file(DASHBOARD_PATH)
        elif path in ("/app", "/app/"):
            self.send_html_file(APP_PATH)
        elif path in ("/logic", "/logic/"):
            self.send_html_file(SYSTEM_LOGIC_PATH)
        elif path == "/api/dashboard":
            try:
                payload = dashboard()
            except pymysql.MySQLError:
                self.send_database_unavailable()
                return
            self.send_json(HTTPStatus.OK, payload)
        elif path in ("/health", "/api/health"):
            self.send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "build": SERVER_BUILD_ID,
                    "server_time": now(),
                    "features": [
                        "fresh-reading-burst-gate",
                        "persistent-reading-id-watermarks",
                        "global-command-revisions",
                        "humidity-hysteresis",
                        "stale-all-stop",
                    ],
                },
            )
        elif path in ("/ready", "/api/ready"):
            ready, error = database_readiness()
            self.send_json(
                HTTPStatus.OK if ready else HTTPStatus.SERVICE_UNAVAILABLE,
                {
                    "ok": ready,
                    "build": SERVER_BUILD_ID,
                    "database": "ready" if ready else "unavailable",
                    "error": error,
                    "server_time": now(),
                },
            )
        elif path == "/api/robot/command":
            try:
                command = robot_command()
            except pymysql.MySQLError:
                self.send_database_unavailable()
                return
            mark_command_delivered(int(command["revision"]))
            self.send_json(HTTPStatus.OK, command)
        elif path == "/api/robot/status":
            try:
                result = report_robot_network(
                    parse_qs(parsed.query, keep_blank_values=False),
                    self.client_address[0],
                )
            except ValueError as error:
                self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
                return
            except pymysql.MySQLError:
                self.send_database_unavailable()
                return
            self.send_json(HTTPStatus.OK, result)
        elif path == "/api/gateway/heartbeat":
            try:
                mark_gateway_heartbeat(self.client_address[0])
                touch_device(
                    "SERVER_GATEWAY_WIFI", "SERVER_GATEWAY",
                    details={"ip": self.client_address[0], "transport": "ESP-01 Wi-Fi"},
                )
            except pymysql.MySQLError:
                self.send_database_unavailable()
                return
            self.send_json(HTTPStatus.OK, {"accepted": True, "device": "SERVER_GATEWAY_WIFI"})
        elif path == "/api/humidity":
            # 기존 Node.js 매뉴얼의 조회 경로를 사용해도 현재 상태를 확인할 수 있게 한다.
            try:
                payload = dashboard()
            except pymysql.MySQLError:
                self.send_database_unavailable()
                return
            self.send_json(HTTPStatus.OK, payload)
        else:
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path not in (
            "/api/readings", "/api/humidity", "/api/events",
            "/api/serial/reconnect", "/api/serial/test-rfid", "/api/settings",
            "/api/control",
        ):
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        client_ip = self.client_address[0]
        if path in REMOTE_CONTROL_PATHS and not control_request_authorized(
            client_ip,
            self.headers.get("Authorization"),
        ):
            self.send_json(
                HTTPStatus.FORBIDDEN,
                {
                    "error": (
                        "remote control is disabled; use localhost or configure "
                        "CONTROL_API_TOKEN and send it as a Bearer token"
                    )
                },
            )
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length < 0:
                raise ValueError("Content-Length must not be negative")
            if length > MAX_REQUEST_BODY_BYTES:
                self.send_json(
                    HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                    {"error": f"request body exceeds {MAX_REQUEST_BODY_BYTES} bytes"},
                )
                return
            raw = self.rfile.read(length).decode("utf-8") if length else "{}"
            payload = json.loads(raw)
            if path == "/api/control":
                control = update_manual_control(payload)
                self.send_json(HTTPStatus.OK, {"accepted": True, "control": control})
                return
            if path == "/api/settings":
                thresholds = update_thresholds(payload)
                self.send_json(HTTPStatus.OK, {"accepted": True, "thresholds": thresholds})
                return
            if path == "/api/serial/test-rfid":
                SERIAL_BRIDGE.send_rfid_test()
                self.send_json(HTTPStatus.OK, {"accepted": True})
                return
            if path == "/api/serial/reconnect":
                port = str(payload.get("port", "")).strip() or None
                SERIAL_BRIDGE.request_reconnect(port)
                self.send_json(HTTPStatus.OK, {"accepted": True, "serial": serial_snapshot()})
                return
            if path == "/api/events":
                source = str(payload.get("source", "EXTERNAL_MODULE")).strip()[:32]
                event_type = str(payload.get("event_type", "MODULE_EVENT")).strip()[:32]
                message = str(payload.get("message", "")).strip()
                device_id = str(payload.get("device_id", source)).strip()[:32]
                if not source or not event_type or not message:
                    raise ValueError("source, event_type and message are required")
                event_data = payload.get("data") if isinstance(payload.get("data"), dict) else {}
                record_event(source, event_type, message, event_data)
                touch_device(
                    device_id or source,
                    str(payload.get("device_type", "EXTERNAL_MODULE"))[:32],
                    details={"ip": self.client_address[0], **event_data},
                )
                self.send_json(HTTPStatus.OK, {"accepted": True})
                return
            # 이전 매뉴얼은 {"zone":"zone2", ...}를 전송합니다.
            # 현재 서버 형식인 {"zone_id":"ZONE2", ...}로 바꿔 같은 DB에 저장합니다.
            if path == "/api/humidity":
                payload["zone_id"] = payload.get("zone_id", payload.get("zone", ""))
            payload["_client_ip"] = self.client_address[0]
            result = record_reading(payload)
        except pymysql.MySQLError:
            self.send_database_unavailable()
            return
        except (AttributeError, RuntimeError, ValueError, json.JSONDecodeError) as error:
            self.send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
            return
        if path == "/api/humidity":
            self.send_json(HTTPStatus.OK, {"status": "ok", "accepted": True, **result})
        else:
            self.send_json(HTTPStatus.OK, {"accepted": True, **result})

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[{time.strftime('%H:%M:%S')}] {self.address_string()} - {fmt % args}")


class LegacyHandler(Handler):
    """Compatibility surface limited to the old humidity-sensor endpoint."""

    def do_GET(self) -> None:  # noqa: N802
        if urlparse(self.path).path not in {
            "/api/humidity", "/health", "/api/health", "/ready", "/api/ready"
        }:
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        super().do_GET()

    def do_POST(self) -> None:  # noqa: N802
        if urlparse(self.path).path != "/api/humidity":
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        super().do_POST()


if __name__ == "__main__":
    if SERIAL_ENABLED and not SERIAL_PORT:
        raise SystemExit("Set SERIAL_PORT when SERIAL_ENABLED=1.")
    initialise_database()
    if SERIAL_ENABLED:
        SERIAL_BRIDGE.start()
    else:
        SERIAL_STATE["error"] = "USB serial bridge disabled (network-only server mode)"
    print(f"Robot controller: http://localhost:{PORT}")
    print(f"MySQL database: {MYSQL_DATABASE} on {MYSQL_HOST}:{MYSQL_PORT}")
    active_thresholds = current_thresholds()
    print(
        "Configured ranges: "
        f"{active_thresholds['temperature_min']:.1f}..{active_thresholds['temperature_max']:.1f} C, "
        f"{active_thresholds['humidity_min']:.1f}..{active_thresholds['humidity_max']:.1f}% RH"
    )
    print("Set each ESP node's server address to this PC's LAN IP, not localhost.")
    print(
        f"USB serial bridge: {'enabled on ' + SERIAL_PORT if SERIAL_ENABLED else 'disabled'}"
    )
    if not is_loopback_address(HOST) and not CONTROL_API_TOKEN:
        print(
            "WARNING: LAN sensor APIs are enabled, but remote dashboard controls "
            "will remain locked because CONTROL_API_TOKEN is empty."
        )
    if LEGACY_ENABLED:
        # 구형 센서에는 humidity 호환 API만 노출한다. 제어 API는 3000 포트로
        # 우회할 수 없다.
        legacy_server = ThreadingHTTPServer((HOST, LEGACY_PORT), LegacyHandler)
        legacy_thread = threading.Thread(target=legacy_server.serve_forever, daemon=True)
        legacy_thread.start()
        print(
            f"Legacy sensor compatibility: http://localhost:{LEGACY_PORT}/api/humidity"
        )
    else:
        print("Legacy sensor compatibility: disabled (set LEGACY_ENABLED=1 to enable)")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
