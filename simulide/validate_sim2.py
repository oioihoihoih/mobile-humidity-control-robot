"""Validate repository SimulIDE 2 circuit files without opening the GUI.

This catches the formatting rules used by SimulIDE R260501 in addition to
ordinary XML errors.  It deliberately validates only static structure; the
firmware must still be compiled and the circuit must be run in SimulIDE.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


EXPECTED_UNOS = {"Uno-1", "Uno-2", "Uno-3"}
EXPECTED_PROGRAMS = {
    "Uno-1": "firmware/sensor_uno_3uno_proxy/sensor_uno_3uno_proxy.ino.hex",
    "Uno-2": "firmware/motor_uno_i2c_proxy/motor_uno_i2c_proxy.ino.hex",
    "Uno-3": "firmware/actuator_uno_i2c_proxy/actuator_uno_i2c_proxy.ino.hex",
}
EXPECTED_SKETCHES = {
    "Uno-1": "firmware/sensor_uno_3uno_proxy/sensor_uno_3uno_proxy.ino",
    "Uno-2": "firmware/motor_uno_i2c_proxy/motor_uno_i2c_proxy.ino",
    "Uno-3": "firmware/actuator_uno_i2c_proxy/actuator_uno_i2c_proxy.ino",
}
BUILD_MANIFEST = "firmware/build-manifest.json"

LCD_EXPANDER_ID = "I2CToParallel-4"
LCD_ID = "Hd44780-5"
CAR_DHT_ID = "Dht22-Car"

# SimulIDE 전용 출력 핀이다. 실제 AFMotor shield 핀맵이 아니라 M1~M4 각
# 채널이 FORWARD/REVERSE 중 어느 논리 상태인지 독립적으로 보여준다.
MOTOR_LED_PIN_MAP = {
    "Led-Motor-M1-F": "2",
    "Led-Motor-M1-R": "3",
    "Led-Motor-M2-F": "4",
    "Led-Motor-M2-R": "5",
    "Led-Motor-M3-F": "6",
    "Led-Motor-M3-R": "7",
    "Led-Motor-M4-F": "8",
    "Led-Motor-M4-R": "11",
}


class DisjointSet:
    def __init__(self) -> None:
        self.parent: dict[str, str] = {}

    def find(self, value: str) -> str:
        self.parent.setdefault(value, value)
        if self.parent[value] != value:
            self.parent[value] = self.find(self.parent[value])
        return self.parent[value]

    def union(self, left: str, right: str) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def fail(errors: list[str], message: str) -> None:
    errors.append(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(64 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def component_for_pin(pin: str, component_ids: set[str]) -> str | None:
    for component_id in sorted(component_ids, key=len, reverse=True):
        if pin.startswith(component_id + "-"):
            return component_id
    return None


def validate(path: Path) -> list[str]:
    errors: list[str] = []
    text = path.read_text(encoding="utf-8")

    manifest_path = path.parent / BUILD_MANIFEST
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        manifest = {}
        fail(errors, f"firmware build manifest is missing: {BUILD_MANIFEST}")
    except json.JSONDecodeError as exc:
        manifest = {}
        fail(errors, f"invalid firmware build manifest: {exc}")

    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        return [f"XML parse error: {exc}"]

    if root.tag != "circuit":
        fail(errors, f"root must be <circuit>, found <{root.tag}>")
    if root.attrib.get("version") != "2.0.0-":
        fail(errors, "circuit version must match SimulIDE 2.0.0 format")

    # R260501 scans line by line. Every item must begin at column zero and
    # occupy one complete line; otherwise components can silently disappear.
    item_lines = [line for line in text.splitlines() if "<item " in line]
    for line_number, line in enumerate(text.splitlines(), start=1):
        if "<item " not in line:
            continue
        if not line.startswith("<item "):
            fail(errors, f"line {line_number}: <item> must start at column zero")
        if not line.rstrip().endswith("/>") and not line.rstrip().endswith(">"):
            fail(errors, f"line {line_number}: item must be complete on one line")
        if not re.match(r'<item itemtype="[^"]+" (?:CircId|uid)="[^"]+"', line):
            fail(errors, f"line {line_number}: itemtype and CircId/uid attribute order is invalid")

    items = list(root.findall("item"))
    if len(item_lines) != len(items):
        fail(errors, "the number of line-scanned items differs from XML items")

    components = [item for item in items if item.attrib.get("itemtype") != "Connector"]
    connectors = [item for item in items if item.attrib.get("itemtype") == "Connector"]
    first_connector = next(
        (index for index, item in enumerate(items) if item.attrib.get("itemtype") == "Connector"),
        len(items),
    )
    if any(item.attrib.get("itemtype") != "Connector" for item in items[first_connector:]):
        fail(errors, "all components and Nodes must be declared before Connectors")

    component_ids = [item.attrib.get("CircId", "") for item in components]
    connector_ids = [item.attrib.get("uid", "") for item in connectors]
    if "" in component_ids:
        fail(errors, "a component is missing CircId")
    if "" in connector_ids:
        fail(errors, "a Connector is missing uid")
    if len(component_ids) != len(set(component_ids)):
        fail(errors, "duplicate component CircId")
    if len(connector_ids) != len(set(connector_ids)):
        fail(errors, "duplicate Connector uid")

    component_id_set = set(component_ids)
    component_by_id = {
        item.attrib.get("CircId", ""): item for item in components
    }
    subcircuits = [item for item in components if item.attrib.get("itemtype") == "Subcircuit"]
    uno_ids = {item.attrib.get("CircId", "") for item in subcircuits}
    if uno_ids != EXPECTED_UNOS:
        fail(errors, f"expected Uno-1/Uno-2/Uno-3, found {sorted(uno_ids)}")
    for uno_id in uno_ids:
        if not re.fullmatch(r"Uno-\d+", uno_id):
            fail(errors, f"{uno_id}: SimulIDE device token must remain 'Uno'")

    used_pins: set[str] = set()
    network = DisjointSet()
    for connector in connectors:
        start = connector.attrib.get("startpinid", "")
        end = connector.attrib.get("endpinid", "")
        for endpoint in (start, end):
            if not endpoint:
                fail(errors, f"{connector.attrib.get('uid')}: missing endpoint")
                continue
            owner = component_for_pin(endpoint, component_id_set)
            if owner is None:
                fail(errors, f"{connector.attrib.get('uid')}: unknown endpoint {endpoint}")
            if endpoint in used_pins:
                fail(errors, f"pin reused without Node branch: {endpoint}")
            used_pins.add(endpoint)
        if start and end:
            network.union(start, end)

        points = connector.attrib.get("pointList", "").split(",")
        if len(points) < 4 or len(points) % 2:
            fail(errors, f"{connector.attrib.get('uid')}: invalid pointList")

    # The three ports of one Node represent a single electrical net.
    for component in components:
        if component.attrib.get("itemtype") != "Node":
            continue
        node_id = component.attrib["CircId"]
        ports = [f"{node_id}-{number}" for number in range(3)]
        network.union(ports[0], ports[1])
        network.union(ports[1], ports[2])

    for bus_pin in ("A4", "A5"):
        pins = [f"Uno-{number}-{bus_pin}" for number in (1, 2, 3)]
        if not all(pin in used_pins for pin in pins):
            fail(errors, f"I2C {bus_pin} is not connected to all three Uno boards")
        elif len({network.find(pin) for pin in pins}) != 1:
            fail(errors, f"I2C {bus_pin} boards are not on one Node net")
    if network.find("Uno-1-A4") == network.find("Uno-1-A5"):
        fail(errors, "SDA and SCL are shorted")

    # 차량 DHT22는 ActuatorUno가 D2에서 직접 읽는다. SensorUno D4에
    # 남아 있으면 자원 분배 목적과 LCD 로컬 측정 계약을 모두 위반한다.
    if CAR_DHT_ID not in component_by_id:
        fail(errors, f"car DHT22 component is missing: {CAR_DHT_ID}")
    else:
        dht_pin = f"{CAR_DHT_ID}-inPin"
        actuator_pin = "Uno-3-2"
        if dht_pin not in used_pins or actuator_pin not in used_pins:
            fail(errors, "car DHT22 must be connected to ActuatorUno D2")
        elif network.find(dht_pin) != network.find(actuator_pin):
            fail(errors, "car DHT22 is not connected to ActuatorUno D2")
        if "Uno-1-4" in used_pins:
            fail(errors, "SensorUno D4 must remain free after moving DHT22")

    # ActuatorUno의 LCD는 별도 software-I2C(D5/D4) 버스를 사용한다. 이 버스가
    # 보드 간 통신용 hardware-I2C(A4/A5)와 합쳐지면 두 프로토콜이 충돌한다.
    for component_id in (LCD_EXPANDER_ID, LCD_ID):
        if component_id not in component_by_id:
            fail(errors, f"LCD component is missing: {component_id}")

    expander = component_by_id.get(LCD_EXPANDER_ID)
    if expander is not None and expander.attrib.get("Control_Code") != "39":
        fail(errors, "LCD I2C expander address must be decimal 39 (0x27)")
    lcd = component_by_id.get(LCD_ID)
    if lcd is not None and (
        lcd.attrib.get("Rows") != "2" or lcd.attrib.get("Cols") != "16"
    ):
        fail(errors, "LCD must be configured as 16 columns x 2 rows")

    expected_lcd_nets = (
        ("Uno-3-5", f"{LCD_EXPANDER_ID}-in0", "software-I2C SDA D5"),
        ("Uno-3-4", f"{LCD_EXPANDER_ID}-in1", "software-I2C SCL D4"),
        (f"{LCD_EXPANDER_ID}-out0", f"{LCD_ID}-PinRS", "LCD RS"),
        (f"{LCD_EXPANDER_ID}-out1", f"{LCD_ID}-PinRW", "LCD RW"),
        (f"{LCD_EXPANDER_ID}-out2", f"{LCD_ID}-PinEn", "LCD Enable"),
        (f"{LCD_EXPANDER_ID}-out4", f"{LCD_ID}-dataPin4", "LCD D4"),
        (f"{LCD_EXPANDER_ID}-out5", f"{LCD_ID}-dataPin5", "LCD D5"),
        (f"{LCD_EXPANDER_ID}-out6", f"{LCD_ID}-dataPin6", "LCD D6"),
        (f"{LCD_EXPANDER_ID}-out7", f"{LCD_ID}-dataPin7", "LCD D7"),
    )
    for left, right, description in expected_lcd_nets:
        if left not in used_pins or right not in used_pins:
            fail(errors, f"{description} connection is missing")
        elif network.find(left) != network.find(right):
            fail(errors, f"{description} endpoints are not on one net")

    lcd_sda_root = network.find("Uno-3-5")
    lcd_scl_root = network.find("Uno-3-4")
    if lcd_sda_root == lcd_scl_root:
        fail(errors, "LCD software-I2C SDA D5 and SCL D4 are shorted")
    hardware_bus_roots = {
        network.find("Uno-3-A4"),
        network.find("Uno-3-A5"),
    }
    if lcd_sda_root in hardware_bus_roots or lcd_scl_root in hardware_bus_roots:
        fail(errors, "LCD D4/D5 bus must be isolated from hardware-I2C A4/A5")

    # M1/M3=LEFT, M2/M4=RIGHT인 실제 4채널 구성을 방향별 LED 8개로
    # 표시한다. 구형 좌/우 2채널 묶음이 섞이면 reverse-home 검증이 모호해진다.
    for led_id, pin_number in MOTOR_LED_PIN_MAP.items():
        if led_id not in component_by_id:
            fail(errors, f"4WD motor direction LED is missing: {led_id}")
            continue
        uno_pin = f"Uno-2-{pin_number}"
        led_pin = f"{led_id}-lPin"
        if uno_pin not in used_pins or led_pin not in used_pins:
            fail(errors, f"{led_id}: MotorUno D{pin_number} connection is missing")
        elif network.find(uno_pin) != network.find(led_pin):
            fail(errors, f"{led_id}: not connected to MotorUno D{pin_number}")

    legacy_motor_leds = {
        "Led-Motor-LF", "Led-Motor-RF", "Led-Motor-LR", "Led-Motor-RR"
    }
    if legacy_motor_leds & component_id_set:
        fail(errors, "legacy left/right 2-channel motor proxy LEDs must be removed")

    for address_bit, ground_id in zip(
        ("in2", "in3", "in4"),
        ("Ground-LCD-A0", "Ground-LCD-A1", "Ground-LCD-A2"),
    ):
        address_pin = f"{LCD_EXPANDER_ID}-{address_bit}"
        ground_pin = f"{ground_id}-Gnd"
        if address_pin not in used_pins or ground_pin not in used_pins:
            fail(errors, f"LCD address pin {address_bit} must be tied low")
        elif network.find(address_pin) != network.find(ground_pin):
            fail(errors, f"LCD address pin {address_bit} is not tied low")

    sketch_sources: dict[str, str] = {}
    for subcircuit in subcircuits:
        uno_id = subcircuit.attrib["CircId"]
        props = subcircuit.find("mainCompProps")
        if props is None:
            fail(errors, f"{uno_id}: missing mainCompProps")
            continue
        program = props.attrib.get("Program", "")
        if EXPECTED_PROGRAMS.get(uno_id) != program:
            fail(errors, f"{uno_id}: unexpected Program path {program!r}")
        if "\\" in program:
            fail(errors, f"{uno_id}: Program must use forward slashes")
        hex_path = path.parent / program
        sketch_path = path.parent / EXPECTED_SKETCHES[uno_id]
        if program and not hex_path.is_file():
            fail(errors, f"{uno_id}: HEX file is missing: {program}")
        if not sketch_path.is_file():
            fail(errors, f"{uno_id}: sketch file is missing: {EXPECTED_SKETCHES[uno_id]}")
        else:
            sketch_sources[uno_id] = sketch_path.read_text(encoding="utf-8")
        artifact = manifest.get("artifacts", {}).get(uno_id, {})
        expected_source = Path("firmware") / artifact.get("source", "")
        expected_hex = Path("firmware") / artifact.get("hex", "")
        if expected_source.as_posix() != EXPECTED_SKETCHES[uno_id]:
            fail(errors, f"{uno_id}: build manifest source path does not match the circuit")
        if expected_hex.as_posix() != EXPECTED_PROGRAMS[uno_id]:
            fail(errors, f"{uno_id}: build manifest HEX path does not match the circuit")
        if sketch_path.is_file() and sha256(sketch_path) != artifact.get("source_sha256"):
            fail(errors, f"{uno_id}: sketch changed after the recorded HEX build")
        if hex_path.is_file() and sha256(hex_path) != artifact.get("hex_sha256"):
            fail(errors, f"{uno_id}: HEX hash does not match the build manifest")

    # 프록시도 운영 계약의 핵심 안전값을 따라야 한다. 문자열 검사는 펌웨어
    # 컴파일을 대신하지 않지만 오래된 HEX/프로토콜 파일이 섞이는 실수를 잡는다.
    expected_source_tokens = {
        "Uno-1": (
            "MOTOR_OUTBOUND = 0x11",
            "MOTOR_REVERSE_HOME = 0x12",
            "MOTOR_HOME_SYNC = 6",
            "MOTOR_PROTOCOL_SYNC = 7",
            "MOTOR_CALIBRATION_REQUIRED = 7",
            "MOTOR_PROTOCOL_REQUIRED = 8",
            "bool routeCalibrated = false",
            "ACTUATOR_CONTROL_MAGIC = 0xA5",
            "STATUS_REPLY_SIZE = 6",
        ),
        "Uno-2": (
            "COMMAND_OUTBOUND = 0x11",
            "COMMAND_RETURN = 0x12",
            "COMMAND_HOME_SYNC = 6",
            "COMMAND_PROTOCOL_SYNC = 7",
            "STATUS_CALIBRATION_REQUIRED = 7",
            "STATUS_PROTOCOL_REQUIRED = 8",
            "bool calibrated = false",
            "bool headingHomebound = false",
            "M1_FORWARD_LED = 2",
            "M4_REVERSE_LED = 11",
            "DIRECTION_CHANGE_DEAD_TIME_MS = 120",
            "STATUS_BYTE_SELECT_BASE = 0xE0",
        ),
        "Uno-3": (
            "CONTROL_FRAME_MAGIC = 0xA5",
            "CONTROL_FRAME_SIZE = 4",
            "STATUS_REPLY_SIZE = 6",
            "STATUS_BYTE_SELECT_BASE = 0xF0",
            "DHT_PIN = 2",
            "serviceLocalDht",
        ),
    }
    for uno_id, tokens in expected_source_tokens.items():
        source = sketch_sources.get(uno_id, "")
        for token in tokens:
            if token not in source:
                fail(errors, f"{uno_id}: missing proxy protocol token {token!r}")

    for uno_id in ("Uno-1", "Uno-2"):
        if "TURN_AROUND" in sketch_sources.get(uno_id, ""):
            fail(errors, f"{uno_id}: obsolete TURN_AROUND semantics remain in proxy source")

    sensor_source = sketch_sources.get("Uno-1", "")
    if "#include <DHT.h>" in sensor_source or "DHT_PIN" in sensor_source:
        fail(errors, "Uno-1: SensorUno proxy must not own the car DHT22")

    home_button = component_by_id.get("Push-Home")
    if home_button is None:
        fail(errors, "CALIBRATE_HOME/HOME proxy button is missing")
    else:
        if "CALIBRATE_HOME" not in home_button.attrib.get("label", ""):
            fail(errors, "Push-Home label must expose CALIBRATE_HOME")
        if home_button.attrib.get("Key") != "h":
            fail(errors, "Push-Home keyboard shortcut must remain h")
        home_pin = "Uno-1-13"
        button_pin = "Push-Home-pinP0"
        if home_pin not in used_pins or button_pin not in used_pins:
            fail(errors, "CALIBRATE_HOME/HOME D13 connection is missing")
        elif network.find(home_pin) != network.find(button_pin):
            fail(errors, "CALIBRATE_HOME/HOME button is not connected to SensorUno D13")

    if "ZONE1" in text:
        fail(errors, "current 3-Uno circuit must not contain legacy ZONE1 labels")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "circuit",
        nargs="?",
        type=Path,
        default=Path(__file__).with_name("mobile_humidity_robot_3uno_proxy.sim2"),
    )
    args = parser.parse_args()
    path = args.circuit.resolve()
    errors = validate(path)
    if errors:
        print(f"FAIL: {path}")
        for error in errors:
            print(f"- {error}")
        return 1
    print(f"PASS: {path}")
    print("- XML and R260501 line format valid")
    print("- Uno-1/Uno-2/Uno-3 device IDs valid")
    print("- I2C A4/A5 shared nets valid")
    print("- car DHT22 is connected to ActuatorUno D2; SensorUno D4 is free")
    print("- LCD D5/D4 software-I2C net is isolated and mapped to PCF8574/Hd44780")
    print("- MotorUno M1-M4 forward/reverse LED nets and straight reverse-home semantics valid")
    print("- protocol-v2/status8 and HOME_SYNC/status7 interlocks present")
    print("- CALIBRATE_HOME/HOME proxy input is connected to SensorUno D13")
    print("- connector endpoints unique and known")
    print("- all three firmware HEX/source hashes match the build manifest")
    return 0


if __name__ == "__main__":
    sys.exit(main())
