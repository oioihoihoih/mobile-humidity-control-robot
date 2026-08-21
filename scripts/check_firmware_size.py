#!/usr/bin/env python3
"""Fail CI before SensorUno exhausts the Arduino Uno's flash or SRAM."""

from __future__ import annotations

import json
import sys
from pathlib import Path


SENSOR_SKETCH = "uno_robot_esp01_rfid_relay"
LIMITS = {
    "flash": 32_100,
    "RAM for global variables": 1_600,
}


def load_reports(report_dir: Path) -> list[dict]:
    reports: list[dict] = []
    for report_path in sorted(report_dir.glob("*.json")):
        with report_path.open(encoding="utf-8") as report_file:
            reports.append(json.load(report_file))
    if not reports:
        raise RuntimeError(f"no Arduino size report found in {report_dir}")
    return reports


def main() -> int:
    report_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "sketches-reports")
    matches: list[dict] = []
    for report in load_reports(report_dir):
        for board in report.get("boards", []):
            for sketch in board.get("sketches", []):
                if SENSOR_SKETCH in sketch.get("name", ""):
                    matches.append(sketch)

    if len(matches) != 1:
        raise RuntimeError(
            f"expected one {SENSOR_SKETCH} size record, found {len(matches)}"
        )

    size_by_name = {entry["name"]: entry for entry in matches[0].get("sizes", [])}
    failed = False
    for size_name, limit in LIMITS.items():
        if size_name not in size_by_name:
            raise RuntimeError(f"missing size metric: {size_name}")
        used = int(size_by_name[size_name]["current"]["absolute"])
        maximum = int(size_by_name[size_name]["maximum"])
        print(f"SensorUno {size_name}: {used}/{maximum} bytes (budget {limit})")
        if used > limit:
            print(f"ERROR: {size_name} exceeds the project budget by {used - limit} bytes")
            failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
