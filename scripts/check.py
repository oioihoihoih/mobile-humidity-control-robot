#!/usr/bin/env python3
"""Run every offline regression gate from one stable entry point."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(label: str, *args: str) -> None:
    print(f"\n==> {label}", flush=True)
    subprocess.run(args, cwd=ROOT, check=True)


def main() -> int:
    run(
        "server unit tests",
        sys.executable,
        "-m",
        "unittest",
        "discover",
        "-s",
        "server",
        "-p",
        "test_*.py",
        "-v",
    )
    run(
        "three-Uno protocol and closed-loop tests",
        sys.executable,
        "-m",
        "unittest",
        "discover",
        "-s",
        "tests",
        "-p",
        "test_*.py",
        "-v",
    )
    run(
        "SimulIDE circuit and firmware-manifest validation",
        sys.executable,
        "simulide/validate_sim2.py",
    )
    print("\nAll offline checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
