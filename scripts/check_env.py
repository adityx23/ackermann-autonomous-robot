#!/usr/bin/env python3

import importlib
import sys

REQUIRED = [
    "numpy",
    "scipy",
    "cv2",
    "depthai",
    "serial",
    "smbus2",
    "can",
    "zmq",
    "fastapi",
    "psutil",
    "yaml",
]

failed = []

for module in REQUIRED:
    try:
        importlib.import_module(module)
        print(f"[OK] {module}")
    except Exception as exc:
        failed.append((module, exc))
        print(f"[FAIL] {module}: {exc}")

if failed:
    print("\nSome imports failed.")
    sys.exit(1)

print("\nRobot Python environment looks ready.")
