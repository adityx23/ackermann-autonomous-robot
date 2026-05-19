#!/usr/bin/env python3
from __future__ import annotations

import sys
from typing import Any


def _display_name(value: Any) -> str:
    return getattr(value, "name", str(value))


def format_camera_feature(feature: Any) -> str:
    socket = _display_name(getattr(feature, "socket", "unknown"))
    types = ", ".join(_display_name(item) for item in getattr(feature, "supportedTypes", []))
    return f"{socket}: {types or 'no reported sensor types'}"


def main() -> int:
    try:
        import depthai as dai
    except ImportError as exc:
        print(f"DepthAI is not installed: {exc}", file=sys.stderr)
        return 1

    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("No OAK devices detected.", file=sys.stderr)
        return 1

    print(f"Detected {len(devices)} OAK device(s):")
    for index, info in enumerate(devices, start=1):
        mx_id = info.getMxId() if hasattr(info, "getMxId") else "unknown"
        name = getattr(info, "name", "unknown")
        print(f"  {index}. name={name} mx_id={mx_id}")

    with dai.Device(devices[0]) as device:
        print("\nConnected OAK device:")
        print(f"  name: {device.getDeviceName()}")
        print(f"  mx_id: {device.getMxId()}")
        print(f"  usb_speed: {_display_name(device.getUsbSpeed())}")
        print("  camera_features:")
        for feature in device.getConnectedCameraFeatures():
            print(f"    - {format_camera_feature(feature)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
