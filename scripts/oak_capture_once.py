#!/usr/bin/env python3
from __future__ import annotations

import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

DEFAULT_OUTPUT_DIR = Path("data/oak_tests")
DEFAULT_TIMEOUT_S = 5.0


def _display_name(value: Any) -> str:
    return getattr(value, "name", str(value))


def default_output_stem(now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_DIR / f"oak_capture_{timestamp}"


def create_pipeline(dai: Any, pipeline: Any | None = None) -> tuple[Any, Any, Any]:
    pipeline = pipeline or dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(1280, 720)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    return pipeline, cam_rgb.preview, stereo.depth


def wait_for_frame(queue: Any, timeout_s: float) -> Any | None:
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        frame = queue.tryGet()
        if frame is not None:
            return frame
        time.sleep(0.02)
    return None


def capture_once(output_stem: Path, timeout_s: float = DEFAULT_TIMEOUT_S) -> dict[str, Any]:
    import cv2
    import depthai as dai
    import numpy as np

    output_stem.parent.mkdir(parents=True, exist_ok=True)
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        raise RuntimeError("no OAK devices detected")

    output_paths: dict[str, Path] = {}
    shapes: dict[str, tuple[int, ...]] = {}

    with dai.Pipeline(dai.Device(devices[0])) as pipeline:
        _, rgb_output, depth_output = create_pipeline(dai, pipeline)
        device = pipeline.getDefaultDevice()
        rgb_queue = rgb_output.createOutputQueue(maxSize=1, blocking=False)
        depth_queue = depth_output.createOutputQueue(maxSize=1, blocking=False)

        pipeline.start()

        rgb_packet = wait_for_frame(rgb_queue, timeout_s)
        depth_packet = wait_for_frame(depth_queue, timeout_s)
        pipeline.stop()

        if rgb_packet is None:
            raise TimeoutError(f"no RGB frame received within {timeout_s:.1f}s")

        rgb_frame = rgb_packet.getCvFrame()
        rgb_path = output_stem.with_name(f"{output_stem.name}_rgb.png")
        if not cv2.imwrite(str(rgb_path), rgb_frame):
            raise RuntimeError(f"failed to write {rgb_path}")
        output_paths["rgb"] = rgb_path
        shapes["rgb"] = tuple(rgb_frame.shape)

        if depth_packet is not None:
            depth_frame = depth_packet.getFrame()
            depth_path = output_stem.with_name(f"{output_stem.name}_depth_mm.npy")
            np.save(depth_path, depth_frame)
            output_paths["depth"] = depth_path
            shapes["depth"] = tuple(depth_frame.shape)

        return {
            "device_name": device.getDeviceName(),
            "mx_id": device.getMxId(),
            "usb_speed": _display_name(device.getUsbSpeed()),
            "shapes": shapes,
            "output_paths": output_paths,
        }


def main() -> int:
    output_stem = default_output_stem()
    try:
        result = capture_once(output_stem)
    except Exception as exc:
        print(f"OAK-D Lite capture failed: {exc}", file=sys.stderr)
        return 1

    print("OAK-D Lite one-shot capture complete.")
    print(f"device_name: {result['device_name']}")
    print(f"mx_id: {result['mx_id']}")
    print(f"usb_speed: {result['usb_speed']}")
    for name, shape in result["shapes"].items():
        print(f"{name}_shape: {shape}")
    for name, path in result["output_paths"].items():
        print(f"{name}_output: {path}")
    if "depth" not in result["output_paths"]:
        print("depth_output: unavailable within capture timeout")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
