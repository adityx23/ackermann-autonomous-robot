#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai

DEFAULT_OUTPUT_DIR = Path("data/oak_tests")


def default_output_stem(dt: datetime | None = None) -> Path:
    timestamp = (dt or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return DEFAULT_OUTPUT_DIR / f"oak_capture_{timestamp}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DepthAI v3 RGB-only OAK-D Lite capture.")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--preview-height", type=int, default=360)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--force-usb2", action="store_true", default=True)
    parser.add_argument("--rgb-only", action="store_true", default=True)
    parser.add_argument("--with-depth", action="store_true", default=False)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.with_depth:
        print("Depth is intentionally disabled in this first stable v3 test.")
        print("Confirm RGB-only capture first, then add depth separately.")

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Starting OAK-D Lite RGB-only capture using DepthAI v3 API...")
    print(f"Requested output: {args.preview_width}x{args.preview_height} @ {args.fps} FPS")

    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

        rgb_output = cam.requestOutput(
            size=(args.preview_width, args.preview_height),
            type=dai.ImgFrame.Type.BGR888p,
            fps=args.fps,
        )

        rgb_queue = rgb_output.createOutputQueue(maxSize=1, blocking=False)

        pipeline.start()

        print("Pipeline started.")
        print("Note: USB speed info may not be available in the same way in DepthAI v3.")

        frame = None
        deadline = time.time() + args.timeout

        while time.time() < deadline and pipeline.isRunning():
            msg = rgb_queue.tryGet()
            if msg is not None:
                frame = msg.getCvFrame()
                break
            time.sleep(0.05)

        if frame is None:
            print(f"No RGB frame received within {args.timeout}s.")
            return 1

        default_stem = default_output_stem()
        output_stem = out_dir / default_stem.name
        out_path = output_stem.with_name(f"{output_stem.name}_rgb.jpg")

        if not cv2.imwrite(str(out_path), frame):
            print(f"Failed to save image: {out_path}")
            return 1

        print("RGB frame shape:", frame.shape)
        print("Saved RGB frame:", out_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
