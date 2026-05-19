#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

FRAME_START = 0x7B
FRAME_END = 0x7D


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a passive C30D binary capture.")
    parser.add_argument("capture", type=Path, help="Path to a .bin capture file.")
    return parser


def first_bytes_hex(data: bytes, limit: int = 128) -> str:
    return data[:limit].hex(" ")


def count_frame_markers(data: bytes) -> tuple[int, int]:
    return data.count(FRAME_START), data.count(FRAME_END)


def extract_candidate_frames(data: bytes) -> list[bytes]:
    frames: list[bytes] = []
    search_from = 0
    while True:
        start = data.find(bytes([FRAME_START]), search_from)
        if start < 0:
            return frames
        end = data.find(bytes([FRAME_END]), start + 1)
        if end < 0:
            return frames
        frames.append(data[start : end + 1])
        search_from = end + 1


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    data = args.capture.read_bytes()
    start_count, end_count = count_frame_markers(data)
    frames = extract_candidate_frames(data)

    print(f"total_bytes: {len(data)}")
    print(f"first_128_bytes_hex: {first_bytes_hex(data)}")
    print(f"0x7B_count: {start_count}")
    print(f"0x7D_count: {end_count}")
    print(f"candidate_frame_count: {len(frames)}")
    for index, frame in enumerate(frames[:20], start=1):
        print(f"candidate_frame_{index}: offset_unknown length={len(frame)} hex={frame.hex(' ')}")
    if len(frames) > 20:
        print(f"candidate_frames_omitted: {len(frames) - 20}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
