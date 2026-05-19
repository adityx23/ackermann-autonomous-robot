#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

FRAME_START = 0x7B
FRAME_END = 0x7D
DEFAULT_CAPTURE_DIR = Path("data/c30d_captures")
DEFAULT_FRAME_PRINT_LIMIT = 10


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Inspect a passive C30D binary capture.")
    parser.add_argument("capture", nargs="?", type=Path, help="Path to a .bin capture file.")
    parser.add_argument(
        "--latest",
        action="store_true",
        help=f"Analyze the newest .bin file in {DEFAULT_CAPTURE_DIR}.",
    )
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


def frame_length_distribution(frames: list[bytes]) -> dict[int, int]:
    return dict(sorted(Counter(len(frame) for frame in frames).items()))


def format_length_distribution(distribution: dict[int, int]) -> str:
    if not distribution:
        return "none"
    return ", ".join(f"{length}:{count}" for length, count in distribution.items())


def newest_capture(capture_dir: Path = DEFAULT_CAPTURE_DIR) -> Path:
    captures = [path for path in capture_dir.glob("*.bin") if path.is_file()]
    if not captures:
        raise FileNotFoundError(f"no .bin captures found in {capture_dir}")
    return max(captures, key=lambda path: (path.stat().st_mtime, path.name))


def resolve_capture_path(capture: Path | None, latest: bool) -> Path:
    if latest:
        if capture is not None:
            raise ValueError("provide either a capture path or --latest, not both")
        return newest_capture()
    if capture is None:
        raise ValueError("provide a capture path or use --latest")
    return capture


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        capture_path = resolve_capture_path(args.capture, args.latest)
    except (FileNotFoundError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2

    data = capture_path.read_bytes()
    start_count, end_count = count_frame_markers(data)
    frames = extract_candidate_frames(data)
    distribution = frame_length_distribution(frames)

    print(f"capture_path: {capture_path}")
    print(f"total_bytes: {len(data)}")
    print(f"first_128_bytes_hex: {first_bytes_hex(data)}")
    print(f"0x7B_count: {start_count}")
    print(f"0x7D_count: {end_count}")
    print(f"candidate_frame_count: {len(frames)}")
    print(f"frame_length_distribution: {format_length_distribution(distribution)}")
    for index, frame in enumerate(frames[:DEFAULT_FRAME_PRINT_LIMIT], start=1):
        print(f"candidate_frame_{index}: offset_unknown length={len(frame)} hex={frame.hex(' ')}")
    if len(frames) > DEFAULT_FRAME_PRINT_LIMIT:
        print(f"candidate_frames_omitted: {len(frames) - DEFAULT_FRAME_PRINT_LIMIT}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
