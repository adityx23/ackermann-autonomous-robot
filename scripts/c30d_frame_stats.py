#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_CAPTURE_DIR = Path("data/c30d_captures")
DEFAULT_FRAME_PRINT_LIMIT = 20


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze candidate C30D frames from a saved passive .bin capture."
    )
    parser.add_argument("capture", nargs="?", type=Path, help="Path to a saved .bin capture file.")
    parser.add_argument(
        "--latest",
        action="store_true",
        help=f"Analyze the newest .bin file in {DEFAULT_CAPTURE_DIR}.",
    )
    return parser


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


def format_distribution(distribution: dict[int, int]) -> str:
    if not distribution:
        return "none"
    return ", ".join(f"{length}:{count}" for length, count in distribution.items())


def format_constant_positions(positions: dict[int, int]) -> str:
    if not positions:
        return "none"
    return ", ".join(f"{position}=0x{value:02x}" for position, value in positions.items())


def format_changing_positions(positions: list[int]) -> str:
    if not positions:
        return "none"
    return ", ".join(str(position) for position in positions)


def print_frame_stats(capture_path: Path, data: bytes) -> None:
    from ackermann_robot.drivers.c30d_frames import extract_frames, summarize_frames

    frames = extract_frames(data)
    summary = summarize_frames(frames)

    print("Read-only C30D frame analysis from saved capture only.")
    print(f"capture_path: {capture_path}")
    print(f"total_bytes: {len(data)}")
    print(f"frame_count: {summary['frame_count']}")
    length_distribution = format_distribution(summary["frame_length_distribution"])
    print(f"frame_length_distribution: {length_distribution}")

    for index, frame in enumerate(frames[:DEFAULT_FRAME_PRINT_LIMIT], start=1):
        print(f"frame_{index}: length={len(frame)} hex={frame.hex(' ')}")
    if len(frames) > DEFAULT_FRAME_PRINT_LIMIT:
        print(f"frames_omitted: {len(frames) - DEFAULT_FRAME_PRINT_LIMIT}")

    print(
        "constant_byte_positions: "
        f"{format_constant_positions(summary['constant_byte_positions'])}"
    )
    print(
        "changing_byte_positions: "
        f"{format_changing_positions(summary['changing_byte_positions'])}"
    )

    repeated = summary["repeated_frame_patterns"]
    if not repeated:
        print("repeated_frame_patterns: none")
        return

    print("repeated_frame_patterns:")
    for frame_hex, count in repeated.items():
        print(f"  count={count} hex={frame_hex}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        capture_path = resolve_capture_path(args.capture, args.latest)
    except (FileNotFoundError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2

    try:
        data = capture_path.read_bytes()
    except OSError as exc:
        print(f"failed to read {capture_path}: {exc}", file=sys.stderr)
        return 1

    print_frame_stats(capture_path, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
