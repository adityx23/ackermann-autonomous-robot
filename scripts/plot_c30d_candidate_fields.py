#!/usr/bin/env python3

import argparse
import math
import statistics
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

DEFAULT_OUTPUT_DIR = Path("data/c30d_analysis")
DEFAULT_PAIR_START = 2
DEFAULT_PAIR_END_FIRST_BYTE = 18
PAIR_LAST_BYTE_LIMIT = 19
ENDIAN_CHOICES = ("be", "le", "both")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot read-only C30D candidate int16 fields from saved .bin captures."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive .bin capture files.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for generated plot PNG files.",
    )
    parser.add_argument(
        "--pairs",
        nargs="+",
        default=None,
        help="Optional adjacent byte pairs such as 02_03 06_07 08_09.",
    )
    parser.add_argument(
        "--endian",
        choices=ENDIAN_CHOICES,
        default="both",
        help="Candidate int16 byte order to plot.",
    )
    return parser


def default_pairs() -> list[tuple[int, int]]:
    return [
        (first, first + 1) for first in range(DEFAULT_PAIR_START, DEFAULT_PAIR_END_FIRST_BYTE + 1)
    ]


def parse_pair(pair_text: str) -> tuple[int, int]:
    parts = pair_text.split("_")
    if len(parts) != 2:
        raise ValueError(f"invalid pair {pair_text!r}; expected format like 02_03")
    try:
        first, second = int(parts[0]), int(parts[1])
    except ValueError as exc:
        raise ValueError(f"invalid pair {pair_text!r}; byte positions must be integers") from exc

    if second != first + 1:
        raise ValueError(f"invalid pair {pair_text!r}; only adjacent byte pairs are supported")
    if first < DEFAULT_PAIR_START or second > PAIR_LAST_BYTE_LIMIT:
        raise ValueError(
            f"invalid pair {pair_text!r}; supported candidate pairs range from 02_03 to 18_19"
        )
    return first, second


def resolve_pairs(pair_texts: list[str] | None) -> list[tuple[int, int]]:
    if pair_texts is None:
        return default_pairs()
    return [parse_pair(pair_text) for pair_text in pair_texts]


def resolve_endians(endian: str) -> list[str]:
    if endian == "both":
        return ["be", "le"]
    return [endian]


def candidate_name(pair: tuple[int, int], endian: str) -> str:
    return f"candidate_int16_{endian}_{pair[0]:02d}_{pair[1]:02d}"


def int16_byte_order(endian: str) -> str:
    if endian == "be":
        return "big"
    if endian == "le":
        return "little"
    raise ValueError(f"unsupported endian {endian!r}")


def decode_candidate_values(frames: list[bytes], pair: tuple[int, int], endian: str) -> list[int]:
    first, second = pair
    return [
        int.from_bytes(frame[first : second + 1], int16_byte_order(endian), signed=True)
        for frame in frames
    ]


def basic_stats(values: list[int]) -> dict[str, float | int]:
    if not values:
        return {"count": 0, "min": math.nan, "max": math.nan, "mean": math.nan, "stdev": math.nan}
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": statistics.fmean(values),
        "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


def output_path_for(capture_path: Path, output_dir: Path, endian: str) -> Path:
    return output_dir / f"{capture_path.stem}_candidate_int16_{endian}.png"


def load_frames(capture_path: Path) -> tuple[list[bytes], int, int]:
    from ackermann_robot.drivers.c30d_frames import extract_frames_with_stats

    extraction = extract_frames_with_stats(capture_path.read_bytes())
    return extraction.frames, extraction.rejected_resync_count, extraction.partial_frame_count


def save_candidate_plot(
    capture_path: Path,
    frames: list[bytes],
    pairs: list[tuple[int, int]],
    endian: str,
    output_dir: Path,
) -> tuple[Path, dict[str, dict[str, float | int]]]:
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_path_for(capture_path, output_dir, endian)
    stats_by_name: dict[str, dict[str, float | int]] = {}

    fig, ax = plt.subplots(figsize=(11, 6))
    frame_indexes = list(range(len(frames)))
    for pair in pairs:
        name = candidate_name(pair, endian)
        values = decode_candidate_values(frames, pair, endian)
        stats_by_name[name] = basic_stats(values)
        ax.plot(frame_indexes, values, linewidth=1.0, label=name)

    ax.set_title(f"{capture_path.name} C30D candidate int16 fields ({endian})")
    ax.set_xlabel("frame_index")
    ax.set_ylabel("candidate_int16_value")
    ax.grid(True, alpha=0.3)
    if pairs:
        ax.legend(loc="best", fontsize="x-small", ncols=2)
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return output_path, stats_by_name


def print_stats(stats_by_name: dict[str, dict[str, float | int]]) -> None:
    for name, stats in stats_by_name.items():
        print(
            f"  {name}: count={stats['count']} min={stats['min']:.0f} "
            f"max={stats['max']:.0f} mean={stats['mean']:.2f} stdev={stats['stdev']:.2f}"
        )


def process_capture(
    capture_path: Path,
    output_dir: Path,
    pairs: list[tuple[int, int]],
    endians: list[str],
) -> list[Path]:
    frames, rejected_resync_count, partial_frame_count = load_frames(capture_path)
    print(f"{capture_path}:")
    print(
        f"  valid_frames={len(frames)} rejected_resync={rejected_resync_count} "
        f"partial_frames={partial_frame_count}"
    )

    output_paths: list[Path] = []
    for endian in endians:
        output_path, stats_by_name = save_candidate_plot(
            capture_path, frames, pairs, endian, output_dir
        )
        output_paths.append(output_path)
        print(f"  saved_plot={output_path}")
        print_stats(stats_by_name)
    return output_paths


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        pairs = resolve_pairs(args.pairs)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    endians = resolve_endians(args.endian)
    try:
        for capture_path in args.captures:
            process_capture(capture_path, args.output_dir, pairs, endians)
    except OSError as exc:
        print(f"failed to read or write analysis file: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
