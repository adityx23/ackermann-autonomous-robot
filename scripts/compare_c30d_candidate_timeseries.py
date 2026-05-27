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
SUGGESTED_PAIR_TEXTS = ["02_03", "06_07", "08_09", "10_11", "12_13", "14_15", "16_17", "18_19"]
MIN_PAIR_FIRST_BYTE = 2
MAX_PAIR_SECOND_BYTE = 19


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare one C30D candidate int16 time series across saved .bin captures."
    )
    parser.add_argument("captures", nargs="+", type=Path, help="Saved passive .bin capture files.")
    pair_group = parser.add_mutually_exclusive_group(required=True)
    pair_group.add_argument("--pair", help="Adjacent byte pair to plot, such as 02_03.")
    pair_group.add_argument(
        "--pairs",
        nargs="*",
        help=(
            "Adjacent byte pairs to plot. If no values follow --pairs, uses the aligned "
            "suggested set: 02_03 06_07 08_09 10_11 12_13 14_15 16_17 18_19."
        ),
    )
    parser.add_argument(
        "--endian",
        choices=("be", "le"),
        default="be",
        help="Candidate int16 byte order.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory for generated comparison PNG files.",
    )
    return parser


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
    if first < MIN_PAIR_FIRST_BYTE or second > MAX_PAIR_SECOND_BYTE:
        raise ValueError(
            f"invalid pair {pair_text!r}; supported candidate pairs range from 02_03 to 18_19"
        )
    return first, second


def suggested_pairs() -> list[tuple[int, int]]:
    return [parse_pair(pair_text) for pair_text in SUGGESTED_PAIR_TEXTS]


def resolve_pairs(pair_text: str | None, pair_texts: list[str] | None) -> list[tuple[int, int]]:
    if pair_text is not None:
        return [parse_pair(pair_text)]
    if pair_texts == []:
        return suggested_pairs()
    if pair_texts is not None:
        return [parse_pair(text) for text in pair_texts]
    raise ValueError("provide --pair or --pairs")


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


def load_frames(capture_path: Path) -> list[bytes]:
    from ackermann_robot.drivers.c30d_frames import extract_frames

    return extract_frames(capture_path.read_bytes())


def output_path_for(pair: tuple[int, int], endian: str, output_dir: Path) -> Path:
    return output_dir / f"compare_candidate_int16_{endian}_{pair[0]:02d}_{pair[1]:02d}.png"


def save_comparison_plot(
    values_by_capture: dict[Path, list[int]],
    pair: tuple[int, int],
    endian: str,
    output_dir: Path,
) -> Path:
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_path_for(pair, endian, output_dir)
    name = candidate_name(pair, endian)

    fig, ax = plt.subplots(figsize=(11, 6))
    for capture_path, values in values_by_capture.items():
        ax.plot(range(len(values)), values, linewidth=1.0, label=capture_path.stem)

    ax.set_title(f"C30D {name} across captures")
    ax.set_xlabel("frame_index")
    ax.set_ylabel("candidate_int16_value")
    ax.grid(True, alpha=0.3)
    if values_by_capture:
        ax.legend(loc="best", fontsize="small")
    fig.tight_layout()
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return output_path


def print_capture_stats(capture_path: Path, stats: dict[str, float | int]) -> None:
    print(
        f"  {capture_path}: count={stats['count']} min={stats['min']:.0f} "
        f"max={stats['max']:.0f} mean={stats['mean']:.2f} stdev={stats['stdev']:.2f}"
    )


def process_pair(
    capture_paths: list[Path],
    pair: tuple[int, int],
    endian: str,
    output_dir: Path,
) -> Path:
    name = candidate_name(pair, endian)
    values_by_capture: dict[Path, list[int]] = {}
    print(name)

    for capture_path in capture_paths:
        frames = load_frames(capture_path)
        values = decode_candidate_values(frames, pair, endian)
        values_by_capture[capture_path] = values
        print_capture_stats(capture_path, basic_stats(values))

    output_path = save_comparison_plot(values_by_capture, pair, endian, output_dir)
    print(f"  saved_plot={output_path}")
    return output_path


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        pairs = resolve_pairs(args.pair, args.pairs)
        for pair in pairs:
            process_pair(args.captures, pair, args.endian, args.output_dir)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except OSError as exc:
        print(f"failed to read or write analysis file: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
