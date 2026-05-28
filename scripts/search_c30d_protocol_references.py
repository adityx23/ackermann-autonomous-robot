#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

DEFAULT_ROOTS = (Path("external"), Path("docs"), Path("src"), Path("scripts"))
DEFAULT_KEYWORDS = (
    "c30d",
    "WHEELTEC",
    "轮趣",
    "轮趣科技",
    "底层主控",
    "通信协议",
    "源码",
    "串口",
    "C10B",
    "R550",
    "STM32F407VET6",
    "ROS bottom controller",
    "cmd_vel",
    "serial protocol",
    "serial.write",
    "motor",
    "steering",
    "servo",
    "speed",
    "checksum",
    "0x7B",
    "0x7D",
)
SKIP_DIRS = {".git", "__pycache__", ".pytest_cache", ".ruff_cache"}
MAX_FILE_SIZE_BYTES = 1_000_000


@dataclass(frozen=True)
class ProtocolReferenceMatch:
    path: Path
    line_number: int
    keyword: str
    line: str


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Search local repository files for C30D protocol reference keywords."
    )
    parser.add_argument(
        "--root",
        action="append",
        type=Path,
        dest="roots",
        help="Root directory to search. May be passed more than once.",
    )
    return parser


def should_skip_path(path: Path) -> bool:
    return any(part in SKIP_DIRS for part in path.parts)


def iter_candidate_files(roots: tuple[Path, ...] = DEFAULT_ROOTS) -> list[Path]:
    files: list[Path] = []
    for root in roots:
        if not root.exists():
            continue
        if root.is_file() and not should_skip_path(root):
            files.append(root)
            continue
        for path in root.rglob("*"):
            if path.is_file() and not should_skip_path(path):
                files.append(path)
    return sorted(files)


def is_probably_text(path: Path, max_size_bytes: int = MAX_FILE_SIZE_BYTES) -> bool:
    try:
        if path.stat().st_size > max_size_bytes:
            return False
        chunk = path.read_bytes()[:4096]
    except OSError:
        return False
    return b"\x00" not in chunk


def line_matches(line: str, keywords: tuple[str, ...] = DEFAULT_KEYWORDS) -> list[str]:
    lower_line = line.lower()
    return [keyword for keyword in keywords if keyword.lower() in lower_line]


def search_file(
    path: Path, keywords: tuple[str, ...] = DEFAULT_KEYWORDS
) -> list[ProtocolReferenceMatch]:
    if not is_probably_text(path):
        return []

    matches: list[ProtocolReferenceMatch] = []
    try:
        with path.open(encoding="utf-8", errors="replace") as handle:
            for line_number, line in enumerate(handle, start=1):
                stripped = line.rstrip()
                for keyword in line_matches(stripped, keywords):
                    matches.append(
                        ProtocolReferenceMatch(
                            path=path,
                            line_number=line_number,
                            keyword=keyword,
                            line=stripped,
                        )
                    )
    except OSError:
        return []
    return matches


def search_roots(
    roots: tuple[Path, ...] = DEFAULT_ROOTS,
    keywords: tuple[str, ...] = DEFAULT_KEYWORDS,
) -> list[ProtocolReferenceMatch]:
    matches: list[ProtocolReferenceMatch] = []
    for path in iter_candidate_files(roots):
        matches.extend(search_file(path, keywords))
    return matches


def format_match(match: ProtocolReferenceMatch) -> str:
    return f"{match.path}:{match.line_number}: [{match.keyword}] {match.line}"


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    roots = tuple(args.roots) if args.roots else DEFAULT_ROOTS
    matches = search_roots(roots)
    for match in matches:
        print(format_match(match))
    print(f"matches: {len(matches)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
