from __future__ import annotations

from collections import Counter
from typing import Iterable, TypedDict

FRAME_START = 0x7B
FRAME_END = 0x7D


class FrameSummary(TypedDict):
    frame_count: int
    frame_length_distribution: dict[int, int]
    constant_byte_positions: dict[int, int]
    changing_byte_positions: list[int]
    repeated_frame_patterns: dict[str, int]


def extract_frames(data: bytes) -> list[bytes]:
    """Extract candidate C30D frames between 0x7B and 0x7D delimiters."""
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


def frame_length_distribution(frames: Iterable[bytes]) -> dict[int, int]:
    return dict(sorted(Counter(len(frame) for frame in frames).items()))


def _materialize_frames(frames: Iterable[bytes]) -> list[bytes]:
    if isinstance(frames, list):
        return frames
    return list(frames)


def constant_byte_positions(frames: Iterable[bytes]) -> dict[int, int]:
    frame_list = _materialize_frames(frames)
    if not frame_list:
        return {}

    shortest_length = min(len(frame) for frame in frame_list)
    constants: dict[int, int] = {}
    for position in range(shortest_length):
        values = {frame[position] for frame in frame_list}
        if len(values) == 1:
            constants[position] = values.pop()
    return constants


def changing_byte_positions(frames: Iterable[bytes]) -> list[int]:
    frame_list = _materialize_frames(frames)
    if not frame_list:
        return []

    longest_length = max(len(frame) for frame in frame_list)
    changing: list[int] = []
    for position in range(longest_length):
        values = {frame[position] for frame in frame_list if position < len(frame)}
        if len(values) > 1:
            changing.append(position)
    return changing


def repeated_frame_patterns(frames: Iterable[bytes]) -> dict[str, int]:
    counts = Counter(frame.hex(" ") for frame in frames)
    return dict(sorted((frame_hex, count) for frame_hex, count in counts.items() if count > 1))


def summarize_frames(frames: Iterable[bytes]) -> FrameSummary:
    frame_list = _materialize_frames(frames)
    return {
        "frame_count": len(frame_list),
        "frame_length_distribution": frame_length_distribution(frame_list),
        "constant_byte_positions": constant_byte_positions(frame_list),
        "changing_byte_positions": changing_byte_positions(frame_list),
        "repeated_frame_patterns": repeated_frame_patterns(frame_list),
    }
