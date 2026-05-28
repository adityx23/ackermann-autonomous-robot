from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Iterable, TypedDict

FRAME_START = 0x7B
FRAME_END = 0x7D
FRAME_LENGTH = 24


class FrameSummary(TypedDict):
    frame_count: int
    frame_length_distribution: dict[int, int]
    constant_byte_positions: dict[int, int]
    changing_byte_positions: list[int]
    repeated_frame_patterns: dict[str, int]


@dataclass(frozen=True)
class FrameExtractionResult:
    frames: list[bytes]
    rejected_resync_count: int
    partial_frame_count: int


def extract_frames(data: bytes) -> list[bytes]:
    """Extract valid fixed-length C30D frames from raw captured bytes."""
    return extract_frames_with_stats(data).frames


def has_valid_checksum(frame: bytes) -> bool:
    from ackermann_robot.drivers.c30d_checksum import is_valid_feedback_checksum

    return is_valid_feedback_checksum(frame)


def filter_frames_by_checksum(frames: Iterable[bytes], require_valid: bool = False) -> list[bytes]:
    frame_list = _materialize_frames(frames)
    if not require_valid:
        return frame_list
    return [frame for frame in frame_list if has_valid_checksum(frame)]


def extract_frames_with_stats(data: bytes) -> FrameExtractionResult:
    frames: list[bytes] = []
    rejected_resync_count = 0
    partial_frame_count = 0
    search_from = 0

    while True:
        start = data.find(bytes([FRAME_START]), search_from)
        if start < 0:
            break

        frame_end = start + FRAME_LENGTH
        if frame_end > len(data):
            partial_frame_count += 1
            break

        frame = data[start:frame_end]
        if frame[FRAME_LENGTH - 1] == FRAME_END:
            frames.append(frame)
            search_from = frame_end
            continue

        rejected_resync_count += 1
        search_from = start + 1

    return FrameExtractionResult(
        frames=frames,
        rejected_resync_count=rejected_resync_count,
        partial_frame_count=partial_frame_count,
    )


def extract_delimited_frames(data: bytes) -> list[bytes]:
    """Extract delimiter-bounded candidates for debugging historical captures."""
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
