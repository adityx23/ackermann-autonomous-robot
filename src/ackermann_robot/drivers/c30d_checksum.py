from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

CHECKSUM_INDEX = 22
FEEDBACK_CHECKSUM_END_EXCLUSIVE = 22
FEEDBACK_FRAME_LENGTH = 24


@dataclass(frozen=True)
class ChecksumRange:
    label: str
    start: int
    end_exclusive: int
    notes: str


@dataclass(frozen=True)
class ChecksumHypothesisResult:
    name: str
    byte_range: ChecksumRange
    match_count: int
    frame_count: int

    @property
    def match_percentage(self) -> float:
        if self.frame_count == 0:
            return 0.0
        return 100.0 * self.match_count / self.frame_count

    @property
    def reached_100_percent(self) -> bool:
        return self.frame_count > 0 and self.match_count == self.frame_count


ChecksumFunction = Callable[[bytes], int]


CHECKSUM_RANGES = (
    ChecksumRange("bytes_01_21", 1, 22, "payload/status bytes before checksum candidate"),
    ChecksumRange("bytes_00_21", 0, 22, "start byte plus payload/status bytes"),
    ChecksumRange("bytes_01_20", 1, 21, "payload bytes excluding possible status/counter byte 21"),
    ChecksumRange("bytes_08_21", 8, 22, "upper payload/status region"),
    ChecksumRange("bytes_02_21", 2, 22, "motion/IMU candidate region plus status byte"),
)


def sum_mod_256(data: bytes) -> int:
    return sum(data) & 0xFF


def twos_complement_sum(data: bytes) -> int:
    return (-sum(data)) & 0xFF


def xor_checksum(data: bytes) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum & 0xFF


def ones_complement_sum(data: bytes) -> int:
    return (~sum(data)) & 0xFF


def compute_feedback_checksum(frame: bytes) -> int:
    return xor_checksum(frame[:FEEDBACK_CHECKSUM_END_EXCLUSIVE])


def is_valid_feedback_checksum(frame: bytes) -> bool:
    return (
        len(frame) == FEEDBACK_FRAME_LENGTH
        and compute_feedback_checksum(frame) == frame[CHECKSUM_INDEX]
    )


CHECKSUM_FUNCTIONS: tuple[tuple[str, ChecksumFunction], ...] = (
    ("sum_mod_256", sum_mod_256),
    ("twos_complement_sum", twos_complement_sum),
    ("xor_checksum", xor_checksum),
    ("ones_complement_sum", ones_complement_sum),
)


def _materialize_frames(frames: Iterable[bytes]) -> list[bytes]:
    if isinstance(frames, list):
        return frames
    return list(frames)


def test_checksum_hypotheses(frames: Iterable[bytes]) -> list[ChecksumHypothesisResult]:
    frame_list = [frame for frame in _materialize_frames(frames) if len(frame) > CHECKSUM_INDEX]
    results: list[ChecksumHypothesisResult] = []

    for range_candidate in CHECKSUM_RANGES:
        for function_name, checksum_function in CHECKSUM_FUNCTIONS:
            match_count = 0
            for frame in frame_list:
                data = frame[range_candidate.start : range_candidate.end_exclusive]
                if checksum_function(data) == frame[CHECKSUM_INDEX]:
                    match_count += 1
            results.append(
                ChecksumHypothesisResult(
                    name=function_name,
                    byte_range=range_candidate,
                    match_count=match_count,
                    frame_count=len(frame_list),
                )
            )

    return sorted(
        results,
        key=lambda result: (
            result.match_count,
            result.match_percentage,
            result.byte_range.label,
            result.name,
        ),
        reverse=True,
    )
