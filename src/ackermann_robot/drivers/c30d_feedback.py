from __future__ import annotations

from dataclasses import dataclass

from ackermann_robot.drivers.c30d_checksum import is_valid_feedback_checksum
from ackermann_robot.drivers.c30d_frames import FRAME_END, FRAME_LENGTH, FRAME_START


@dataclass(frozen=True)
class C30DFeedbackCandidate:
    frame_index: int
    candidate_forward_motion: int
    candidate_yaw_motion: int
    candidate_imu_12_13: int
    candidate_imu_14_15: int
    candidate_imu_16_17: int
    candidate_imu_18_19: int
    checksum_candidate: int
    checksum_valid: bool
    raw_frame_hex: str


def _decode_int16_be(frame: bytes, first_byte: int) -> int:
    return int.from_bytes(frame[first_byte : first_byte + 2], "big", signed=True)


def _validate_frame(frame: bytes) -> None:
    if len(frame) != FRAME_LENGTH:
        raise ValueError(f"expected {FRAME_LENGTH}-byte C30D frame, got {len(frame)} bytes")
    if frame[0] != FRAME_START:
        raise ValueError(f"expected C30D frame start 0x{FRAME_START:02x}, got 0x{frame[0]:02x}")
    if frame[FRAME_LENGTH - 1] != FRAME_END:
        raise ValueError(
            f"expected C30D frame end 0x{FRAME_END:02x}, got 0x{frame[FRAME_LENGTH - 1]:02x}"
        )


def parse_feedback_candidates(frames: list[bytes]) -> list[C30DFeedbackCandidate]:
    """Parse read-only candidate feedback fields from fixed-length C30D frames."""
    candidates: list[C30DFeedbackCandidate] = []
    for frame_index, frame in enumerate(frames):
        _validate_frame(frame)
        candidates.append(
            C30DFeedbackCandidate(
                frame_index=frame_index,
                candidate_forward_motion=_decode_int16_be(frame, 2),
                candidate_yaw_motion=_decode_int16_be(frame, 6),
                candidate_imu_12_13=_decode_int16_be(frame, 12),
                candidate_imu_14_15=_decode_int16_be(frame, 14),
                candidate_imu_16_17=_decode_int16_be(frame, 16),
                candidate_imu_18_19=_decode_int16_be(frame, 18),
                checksum_candidate=frame[22],
                checksum_valid=is_valid_feedback_checksum(frame),
                raw_frame_hex=frame.hex(" "),
            )
        )
    return candidates
