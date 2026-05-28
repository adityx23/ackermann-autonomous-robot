from __future__ import annotations

import pytest

from ackermann_robot.drivers.c30d_feedback import parse_feedback_candidates
from ackermann_robot.drivers.c30d_frames import FRAME_LENGTH


def make_frame(overrides: dict[int, int] | None = None) -> bytes:
    frame = bytearray([0x7B, *([0x00] * (FRAME_LENGTH - 2)), 0x7D])
    for position, value in (overrides or {}).items():
        frame[position] = value
    return bytes(frame)


def test_parse_feedback_candidates_decodes_named_candidate_fields():
    frame = make_frame(
        {
            2: 0x12,
            3: 0x34,
            6: 0xFF,
            7: 0xFE,
            12: 0x80,
            13: 0x00,
            14: 0x7F,
            15: 0xFF,
            16: 0x00,
            17: 0x01,
            18: 0xFF,
            19: 0xFF,
            22: 0xA5,
        }
    )

    parsed = parse_feedback_candidates([frame])

    assert len(parsed) == 1
    candidate = parsed[0]
    assert candidate.frame_index == 0
    assert candidate.candidate_forward_motion == 0x1234
    assert candidate.candidate_yaw_motion == -2
    assert candidate.candidate_imu_12_13 == -32768
    assert candidate.candidate_imu_14_15 == 32767
    assert candidate.candidate_imu_16_17 == 1
    assert candidate.candidate_imu_18_19 == -1
    assert candidate.checksum_candidate == 0xA5
    assert candidate.raw_frame_hex == frame.hex(" ")


def test_parse_feedback_candidates_uses_zero_based_frame_index():
    frames = [make_frame({2: 0x00, 3: value}) for value in (0x01, 0x02, 0x03)]

    parsed = parse_feedback_candidates(frames)

    assert [candidate.frame_index for candidate in parsed] == [0, 1, 2]
    assert [candidate.candidate_forward_motion for candidate in parsed] == [1, 2, 3]


def test_parse_feedback_candidates_rejects_non_fixed_frame():
    with pytest.raises(ValueError, match="expected 24-byte C30D frame"):
        parse_feedback_candidates([bytes([0x7B, 0x00, 0x7D])])


def test_parse_feedback_candidates_rejects_bad_markers():
    with pytest.raises(ValueError, match="expected C30D frame start"):
        parse_feedback_candidates([make_frame({0: 0x00})])

    with pytest.raises(ValueError, match="expected C30D frame end"):
        parse_feedback_candidates([make_frame({23: 0x00})])
