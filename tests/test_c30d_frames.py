from __future__ import annotations

from ackermann_robot.drivers.c30d_frames import (
    changing_byte_positions,
    constant_byte_positions,
    extract_frames,
    frame_length_distribution,
    repeated_frame_patterns,
    summarize_frames,
)


def test_extract_frames_uses_candidate_delimiters():
    data = bytes(
        [
            0x00,
            0x7B,
            0x10,
            0x20,
            0x7D,
            0x55,
            0x7B,
            0x30,
            0x7D,
            0x7B,
            0x40,
        ]
    )

    assert extract_frames(data) == [
        bytes([0x7B, 0x10, 0x20, 0x7D]),
        bytes([0x7B, 0x30, 0x7D]),
    ]


def test_frame_length_distribution_is_sorted():
    frames = [
        bytes([0x7B, 0x01, 0x7D]),
        bytes([0x7B, 0x02, 0x03, 0x7D]),
        bytes([0x7B, 0x04, 0x7D]),
    ]

    assert frame_length_distribution(frames) == {3: 2, 4: 1}


def test_constant_and_changing_byte_positions_do_not_assign_meanings():
    frames = [
        bytes([0x7B, 0x01, 0xAA, 0x7D]),
        bytes([0x7B, 0x02, 0xAA, 0x7D]),
        bytes([0x7B, 0x03, 0xAA, 0x7D]),
    ]

    assert constant_byte_positions(frames) == {0: 0x7B, 2: 0xAA, 3: 0x7D}
    assert changing_byte_positions(frames) == [1]


def test_variable_length_frames_only_compare_observed_bytes_for_changes():
    frames = [
        bytes([0x7B, 0x01, 0x7D]),
        bytes([0x7B, 0x02, 0x03, 0x7D]),
    ]

    assert constant_byte_positions(frames) == {0: 0x7B}
    assert changing_byte_positions(frames) == [1, 2]


def test_repeated_frame_patterns_are_exact_frame_matches():
    repeated = bytes([0x7B, 0x10, 0x7D])
    frames = [repeated, bytes([0x7B, 0x20, 0x7D]), repeated]

    assert repeated_frame_patterns(frames) == {"7b 10 7d": 2}


def test_summarize_frames_returns_read_only_statistics():
    frames = [
        bytes([0x7B, 0x10, 0xAA, 0x7D]),
        bytes([0x7B, 0x11, 0xAA, 0x7D]),
        bytes([0x7B, 0x10, 0xAA, 0x7D]),
    ]

    assert summarize_frames(frames) == {
        "frame_count": 3,
        "frame_length_distribution": {4: 3},
        "constant_byte_positions": {0: 0x7B, 2: 0xAA, 3: 0x7D},
        "changing_byte_positions": [1],
        "repeated_frame_patterns": {"7b 10 aa 7d": 2},
    }
