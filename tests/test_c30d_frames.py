from __future__ import annotations

from ackermann_robot.drivers.c30d_frames import (
    FRAME_LENGTH,
    changing_byte_positions,
    constant_byte_positions,
    extract_delimited_frames,
    extract_frames,
    extract_frames_with_stats,
    frame_length_distribution,
    repeated_frame_patterns,
    summarize_frames,
)


def make_frame(fill: int, overrides: dict[int, int] | None = None) -> bytes:
    frame = bytearray([0x7B, *([fill] * (FRAME_LENGTH - 2)), 0x7D])
    for position, value in (overrides or {}).items():
        frame[position] = value
    return bytes(frame)


def test_extract_frames_uses_fixed_24_byte_frames():
    first = make_frame(0x10)
    second = make_frame(0x20)

    assert extract_frames(bytes([0x00, 0x55]) + first + second) == [first, second]


def test_payload_containing_end_byte_does_not_truncate_frame():
    frame = make_frame(0x10, {5: 0x7D, 12: 0x7D})

    assert len(frame) == 24
    assert extract_frames(frame) == [frame]
    assert extract_delimited_frames(frame) == [frame[:6]]


def test_partial_frame_at_beginning_and_end_is_ignored_safely():
    frame = make_frame(0x22)
    leading_partial = bytes([0x7B, *([0x55] * 10)])
    data = leading_partial + frame + frame[:12]

    extraction = extract_frames_with_stats(data)

    assert extraction.frames == [frame]
    assert extraction.rejected_resync_count == 1
    assert extraction.partial_frame_count == 1


def test_invalid_end_byte_is_rejected_and_resyncs():
    invalid = make_frame(0x33, {FRAME_LENGTH - 1: 0x00})
    valid = make_frame(0x44)

    extraction = extract_frames_with_stats(invalid + valid)

    assert extraction.frames == [valid]
    assert extraction.rejected_resync_count == 1
    assert extraction.partial_frame_count == 0


def test_frame_length_distribution_is_sorted():
    frames = [
        make_frame(0x01),
        make_frame(0x02),
        make_frame(0x03),
    ]

    assert frame_length_distribution(frames) == {24: 3}


def test_constant_and_changing_byte_positions_do_not_assign_meanings():
    frames = [
        make_frame(0x01, {2: 0xAA}),
        make_frame(0x02, {2: 0xAA}),
        make_frame(0x03, {2: 0xAA}),
    ]

    assert constant_byte_positions(frames) == {0: 0x7B, 2: 0xAA, 23: 0x7D}
    assert changing_byte_positions(frames) == [
        1,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        17,
        18,
        19,
        20,
        21,
        22,
    ]


def test_variable_length_frames_only_compare_observed_bytes_for_changes():
    frames = [
        bytes([0x7B, 0x01, 0x7D]),
        bytes([0x7B, 0x02, 0x03, 0x7D]),
    ]

    assert constant_byte_positions(frames) == {0: 0x7B}
    assert changing_byte_positions(frames) == [1, 2]


def test_repeated_frame_patterns_are_exact_frame_matches():
    repeated = make_frame(0x10)
    frames = [repeated, make_frame(0x20), repeated]

    assert repeated_frame_patterns(frames) == {repeated.hex(" "): 2}


def test_summarize_frames_returns_read_only_statistics():
    frames = [
        make_frame(0x10, {2: 0xAA}),
        make_frame(0x11, {2: 0xAA}),
        make_frame(0x10, {2: 0xAA}),
    ]

    assert summarize_frames(frames) == {
        "frame_count": 3,
        "frame_length_distribution": {24: 3},
        "constant_byte_positions": {0: 0x7B, 2: 0xAA, 23: 0x7D},
        "changing_byte_positions": [
            1,
            3,
            4,
            5,
            6,
            7,
            8,
            9,
            10,
            11,
            12,
            13,
            14,
            15,
            16,
            17,
            18,
            19,
            20,
            21,
            22,
        ],
        "repeated_frame_patterns": {frames[0].hex(" "): 2},
    }
