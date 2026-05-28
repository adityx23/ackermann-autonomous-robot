from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_checksum import (
    CHECKSUM_INDEX,
    compute_feedback_checksum,
    is_valid_feedback_checksum,
    sum_mod_256,
    test_checksum_hypotheses as run_checksum_hypotheses,
    twos_complement_sum,
    xor_checksum,
)
from ackermann_robot.drivers.c30d_frames import FRAME_END, FRAME_START


def load_checksum_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "analyze_c30d_checksum.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def build_frame(payload: list[int], checksum: int) -> bytes:
    assert len(payload) == 21
    return bytes([FRAME_START, *payload, checksum, FRAME_END])


def test_basic_checksum_functions():
    data = bytes([0x01, 0x02, 0x03])

    assert sum_mod_256(data) == 0x06
    assert twos_complement_sum(data) == 0xFA
    assert xor_checksum(data) == 0x00


def test_checksum_hypotheses_detect_sum_mod_256_for_bytes_1_through_21():
    frames = []
    for offset in range(3):
        payload = [(index + offset) & 0xFF for index in range(1, 22)]
        checksum = sum_mod_256(bytes(payload))
        frames.append(build_frame(payload, checksum))

    results = run_checksum_hypotheses(frames)
    best = results[0]

    assert best.name == "sum_mod_256"
    assert best.byte_range.label == "bytes_01_21"
    assert best.match_count == 3
    assert best.match_percentage == 100.0
    assert best.reached_100_percent is True


def test_checksum_hypotheses_detect_twos_complement_range():
    frames = []
    for offset in range(4):
        payload = [0x2A, 0x10 + offset, *([offset] * 19)]
        checksum = twos_complement_sum(bytes(payload[:20]))
        frames.append(build_frame(payload, checksum))

    results = run_checksum_hypotheses(frames)
    matching = [
        result
        for result in results
        if result.name == "twos_complement_sum" and result.byte_range.label == "bytes_01_20"
    ][0]

    assert matching.match_count == 4
    assert matching.reached_100_percent


def test_checksum_hypotheses_use_byte_22_as_candidate():
    payload = [1] * 21
    frame = build_frame(payload, checksum=0xA5)

    assert frame[CHECKSUM_INDEX] == 0xA5
    assert all(result.frame_count == 1 for result in run_checksum_hypotheses([frame]))


def test_feedback_checksum_uses_xor_of_bytes_0_through_21():
    payload = [index & 0xFF for index in range(1, 22)]
    frame = build_frame(payload, checksum=0x00)
    expected = xor_checksum(frame[:CHECKSUM_INDEX])

    assert compute_feedback_checksum(frame) == expected


def test_feedback_checksum_validation_detects_corruption():
    payload = [index & 0xFF for index in range(1, 22)]
    valid = bytearray(build_frame(payload, checksum=0x00))
    valid[CHECKSUM_INDEX] = compute_feedback_checksum(valid)

    corrupted = bytearray(valid)
    corrupted[3] ^= 0x01

    assert is_valid_feedback_checksum(bytes(valid)) is True
    assert is_valid_feedback_checksum(bytes(corrupted)) is False


def test_analyze_c30d_checksum_script_prints_top_hypotheses(tmp_path: Path, capsys):
    module = load_checksum_script()
    capture_a = tmp_path / "a.bin"
    capture_b = tmp_path / "b.bin"
    frames = []
    for offset in range(2):
        payload = [(index + offset) & 0xFF for index in range(1, 22)]
        frames.append(build_frame(payload, sum_mod_256(bytes(payload))))
    capture_a.write_bytes(b"noise" + b"".join(frames))
    capture_b.write_bytes(b"".join(frames))

    exit_code = module.main([str(capture_a), str(capture_b), "--top", "3"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert f"file: {capture_a}" in output
    assert "frame_count: 2" in output
    assert "sum_mod_256 bytes_01_21" in output
    assert "matches=2/2" in output
    assert "match_percentage=100.00%" in output
    assert "any_hypothesis_100_percent: true" in output
    assert "confirmed_across_multiple_captures: candidate_requires_review" in output
    assert "real_motor_command_path: disabled" in output


def test_analyze_c30d_checksum_rejects_invalid_top(capsys):
    module = load_checksum_script()

    exit_code = module.main(["capture.bin", "--top", "0"])

    assert exit_code == 2
    assert "--top must be positive" in capsys.readouterr().err
