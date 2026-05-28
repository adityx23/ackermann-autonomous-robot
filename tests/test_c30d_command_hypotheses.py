from __future__ import annotations

import importlib.util
import inspect
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_checksum import xor_checksum
from ackermann_robot.drivers.c30d_command_hypotheses import (
    CHECKSUM_INDEX,
    C30DCommandHypothesis,
    C30DCommandHypothesisFrame,
    build_command_hypothesis,
    build_hypothesis_frame,
)


def load_script(name: str):
    script_path = Path(__file__).resolve().parents[1] / "scripts" / name
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def test_hypothesis_frame_shape_and_checksum():
    payload = bytes(range(21))

    frame = build_hypothesis_frame(payload)

    assert len(frame) == 24
    assert frame[0] == 0x7B
    assert frame[1:22] == payload
    assert frame[23] == 0x7D
    assert frame[22] == xor_checksum(frame[:22])


def test_hypothesis_frame_requires_exact_payload_length():
    for payload in (bytes(range(20)), bytes(range(22))):
        try:
            build_hypothesis_frame(payload)
        except ValueError as exc:
            assert "exactly 21 bytes" in str(exc)
        else:
            raise AssertionError("expected ValueError")


def test_hypothesis_dataclasses_label_outputs_unverified():
    payload = bytes([0x00] * 21)

    hypothesis_frame = build_command_hypothesis(payload)

    assert isinstance(hypothesis_frame.hypothesis, C30DCommandHypothesis)
    assert isinstance(hypothesis_frame, C30DCommandHypothesisFrame)
    assert hypothesis_frame.label == "unverified_hypothesis"
    assert hypothesis_frame.hypothesis.label == "unverified_hypothesis"
    assert hypothesis_frame.protocol_known is False
    assert hypothesis_frame.transmit_allowed is False
    assert hypothesis_frame.checksum == hypothesis_frame.frame[CHECKSUM_INDEX]


def assert_hypothesis_script_safety_output(output: str) -> None:
    assert "label: unverified_hypothesis" in output
    assert "full_frame_hex:" in output
    assert "checksum_byte:" in output
    assert "protocol_known: false" in output
    assert "transmit_allowed: false" in output
    assert "must not be sent to C30D" in output


def test_build_hypothesis_script_positional_payload_still_works(capsys):
    module = load_script("build_c30d_hypothesis_frame.py")
    payload_args = ["00"] * 21

    exit_code = module.main(payload_args)

    output = capsys.readouterr().out
    assert exit_code == 0
    assert_hypothesis_script_safety_output(output)


def test_build_hypothesis_script_payload_hex_option_works(capsys):
    module = load_script("build_c30d_hypothesis_frame.py")
    payload_hex = " ".join(["00"] * 21)

    exit_code = module.main(["--payload-hex", payload_hex])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert_hypothesis_script_safety_output(output)


def checked_feedback_frame(overrides: dict[int, int]) -> bytes:
    from ackermann_robot.drivers.c30d_checksum import compute_feedback_checksum

    frame = bytearray([0x7B, *([0x00] * 22), 0x7D])
    for position, value in overrides.items():
        frame[position] = value
    frame[22] = compute_feedback_checksum(frame)
    return bytes(frame)


def test_frame_structure_script_prints_feedback_template(tmp_path: Path, capsys):
    module = load_script("analyze_c30d_frame_structure.py")
    capture = tmp_path / "capture.bin"
    capture.write_bytes(checked_feedback_frame({20: 0x2A, 21: 0xF6}))

    exit_code = module.main([str(capture)])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "byte_0_start: 0x7b" in output
    assert "byte_22_checksum: xor_bytes_0_through_21" in output
    assert "byte_23_end: 0x7d" in output
    assert "uint16_be_20_21: candidate_battery_mV" in output
    assert "unknown_bytes:" in output


def test_frame_structure_script_reports_all_candidate_fields(tmp_path: Path, capsys):
    module = load_script("analyze_c30d_frame_structure.py")
    capture = tmp_path / "capture.bin"
    capture.write_bytes(
        checked_feedback_frame(
            {
                2: 0x00,
                3: 0x10,
                6: 0xFF,
                7: 0xF0,
                12: 0x00,
                13: 0x01,
                14: 0x00,
                15: 0x02,
                16: 0x00,
                17: 0x03,
                18: 0x00,
                19: 0x04,
                20: 0x2A,
                21: 0xF6,
            }
        )
    )

    exit_code = module.main([str(capture)])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "int16_be_02_03: candidate_forward_motion" in output
    assert "int16_be_06_07: candidate_yaw_motion" in output
    assert "int16_be_12_13: candidate_imu_12_13" in output
    assert "int16_be_14_15: candidate_imu_14_15" in output
    assert "int16_be_16_17: candidate_imu_16_17" in output
    assert "int16_be_18_19: candidate_imu_18_19" in output
    assert "uint16_be_20_21: candidate_battery_mV" in output
    assert "min=" in output
    assert "max=" in output
    assert "mean=" in output


def test_frame_structure_unknown_bytes_exclude_candidate_field_bytes(tmp_path: Path, capsys):
    module = load_script("analyze_c30d_frame_structure.py")
    capture = tmp_path / "capture.bin"
    capture.write_bytes(checked_feedback_frame({20: 0x2A, 21: 0xF6}))

    exit_code = module.main([str(capture)])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "unknown_bytes: 1, 4, 5, 8, 9, 10, 11" in output


def test_no_serial_write_path_exists_in_hypothesis_lab():
    import ackermann_robot.drivers.c30d_command_hypotheses as hypotheses

    build_script = load_script("build_c30d_hypothesis_frame.py")
    analyze_script = load_script("analyze_c30d_frame_structure.py")
    sources = [
        inspect.getsource(hypotheses),
        inspect.getsource(build_script),
        inspect.getsource(analyze_script),
    ]

    forbidden = (
        "os.open",
        "os.write",
        ".write(",
        "serial.Serial",
        "/dev/c30d",
        "send_drive_command",
    )
    for source in sources:
        for token in forbidden:
            assert token not in source
