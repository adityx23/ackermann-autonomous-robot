from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_checksum import compute_feedback_checksum
from ackermann_robot.drivers.c30d_frames import FRAME_LENGTH


def load_payload_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "analyze_c30d_payload_fields.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def make_frame(byte_20: int, byte_21: int, valid_checksum: bool = True) -> bytes:
    frame = bytearray([0x7B, *([0x00] * (FRAME_LENGTH - 2)), 0x7D])
    frame[20] = byte_20
    frame[21] = byte_21
    frame[22] = compute_feedback_checksum(frame)
    if not valid_checksum:
        frame[22] ^= 0xFF
    return bytes(frame)


def test_decode_uint16_be_20_21():
    module = load_payload_script()

    assert module.decode_uint16_be_20_21(make_frame(0x2A, 0xF6)) == 10998


def test_payload_analyzer_reports_candidate_battery_and_checksum_counts(tmp_path: Path):
    module = load_payload_script()
    capture = tmp_path / "capture.bin"
    capture.write_bytes(
        b"noise"
        + make_frame(0x2A, 0xF6)
        + make_frame(0x2A, 0xF7)
        + make_frame(0x2A, 0xF8, valid_checksum=False)
    )

    analysis = module.analyze_payload_fields(capture)

    assert analysis.extracted_frame_count == 3
    assert analysis.valid_checksum_count == 2
    assert analysis.invalid_checksum_count == 1
    assert analysis.uint16_be_20_21.count == 2
    assert analysis.uint16_be_20_21.minimum == 10998.0
    assert analysis.uint16_be_20_21.maximum == 10999.0
    assert analysis.candidate_battery_voltage_V.minimum == 10.998
    assert analysis.candidate_battery_voltage_V.maximum == 10.999
    assert analysis.byte_20_counts == {0x2A: 2}
    assert analysis.byte_21_counts == {0xF6: 1, 0xF7: 1}


def test_payload_analyzer_script_prints_candidate_only_language(tmp_path: Path, capsys):
    module = load_payload_script()
    capture = tmp_path / "capture.bin"
    capture.write_bytes(make_frame(0x2A, 0xF6))

    exit_code = module.main([str(capture)])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "uint16_be_20_21:" in output
    assert "byte_20_unique_counts: 0x2a:1" in output
    assert "byte_21_unique_counts: 0xf6:1" in output
    assert "candidate_battery_voltage_V:" in output
    assert "interpretation: candidate_only_not_confirmed_battery_voltage" in output
