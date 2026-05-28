from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_checksum import compute_feedback_checksum
from ackermann_robot.drivers.c30d_frames import FRAME_LENGTH


def load_status_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "analyze_c30d_status_byte.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def make_frame(status_byte: int, valid_checksum: bool = True) -> bytes:
    frame = bytearray([0x7B, *([0x00] * (FRAME_LENGTH - 2)), 0x7D])
    frame[21] = status_byte
    frame[22] = compute_feedback_checksum(frame)
    if not valid_checksum:
        frame[22] ^= 0xFF
    return bytes(frame)


def test_status_byte_analyzer_reports_counts_and_transitions_from_valid_frames(tmp_path: Path):
    module = load_status_script()
    capture = tmp_path / "capture.bin"
    capture.write_bytes(
        b"noise"
        + make_frame(0x01)
        + make_frame(0x01)
        + make_frame(0x02)
        + make_frame(0x03, valid_checksum=False)
    )

    analysis = module.analyze_status_byte(capture)

    assert analysis.frame_count == 3
    assert analysis.invalid_checksum_count == 1
    assert analysis.minimum == 0x01
    assert analysis.maximum == 0x02
    assert analysis.unique_values == (0x01, 0x02)
    assert analysis.transitions == 1
    assert analysis.counts == {0x01: 2, 0x02: 1}


def test_status_byte_script_prints_unassigned_meaning(tmp_path: Path, capsys):
    module = load_status_script()
    capture = tmp_path / "capture.bin"
    capture.write_bytes(make_frame(0x2A))

    exit_code = module.main([str(capture)])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "valid_frame_count: 1" in output
    assert "byte_index: 21" in output
    assert "unique_values: 0x2a" in output
    assert "meaning: unassigned_status_counter_or_mode_candidate" in output
