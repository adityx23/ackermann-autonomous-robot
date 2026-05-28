from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_checksum import xor_checksum
from ackermann_robot.drivers.c30d_host_command_frame import (
    CHECKSUM_INDEX,
    build_ackermann_host_command_frame,
    build_ackermann_host_command_frame_from_floats,
    build_host_command_frame,
    scale_documentation_candidate,
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


def test_host_command_frame_shape_and_checksum():
    frame = build_host_command_frame(0x00, 0x01, 1000, 0, -250)

    assert len(frame) == 11
    assert frame[0] == 0x7B
    assert frame[10] == 0x7D
    assert frame[CHECKSUM_INDEX] == xor_checksum(frame[:9])


def test_host_command_frame_signed_int16_big_endian_positive_and_negative():
    frame = build_host_command_frame(0x00, 0x00, 1000, -1000, -1)

    assert frame[3:5] == (1000).to_bytes(2, "big", signed=True)
    assert frame[5:7] == (-1000).to_bytes(2, "big", signed=True)
    assert frame[7:9] == (-1).to_bytes(2, "big", signed=True)


def test_scale_documentation_candidate_uses_1000_multiplier():
    assert scale_documentation_candidate(0.25) == 250
    assert scale_documentation_candidate(-0.125) == -125


def test_ackermann_helper_keeps_y_zero_by_default():
    frame = build_ackermann_host_command_frame(0x00, 0x00, target_x=123, target_z=-456)

    assert frame[5:7] == (0).to_bytes(2, "big", signed=True)
    assert frame[3:5] == (123).to_bytes(2, "big", signed=True)
    assert frame[7:9] == (-456).to_bytes(2, "big", signed=True)


def test_native_host_command_builder_zero_frame_is_unchanged():
    frame = build_ackermann_host_command_frame(0x00, 0x00, target_x=0, target_z=0)

    assert frame.hex(" ") == "7b 00 00 00 00 00 00 00 00 7b 7d"


def test_ackermann_float_helper_keeps_y_zero_by_default():
    candidate = build_ackermann_host_command_frame_from_floats(
        0x00,
        0x00,
        target_x=0.1,
        target_z=-0.2,
    )

    assert candidate.target_x == 100
    assert candidate.target_y == 0
    assert candidate.target_z == -200
    assert candidate.transmit_allowed is False
    assert candidate.real_write_disabled is True


def test_build_host_command_frame_script_prints_disabled_status(capsys):
    module = load_script("build_c30d_host_command_frame.py")

    exit_code = module.main(["--target-x", "0.1", "--target-z", "-0.2"])

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "protocol_source: wheeltec_documentation_candidate" in output
    assert "frame_hex:" in output
    assert "checksum:" in output
    assert "transmit_allowed: false" in output
    assert "real_write_disabled: true" in output
    assert "native host-to-C30D" in output
    assert "must not be sent yet" in output
