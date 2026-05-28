from __future__ import annotations

import importlib.util
import inspect
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_command_hypotheses import build_hypothesis_frame
from ackermann_robot.drivers.c30d_host_command_frame import build_ackermann_host_command_frame


def load_harness_script():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "c30d_write_test_harness.py"
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def guarded_args(packet_hex: str) -> list[str]:
    return [
        "--armed",
        "--wheels-lifted",
        "--manual-enable",
        "--i-understand-risk",
        "--packet-hex",
        packet_hex,
    ]


def good_packet_hex() -> str:
    frame = build_hypothesis_frame(bytes([0x00] * 21))
    return frame.hex(" ")


def good_host_command_packet_hex() -> str:
    frame = build_ackermann_host_command_frame(0x00, 0x00, target_x=0, target_z=0)
    return frame.hex(" ")


def test_write_test_harness_refuses_without_flags(capsys):
    module = load_harness_script()

    exit_code = module.main([])

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "serial_write_allowed: false" in output
    assert "real_write_disabled_in_code: true" in output
    assert "no bytes sent" in output
    assert "refused: missing_required_safety_inputs" in output
    assert "armed: false" in output
    assert "packet_hex_provided: false" in output


def test_write_test_harness_validates_good_checksum(capsys):
    module = load_harness_script()

    exit_code = module.main(guarded_args(good_packet_hex()))

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "packet_valid: true" in output
    assert "packet_validation_reasons: ok" in output
    assert "serial_write_allowed: false" in output
    assert "real_write_disabled_in_code: true" in output
    assert "no bytes sent" in output
    assert "refused: real_write_disabled_in_code" in output


def test_write_test_harness_validates_11_byte_host_command_without_transmit(capsys):
    module = load_harness_script()

    exit_code = module.main(guarded_args(good_host_command_packet_hex()))

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "packet_valid: true" in output
    assert "packet_frame_type: host_command_candidate_11_byte" in output
    assert "serial_write_allowed: false" in output
    assert "real_write_disabled_in_code: true" in output
    assert "no bytes sent" in output
    assert "refused: real_write_disabled_in_code" in output


def test_write_test_harness_rejects_bad_checksum(capsys):
    module = load_harness_script()
    frame = bytearray(build_hypothesis_frame(bytes([0x00] * 21)))
    frame[22] ^= 0xFF

    exit_code = module.main(guarded_args(bytes(frame).hex(" ")))

    output = capsys.readouterr().out
    assert exit_code == 1
    assert "packet_valid: false" in output
    assert "checksum_byte_22_not_xor_bytes_0_through_21" in output
    assert "serial_write_allowed: false" in output
    assert "no bytes sent" in output


def test_write_test_harness_never_opens_serial_or_calls_write():
    module = load_harness_script()
    source = inspect.getsource(module)

    forbidden = (
        "import serial",
        "serial.Serial",
        "os.open",
        "open('/dev/c30d'",
        'open("/dev/c30d"',
        ".write(",
        "os.write",
        "/dev/c30d",
    )
    for token in forbidden:
        assert token not in source
