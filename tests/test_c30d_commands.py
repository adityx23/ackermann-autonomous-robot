from __future__ import annotations

import importlib.util
import inspect
import sys
from pathlib import Path

from ackermann_robot.drivers.c30d_commands import (
    C30DCommandCandidate,
    UNIMPLEMENTED_PACKET_HEX,
    build_dry_run_command_packet,
)


def load_protocol_status_script():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "c30d_command_protocol_status.py"
    )
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def test_command_packet_builder_returns_unknown_protocol_placeholder():
    command = C30DCommandCandidate(
        speed_mps=0.1,
        steering_deg=5.0,
        duration_s=1.0,
        source="test",
    )

    packet = build_dry_run_command_packet(command)

    assert packet.command == command
    assert packet.protocol_known is False
    assert packet.packet_hex == UNIMPLEMENTED_PACKET_HEX
    assert "not implemented" in packet.notes
    assert "never returns serial bytes" in packet.notes


def test_placeholder_packet_hex_does_not_contain_real_bytes():
    command = C30DCommandCandidate(0.1, 5.0, 1.0, "test")
    packet = build_dry_run_command_packet(command)

    assert packet.packet_hex in ("", "UNIMPLEMENTED")
    assert " " not in packet.packet_hex
    assert not packet.packet_hex.startswith("7b")


def test_c30d_command_protocol_status_reports_disabled(capsys):
    module = load_protocol_status_script()

    exit_code = module.main()

    output = capsys.readouterr().out
    assert exit_code == 0
    assert "hardware_role: integrated_motor_servo_encoder_imu_controller" in output
    assert "imu_location: integrated_on_c30d" in output
    assert "movement_requires_c30d_command_protocol: true" in output
    assert "command_protocol_known: false" in output
    assert "command_protocol_implemented: false" in output
    assert "command_hypothesis_builder_exists: true" in output
    assert "known_good_command_found: false" in output
    assert "real_motor_command_path: disabled" in output
    assert "command_transmission: disabled" in output
    assert "movement_blocked_until: command_protocol_discovered_safely" in output
    assert "serial_write_path: absent" in output


def test_no_serial_write_path_exists_in_command_placeholder_or_status_script():
    import ackermann_robot.drivers.c30d_commands as commands

    status_module = load_protocol_status_script()
    commands_source = inspect.getsource(commands)
    status_source = inspect.getsource(status_module)

    forbidden = ("os.open", "os.write", ".write(", "serial.Serial", "send_drive_command")
    for token in forbidden:
        assert token not in commands_source
        assert token not in status_source
