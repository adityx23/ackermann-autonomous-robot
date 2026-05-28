#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))


def protocol_status_lines() -> list[str]:
    return [
        "C30D Command Protocol Status",
        "hardware_role: integrated_motor_servo_encoder_imu_controller",
        "board_family: WHEELTEC_C30D_candidate",
        "host_uart: serial_port_3_candidate",
        "mcu: STM32F407VET6",
        "imu_candidate: ICM20948_new_version_or_MPU6050_old_version",
        "motor_wiring: drive_motors_connected_directly_to_c30d_6_pin_jst",
        "steering_wiring: steering_servo_connected_directly_to_c30d",
        "imu_location: integrated_on_c30d",
        "feedback_decoding: partially_understood_candidate_fields",
        "feedback_protocol: confirmed_from_observation_and_docs",
        "feedback_access: read_only",
        "movement_requires_c30d_command_protocol: true",
        "command_protocol: documented_candidate_not_live_tested",
        "command_protocol_known: false",
        "command_protocol_implemented: false",
        "command_hypothesis_builder_exists: true",
        "command_hypothesis_label: unverified_hypothesis",
        "known_good_command_found: false",
        "guarded_write_harness_exists: true",
        "first_write_plan_exists: true",
        "first_write_plan_approved: false",
        "real_motor_command_path: disabled",
        "real_write_enabled: false",
        "real_write_still_disabled: true",
        "command_transmission: disabled",
        "serial_write_path: absent",
        "movement_enabled: false",
        "movement_blocked_until: command_protocol_discovered_safely",
        "requirement_before_movement: official C30D command documentation or known-good examples",
    ]


def main() -> int:
    for line in protocol_status_lines():
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
