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
        "feedback_decoding: partially_understood_candidate_fields",
        "feedback_access: read_only",
        "command_protocol_known: false",
        "command_protocol_implemented: false",
        "real_motor_command_path: disabled",
        "serial_write_path: absent",
        "movement_enabled: false",
        "requirement_before_movement: official C30D command documentation or known-good examples",
    ]


def main() -> int:
    for line in protocol_status_lines():
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
