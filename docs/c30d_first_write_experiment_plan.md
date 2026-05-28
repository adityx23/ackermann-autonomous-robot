# C30D First Write Experiment Plan

No C30D write experiment is approved yet. This document is a safety planning scaffold for a possible future test after a verified stop/neutral command packet is found and reviewed. It does not enable writing, does not approve motor movement, and does not claim any command packet is valid.

## Current Blocker

- No verified stop/neutral command packet exists.

Because this blocker remains unresolved, no first write experiment may run. The guarded harness in `scripts/c30d_write_test_harness.py` remains validation-only and prints `serial_write_allowed: false`, `real_write_disabled_in_code: true`, and `no bytes sent`. The readiness checker in `scripts/c30d_first_write_readiness.py` is also read-only: it runs or consumes preflight results, builds the native zero host command frame internally, validates that frame shape, and still sends no bytes.

## Required Preconditions

All of these must be true before any future write experiment is even considered. The readiness checker reports these as `readiness_allowed: true/false`; it does not enable writing.

- Battery charged above warning threshold.
- Wheels lifted.
- Robot physically restrained.
- Manual power cutoff available and reachable.
- Read-only preflight PASS.
- Checksum-valid packet only.
- Known stop/neutral hypothesis selected.
- Maximum test duration under 0.25 seconds for any future pulse.
- Stop/neutral packet repeated before and after the pulse, once a stop/neutral packet is known.

Readiness checker command shape:

    python scripts/c30d_first_write_readiness.py --wheels-lifted --robot-restrained --manual-power-cutoff-ready --motor-enable-switch-reviewed --i-understand-this-is-not-a-motor-test --c30d-only-preflight

The checker builds the zero native host command frame (`target_x=0`, `target_y=0`, `target_z=0`) internally and validates: 11-byte length, `0x7B` at byte 0, `0x7D` at byte 10, and XOR checksum over bytes 0-8. It reports `real_write_enabled: false` and `no bytes sent`. Readiness fails if the candidate battery voltage is below the warning threshold.

These are minimum planning preconditions, not approval to transmit. A future experiment still requires an explicit code change that enables a controlled write path, plus human approval at the robot.

## Abort Conditions

Abort immediately and remove power if any of these occur:

- Unexpected wheel motion.
- Steering twitch.
- Checksum invalid.
- C30D feedback drops.
- Battery warning or block condition.
- User not physically next to the robot.

## Non-Approval Statement

This plan documents a future experiment shape only. It does not enable serial writes, does not open `/dev/c30d`, does not send bytes, and does not move motors or steering. Real motor or steering commands remain blocked until official C30D command documentation or known-good command examples establish the command protocol and a verified stop/neutral packet.
