# C30D First Write Experiment Plan

A zero/neutral-only C30D first write path now exists at `scripts/send_c30d_zero_frame_once.py`. It is restricted to the hardcoded native host command frame `7b 00 00 00 00 00 00 00 00 7b 7d`. It does not approve motor movement, does not implement motor pulse commands, does not implement steering commands, accepts no speed/steering/target/packet inputs, and does not use ROS or ROS2.

## Approved First Write Scope

- Send exactly one zero/neutral native host command frame.
- Target X, Y, and Z must all be zero.
- Frame length must be 11 bytes.
- Byte 0 must be `0x7B` and byte 10 must be `0x7D`.
- Byte 9 must be the XOR checksum over bytes 0 through 8.
- The exact transmitted frame must be `7b 00 00 00 00 00 00 00 00 7b 7d`.

The guarded harness in `scripts/c30d_write_test_harness.py` remains validation-only and prints `serial_write_allowed: false`, `real_write_disabled_in_code: true`, and `no bytes sent`. The readiness checker in `scripts/c30d_first_write_readiness.py` is still read-only: it runs or consumes preflight results, builds the native zero host command frame internally, validates that frame shape, and sends no bytes.

## Required Preconditions

All of these must be true before the zero-frame first-write script may open `/dev/c30d`. The readiness checker reports these as `readiness_allowed: true/false`; the write script refuses if readiness is false. First-write readiness defaults to `preflight_mode: c30d_only` and a 5 second C30D stability window because C30D feedback stability and battery are the critical checks for a zero/neutral C30D serial frame.

- Battery charged above warning threshold.
- Wheels lifted.
- Robot physically restrained.
- Manual power cutoff available and reachable.
- Read-only C30D-only preflight PASS, including data directories, C30D feedback, and battery.
- C30D feedback frame rate at or above threshold.
- Zero invalid C30D feedback checksum frames during the stability window.
- Checksum-valid packet only.
- Known stop/neutral hypothesis selected.
- Maximum test duration under 0.25 seconds for any future pulse.
- Stop/neutral packet repeated before and after the pulse, once a stop/neutral packet is known.

Readiness checker command shape:

    python scripts/c30d_first_write_readiness.py --wheels-lifted --robot-restrained --manual-power-cutoff-ready --motor-enable-switch-reviewed --i-understand-this-is-not-a-motor-test

The checker builds the zero native host command frame (`target_x=0`, `target_y=0`, `target_z=0`) internally and validates: 11-byte length, `0x7B` at byte 0, `0x7D` at byte 10, and XOR checksum over bytes 0-8. It reports `preflight_mode`, `preflight_duration_s`, `invalid_checksum_count`, `real_write_enabled: false`, and `no bytes sent`. Readiness fails if candidate battery voltage is below the warning threshold, C30D frame rate is below threshold, or invalid checksum count is nonzero. If invalid checksums appear, the output says to rerun after checking USB/serial stability. RPLIDAR and OAK are not required in default C30D-only mode; use `--full-sensor-preflight` only for manual whole-sensor checks.

Zero-frame write command shape:

    python scripts/send_c30d_zero_frame_once.py --armed --manual-enable --wheels-lifted --robot-restrained --manual-power-cutoff-ready --motor-enable-switch-reviewed --i-understand-this-sends-a-real-serial-frame --c30d-only-preflight

Default behavior refuses to write. The script requires every listed guard flag, runs readiness internally, prints the exact frame hex before writing, opens `/dev/c30d` only after all checks pass, writes the 11-byte frame once, flushes, and closes. On success it reports `real_write_performed: true`, `bytes_written: 11`, `frame_hex`, and `warning: zero/neutral frame only, not a motor pulse`.

## Abort Conditions

Abort immediately and remove power if any of these occur:

- Unexpected wheel motion.
- Steering twitch.
- Checksum invalid.
- C30D feedback drops.
- Battery warning or block condition.
- User not physically next to the robot.

## Non-Approval Statement

This plan approves only the hardcoded zero/neutral first write described above. It does not approve motor pulses, steering commands, arbitrary packet sending, speed inputs, steering inputs, nonzero target values, or any ROS/ROS2 path. Real motor or steering commands remain blocked until official C30D command documentation or known-good command examples establish the command protocol beyond the zero/neutral frame.
