# C30D Protocol Research

This document tracks evidence for the C30D controller protocol. It is intentionally a
research scaffold, not an implementation plan for motor movement.

## Current Known Facts

- The C30D is the integrated motor, steering-servo, encoder, and onboard IMU controller
  in the current robot wiring.
- Both drive motors connect directly to the C30D through 6-pin JST connectors.
- The steering servo connects directly to the C30D.
- The 12V battery connects to the C30D, while the Raspberry Pi is powered separately.
- The IMU chip is integrated on the C30D, not on the Raspberry Pi.
- Passive read-only feedback captures show a fixed 24-byte frame shape.
- Feedback byte 0 is `0x7B`.
- Feedback byte 23 is `0x7D`.
- Feedback byte 22 is a feedback checksum byte.
- The feedback checksum is confirmed from saved captures as XOR of bytes 0 through 21,
  with the expected checksum stored at byte 22.
- Feedback byte 20 is often observed as `0x2A`.
- Feedback byte 21 may be status, counter, or mode data, but this is not confirmed.
- Feedback bytes 20-21 interpreted as big-endian uint16 produce values around
  10750-11010 in current captures. Because the 12V battery is connected to the C30D,
  this is tracked as `candidate_battery_mV`, but it is not confirmed as battery voltage.
- Provisional candidate battery thresholds live in `config/battery_safety.yaml`:
  warn below 10800 mV, block motor-test readiness below 10500 mV, and critical below
  10200 mV. These thresholds are safety scaffolding around a candidate field, not a
  confirmed C30D battery decoder.
- Feedback candidate fields exist in the current analysis scripts.
- Current feedback candidate field map:
  - `int16_be_02_03`: `candidate_forward_motion`
  - `int16_be_06_07`: `candidate_yaw_motion`
  - `int16_be_12_13`: `candidate_imu_12_13`
  - `int16_be_14_15`: `candidate_imu_14_15`
  - `int16_be_16_17`: `candidate_imu_16_17`
  - `int16_be_18_19`: `candidate_imu_18_19`
  - `uint16_be_20_21`: `candidate_battery_mV`
- Candidate feedback field names are provisional and not confirmed protocol labels.
- The C30D command protocol is unknown.
- C30D command protocol knowledge is required for movement with the current wiring.
- The real motor/steering command path is disabled.
- The dry-run command path only creates an `UNIMPLEMENTED` placeholder and never returns
  bytes to transmit.
- An offline command hypothesis builder exists for constructing C30D-like 24-byte frames
  labeled `unverified_hypothesis`. It is not a command implementation, does not transmit,
  and must not be sent to the C30D.

Architecture reference:

    docs/c30d_integrated_architecture.md

## Protocol Evidence Sources

Evidence required before any real motor command path is added:

- Official manual:
  - Not yet present in this repository.
- Vendor examples:
  - Not yet present in this repository.
- ROS driver references:
  - Not yet present in this repository.
- Arduino/Python demos:
  - Not yet present in this repository.

Local reference search:

    python scripts/search_c30d_protocol_references.py

The search tool reads local files under `external/`, `docs/`, `src/`, and `scripts/` and
prints keyword matches only. It does not access hardware and does not infer protocol
details from matches.

Checksum hypothesis analysis:

    python scripts/analyze_c30d_checksum.py data/c30d_captures/*.bin

The checksum analyzer reads only saved passive `.bin` captures. It showed a 100% match
for feedback byte 22 as XOR of bytes 0 through 21 across stationary, wheel-spin, and
manual-roll captures. The feedback frame parser now reports `checksum_valid` for every
parsed frame.

Analyze payload bytes 20-21 without assigning confirmed physical meaning:

    python scripts/analyze_c30d_payload_fields.py data/c30d_captures/*.bin

The payload-field analyzer reads only saved `.bin` captures, filters to checksum-valid
fixed-length frames, and reports uint16 big-endian bytes 20-21 stats, byte 20 unique
values, byte 21 unique values, checksum-valid/invalid frame counts, and
`candidate_battery_voltage_V = uint16_be_20_21 / 1000.0`. This remains candidate-only
analysis, not a confirmed battery-voltage decoder.

The command-packet checksum may use a similar rule, but that is not confirmed. Do not
apply the feedback checksum rule to commands without command-side evidence.

Pi-side preflight uses checksum validity and `candidate_battery_mV` only for health and
future motor-test readiness decisions. It does not send commands, and the real movement
path remains disabled.

## Command Packet Hypotheses

No known-good command packet has been found.

The repository includes a non-transmitting hypothesis lab for offline byte-shape work:

    python scripts/build_c30d_hypothesis_frame.py 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
    python scripts/build_c30d_hypothesis_frame.py --payload-hex "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"

This builds a 24-byte C30D-like frame with byte 0 set to `0x7B`, bytes 1-21 supplied by
the caller, byte 22 set to XOR of bytes 0-21, and byte 23 set to `0x7D`. Every output is
labeled `unverified_hypothesis`, `protocol_known: false`, and `transmit_allowed: false`.
The builder has no serial transmission path.

Summarize saved feedback captures into the stable observed frame template:

    python scripts/analyze_c30d_frame_structure.py data/c30d_captures/*.bin

This analyzer reads only saved `.bin` files. It prints the observed feedback start byte,
checksum rule, end byte, the current candidate field map with min/max/mean stats for
each capture, and unknown payload byte positions excluding bytes already covered by
candidate fields. It does not infer command fields.

Do not add a hypothesis here unless it is tied to a concrete evidence source such as an
official manual, vendor example, known driver, or tested demo. Do not infer command bytes
from feedback frames alone.

## Blocking Rule

Real motor or steering commands remain blocked until official C30D command documentation
or known-good command examples are available and reviewed. No code should write to the
C30D controller based on guesses.
