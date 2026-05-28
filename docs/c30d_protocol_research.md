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
- Candidate feedback field names are provisional and not confirmed protocol labels.
- The C30D command protocol is unknown.
- C30D command protocol knowledge is required for movement with the current wiring.
- The real motor/steering command path is disabled.
- The dry-run command path only creates an `UNIMPLEMENTED` placeholder and never returns
  bytes to transmit.

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

No command packet hypotheses are recorded yet.

Do not add a hypothesis here unless it is tied to a concrete evidence source such as an
official manual, vendor example, known driver, or tested demo. Do not infer command bytes
from feedback frames alone.

## Blocking Rule

Real motor or steering commands remain blocked until official C30D command documentation
or known-good command examples are available and reviewed. No code should write to the
C30D controller based on guesses.
