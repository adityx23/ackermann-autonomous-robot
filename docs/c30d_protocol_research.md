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

## Command Packet Hypotheses

No command packet hypotheses are recorded yet.

Do not add a hypothesis here unless it is tied to a concrete evidence source such as an
official manual, vendor example, known driver, or tested demo. Do not infer command bytes
from feedback frames alone.

## Blocking Rule

Real motor or steering commands remain blocked until official C30D command documentation
or known-good command examples are available and reviewed. No code should write to the
C30D controller based on guesses.
