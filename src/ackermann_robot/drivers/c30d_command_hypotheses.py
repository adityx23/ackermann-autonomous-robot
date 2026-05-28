from __future__ import annotations

from dataclasses import dataclass

from ackermann_robot.drivers.c30d_checksum import xor_checksum

FRAME_START = 0x7B
FRAME_END = 0x7D
FRAME_LENGTH = 24
PAYLOAD_LENGTH = 21
CHECKSUM_INDEX = 22
HYPOTHESIS_LABEL = "unverified_hypothesis"


@dataclass(frozen=True)
class C30DCommandHypothesis:
    payload_1_to_21: bytes
    label: str = HYPOTHESIS_LABEL
    protocol_known: bool = False
    transmit_allowed: bool = False
    notes: str = (
        "Offline C30D-like frame hypothesis only. The real command protocol is unknown, "
        "and this must not be sent to a C30D controller."
    )


@dataclass(frozen=True)
class C30DCommandHypothesisFrame:
    hypothesis: C30DCommandHypothesis
    frame: bytes
    checksum: int
    label: str = HYPOTHESIS_LABEL
    protocol_known: bool = False
    transmit_allowed: bool = False


def build_hypothesis_frame(payload_1_to_21: bytes) -> bytes:
    """Build a C30D-like 24-byte frame hypothesis without transmitting anything."""
    if len(payload_1_to_21) != PAYLOAD_LENGTH:
        raise ValueError(f"payload_1_to_21 must contain exactly {PAYLOAD_LENGTH} bytes")

    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_START
    frame[1:CHECKSUM_INDEX] = payload_1_to_21
    frame[CHECKSUM_INDEX] = xor_checksum(bytes(frame[:CHECKSUM_INDEX]))
    frame[FRAME_LENGTH - 1] = FRAME_END
    return bytes(frame)


def build_command_hypothesis(payload_1_to_21: bytes) -> C30DCommandHypothesisFrame:
    hypothesis = C30DCommandHypothesis(payload_1_to_21=payload_1_to_21)
    frame = build_hypothesis_frame(payload_1_to_21)
    return C30DCommandHypothesisFrame(
        hypothesis=hypothesis,
        frame=frame,
        checksum=frame[CHECKSUM_INDEX],
    )
