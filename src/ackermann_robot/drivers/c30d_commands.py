from __future__ import annotations

from dataclasses import dataclass

UNIMPLEMENTED_PACKET_HEX = "UNIMPLEMENTED"
UNIMPLEMENTED_NOTES = (
    "Real C30D motor/steering command protocol is not implemented. "
    "This dry-run placeholder never returns serial bytes and must not be transmitted."
)


@dataclass(frozen=True)
class C30DCommandCandidate:
    speed_mps: float
    steering_deg: float
    duration_s: float
    source: str


@dataclass(frozen=True)
class C30DCommandPacketCandidate:
    command: C30DCommandCandidate
    packet_hex: str
    protocol_known: bool
    notes: str


def build_dry_run_command_packet(command: C30DCommandCandidate) -> C30DCommandPacketCandidate:
    return C30DCommandPacketCandidate(
        command=command,
        packet_hex=UNIMPLEMENTED_PACKET_HEX,
        protocol_known=False,
        notes=UNIMPLEMENTED_NOTES,
    )
