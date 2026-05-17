from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class DriveCommand:
    speed_mps: float = 0.0
    steering_deg: float = 0.0
    timestamp_s: float = 0.0
    source: str = "unknown"


@dataclass(frozen=True)
class FilteredCommand:
    command: DriveCommand
    reasons: tuple[str, ...] = field(default_factory=tuple)

    @property
    def speed_mps(self) -> float:
        return self.command.speed_mps

    @property
    def steering_deg(self) -> float:
        return self.command.steering_deg


@dataclass(frozen=True)
class MotorFeedback:
    left_velocity_mps: float = 0.0
    right_velocity_mps: float = 0.0
    timestamp_s: float = 0.0
    valid: bool = False


@dataclass(frozen=True)
class C30DStatus:
    connected: bool = False
    last_communication_s: float = 0.0
    failed: bool = False
    fault: str | None = None


@dataclass(frozen=True)
class SafetyStatus:
    enabled: bool = False
    stopped: bool = True
    reasons: tuple[str, ...] = field(default_factory=tuple)


SAFE_STOP_COMMAND = DriveCommand(speed_mps=0.0, steering_deg=0.0, source="safety")
