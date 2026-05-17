from __future__ import annotations

from dataclasses import dataclass

from ackermann_robot.messages import DriveCommand, FilteredCommand, SafetyStatus
from ackermann_robot.control.safety import SafetyManager


@dataclass(frozen=True)
class CommandLimits:
    max_speed_mps: float = 0.5
    max_reverse_speed_mps: float = 0.2
    max_steering_deg: float = 25.0
    max_accel_mps2: float = 0.5


class CommandFilter:
    def __init__(self, limits: CommandLimits | None = None) -> None:
        self.limits = limits or CommandLimits()

    def filter(
        self,
        command: DriveCommand,
        *,
        safety_status: SafetyStatus,
        previous_command: DriveCommand | None = None,
        now_s: float | None = None,
    ) -> FilteredCommand:
        if safety_status.stopped:
            return FilteredCommand(
                SafetyManager.safe_stop_command(timestamp_s=now_s or command.timestamp_s),
                safety_status.reasons,
            )

        speed, speed_reasons = self._clamp_speed(command.speed_mps)
        steering, steering_reasons = self._clamp_steering(command.steering_deg)
        reasons = [*speed_reasons, *steering_reasons]

        if previous_command is not None:
            elapsed_s = max(0.0, command.timestamp_s - previous_command.timestamp_s)
            max_delta = self.limits.max_accel_mps2 * elapsed_s
            delta = speed - previous_command.speed_mps
            if delta > max_delta:
                speed = previous_command.speed_mps + max_delta
                reasons.append("clamped_acceleration")
            elif delta < -max_delta:
                speed = previous_command.speed_mps - max_delta
                reasons.append("clamped_acceleration")

        if not reasons:
            reasons.append("ok")

        return FilteredCommand(
            DriveCommand(
                speed_mps=speed,
                steering_deg=steering,
                timestamp_s=command.timestamp_s,
                source=command.source,
            ),
            tuple(reasons),
        )

    def _clamp_speed(self, speed_mps: float) -> tuple[float, list[str]]:
        reasons: list[str] = []
        if speed_mps > self.limits.max_speed_mps:
            speed_mps = self.limits.max_speed_mps
            reasons.append("clamped_speed")
        elif speed_mps < -self.limits.max_reverse_speed_mps:
            speed_mps = -self.limits.max_reverse_speed_mps
            reasons.append("clamped_reverse_speed")
        return speed_mps, reasons

    def _clamp_steering(self, steering_deg: float) -> tuple[float, list[str]]:
        reasons: list[str] = []
        if steering_deg > self.limits.max_steering_deg:
            steering_deg = self.limits.max_steering_deg
            reasons.append("clamped_steering")
        elif steering_deg < -self.limits.max_steering_deg:
            steering_deg = -self.limits.max_steering_deg
            reasons.append("clamped_steering")
        return steering_deg, reasons
