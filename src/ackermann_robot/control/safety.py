from __future__ import annotations

from dataclasses import dataclass

from ackermann_robot.messages import C30DStatus, DriveCommand, SAFE_STOP_COMMAND, SafetyStatus


@dataclass(frozen=True)
class SafetyConfig:
    require_manual_enable: bool = True
    command_timeout_s: float = 0.5
    jetson_timeout_s: float = 0.5
    stop_on_jetson_disconnect: bool = True
    stop_on_c30d_failure: bool = True


class SafetyManager:
    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.config = config or SafetyConfig()
        self.manual_enabled = False

    def set_manual_enabled(self, enabled: bool) -> None:
        self.manual_enabled = enabled

    def evaluate(
        self,
        *,
        now_s: float,
        command: DriveCommand | None,
        c30d_status: C30DStatus,
        last_jetson_command_s: float | None = None,
    ) -> SafetyStatus:
        reasons: list[str] = []

        if self.config.require_manual_enable and not self.manual_enabled:
            reasons.append("manual_enable_required")

        if command is None or now_s - command.timestamp_s > self.config.command_timeout_s:
            reasons.append("command_stale")

        if (
            self.config.stop_on_jetson_disconnect
            and last_jetson_command_s is not None
            and now_s - last_jetson_command_s > self.config.jetson_timeout_s
        ):
            reasons.append("jetson_command_stale")

        if self.config.stop_on_c30d_failure and (c30d_status.failed or not c30d_status.connected):
            reasons.append("c30d_failed")

        stopped = bool(reasons)
        return SafetyStatus(enabled=not stopped, stopped=stopped, reasons=tuple(reasons))

    @staticmethod
    def safe_stop_command(timestamp_s: float = 0.0) -> DriveCommand:
        return DriveCommand(
            speed_mps=SAFE_STOP_COMMAND.speed_mps,
            steering_deg=SAFE_STOP_COMMAND.steering_deg,
            timestamp_s=timestamp_s,
            source=SAFE_STOP_COMMAND.source,
        )
