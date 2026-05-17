from __future__ import annotations

from dataclasses import dataclass, field

from ackermann_robot.messages import (
    C30DStatus,
    DriveCommand,
    FilteredCommand,
    MotorFeedback,
    SAFE_STOP_COMMAND,
    SafetyStatus,
)


@dataclass
class RobotState:
    latest_command: DriveCommand | None = None
    latest_filtered_command: FilteredCommand = field(
        default_factory=lambda: FilteredCommand(SAFE_STOP_COMMAND, ("initial_stop",))
    )
    c30d_status: C30DStatus = field(default_factory=C30DStatus)
    motor_feedback: MotorFeedback = field(default_factory=MotorFeedback)
    safety_status: SafetyStatus = field(default_factory=SafetyStatus)
    last_jetson_command_s: float | None = None
    last_local_command_s: float | None = None

    @staticmethod
    def is_stale(timestamp_s: float | None, now_s: float, timeout_s: float) -> bool:
        if timestamp_s is None:
            return True
        return now_s - timestamp_s > timeout_s
