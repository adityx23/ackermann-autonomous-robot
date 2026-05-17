from __future__ import annotations

from dataclasses import dataclass, field

from ackermann_robot.messages import C30DStatus, DriveCommand, SAFE_STOP_COMMAND


class C30DDriverError(RuntimeError):
    """Raised by the mock driver when a configured failure is active."""


@dataclass
class MockC30DDriver:
    connected: bool = False
    fail_commands: bool = False
    fail_status: bool = False
    last_communication_s: float = 0.0
    commands: list[DriveCommand] = field(default_factory=list)
    fault: str | None = None

    def connect(self) -> None:
        self.connected = True
        self.fault = None

    def disconnect(self) -> None:
        self.connected = False

    @property
    def is_connected(self) -> bool:
        return self.connected

    def send_drive_command(self, command: DriveCommand) -> None:
        if not self.connected:
            raise C30DDriverError("mock C30D driver is not connected")
        if self.fail_commands:
            self.fault = "command_failure"
            raise C30DDriverError("mock C30D command failure")
        self.commands.append(command)
        self.last_communication_s = command.timestamp_s

    def stop(self, timestamp_s: float = 0.0) -> None:
        self.send_drive_command(
            DriveCommand(
                speed_mps=SAFE_STOP_COMMAND.speed_mps,
                steering_deg=SAFE_STOP_COMMAND.steering_deg,
                timestamp_s=timestamp_s,
                source="mock_c30d_stop",
            )
        )

    def read_status(self) -> C30DStatus:
        if self.fail_status:
            self.fault = "status_failure"
            return C30DStatus(
                connected=self.connected,
                last_communication_s=self.last_communication_s,
                failed=True,
                fault=self.fault,
            )
        return C30DStatus(
            connected=self.connected,
            last_communication_s=self.last_communication_s,
            failed=not self.connected or self.fault is not None,
            fault=self.fault,
        )
