from __future__ import annotations

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from ackermann_robot.control.command_filter import CommandFilter, CommandLimits
from ackermann_robot.control.safety import SafetyConfig, SafetyManager
from ackermann_robot.drivers.c30d import C30DDriverError, MockC30DDriver
from ackermann_robot.messages import DriveCommand, SafetyStatus
from ackermann_robot.state import RobotState

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class SupervisorDefaults:
    control_rate_hz: int = 30
    limits: CommandLimits = CommandLimits()
    safety: SafetyConfig = SafetyConfig()


class RobotSupervisor:
    """Dry-run robot supervisor that composes safety, filtering, and the mock C30D."""

    def __init__(
        self,
        *,
        config_dir: str | Path = "config",
        dry_run: bool = True,
        driver: MockC30DDriver | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        if not dry_run:
            raise ValueError("RobotSupervisor currently supports dry-run mode only")

        self.dry_run = dry_run
        self.clock = clock
        self.sleeper = sleeper
        self.state = RobotState()
        self.driver = driver or MockC30DDriver()

        defaults = SupervisorDefaults()
        app_config = self._load_config(config_dir)
        if app_config is None:
            self.control_rate_hz = defaults.control_rate_hz
            safety_config = defaults.safety
            command_limits = defaults.limits
        else:
            self.control_rate_hz = app_config.robot.control.control_rate_hz
            safety_config = app_config.safety
            command_limits = CommandLimits(
                max_speed_mps=app_config.robot.limits.max_speed_mps,
                max_reverse_speed_mps=app_config.robot.limits.max_reverse_speed_mps,
                max_steering_deg=app_config.robot.limits.max_steering_deg,
                max_accel_mps2=app_config.robot.limits.max_accel_mps2,
            )

        self.safety_manager = SafetyManager(safety_config)
        self.command_filter = CommandFilter(command_limits)
        self._last_safety_status: SafetyStatus | None = None
        self._last_filtered_reasons: tuple[str, ...] | None = None

        self.driver.connect()
        self.state.c30d_status = self.driver.read_status()
        LOGGER.info("dry-run supervisor initialized at %s Hz", self.control_rate_hz)

    def run(self, *, cycles: int = 30, sleep: bool = True) -> RobotState:
        if cycles < 0:
            raise ValueError("cycles must be non-negative")

        period_s = 1.0 / max(1, self.control_rate_hz)
        for _ in range(cycles):
            started_s = self.clock()
            self.step(now_s=started_s)
            if sleep:
                elapsed_s = self.clock() - started_s
                self.sleeper(max(0.0, period_s - elapsed_s))
        return self.state

    def step(
        self, *, now_s: float | None = None, command: DriveCommand | None = None
    ) -> RobotState:
        now = self.clock() if now_s is None else now_s
        requested_command = command or DriveCommand(
            speed_mps=0.0,
            steering_deg=0.0,
            timestamp_s=now,
            source="dry_supervisor",
        )

        self.state.latest_command = requested_command
        self.state.last_local_command_s = now
        self.state.c30d_status = self.driver.read_status()
        self.state.safety_status = self.safety_manager.evaluate(
            now_s=now,
            command=requested_command,
            c30d_status=self.state.c30d_status,
            last_jetson_command_s=self.state.last_jetson_command_s,
        )
        self._log_safety_transition(self.state.safety_status)

        previous_filtered = self.state.latest_filtered_command.command
        self.state.latest_filtered_command = self.command_filter.filter(
            requested_command,
            safety_status=self.state.safety_status,
            previous_command=previous_filtered,
            now_s=now,
        )
        self._log_filter_transition(self.state.latest_filtered_command.reasons)

        try:
            self.driver.send_drive_command(self.state.latest_filtered_command.command)
        except C30DDriverError:
            LOGGER.exception("mock C30D command failed")
            self.state.c30d_status = self.driver.read_status()
            self.state.safety_status = self.safety_manager.evaluate(
                now_s=now,
                command=requested_command,
                c30d_status=self.state.c30d_status,
                last_jetson_command_s=self.state.last_jetson_command_s,
            )
            self.state.latest_filtered_command = self.command_filter.filter(
                requested_command,
                safety_status=self.state.safety_status,
                previous_command=previous_filtered,
                now_s=now,
            )
            if self.driver.is_connected:
                self.driver.stop(timestamp_s=now)
            raise

        self.state.c30d_status = self.driver.read_status()
        return self.state

    def _log_safety_transition(self, status: SafetyStatus) -> None:
        if self._last_safety_status == status:
            return
        LOGGER.info(
            "safety status changed: stopped=%s enabled=%s reasons=%s",
            status.stopped,
            status.enabled,
            ",".join(status.reasons) or "none",
        )
        self._last_safety_status = status

    def _log_filter_transition(self, reasons: tuple[str, ...]) -> None:
        if self._last_filtered_reasons == reasons:
            return
        LOGGER.info("command filter reasons changed: %s", ",".join(reasons) or "none")
        self._last_filtered_reasons = reasons

    @staticmethod
    def _load_config(config_dir: str | Path):
        try:
            from ackermann_robot.utils.config import load_config
        except ImportError:
            LOGGER.warning("config utilities unavailable; using safe supervisor defaults")
            return None

        return load_config(config_dir)
