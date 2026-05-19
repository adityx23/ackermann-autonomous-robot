import pytest

from ackermann_robot.drivers.c30d import C30DDriverError, MockC30DDriver
from ackermann_robot.main import RobotSupervisor
from ackermann_robot.messages import DriveCommand


def test_supervisor_defaults_to_dry_run_safe_stop():
    driver = MockC30DDriver()
    supervisor = RobotSupervisor(driver=driver, clock=lambda: 1.0)

    state = supervisor.run(cycles=3, sleep=False)

    assert driver.is_connected
    assert len(driver.commands) == 3
    assert all(command.speed_mps == 0.0 for command in driver.commands)
    assert all(command.steering_deg == 0.0 for command in driver.commands)
    assert state.safety_status.stopped
    assert "manual_enable_required" in state.safety_status.reasons


def test_supervisor_rejects_non_dry_run_mode():
    with pytest.raises(ValueError, match="dry-run mode only"):
        RobotSupervisor(dry_run=False)


def test_supervisor_uses_mock_driver_without_serial_or_sensor_access():
    driver = MockC30DDriver()
    supervisor = RobotSupervisor(driver=driver, clock=lambda: 2.0)

    supervisor.step()

    assert isinstance(supervisor.driver, MockC30DDriver)
    assert driver.commands[-1].source == "safety"
    assert driver.read_status().connected


def test_supervisor_filters_enabled_command_through_limits():
    driver = MockC30DDriver()
    supervisor = RobotSupervisor(driver=driver, clock=lambda: 1.0)
    supervisor.safety_manager.set_manual_enabled(True)

    supervisor.step(
        now_s=1.0,
        command=DriveCommand(speed_mps=2.0, steering_deg=40.0, timestamp_s=1.0),
    )

    assert driver.commands[-1].speed_mps == 0.5
    assert driver.commands[-1].steering_deg == 25.0
    assert "clamped_speed" in supervisor.state.latest_filtered_command.reasons
    assert "clamped_steering" in supervisor.state.latest_filtered_command.reasons


def test_supervisor_surfaces_mock_command_failures():
    driver = MockC30DDriver(fail_commands=True)
    supervisor = RobotSupervisor(driver=driver, clock=lambda: 1.0)

    with pytest.raises(C30DDriverError):
        supervisor.step()

    assert driver.fault == "command_failure"
