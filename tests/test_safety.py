from ackermann_robot.control.safety import SafetyConfig, SafetyManager
from ackermann_robot.messages import C30DStatus, DriveCommand


def test_manual_enable_required_defaults_to_stop():
    manager = SafetyManager()

    status = manager.evaluate(
        now_s=1.0,
        command=DriveCommand(timestamp_s=1.0),
        c30d_status=C30DStatus(connected=True),
    )

    assert status.stopped
    assert "manual_enable_required" in status.reasons


def test_stale_command_stops_robot():
    manager = SafetyManager(SafetyConfig(require_manual_enable=False))

    status = manager.evaluate(
        now_s=2.0,
        command=DriveCommand(timestamp_s=1.0),
        c30d_status=C30DStatus(connected=True),
    )

    assert status.stopped
    assert "command_stale" in status.reasons


def test_stale_jetson_command_stops_robot():
    manager = SafetyManager(SafetyConfig(require_manual_enable=False))

    status = manager.evaluate(
        now_s=2.0,
        command=DriveCommand(timestamp_s=2.0),
        c30d_status=C30DStatus(connected=True),
        last_jetson_command_s=1.0,
    )

    assert status.stopped
    assert "jetson_command_stale" in status.reasons


def test_c30d_failure_stops_robot():
    manager = SafetyManager(SafetyConfig(require_manual_enable=False))

    status = manager.evaluate(
        now_s=1.0,
        command=DriveCommand(timestamp_s=1.0),
        c30d_status=C30DStatus(connected=True, failed=True),
    )

    assert status.stopped
    assert "c30d_failed" in status.reasons


def test_safe_stop_command_is_neutral():
    command = SafetyManager.safe_stop_command(timestamp_s=3.0)

    assert command.speed_mps == 0.0
    assert command.steering_deg == 0.0
    assert command.timestamp_s == 3.0


def test_all_clear_enables_command_path():
    manager = SafetyManager(SafetyConfig(require_manual_enable=False))

    status = manager.evaluate(
        now_s=1.0,
        command=DriveCommand(timestamp_s=1.0),
        c30d_status=C30DStatus(connected=True),
    )

    assert not status.stopped
    assert status.enabled
