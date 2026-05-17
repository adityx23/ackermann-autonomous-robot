from ackermann_robot.control.command_filter import CommandFilter, CommandLimits
from ackermann_robot.messages import DriveCommand, SafetyStatus


def test_filter_clamps_forward_speed():
    command_filter = CommandFilter(CommandLimits(max_speed_mps=0.5))

    filtered = command_filter.filter(
        DriveCommand(speed_mps=2.0, timestamp_s=1.0),
        safety_status=SafetyStatus(enabled=True, stopped=False),
    )

    assert filtered.speed_mps == 0.5
    assert "clamped_speed" in filtered.reasons


def test_filter_clamps_reverse_speed():
    command_filter = CommandFilter(CommandLimits(max_reverse_speed_mps=0.2))

    filtered = command_filter.filter(
        DriveCommand(speed_mps=-1.0, timestamp_s=1.0),
        safety_status=SafetyStatus(enabled=True, stopped=False),
    )

    assert filtered.speed_mps == -0.2
    assert "clamped_reverse_speed" in filtered.reasons


def test_filter_clamps_steering():
    command_filter = CommandFilter(CommandLimits(max_steering_deg=25.0))

    filtered = command_filter.filter(
        DriveCommand(steering_deg=40.0, timestamp_s=1.0),
        safety_status=SafetyStatus(enabled=True, stopped=False),
    )

    assert filtered.steering_deg == 25.0
    assert "clamped_steering" in filtered.reasons


def test_filter_limits_acceleration():
    command_filter = CommandFilter(CommandLimits(max_accel_mps2=0.5))

    filtered = command_filter.filter(
        DriveCommand(speed_mps=1.0, timestamp_s=1.5),
        previous_command=DriveCommand(speed_mps=0.0, timestamp_s=1.0),
        safety_status=SafetyStatus(enabled=True, stopped=False),
    )

    assert filtered.speed_mps == 0.25
    assert "clamped_acceleration" in filtered.reasons


def test_filter_returns_safe_stop_when_safety_is_stopped():
    command_filter = CommandFilter()

    filtered = command_filter.filter(
        DriveCommand(speed_mps=0.2, steering_deg=10.0, timestamp_s=1.0),
        safety_status=SafetyStatus(stopped=True, reasons=("command_stale",)),
    )

    assert filtered.speed_mps == 0.0
    assert filtered.steering_deg == 0.0
    assert filtered.reasons == ("command_stale",)


def test_filter_marks_unchanged_command_ok():
    command_filter = CommandFilter()

    filtered = command_filter.filter(
        DriveCommand(speed_mps=0.2, steering_deg=10.0, timestamp_s=1.0),
        safety_status=SafetyStatus(enabled=True, stopped=False),
    )

    assert filtered.reasons == ("ok",)
