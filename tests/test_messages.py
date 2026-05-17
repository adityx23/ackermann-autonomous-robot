from ackermann_robot.messages import C30DStatus, DriveCommand, FilteredCommand, SafetyStatus


def test_drive_command_defaults_to_neutral():
    command = DriveCommand()

    assert command.speed_mps == 0.0
    assert command.steering_deg == 0.0


def test_filtered_command_exposes_command_values():
    filtered = FilteredCommand(DriveCommand(speed_mps=0.1, steering_deg=2.0), ("ok",))

    assert filtered.speed_mps == 0.1
    assert filtered.steering_deg == 2.0
    assert filtered.reasons == ("ok",)


def test_status_defaults_are_safe():
    assert C30DStatus().connected is False
    assert SafetyStatus().stopped is True
