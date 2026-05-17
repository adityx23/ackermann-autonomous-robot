import pytest

from ackermann_robot.drivers.c30d import C30DDriverError, MockC30DDriver
from ackermann_robot.messages import DriveCommand


def test_mock_driver_records_commands_in_memory():
    driver = MockC30DDriver()
    driver.connect()
    command = DriveCommand(speed_mps=0.1, steering_deg=3.0, timestamp_s=1.0)

    driver.send_drive_command(command)

    assert driver.commands == [command]
    assert driver.read_status().connected


def test_mock_driver_stop_records_neutral_command():
    driver = MockC30DDriver()
    driver.connect()

    driver.stop(timestamp_s=2.0)

    assert driver.commands[-1].speed_mps == 0.0
    assert driver.commands[-1].steering_deg == 0.0
    assert driver.commands[-1].timestamp_s == 2.0


def test_mock_driver_rejects_command_when_disconnected():
    driver = MockC30DDriver()

    with pytest.raises(C30DDriverError):
        driver.send_drive_command(DriveCommand())


def test_mock_driver_reports_status_failure():
    driver = MockC30DDriver(connected=True, fail_status=True)

    status = driver.read_status()

    assert status.failed
    assert status.fault == "status_failure"
