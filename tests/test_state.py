from ackermann_robot.state import RobotState


def test_missing_timestamp_is_stale():
    assert RobotState.is_stale(None, now_s=10.0, timeout_s=0.5)


def test_timestamp_older_than_timeout_is_stale():
    assert RobotState.is_stale(9.0, now_s=10.0, timeout_s=0.5)


def test_recent_timestamp_is_not_stale():
    assert not RobotState.is_stale(9.8, now_s=10.0, timeout_s=0.5)
