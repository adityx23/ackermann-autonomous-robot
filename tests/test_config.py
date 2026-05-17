from __future__ import annotations

import pytest

from ackermann_robot.control.safety import SafetyConfig
from ackermann_robot.utils.config import ConfigError, load_config, load_robot_config


def test_load_config_reads_all_yaml_files(tmp_path):
    write_config_files(tmp_path)

    config = load_config(tmp_path)

    assert config.robot.name == "test_robot"
    assert config.robot.geometry.wheelbase_m == 0.3
    assert config.robot.geometry.wheel_radius_m is None
    assert isinstance(config.safety, SafetyConfig)
    assert config.safety.require_manual_enable is True
    assert config.sensors.c30d.port == "/dev/mock-c30d"
    assert config.network.jetson.command_port == 5555


def test_missing_required_file_raises_clear_error(tmp_path):
    with pytest.raises(ConfigError, match="missing required config file"):
        load_robot_config(tmp_path)


def test_malformed_yaml_raises_clear_error(tmp_path):
    (tmp_path / "robot.yaml").write_text("robot: [", encoding="utf-8")

    with pytest.raises(ConfigError, match="malformed YAML"):
        load_robot_config(tmp_path)


def test_missing_required_value_raises_clear_error(tmp_path):
    (tmp_path / "robot.yaml").write_text("robot:\n  name: test\n", encoding="utf-8")

    with pytest.raises(ConfigError, match="missing required config value: mode"):
        load_robot_config(tmp_path)


def write_config_files(config_dir):
    (config_dir / "robot.yaml").write_text(
        """
robot:
  name: test_robot
  mode: test
  geometry:
    wheelbase_m: 0.30
    track_width_m: 0.20
    wheel_radius_m: null
  limits:
    max_speed_mps: 0.5
    max_reverse_speed_mps: 0.2
    max_steering_deg: 25.0
    max_accel_mps2: 0.5
  control:
    control_rate_hz: 30
    dry_run_default: true
""",
        encoding="utf-8",
    )
    (config_dir / "safety.yaml").write_text(
        """
safety:
  require_manual_enable: true
  command_timeout_s: 0.5
  jetson_timeout_s: 0.5
  stop_on_jetson_disconnect: true
  stop_on_c30d_failure: true
""",
        encoding="utf-8",
    )
    (config_dir / "sensors.yaml").write_text(
        """
sensors:
  c30d:
    interface: serial
    port: /dev/mock-c30d
    baudrate: 115200
    timeout_s: 0.1
  imu:
    model: ICM-20948
    interface: i2c
    i2c_bus: 1
    address: null
  lidar:
    model: RPLIDAR C1
    interface: usb_serial
    port: /dev/mock-lidar
    baudrate: 460800
  camera:
    model: OAK-D Lite
    interface: usb
    fps: 30
    resolution: 720p
""",
        encoding="utf-8",
    )
    (config_dir / "network.yaml").write_text(
        """
network:
  jetson:
    hostname: jetson.local
    ip: null
    command_port: 5555
    telemetry_port: 5556
    timeout_s: 0.5
  dashboard:
    host: 127.0.0.1
    port: 8000
""",
        encoding="utf-8",
    )
