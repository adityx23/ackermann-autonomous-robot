from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from ackermann_robot.control.safety import SafetyConfig


class ConfigError(RuntimeError):
    """Raised when configuration files are missing or malformed."""


@dataclass(frozen=True)
class RobotGeometry:
    wheelbase_m: float
    track_width_m: float
    wheel_radius_m: float | None


@dataclass(frozen=True)
class RobotLimits:
    max_speed_mps: float
    max_reverse_speed_mps: float
    max_steering_deg: float
    max_accel_mps2: float


@dataclass(frozen=True)
class RobotControlConfig:
    control_rate_hz: int
    dry_run_default: bool


@dataclass(frozen=True)
class RobotConfig:
    name: str
    mode: str
    geometry: RobotGeometry
    limits: RobotLimits
    control: RobotControlConfig


@dataclass(frozen=True)
class C30DConfig:
    interface: str
    port: str
    baudrate: int
    timeout_s: float


@dataclass(frozen=True)
class ImuConfig:
    model: str
    interface: str
    i2c_bus: int | None
    address: int | None
    source: str | None


@dataclass(frozen=True)
class LidarConfig:
    model: str
    interface: str
    port: str
    baudrate: int


@dataclass(frozen=True)
class CameraConfig:
    model: str
    interface: str
    fps: int
    resolution: str


@dataclass(frozen=True)
class SensorConfig:
    c30d: C30DConfig
    imu: ImuConfig
    lidar: LidarConfig
    camera: CameraConfig


@dataclass(frozen=True)
class JetsonNetworkConfig:
    hostname: str
    ip: str | None
    command_port: int
    telemetry_port: int
    timeout_s: float


@dataclass(frozen=True)
class DashboardConfig:
    host: str
    port: int


@dataclass(frozen=True)
class NetworkConfig:
    jetson: JetsonNetworkConfig
    dashboard: DashboardConfig


@dataclass(frozen=True)
class AppConfig:
    robot: RobotConfig
    safety: SafetyConfig
    sensors: SensorConfig
    network: NetworkConfig


def load_config(config_dir: str | Path = "config") -> AppConfig:
    config_path = Path(config_dir)
    return AppConfig(
        robot=load_robot_config(config_path),
        safety=load_safety_config(config_path),
        sensors=load_sensor_config(config_path),
        network=load_network_config(config_path),
    )


def load_robot_config(config_dir: str | Path = "config") -> RobotConfig:
    data = _load_section(Path(config_dir), "robot.yaml", "robot")
    return RobotConfig(
        name=_required(data, "name", str),
        mode=_required(data, "mode", str),
        geometry=RobotGeometry(
            wheelbase_m=_required(data, "geometry.wheelbase_m", float),
            track_width_m=_required(data, "geometry.track_width_m", float),
            wheel_radius_m=_optional(data, "geometry.wheel_radius_m", float),
        ),
        limits=RobotLimits(
            max_speed_mps=_required(data, "limits.max_speed_mps", float),
            max_reverse_speed_mps=_required(data, "limits.max_reverse_speed_mps", float),
            max_steering_deg=_required(data, "limits.max_steering_deg", float),
            max_accel_mps2=_required(data, "limits.max_accel_mps2", float),
        ),
        control=RobotControlConfig(
            control_rate_hz=_required(data, "control.control_rate_hz", int),
            dry_run_default=_required(data, "control.dry_run_default", bool),
        ),
    )


def load_safety_config(config_dir: str | Path = "config") -> SafetyConfig:
    data = _load_section(Path(config_dir), "safety.yaml", "safety")
    return SafetyConfig(
        require_manual_enable=_required(data, "require_manual_enable", bool),
        command_timeout_s=_required(data, "command_timeout_s", float),
        jetson_timeout_s=_required(data, "jetson_timeout_s", float),
        stop_on_jetson_disconnect=_required(data, "stop_on_jetson_disconnect", bool),
        stop_on_c30d_failure=_required(data, "stop_on_c30d_failure", bool),
    )


def load_sensor_config(config_dir: str | Path = "config") -> SensorConfig:
    data = _load_section(Path(config_dir), "sensors.yaml", "sensors")
    return SensorConfig(
        c30d=C30DConfig(
            interface=_required(data, "c30d.interface", str),
            port=_required(data, "c30d.port", str),
            baudrate=_required(data, "c30d.baudrate", int),
            timeout_s=_required(data, "c30d.timeout_s", float),
        ),
        imu=_load_imu_config(data),
        lidar=LidarConfig(
            model=_required(data, "lidar.model", str),
            interface=_required(data, "lidar.interface", str),
            port=_required(data, "lidar.port", str),
            baudrate=_required(data, "lidar.baudrate", int),
        ),
        camera=CameraConfig(
            model=_required(data, "camera.model", str),
            interface=_required(data, "camera.interface", str),
            fps=_required(data, "camera.fps", int),
            resolution=_required(data, "camera.resolution", str),
        ),
    )


def _load_imu_config(data: dict[str, Any]) -> ImuConfig:
    model = _required(data, "imu.model", str)
    interface = _required(data, "imu.interface", str)

    if interface == "i2c":
        return ImuConfig(
            model=model,
            interface=interface,
            i2c_bus=_required(data, "imu.i2c_bus", int),
            address=_optional(data, "imu.address", int),
            source=None,
        )

    if interface == "c30d":
        return ImuConfig(
            model=model,
            interface=interface,
            i2c_bus=None,
            address=None,
            source=_required(data, "imu.source", str),
        )

    raise ConfigError(f"unsupported imu.interface: {interface}")


def load_network_config(config_dir: str | Path = "config") -> NetworkConfig:
    data = _load_section(Path(config_dir), "network.yaml", "network")
    return NetworkConfig(
        jetson=JetsonNetworkConfig(
            hostname=_required(data, "jetson.hostname", str),
            ip=_optional(data, "jetson.ip", str),
            command_port=_required(data, "jetson.command_port", int),
            telemetry_port=_required(data, "jetson.telemetry_port", int),
            timeout_s=_required(data, "jetson.timeout_s", float),
        ),
        dashboard=DashboardConfig(
            host=_required(data, "dashboard.host", str),
            port=_required(data, "dashboard.port", int),
        ),
    )


def _load_section(config_dir: Path, filename: str, section: str) -> dict[str, Any]:
    path = config_dir / filename
    if not path.exists():
        raise ConfigError(f"missing required config file: {path}")

    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"malformed YAML in {path}: {exc}") from exc

    if not isinstance(loaded, dict):
        raise ConfigError(f"config file must contain a mapping: {path}")

    section_data = loaded.get(section)
    if not isinstance(section_data, dict):
        raise ConfigError(f"missing or malformed '{section}' section in {path}")

    return section_data


def _required(data: dict[str, Any], dotted_key: str, expected_type: type) -> Any:
    value = _lookup(data, dotted_key)
    if value is None:
        raise ConfigError(f"missing required config value: {dotted_key}")
    if expected_type is float and isinstance(value, int):
        return float(value)
    if not isinstance(value, expected_type):
        raise ConfigError(
            f"config value '{dotted_key}' must be {expected_type.__name__}, "
            f"got {type(value).__name__}"
        )
    return value


def _optional(data: dict[str, Any], dotted_key: str, expected_type: type) -> Any:
    value = _lookup(data, dotted_key)
    if value is None:
        return None
    if expected_type is float and isinstance(value, int):
        return float(value)
    if not isinstance(value, expected_type):
        raise ConfigError(
            f"config value '{dotted_key}' must be {expected_type.__name__} or null, "
            f"got {type(value).__name__}"
        )
    return value


def _lookup(data: dict[str, Any], dotted_key: str) -> Any:
    value: Any = data
    for part in dotted_key.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value
