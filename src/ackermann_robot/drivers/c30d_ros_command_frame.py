from __future__ import annotations

from dataclasses import dataclass

from ackermann_robot.drivers.c30d_checksum import xor_checksum

FRAME_START = 0x7B
FRAME_END = 0x7D
FRAME_LENGTH = 11
CHECKSUM_INDEX = 9
SCALING_LABEL = "documentation_derived_candidate_scaled_by_1000_not_live_tested"
PROTOCOL_SOURCE = "wheeltec_documentation_candidate"


@dataclass(frozen=True)
class C30DRosCommandFrameCandidate:
    reserved_1: int
    reserved_2: int
    target_x: int
    target_y: int
    target_z: int
    frame: bytes
    checksum: int
    protocol_source: str = PROTOCOL_SOURCE
    scaling_label: str = SCALING_LABEL
    transmit_allowed: bool = False
    real_write_disabled: bool = True


def _validate_byte(value: int, name: str) -> int:
    if not 0 <= value <= 0xFF:
        raise ValueError(f"{name} must fit in one unsigned byte")
    return value


def _encode_int16_be(value: int, name: str) -> bytes:
    if not -32768 <= value <= 32767:
        raise ValueError(f"{name} must fit in signed int16")
    return int(value).to_bytes(2, "big", signed=True)


def scale_documentation_candidate(value: float, scale: int = 1000) -> int:
    """Scale a m/s or rad/s style value per Wheeltec docs; not live-tested here."""
    scaled = int(round(value * scale))
    if not -32768 <= scaled <= 32767:
        raise ValueError("scaled value must fit in signed int16")
    return scaled


def build_ros_command_frame(
    reserved_1: int,
    reserved_2: int,
    target_x: int,
    target_y: int,
    target_z: int,
) -> bytes:
    frame = bytearray()
    frame.append(FRAME_START)
    frame.append(_validate_byte(reserved_1, "reserved_1"))
    frame.append(_validate_byte(reserved_2, "reserved_2"))
    frame.extend(_encode_int16_be(target_x, "target_x"))
    frame.extend(_encode_int16_be(target_y, "target_y"))
    frame.extend(_encode_int16_be(target_z, "target_z"))
    frame.append(xor_checksum(bytes(frame[:CHECKSUM_INDEX])))
    frame.append(FRAME_END)
    return bytes(frame)


def build_ros_command_candidate(
    reserved_1: int,
    reserved_2: int,
    target_x: int,
    target_y: int,
    target_z: int,
) -> C30DRosCommandFrameCandidate:
    frame = build_ros_command_frame(reserved_1, reserved_2, target_x, target_y, target_z)
    return C30DRosCommandFrameCandidate(
        reserved_1=reserved_1,
        reserved_2=reserved_2,
        target_x=target_x,
        target_y=target_y,
        target_z=target_z,
        frame=frame,
        checksum=frame[CHECKSUM_INDEX],
    )


def build_ackermann_ros_command_frame(
    reserved_1: int,
    reserved_2: int,
    target_x: int,
    target_z: int,
    target_y: int = 0,
) -> bytes:
    return build_ros_command_frame(
        reserved_1=reserved_1,
        reserved_2=reserved_2,
        target_x=target_x,
        target_y=target_y,
        target_z=target_z,
    )


def build_ackermann_ros_command_frame_from_floats(
    reserved_1: int,
    reserved_2: int,
    target_x: float,
    target_z: float,
    target_y: float = 0.0,
) -> C30DRosCommandFrameCandidate:
    return build_ros_command_candidate(
        reserved_1=reserved_1,
        reserved_2=reserved_2,
        target_x=scale_documentation_candidate(target_x),
        target_y=scale_documentation_candidate(target_y),
        target_z=scale_documentation_candidate(target_z),
    )
