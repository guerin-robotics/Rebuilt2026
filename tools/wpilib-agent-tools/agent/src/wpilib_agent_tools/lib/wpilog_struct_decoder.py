"""Decoders for common WPILib struct payloads in WPILOG entries."""

from __future__ import annotations

import struct
from collections.abc import Callable
from typing import Any


StructValue = dict[str, Any]
FixedDecoder = tuple[int, Callable[[bytes], StructValue | None]]

DOUBLE_SIZE = 8

ROTATION2D_SIZE = DOUBLE_SIZE
TRANSLATION2D_SIZE = DOUBLE_SIZE * 2
POSE2D_SIZE = TRANSLATION2D_SIZE + ROTATION2D_SIZE

QUATERNION_SIZE = DOUBLE_SIZE * 4
ROTATION3D_SIZE = QUATERNION_SIZE
TRANSLATION3D_SIZE = DOUBLE_SIZE * 3
POSE3D_SIZE = TRANSLATION3D_SIZE + ROTATION3D_SIZE

TWIST2D_SIZE = DOUBLE_SIZE * 3
TWIST3D_SIZE = DOUBLE_SIZE * 6

SWERVE_MODULE_SIZE = DOUBLE_SIZE + ROTATION2D_SIZE
DIFFERENTIAL_DRIVE_WHEELS_SIZE = DOUBLE_SIZE * 2
MECANUM_DRIVE_WHEELS_SIZE = DOUBLE_SIZE * 4
DIFFERENTIAL_DRIVE_KINEMATICS_SIZE = DOUBLE_SIZE
MECANUM_DRIVE_KINEMATICS_SIZE = TRANSLATION2D_SIZE * 4
SIMPLE_SHAPE2D_SIZE = POSE2D_SIZE + (DOUBLE_SIZE * 2)


def _unpack_doubles(raw: bytes, count: int) -> tuple[float, ...] | None:
    if len(raw) != count * DOUBLE_SIZE:
        return None
    return struct.unpack(f"<{count}d", raw)


def _decode_rotation2d(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 1)
    if values is None:
        return None
    (radians,) = values
    return {
        "type": "Rotation2d",
        "radians": radians,
    }


def _decode_translation2d(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 2)
    if values is None:
        return None
    x, y = values
    return {
        "type": "Translation2d",
        "x_meters": x,
        "y_meters": y,
    }


def _decode_pose2d(raw: bytes) -> dict[str, Any] | None:
    if len(raw) != POSE2D_SIZE:
        return None
    translation = _decode_translation2d(raw[:TRANSLATION2D_SIZE])
    rotation = _decode_rotation2d(raw[TRANSLATION2D_SIZE:])
    if translation is None or rotation is None:
        return None
    return {
        "type": "Pose2d",
        "translation": translation,
        "rotation": rotation,
    }


def _decode_quaternion(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 4)
    if values is None:
        return None
    w, x, y, z = values
    return {
        "type": "Quaternion",
        "w": w,
        "x": x,
        "y": y,
        "z": z,
    }


def _decode_rotation3d(raw: bytes) -> dict[str, Any] | None:
    quaternion = _decode_quaternion(raw)
    if quaternion is None:
        return None
    return {
        "type": "Rotation3d",
        "quaternion": quaternion,
    }


def _decode_translation3d(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 3)
    if values is None:
        return None
    x, y, z = values
    return {
        "type": "Translation3d",
        "x_meters": x,
        "y_meters": y,
        "z_meters": z,
    }


def _decode_pose3d(raw: bytes) -> dict[str, Any] | None:
    if len(raw) != POSE3D_SIZE:
        return None
    translation = _decode_translation3d(raw[:TRANSLATION3D_SIZE])
    rotation = _decode_rotation3d(raw[TRANSLATION3D_SIZE:])
    if translation is None or rotation is None:
        return None
    return {
        "type": "Pose3d",
        "translation": translation,
        "rotation": rotation,
    }


def _decode_transform2d(raw: bytes) -> dict[str, Any] | None:
    pose_like = _decode_pose2d(raw)
    if pose_like is None:
        return None
    return {
        "type": "Transform2d",
        "translation": pose_like["translation"],
        "rotation": pose_like["rotation"],
    }


def _decode_transform3d(raw: bytes) -> dict[str, Any] | None:
    pose_like = _decode_pose3d(raw)
    if pose_like is None:
        return None
    return {
        "type": "Transform3d",
        "translation": pose_like["translation"],
        "rotation": pose_like["rotation"],
    }


def _decode_twist2d(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 3)
    if values is None:
        return None
    dx, dy, dtheta = values
    return {
        "type": "Twist2d",
        "dx_meters": dx,
        "dy_meters": dy,
        "dtheta_radians": dtheta,
    }


def _decode_twist3d(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 6)
    if values is None:
        return None
    dx, dy, dz, rx, ry, rz = values
    return {
        "type": "Twist3d",
        "dx_meters": dx,
        "dy_meters": dy,
        "dz_meters": dz,
        "rx_radians": rx,
        "ry_radians": ry,
        "rz_radians": rz,
    }


def _decode_rectangle2d(raw: bytes) -> dict[str, Any] | None:
    if len(raw) != SIMPLE_SHAPE2D_SIZE:
        return None
    center = _decode_pose2d(raw[:POSE2D_SIZE])
    sizes = _unpack_doubles(raw[POSE2D_SIZE:], 2)
    if center is None or sizes is None:
        return None
    x_width, y_width = sizes
    return {
        "type": "Rectangle2d",
        "center": center,
        "x_width_meters": x_width,
        "y_width_meters": y_width,
    }


def _decode_ellipse2d(raw: bytes) -> dict[str, Any] | None:
    if len(raw) != SIMPLE_SHAPE2D_SIZE:
        return None
    center = _decode_pose2d(raw[:POSE2D_SIZE])
    radii = _unpack_doubles(raw[POSE2D_SIZE:], 2)
    if center is None or radii is None:
        return None
    x_axis, y_axis = radii
    return {
        "type": "Ellipse2d",
        "center": center,
        "x_semi_axis_meters": x_axis,
        "y_semi_axis_meters": y_axis,
    }


def _decode_chassis_speeds(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 3)
    if values is None:
        return None
    vx, vy, omega = values
    return {
        "type": "ChassisSpeeds",
        "vx_mps": vx,
        "vy_mps": vy,
        "omega_radps": omega,
    }


def _decode_swerve_module_state(raw: bytes) -> dict[str, Any] | None:
    if len(raw) != SWERVE_MODULE_SIZE:
        return None
    speed_values = _unpack_doubles(raw[:DOUBLE_SIZE], 1)
    angle = _decode_rotation2d(raw[DOUBLE_SIZE:])
    if speed_values is None or angle is None:
        return None
    (speed,) = speed_values
    return {
        "type": "SwerveModuleState",
        "speed_mps": speed,
        "angle": angle,
    }


def _decode_swerve_module_position(raw: bytes) -> dict[str, Any] | None:
    if len(raw) != SWERVE_MODULE_SIZE:
        return None
    distance_values = _unpack_doubles(raw[:DOUBLE_SIZE], 1)
    angle = _decode_rotation2d(raw[DOUBLE_SIZE:])
    if distance_values is None or angle is None:
        return None
    (distance,) = distance_values
    return {
        "type": "SwerveModulePosition",
        "distance_meters": distance,
        "angle": angle,
    }


def _decode_differential_drive_wheel_speeds(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 2)
    if values is None:
        return None
    left, right = values
    return {
        "type": "DifferentialDriveWheelSpeeds",
        "left_mps": left,
        "right_mps": right,
    }


def _decode_differential_drive_wheel_positions(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 2)
    if values is None:
        return None
    left, right = values
    return {
        "type": "DifferentialDriveWheelPositions",
        "left_meters": left,
        "right_meters": right,
    }


def _decode_mecanum_drive_wheel_speeds(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 4)
    if values is None:
        return None
    front_left, front_right, rear_left, rear_right = values
    return {
        "type": "MecanumDriveWheelSpeeds",
        "front_left_mps": front_left,
        "front_right_mps": front_right,
        "rear_left_mps": rear_left,
        "rear_right_mps": rear_right,
    }


def _decode_mecanum_drive_wheel_positions(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 4)
    if values is None:
        return None
    front_left, front_right, rear_left, rear_right = values
    return {
        "type": "MecanumDriveWheelPositions",
        "front_left_meters": front_left,
        "front_right_meters": front_right,
        "rear_left_meters": rear_left,
        "rear_right_meters": rear_right,
    }


def _decode_differential_drive_kinematics(raw: bytes) -> dict[str, Any] | None:
    values = _unpack_doubles(raw, 1)
    if values is None:
        return None
    (track_width,) = values
    return {
        "type": "DifferentialDriveKinematics",
        "track_width_meters": track_width,
    }


def _decode_mecanum_drive_kinematics(raw: bytes) -> dict[str, Any] | None:
    if len(raw) != MECANUM_DRIVE_KINEMATICS_SIZE:
        return None
    wheels: list[dict[str, Any]] = []
    for index in range(0, len(raw), TRANSLATION2D_SIZE):
        wheel = _decode_translation2d(raw[index : index + TRANSLATION2D_SIZE])
        if wheel is None:
            return None
        wheels.append(wheel)
    return {
        "type": "MecanumDriveKinematics",
        "wheel_translations": {
            "front_left": wheels[0],
            "front_right": wheels[1],
            "rear_left": wheels[2],
            "rear_right": wheels[3],
        },
    }


def _decode_swerve_drive_kinematics(raw: bytes, module_count: int | None = None) -> dict[str, Any] | None:
    if module_count is None:
        if len(raw) % TRANSLATION2D_SIZE != 0:
            return None
        module_count = len(raw) // TRANSLATION2D_SIZE

    if module_count < 0:
        return None

    expected_size = module_count * TRANSLATION2D_SIZE
    if len(raw) != expected_size:
        return None

    modules: list[dict[str, Any]] = []
    for index in range(0, expected_size, TRANSLATION2D_SIZE):
        module = _decode_translation2d(raw[index : index + TRANSLATION2D_SIZE])
        if module is None:
            return None
        modules.append(module)

    return {
        "type": "SwerveDriveKinematics",
        "module_count": module_count,
        "module_translations": modules,
    }


def _decode_struct_array(
    raw: bytes,
    element_type_name: str,
    element_size: int,
    element_decoder: Callable[[bytes], dict[str, Any] | None],
) -> dict[str, Any] | None:
    if element_size <= 0:
        return None
    if len(raw) % element_size != 0:
        return None

    values: list[dict[str, Any]] = []
    for index in range(0, len(raw), element_size):
        decoded = element_decoder(raw[index : index + element_size])
        if decoded is None:
            return None
        values.append(decoded)

    return {
        "type": f"{element_type_name}[]",
        "values": values,
    }


FIXED_DECODERS: dict[str, FixedDecoder] = {
    "Rotation2d": (ROTATION2D_SIZE, _decode_rotation2d),
    "Translation2d": (TRANSLATION2D_SIZE, _decode_translation2d),
    "Pose2d": (POSE2D_SIZE, _decode_pose2d),
    "Quaternion": (QUATERNION_SIZE, _decode_quaternion),
    "Rotation3d": (ROTATION3D_SIZE, _decode_rotation3d),
    "Translation3d": (TRANSLATION3D_SIZE, _decode_translation3d),
    "Pose3d": (POSE3D_SIZE, _decode_pose3d),
    "Transform2d": (POSE2D_SIZE, _decode_transform2d),
    "Transform3d": (POSE3D_SIZE, _decode_transform3d),
    "Twist2d": (TWIST2D_SIZE, _decode_twist2d),
    "Twist3d": (TWIST3D_SIZE, _decode_twist3d),
    "Rectangle2d": (SIMPLE_SHAPE2D_SIZE, _decode_rectangle2d),
    "Ellipse2d": (SIMPLE_SHAPE2D_SIZE, _decode_ellipse2d),
    "ChassisSpeeds": (TWIST2D_SIZE, _decode_chassis_speeds),
    "SwerveModuleState": (SWERVE_MODULE_SIZE, _decode_swerve_module_state),
    "SwerveModulePosition": (SWERVE_MODULE_SIZE, _decode_swerve_module_position),
    "DifferentialDriveWheelSpeeds": (DIFFERENTIAL_DRIVE_WHEELS_SIZE, _decode_differential_drive_wheel_speeds),
    "DifferentialDriveWheelPositions": (
        DIFFERENTIAL_DRIVE_WHEELS_SIZE,
        _decode_differential_drive_wheel_positions,
    ),
    "MecanumDriveWheelSpeeds": (MECANUM_DRIVE_WHEELS_SIZE, _decode_mecanum_drive_wheel_speeds),
    "MecanumDriveWheelPositions": (MECANUM_DRIVE_WHEELS_SIZE, _decode_mecanum_drive_wheel_positions),
    "DifferentialDriveKinematics": (DIFFERENTIAL_DRIVE_KINEMATICS_SIZE, _decode_differential_drive_kinematics),
    "MecanumDriveKinematics": (MECANUM_DRIVE_KINEMATICS_SIZE, _decode_mecanum_drive_kinematics),
}


def decode_struct_value(raw: bytes, type_str: str) -> dict[str, Any] | None:
    """Decode known `struct:*` WPILOG payloads into readable dicts."""
    type_name = type_str[7:] if type_str.startswith("struct:") else type_str

    if type_name.endswith("[]"):
        element_type = type_name[:-2]
        decoder_info = FIXED_DECODERS.get(element_type)
        if decoder_info is None:
            return None
        element_size, element_decoder = decoder_info
        return _decode_struct_array(raw, element_type, element_size, element_decoder)

    if type_name.startswith("SwerveDriveKinematics__"):
        module_count_str = type_name.split("__", 1)[1]
        if not module_count_str.isdigit():
            return None
        return _decode_swerve_drive_kinematics(raw, module_count=int(module_count_str))

    if type_name == "SwerveDriveKinematics":
        return _decode_swerve_drive_kinematics(raw)

    decoder_info = FIXED_DECODERS.get(type_name)
    if decoder_info is None:
        return None
    _, decoder = decoder_info
    return decoder(raw)
