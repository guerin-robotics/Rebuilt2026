from __future__ import annotations

import struct

import pytest

from wpilib_agent_tools.lib.wpilog_struct_decoder import decode_struct_value


def _rotation2d(radians: float) -> dict[str, float | str]:
    return {"type": "Rotation2d", "radians": radians}


def _translation2d(x: float, y: float) -> dict[str, float | str]:
    return {"type": "Translation2d", "x_meters": x, "y_meters": y}


def _pose2d(x: float, y: float, radians: float) -> dict[str, object]:
    return {"type": "Pose2d", "translation": _translation2d(x, y), "rotation": _rotation2d(radians)}


def _quaternion(w: float, x: float, y: float, z: float) -> dict[str, float | str]:
    return {"type": "Quaternion", "w": w, "x": x, "y": y, "z": z}


def _rotation3d(w: float, x: float, y: float, z: float) -> dict[str, object]:
    return {"type": "Rotation3d", "quaternion": _quaternion(w, x, y, z)}


def _translation3d(x: float, y: float, z: float) -> dict[str, float | str]:
    return {"type": "Translation3d", "x_meters": x, "y_meters": y, "z_meters": z}


def _pose3d(tx: float, ty: float, tz: float, w: float, x: float, y: float, z: float) -> dict[str, object]:
    return {
        "type": "Pose3d",
        "translation": _translation3d(tx, ty, tz),
        "rotation": _rotation3d(w, x, y, z),
    }


@pytest.mark.parametrize(
    ("type_str", "raw", "expected"),
    [
        ("struct:Rotation2d", struct.pack("<d", 0.75), _rotation2d(0.75)),
        ("struct:Translation2d", struct.pack("<dd", -1.25, 2.5), _translation2d(-1.25, 2.5)),
        ("struct:Pose2d", struct.pack("<ddd", 2.2, -0.7, 1.0), _pose2d(2.2, -0.7, 1.0)),
        ("struct:Quaternion", struct.pack("<dddd", 0.9, 0.1, 0.2, 0.3), _quaternion(0.9, 0.1, 0.2, 0.3)),
        ("struct:Rotation3d", struct.pack("<dddd", 0.8, -0.1, 0.4, -0.6), _rotation3d(0.8, -0.1, 0.4, -0.6)),
        ("struct:Translation3d", struct.pack("<ddd", 1.1, 2.2, 3.3), _translation3d(1.1, 2.2, 3.3)),
        (
            "struct:Pose3d",
            struct.pack("<ddddddd", 0.5, -1.0, 2.0, 1.0, 0.0, 0.1, -0.2),
            _pose3d(0.5, -1.0, 2.0, 1.0, 0.0, 0.1, -0.2),
        ),
        (
            "struct:Transform2d",
            struct.pack("<ddd", -3.0, 4.0, -0.25),
            {"type": "Transform2d", "translation": _translation2d(-3.0, 4.0), "rotation": _rotation2d(-0.25)},
        ),
        (
            "struct:Transform3d",
            struct.pack("<ddddddd", -1.0, 2.0, -3.0, 0.7, 0.2, 0.3, 0.4),
            {
                "type": "Transform3d",
                "translation": _translation3d(-1.0, 2.0, -3.0),
                "rotation": _rotation3d(0.7, 0.2, 0.3, 0.4),
            },
        ),
        (
            "struct:Twist2d",
            struct.pack("<ddd", 0.1, -0.2, 0.3),
            {"type": "Twist2d", "dx_meters": 0.1, "dy_meters": -0.2, "dtheta_radians": 0.3},
        ),
        (
            "struct:Twist3d",
            struct.pack("<dddddd", 1.0, -2.0, 3.0, 0.4, -0.5, 0.6),
            {
                "type": "Twist3d",
                "dx_meters": 1.0,
                "dy_meters": -2.0,
                "dz_meters": 3.0,
                "rx_radians": 0.4,
                "ry_radians": -0.5,
                "rz_radians": 0.6,
            },
        ),
        (
            "struct:Rectangle2d",
            struct.pack("<ddddd", 1.0, 2.0, 0.3, 4.0, 5.0),
            {"type": "Rectangle2d", "center": _pose2d(1.0, 2.0, 0.3), "x_width_meters": 4.0, "y_width_meters": 5.0},
        ),
        (
            "struct:Ellipse2d",
            struct.pack("<ddddd", -1.0, 0.5, -0.2, 1.2, 3.4),
            {
                "type": "Ellipse2d",
                "center": _pose2d(-1.0, 0.5, -0.2),
                "x_semi_axis_meters": 1.2,
                "y_semi_axis_meters": 3.4,
            },
        ),
        (
            "struct:ChassisSpeeds",
            struct.pack("<ddd", 1.5, -2.0, 3.25),
            {"type": "ChassisSpeeds", "vx_mps": 1.5, "vy_mps": -2.0, "omega_radps": 3.25},
        ),
        (
            "struct:SwerveModuleState",
            struct.pack("<dd", 4.1, 0.2),
            {"type": "SwerveModuleState", "speed_mps": 4.1, "angle": _rotation2d(0.2)},
        ),
        (
            "struct:SwerveModulePosition",
            struct.pack("<dd", 6.2, -0.4),
            {"type": "SwerveModulePosition", "distance_meters": 6.2, "angle": _rotation2d(-0.4)},
        ),
        (
            "struct:DifferentialDriveWheelSpeeds",
            struct.pack("<dd", 2.0, -2.1),
            {"type": "DifferentialDriveWheelSpeeds", "left_mps": 2.0, "right_mps": -2.1},
        ),
        (
            "struct:DifferentialDriveWheelPositions",
            struct.pack("<dd", 5.4, 5.9),
            {"type": "DifferentialDriveWheelPositions", "left_meters": 5.4, "right_meters": 5.9},
        ),
        (
            "struct:MecanumDriveWheelSpeeds",
            struct.pack("<dddd", 1.0, 2.0, 3.0, 4.0),
            {
                "type": "MecanumDriveWheelSpeeds",
                "front_left_mps": 1.0,
                "front_right_mps": 2.0,
                "rear_left_mps": 3.0,
                "rear_right_mps": 4.0,
            },
        ),
        (
            "struct:MecanumDriveWheelPositions",
            struct.pack("<dddd", -1.0, -2.0, -3.0, -4.0),
            {
                "type": "MecanumDriveWheelPositions",
                "front_left_meters": -1.0,
                "front_right_meters": -2.0,
                "rear_left_meters": -3.0,
                "rear_right_meters": -4.0,
            },
        ),
        (
            "struct:DifferentialDriveKinematics",
            struct.pack("<d", 0.66),
            {"type": "DifferentialDriveKinematics", "track_width_meters": 0.66},
        ),
        (
            "struct:MecanumDriveKinematics",
            struct.pack("<dddddddd", 1.0, 2.0, -1.0, 2.0, 1.0, -2.0, -1.0, -2.0),
            {
                "type": "MecanumDriveKinematics",
                "wheel_translations": {
                    "front_left": _translation2d(1.0, 2.0),
                    "front_right": _translation2d(-1.0, 2.0),
                    "rear_left": _translation2d(1.0, -2.0),
                    "rear_right": _translation2d(-1.0, -2.0),
                },
            },
        ),
        (
            "struct:SwerveDriveKinematics__3",
            struct.pack("<dddddd", 0.5, 0.5, 0.5, -0.5, -0.5, -0.5),
            {
                "type": "SwerveDriveKinematics",
                "module_count": 3,
                "module_translations": [
                    _translation2d(0.5, 0.5),
                    _translation2d(0.5, -0.5),
                    _translation2d(-0.5, -0.5),
                ],
            },
        ),
    ],
)
def test_decode_supported_structs(type_str: str, raw: bytes, expected: dict[str, object]) -> None:
    assert decode_struct_value(raw, type_str) == expected


def test_decode_struct_arrays() -> None:
    pose_array_raw = struct.pack("<dddddd", 1.0, 2.0, 0.1, -3.0, 4.0, -0.2)
    pose_array = decode_struct_value(pose_array_raw, "struct:Pose2d[]")
    assert pose_array == {
        "type": "Pose2d[]",
        "values": [_pose2d(1.0, 2.0, 0.1), _pose2d(-3.0, 4.0, -0.2)],
    }

    state_array_raw = struct.pack("<dddd", 4.1, 0.2, -1.0, -0.3)
    state_array = decode_struct_value(state_array_raw, "struct:SwerveModuleState[]")
    assert state_array == {
        "type": "SwerveModuleState[]",
        "values": [
            {"type": "SwerveModuleState", "speed_mps": 4.1, "angle": _rotation2d(0.2)},
            {"type": "SwerveModuleState", "speed_mps": -1.0, "angle": _rotation2d(-0.3)},
        ],
    }

    empty = decode_struct_value(b"", "struct:Translation3d[]")
    assert empty == {"type": "Translation3d[]", "values": []}


@pytest.mark.parametrize(
    "type_str",
    [
        "struct:Rotation2d",
        "struct:Translation2d",
        "struct:Pose2d",
        "struct:Quaternion",
        "struct:Rotation3d",
        "struct:Translation3d",
        "struct:Pose3d",
        "struct:Transform2d",
        "struct:Transform3d",
        "struct:Twist2d",
        "struct:Twist3d",
        "struct:Rectangle2d",
        "struct:Ellipse2d",
        "struct:ChassisSpeeds",
        "struct:SwerveModuleState",
        "struct:SwerveModulePosition",
        "struct:DifferentialDriveWheelSpeeds",
        "struct:DifferentialDriveWheelPositions",
        "struct:MecanumDriveWheelSpeeds",
        "struct:MecanumDriveWheelPositions",
        "struct:DifferentialDriveKinematics",
        "struct:MecanumDriveKinematics",
        "struct:SwerveDriveKinematics__4",
    ],
)
def test_decode_known_struct_rejects_invalid_size(type_str: str) -> None:
    assert decode_struct_value(b"\x01\x02\x03", type_str) is None


def test_decode_swerve_drive_kinematics_without_embedded_module_count() -> None:
    raw = struct.pack("<dddd", 1.0, 0.0, -1.0, 0.0)
    decoded = decode_struct_value(raw, "struct:SwerveDriveKinematics")
    assert decoded == {
        "type": "SwerveDriveKinematics",
        "module_count": 2,
        "module_translations": [_translation2d(1.0, 0.0), _translation2d(-1.0, 0.0)],
    }


def test_decode_unknown_array_struct_returns_none() -> None:
    assert decode_struct_value(b"\x00\x01", "struct:UnknownType[]") is None


def test_decode_malformed_swerve_kinematics_type_returns_none() -> None:
    assert decode_struct_value(b"\x00\x01", "struct:SwerveDriveKinematics__abc") is None


def test_decode_unknown_struct_returns_none() -> None:
    assert decode_struct_value(b"\x01\x02", "struct:UnknownType") is None
