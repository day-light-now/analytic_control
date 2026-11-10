"""Authoritative shoulder->arm-base mounting presets for the power-port facing.

Each Nero base is bolted to its shoulder bracket. Re-bolting the base one
quarter turn turns the connector panel (the local -X face of base_link) from
facing forward to facing down; the operator currently edits the
"shoulder to base RPY" row by hand every time that happens. This module owns
the one derivation of that installation rotation, so frames/IK/persistence/3D
all consume the same rigid transform.

Conventions match transforms.py: R = Rz(yaw) @ Ry(pitch) @ Rx(roll) is the
active rotation from the arm-base frame into the parent (shoulder/horizon)
frame; a homogeneous transform T = [R | t] maps child coordinates to parent
coordinates. "Front" is the original URDF/model installation and must stay the
historical config.py defaults; "Down" is Front rotated by the derived rigid
90 deg installation rotation. RPY output is presentation only: gimbal-lock
comparisons are done on rotation matrices.

The physical axis assumptions are documented and flagged in
docs/power_port_mounting_20261107/设计节点.md §3; they are NOT a measurement of
the real arms.
"""
from __future__ import annotations

import math
from typing import Mapping

import numpy as np

from transforms import (
    rotation_matrix_from_rpy_deg,
    rpy_deg_from_rotation_matrix,
    transform_from_rotation_and_translation,
)

SIDES = ("left", "right")

POWER_PORT_FACING_FRONT = "front"
POWER_PORT_FACING_DOWN = "down"
POWER_PORT_FACING_CUSTOM = "custom"
POWER_PORT_FACING_OPTIONS = (
    POWER_PORT_FACING_FRONT,
    POWER_PORT_FACING_DOWN,
    POWER_PORT_FACING_CUSTOM,
)
POWER_PORT_FACING_LABELS = {
    POWER_PORT_FACING_FRONT: "Front",
    POWER_PORT_FACING_DOWN: "Down",
    POWER_PORT_FACING_CUSTOM: "Custom",
}

# Front = original model installation. These must remain identical to the
# historical config.py left/right base RPY defaults: Front is the
# backward-compatible baseline and is never re-derived from Euler strings.
FRONT_RPY_DEG: Mapping[str, tuple[float, float, float]] = {
    "left": (-90.0, -90.0, 90.0),
    "right": (0.0, -90.0, 180.0),
}
FRONT_ROTATION: Mapping[str, np.ndarray] = {
    side: rotation_matrix_from_rpy_deg(rpy) for side, rpy in FRONT_RPY_DEG.items()
}

# Connector / power-port panel normal in the arm-base frame. base_link.stl has
# the connector panel on its local -X face (and the -X side carries the
# protruding sockets); base_link +Z is the mounting / joint1 axis. See the
# design node for the mesh inspection evidence.
POWER_PORT_LOCAL_VECTOR = np.array([-1.0, 0.0, 0.0])
# Horizon/shoulder frame: +X right, +Y up, -Z forward (README "PICO input").
HORIZON_FORWARD = np.array([0.0, 0.0, -1.0])
HORIZON_DOWN = np.array([0.0, -1.0, 0.0])

# Shortest 90 deg rigid rotation taking the port direction from forward to
# down, expressed in horizon coordinates: axis = normalize(forward x down) =
# (-1, 0, 0), angle = acos(forward . down) = 90 deg, i.e. Rx(-90 deg) about
# horizon +X (equivalently Rx(+90 deg) about horizon -X, the operator's left
# axis). Derived from the horizon vectors instead of a second hard-coded axis.
FRONT_TO_DOWN_AXIS_HORIZON = np.cross(HORIZON_FORWARD, HORIZON_DOWN)
FRONT_TO_DOWN_AXIS_HORIZON = FRONT_TO_DOWN_AXIS_HORIZON / np.linalg.norm(
    FRONT_TO_DOWN_AXIS_HORIZON
)
FRONT_TO_DOWN_ANGLE_DEG = math.degrees(
    math.acos(max(-1.0, min(1.0, float(np.dot(HORIZON_FORWARD, HORIZON_DOWN)))))
)


def _axis_angle_rotation(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(3)
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-12:
        raise ValueError("installation rotation axis is degenerate")
    x, y, z = axis / norm
    angle = math.radians(angle_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    one = 1.0 - cosine
    return np.array(
        [
            [x * x * one + cosine, x * y * one - z * sine, x * z * one + y * sine],
            [y * x * one + z * sine, y * y * one + cosine, y * z * one - x * sine],
            [z * x * one - y * sine, z * y * one + x * sine, z * z * one + cosine],
        ],
        dtype=float,
    )


# Active rotation in the parent/horizon frame: R_down = A @ R_front. This is
# exactly the body-fixed "re-bolt the base about its own mounting axis" motion
# conjugated into the base frame, so the base origin translation is untouched.
FRONT_TO_DOWN_ROTATION = _axis_angle_rotation(
    FRONT_TO_DOWN_AXIS_HORIZON, FRONT_TO_DOWN_ANGLE_DEG
)
DOWN_ROTATION: Mapping[str, np.ndarray] = {
    side: FRONT_TO_DOWN_ROTATION @ front for side, front in FRONT_ROTATION.items()
}
# RPY presentation of the derived matrices (ZYX convention). The matrices above
# stay authoritative; settings_store validates any stored RPY against them.
DOWN_RPY_DEG: Mapping[str, tuple[float, float, float]] = {
    side: rpy_deg_from_rotation_matrix(rotation)
    for side, rotation in DOWN_ROTATION.items()
}

PRESET_RPY_DEG: Mapping[str, Mapping[str, tuple[float, float, float]]] = {
    POWER_PORT_FACING_FRONT: FRONT_RPY_DEG,
    POWER_PORT_FACING_DOWN: DOWN_RPY_DEG,
}
PRESET_ROTATION: Mapping[str, Mapping[str, np.ndarray]] = {
    POWER_PORT_FACING_FRONT: FRONT_ROTATION,
    POWER_PORT_FACING_DOWN: DOWN_ROTATION,
}


def normalize_facing(facing: str) -> str:
    value = str(facing).strip().lower()
    if value not in POWER_PORT_FACING_OPTIONS:
        raise ValueError(
            f"power port facing must be one of {POWER_PORT_FACING_OPTIONS}, got {facing!r}"
        )
    return value


def _check_side(side: str) -> str:
    if side not in SIDES:
        raise ValueError(f"unknown arm side {side!r}")
    return side


def preset_rotation(side: str, facing: str) -> np.ndarray:
    """Preset base rotation for one side; ``custom`` has no preset."""
    facing = normalize_facing(facing)
    _check_side(side)
    if facing == POWER_PORT_FACING_CUSTOM:
        raise ValueError("custom facing has no preset rotation; read the manual RPY instead")
    return PRESET_ROTATION[facing][side].copy()


def shoulder_to_base_rotation(
    side: str,
    facing: str,
    rpy_deg: tuple[float, float, float],
) -> np.ndarray:
    """Effective shoulder->arm-base rotation for preset or manual RPY."""
    facing = normalize_facing(facing)
    if facing == POWER_PORT_FACING_CUSTOM:
        return rotation_matrix_from_rpy_deg(rpy_deg)
    return preset_rotation(side, facing)


def shoulder_to_base_transform(
    side: str,
    translation_m: tuple[float, float, float],
    facing: str,
    rpy_deg: tuple[float, float, float],
) -> np.ndarray:
    """Effective shoulder->arm-base rigid transform: rotation + untouched origin."""
    rotation = shoulder_to_base_rotation(side, facing, rpy_deg)
    return transform_from_rotation_and_translation(rotation, translation_m)


def preset_rotation_matches(
    side: str,
    facing: str,
    rotation: np.ndarray,
    atol: float = 1e-6,
) -> bool:
    """True when ``rotation`` equals the preset rotation as a matrix (SO(3) compare)."""
    facing = normalize_facing(facing)
    _check_side(side)
    if facing == POWER_PORT_FACING_CUSTOM:
        return False
    return bool(np.allclose(rotation, PRESET_ROTATION[facing][side], atol=atol, rtol=0.0))


def power_port_direction_in_horizon(
    side: str,
    facing: str,
    rpy_deg: tuple[float, float, float],
) -> np.ndarray:
    """Where the connector-panel normal points in the horizon/shoulder frame."""
    return shoulder_to_base_rotation(side, facing, rpy_deg) @ POWER_PORT_LOCAL_VECTOR


def validate_preset_rotation(
    side: str,
    facing: str,
    rpy_deg: tuple[float, float, float],
    atol: float = 1e-6,
) -> None:
    """Reject persisted data that claims a preset while storing another rotation."""
    facing = normalize_facing(facing)
    _check_side(side)
    if facing == POWER_PORT_FACING_CUSTOM:
        return
    rotation = rotation_matrix_from_rpy_deg(rpy_deg)
    if not preset_rotation_matches(side, facing, rotation, atol=atol):
        raise ValueError(
            f"power_port_facing_{side}={facing} does not match the stored "
            f"shoulder-to-base RPY; use power_port_facing_{side}=custom for a manual RPY"
        )


def _direction_text(direction: np.ndarray) -> str:
    if np.allclose(direction, HORIZON_FORWARD, atol=1e-9):
        return "forward (horizon -Z)"
    if np.allclose(direction, HORIZON_DOWN, atol=1e-9):
        return "down (horizon -Y)"
    return f"({direction[0]:+.3f}, {direction[1]:+.3f}, {direction[2]:+.3f})"


def describe_facing(side: str, facing: str) -> str:
    """Read-only effective-transform hint for the Settings row."""
    facing = normalize_facing(facing)
    _check_side(side)
    if facing in PRESET_RPY_DEG:
        direction = power_port_direction_in_horizon(side, facing, PRESET_RPY_DEG[facing][side])
        title = "Front preset" if facing == POWER_PORT_FACING_FRONT else "Down preset"
        return (
            f"{title}: power port faces {_direction_text(direction)}; "
            f"effective RPY {PRESET_RPY_DEG[facing][side]}. "
            "Editing an RPY field switches the preset to Custom."
        )
    return "Custom: the shoulder-to-base RPY is edited manually."
