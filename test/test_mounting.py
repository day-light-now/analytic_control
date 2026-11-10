from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest

import mounting
from config import ControlConfig
from frames import (
    ControllerPose,
    arm_base_in_horizon,
    build_coordinate_frames,
    shoulder_to_arm_base,
)
from transforms import (
    assert_rotation_matrix,
    rotation_matrix_from_rpy_deg,
    rpy_deg_from_rotation_matrix,
)

FRONT_MATRICES = {
    "left": np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]),
    "right": np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]]),
}
DOWN_MATRICES = {
    "left": np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]),
    "right": np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
}
FRONT_PORT_DIRECTION = (0.0, 0.0, -1.0)
DOWN_PORT_DIRECTION = (0.0, -1.0, 0.0)


def _down_frames(front_frames, sides=("left", "right")):
    for side in sides:
        front_frames = replace(
            front_frames,
            **{
                f"power_port_facing_{side}": mounting.POWER_PORT_FACING_DOWN,
                f"rotation_{side}_shoulder_to_{side}_arm_base_rpy_deg": mounting.DOWN_RPY_DEG[side],
            },
        )
    return front_frames


def test_front_keeps_existing_config_defaults():
    frames = ControlConfig().frames
    assert frames.rotation_left_shoulder_to_left_arm_base_rpy_deg == (-90.0, -90.0, 90.0)
    assert frames.rotation_right_shoulder_to_right_arm_base_rpy_deg == (0.0, -90.0, 180.0)
    assert frames.power_port_facing_left == mounting.POWER_PORT_FACING_FRONT
    assert frames.power_port_facing_right == mounting.POWER_PORT_FACING_FRONT
    for side in mounting.SIDES:
        assert np.allclose(mounting.FRONT_ROTATION[side], FRONT_MATRICES[side], atol=1e-12)
        expected = shoulder_to_arm_base(frames, side)
        assert np.allclose(expected[:3, :3], FRONT_MATRICES[side], atol=1e-12)
        assert np.allclose(expected[:3, 3], 0.0)


def test_down_is_a_rigid_90_degree_installation_rotation():
    rotation = mounting.FRONT_TO_DOWN_ROTATION
    assert_rotation_matrix(rotation)
    assert abs(float(np.linalg.det(rotation)) - 1.0) < 1e-12
    angle = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) / 2.0))))
    assert abs(angle - 90.0) < 1e-9
    assert np.allclose(rotation, rotation_matrix_from_rpy_deg((-90.0, 0.0, 0.0)), atol=1e-12)
    assert np.allclose(rotation @ mounting.HORIZON_FORWARD, mounting.HORIZON_DOWN, atol=1e-12)
    for side in mounting.SIDES:
        assert np.allclose(mounting.DOWN_ROTATION[side], rotation @ mounting.FRONT_ROTATION[side], atol=1e-12)
        assert np.allclose(mounting.DOWN_ROTATION[side], DOWN_MATRICES[side], atol=1e-12)
        assert_rotation_matrix(mounting.DOWN_ROTATION[side])


def test_power_port_points_forward_for_front_and_down_for_down():
    for side in mounting.SIDES:
        front = mounting.power_port_direction_in_horizon(
            side, mounting.POWER_PORT_FACING_FRONT, mounting.FRONT_RPY_DEG[side]
        )
        down = mounting.power_port_direction_in_horizon(
            side, mounting.POWER_PORT_FACING_DOWN, mounting.DOWN_RPY_DEG[side]
        )
        assert np.allclose(front, FRONT_PORT_DIRECTION, atol=1e-12)
        assert np.allclose(down, DOWN_PORT_DIRECTION, atol=1e-12)
        # The stored RPY presentation must rebuild the authoritative matrices.
        assert np.allclose(
            rotation_matrix_from_rpy_deg(mounting.DOWN_RPY_DEG[side]),
            mounting.DOWN_ROTATION[side],
            atol=1e-9,
        )


def test_down_rpy_presentation_is_expected_and_pitch_moves_90_degrees():
    # ZYX RPY as shown by the Settings row; matrices remain the authority.
    assert mounting.DOWN_RPY_DEG["left"] == (-90.0, 0.0, 90.0)
    assert mounting.DOWN_RPY_DEG["right"] == (90.0, 0.0, 90.0)
    for side in mounting.SIDES:
        assert abs(mounting.DOWN_RPY_DEG[side][1] - mounting.FRONT_RPY_DEG[side][1] - 90.0) < 1e-9


def test_body_fixed_composition_matches_world_rotation():
    """R_front @ M == A @ R_front; left/right rotate opposite ways about local +Z."""
    bodies = {}
    for side in mounting.SIDES:
        front = mounting.FRONT_ROTATION[side]
        body = front.T @ mounting.FRONT_TO_DOWN_ROTATION @ front
        bodies[side] = body
        assert np.allclose(front @ body, mounting.FRONT_TO_DOWN_ROTATION @ front, atol=1e-12)
        assert np.allclose(rotation_matrix_from_rpy_deg(rpy_deg_from_rotation_matrix(body)), body, atol=1e-9)
    left_angle = rpy_deg_from_rotation_matrix(bodies["left"])[2]
    right_angle = rpy_deg_from_rotation_matrix(bodies["right"])[2]
    assert abs(left_angle - 90.0) < 1e-9
    assert abs(right_angle + 90.0) < 1e-9


def test_front_to_down_to_front_is_reversible_without_drift():
    for side in mounting.SIDES:
        front = mounting.FRONT_ROTATION[side]
        current = front.copy()
        for _ in range(100):
            current = mounting.FRONT_TO_DOWN_ROTATION @ current
            current = mounting.FRONT_TO_DOWN_ROTATION.T @ current
        assert np.allclose(current, front, atol=1e-12)
    # The preset resolver itself is stateless: repeated resolutions are identical.
    for _ in range(5):
        for side in mounting.SIDES:
            assert np.array_equal(
                mounting.preset_rotation(side, mounting.POWER_PORT_FACING_DOWN),
                mounting.DOWN_ROTATION[side],
            )


def test_left_and_right_installations_are_independent():
    frames = ControlConfig().frames
    left_down = _down_frames(frames, sides=("left",))
    bases = arm_base_in_horizon(frames)
    assert np.allclose(bases.left[:3, :3], FRONT_MATRICES["left"], atol=1e-12)
    assert np.allclose(bases.right[:3, :3], FRONT_MATRICES["right"], atol=1e-12)

    bases_left_down = arm_base_in_horizon(left_down)
    assert np.allclose(bases_left_down.left[:3, :3], DOWN_MATRICES["left"], atol=1e-12)
    assert np.allclose(bases_left_down.right[:3, :3], FRONT_MATRICES["right"], atol=1e-12)

    both_down = _down_frames(left_down)
    assert np.allclose(arm_base_in_horizon(both_down).right[:3, :3], DOWN_MATRICES["right"], atol=1e-12)


def test_down_keeps_translations_startup_pose_and_joint_relation():
    config = ControlConfig()
    moved = replace(
        config,
        frames=replace(
            config.frames,
            translation_left_shoulder_to_left_arm_base_m=(0.01, -0.02, 0.03),
            translation_right_shoulder_to_right_arm_base_m=(-0.04, 0.05, 0.06),
            translation_horizon_to_left_shoulder_m=(-0.2, 0.01, 0.02),
        ),
    )
    moved = replace(moved, frames=_down_frames(moved.frames))
    bases = arm_base_in_horizon(moved.frames)
    assert np.allclose(bases.left[:3, 3], (-0.19, -0.01, 0.05), atol=1e-12)
    assert np.allclose(bases.right[:3, 3], (0.12, 0.05, 0.06), atol=1e-12)
    assert moved.startup == config.startup
    # The FK relation inside the arm base frame is untouched: the target mapping
    # is recomputed from the same PICO pose through the one base transform.
    pose = ControllerPose((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0), 0.0)
    down_only = _down_frames(config.frames)
    front_frames = build_coordinate_frames(pose, pose, config.frames)
    down_frames = build_coordinate_frames(pose, pose, down_only)
    from transforms import invert_transform

    for side in mounting.SIDES:
        expected = (
            invert_transform(shoulder_to_arm_base(down_only, side))
            @ shoulder_to_arm_base(config.frames, side)
            @ getattr(front_frames, side).transform_arm_base_to_flange
        )
        assert np.allclose(getattr(down_frames, side).transform_arm_base_to_flange, expected, atol=1e-12)
        # Base mounting rotation is the derived horizon-frame rotation.
        front_hb = getattr(front_frames, side).transform_horizon_to_arm_base
        down_hb = getattr(down_frames, side).transform_horizon_to_arm_base
        assert np.allclose(down_hb[:3, :3], mounting.FRONT_TO_DOWN_ROTATION @ front_hb[:3, :3], atol=1e-12)


def test_coordinate_frames_use_the_one_authoritative_base_transform():
    frames = _down_frames(ControlConfig().frames)
    pose = ControllerPose((0.05, 0.1, -0.2), (0.0, 0.0, 0.0, 1.0), 0.0)
    coordinate = build_coordinate_frames(pose, pose, frames)
    bases = arm_base_in_horizon(frames)
    for side in mounting.SIDES:
        arm = getattr(coordinate, side)
        base = getattr(bases, side)
        assert np.allclose(arm.transform_horizon_to_arm_base, base, atol=1e-12)
        assert np.allclose(
            arm.transform_horizon_to_arm_base @ arm.transform_arm_base_to_flange,
            arm.transform_horizon_to_flange,
            atol=1e-12,
        )


def test_preset_matrix_is_authoritative_over_stale_rpy():
    frames = replace(
        ControlConfig().frames,
        power_port_facing_left="down",
        # A stale manual RPY must not influence the preset transform.
        rotation_left_shoulder_to_left_arm_base_rpy_deg=(12.0, 34.0, 56.0),
    )
    rotation = shoulder_to_arm_base(frames, "left")[:3, :3]
    assert np.allclose(rotation, DOWN_MATRICES["left"], atol=1e-12)


def test_custom_uses_the_manual_rpy_matrix():
    frames = replace(
        ControlConfig().frames,
        power_port_facing_right="custom",
        rotation_right_shoulder_to_right_arm_base_rpy_deg=(10.0, 20.0, 30.0),
    )
    right = shoulder_to_arm_base(frames, "right")
    assert np.allclose(
        right[:3, :3], rotation_matrix_from_rpy_deg((10.0, 20.0, 30.0)), atol=1e-12
    )
    assert np.allclose(arm_base_in_horizon(frames).left[:3, :3], FRONT_MATRICES["left"], atol=1e-12)


def test_gimbal_lock_rpy_roundtrip_and_matrix_comparison():
    for side in mounting.SIDES:
        matrix = mounting.FRONT_ROTATION[side]
        # At pitch = -90 deg the Euler encoding is not unique; matrices must round
        # trip and validation must accept any equivalent encoding.
        rpy = rpy_deg_from_rotation_matrix(matrix)
        assert np.allclose(rotation_matrix_from_rpy_deg(rpy), matrix, atol=1e-9)
        mounting.validate_preset_rotation(side, "front", rpy)
    # An equivalent gimbal encoding of the left Front matrix.
    mounting.validate_preset_rotation("left", "front", (0.0, -90.0, 0.0))


def test_preset_validation_rejects_contradictory_labels():
    for side in mounting.SIDES:
        with pytest.raises(ValueError):
            mounting.validate_preset_rotation(side, "front", mounting.DOWN_RPY_DEG[side])
        with pytest.raises(ValueError):
            mounting.validate_preset_rotation(side, "down", mounting.FRONT_RPY_DEG[side])
        # Custom accepts any finite RPY.
        mounting.validate_preset_rotation(side, "custom", (1.0, 2.0, 3.0))
    with pytest.raises(ValueError):
        mounting.normalize_facing("sideways")
    with pytest.raises(ValueError):
        mounting.shoulder_to_base_rotation("middle", "front", (0.0, 0.0, 0.0))


def test_facing_hints_describe_effective_transform():
    for side in mounting.SIDES:
        front = mounting.describe_facing(side, "front")
        down = mounting.describe_facing(side, "down")
        custom = mounting.describe_facing(side, "custom")
        assert "-Z" in front and "effective RPY" in front
        assert "-Y" in down and "effective RPY" in down
        assert "Custom" in custom
        assert "switches the preset to Custom" in front
        assert "switches the preset to Custom" in down
