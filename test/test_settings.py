from __future__ import annotations

import math

import numpy as np
import pytest

import mounting
from config import ControlConfig
from settings_store import apply_settings, load_settings, reset_settings, save_settings, settings_data


def test_apply_settings_overrides_startup_and_frames():
    config = ControlConfig()
    data = settings_data(config)
    data["startup"]["left_pose_rad"][0] = 0.5
    data["frames"]["translation_horizon_to_left_shoulder_m"][0] = -0.2
    updated = apply_settings(config, data)
    assert updated.startup.left_pose_rad[0] == 0.5
    assert updated.frames.translation_horizon_to_left_shoulder_m[0] == -0.2
    assert updated.startup.right_pose_rad == config.startup.right_pose_rad


def test_save_and_load_roundtrip(tmp_path):
    path = tmp_path / "settings.json"
    data = settings_data(ControlConfig())
    data["startup"]["right_pose_rad"][3] = 1.0
    save_settings(data, path)
    loaded, error = load_settings(path)
    assert error is None
    assert loaded["startup"]["right_pose_rad"][3] == 1.0


def test_invalid_json_returns_error(tmp_path):
    path = tmp_path / "settings.json"
    path.write_text("{not json", encoding="utf-8")
    data, error = load_settings(path)
    assert data == {}
    assert error is not None


def test_invalid_settings_are_rejected(tmp_path):
    path = tmp_path / "settings.json"
    cases = [
        lambda data: data["startup"]["left_pose_rad"].pop(),
        lambda data: data["startup"]["left_pose_rad"].__setitem__(0, math.nan),
        lambda data: data["startup"]["left_pose_rad"].__setitem__(0, 100.0),
        lambda data: data["frames"]["translation_horizon_to_left_shoulder_m"].pop(),
    ]
    for mutate in cases:
        data = settings_data(ControlConfig())
        mutate(data)
        with pytest.raises(ValueError):
            apply_settings(ControlConfig(), data)
        with pytest.raises(ValueError):
            save_settings(data, path)
        assert not path.exists()


def test_reset_settings_removes_override(tmp_path):
    path = tmp_path / "settings.json"
    save_settings(settings_data(ControlConfig()), path)
    assert path.exists()
    reset_settings(path)
    assert not path.exists()


def test_settings_data_includes_power_port_mounts():
    data = settings_data(ControlConfig())
    assert data["mounts"] == {"left": "front", "right": "front"}


def test_defaults_are_front_for_both_arms():
    config = ControlConfig()
    assert config.frames.power_port_facing_left == "front"
    assert config.frames.power_port_facing_right == "front"
    assert config.frames.rotation_left_shoulder_to_left_arm_base_rpy_deg == (-90.0, -90.0, 90.0)
    assert config.frames.rotation_right_shoulder_to_right_arm_base_rpy_deg == (0.0, -90.0, 180.0)


def test_new_settings_roundtrip_keeps_front_down_and_custom():
    data = settings_data(ControlConfig())
    data["mounts"] = {"left": "down", "right": "custom"}
    data["frames"]["rotation_left_shoulder_to_left_arm_base_rpy_deg"] = list(
        mounting.DOWN_RPY_DEG["left"]
    )
    data["frames"]["rotation_right_shoulder_to_right_arm_base_rpy_deg"] = [10.0, 20.0, 30.0]
    config = apply_settings(ControlConfig(), data)
    assert config.frames.power_port_facing_left == "down"
    assert config.frames.power_port_facing_right == "custom"
    assert np.allclose(
        config.frames.rotation_left_shoulder_to_left_arm_base_rpy_deg,
        mounting.DOWN_RPY_DEG["left"],
    )
    # A second application of the resulting file must be idempotent.
    again = apply_settings(config, data)
    assert again.frames == config.frames


def test_legacy_settings_without_mounts_restore_rpy_as_custom(tmp_path):
    config = ControlConfig()
    legacy = settings_data(config)
    del legacy["mounts"]
    legacy["frames"]["rotation_left_shoulder_to_left_arm_base_rpy_deg"] = [1.5, -2.5, 3.5]
    legacy["frames"]["translation_left_shoulder_to_left_arm_base_m"] = [0.01, 0.02, 0.03]
    migrated = apply_settings(ControlConfig(), legacy)
    assert migrated.frames.power_port_facing_left == "custom"
    assert migrated.frames.power_port_facing_right == "custom"
    assert migrated.frames.rotation_left_shoulder_to_left_arm_base_rpy_deg == (1.5, -2.5, 3.5)
    assert migrated.frames.translation_left_shoulder_to_left_arm_base_m == (0.01, 0.02, 0.03)
    assert migrated.startup == ControlConfig().startup
    # Legacy files with the historical Front RPY still resolve to the same matrix.
    plain = settings_data(ControlConfig())
    del plain["mounts"]
    plain_migrated = apply_settings(ControlConfig(), plain)
    assert plain_migrated.frames.power_port_facing_left == "custom"
    assert np.allclose(
        plain_migrated.frames.rotation_left_shoulder_to_left_arm_base_rpy_deg,
        (-90.0, -90.0, 90.0),
    )


def test_save_and_load_roundtrip_preserves_power_port_mounts(tmp_path):
    path = tmp_path / "settings.json"
    data = settings_data(ControlConfig())
    data["mounts"] = {"left": "front", "right": "down"}
    data["frames"]["rotation_right_shoulder_to_right_arm_base_rpy_deg"] = list(
        mounting.DOWN_RPY_DEG["right"]
    )
    saved = save_settings(data, path)
    assert saved.frames.power_port_facing_right == "down"
    loaded, error = load_settings(path)
    assert error is None
    assert loaded["mounts"] == {"left": "front", "right": "down"}
    restored = apply_settings(ControlConfig(), loaded)
    assert restored.frames.power_port_facing_right == "down"
    assert restored.frames.rotation_right_shoulder_to_right_arm_base_rpy_deg == pytest.approx(
        mounting.DOWN_RPY_DEG["right"]
    )


def test_mounts_label_must_match_stored_rpy(tmp_path):
    path = tmp_path / "settings.json"
    data = settings_data(ControlConfig())
    data["mounts"] = {"left": "down", "right": "front"}
    # Front RPY while claiming Down must be rejected, not silently relabelled.
    with pytest.raises(ValueError):
        apply_settings(ControlConfig(), data)
    with pytest.raises(ValueError):
        save_settings(data, path)
    assert not path.exists()
    # Alternative gimbal encoding of the same Front matrix is still accepted.
    data["frames"]["rotation_left_shoulder_to_left_arm_base_rpy_deg"] = [0.0, -90.0, 0.0]
    data["mounts"]["left"] = "front"
    assert apply_settings(ControlConfig(), data).frames.power_port_facing_left == "front"


def test_invalid_power_port_mounts_are_rejected(tmp_path):
    path = tmp_path / "settings.json"
    for mounts in (
        {"left": "front", "right": "sideways"},
        {"left": "front"},
        {"left": "front", "right": "front", "middle": "custom"},
        ["front", "front"],
        None,
    ):
        data = settings_data(ControlConfig())
        if mounts is None:
            data["mounts"] = None
        else:
            data["mounts"] = mounts
        with pytest.raises(ValueError):
            apply_settings(ControlConfig(), data)
        with pytest.raises(ValueError):
            save_settings(data, path)
        assert not path.exists()


def test_restore_defaults_returns_to_front_after_down(tmp_path):
    path = tmp_path / "settings.json"
    data = settings_data(ControlConfig())
    data["mounts"] = {"left": "down", "right": "down"}
    data["frames"]["rotation_left_shoulder_to_left_arm_base_rpy_deg"] = list(
        mounting.DOWN_RPY_DEG["left"]
    )
    data["frames"]["rotation_right_shoulder_to_right_arm_base_rpy_deg"] = list(
        mounting.DOWN_RPY_DEG["right"]
    )
    save_settings(data, path)
    reset_settings(path)
    restored = apply_settings(ControlConfig(), {})
    assert restored.frames.power_port_facing_left == "front"
    assert restored.frames.power_port_facing_right == "front"
    assert restored == ControlConfig()
