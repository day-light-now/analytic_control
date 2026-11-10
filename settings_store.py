from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

import mounting
from config import ControlConfig
from ik_solver import NeroParams

SETTINGS_PATH = Path(__file__).resolve().parent / "settings.json"
STARTUP_FIELDS = ("left_pose_rad", "right_pose_rad")
FRAME_FIELDS = (
    "translation_horizon_to_left_shoulder_m",
    "translation_horizon_to_right_shoulder_m",
    "translation_left_shoulder_to_left_arm_base_m",
    "rotation_left_shoulder_to_left_arm_base_rpy_deg",
    "translation_right_shoulder_to_right_arm_base_m",
    "rotation_right_shoulder_to_right_arm_base_rpy_deg",
)
MOUNTS_KEY = "mounts"


def load_settings(path: Path | None = None) -> tuple[dict, str | None]:
    path = SETTINGS_PATH if path is None else path
    if not path.is_file():
        return {}, None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {}, f"invalid settings.json: {exc}"
    if not isinstance(data, dict):
        return {}, "invalid settings.json: root must be an object"
    return data, None


def apply_settings(config: ControlConfig, data: dict) -> ControlConfig:
    startup_data = data.get("startup", {})
    frame_data = data.get("frames", {})
    mounts_data = data.get(MOUNTS_KEY)
    if not isinstance(startup_data, dict) or not isinstance(frame_data, dict):
        raise ValueError("settings startup and frames must be objects")
    if MOUNTS_KEY in data and not isinstance(mounts_data, dict):
        raise ValueError("settings mounts must be an object")

    startup = config.startup
    frames = config.frames
    limits = NeroParams.default().joint_limits

    for field in STARTUP_FIELDS:
        if field not in startup_data:
            continue
        values = _floats(startup_data[field], 7, field)
        for index, value in enumerate(values):
            if value < float(limits[index, 0]) or value > float(limits[index, 1]):
                raise ValueError(f"{field} joint {index + 1} is outside the Nero joint limits")
        startup = replace(startup, **{field: tuple(values)})

    for field in FRAME_FIELDS:
        if field not in frame_data:
            continue
        frames = replace(frames, **{field: tuple(_floats(frame_data[field], 3, field))})

    if mounts_data is None:
        # Files written before the preset existed only carry manual RPY. Keep
        # that RPY verbatim and expose it through Custom instead of guessing a
        # direction label for it.
        if "frames" in data:
            frames = replace(
                frames,
                power_port_facing_left=mounting.POWER_PORT_FACING_CUSTOM,
                power_port_facing_right=mounting.POWER_PORT_FACING_CUSTOM,
            )
    else:
        unknown = sorted(set(mounts_data) - set(mounting.SIDES))
        missing = sorted(set(mounting.SIDES) - set(mounts_data))
        if unknown or missing:
            raise ValueError(f"settings mounts must contain exactly the sides {mounting.SIDES}")
        facings = {}
        for side in mounting.SIDES:
            raw = mounts_data[side]
            if not isinstance(raw, str):
                raise ValueError(f"settings mounts.{side} must be a string")
            facings[side] = mounting.normalize_facing(raw)
        frames = replace(
            frames,
            power_port_facing_left=facings["left"],
            power_port_facing_right=facings["right"],
        )

    for side in mounting.SIDES:
        mounting.validate_preset_rotation(
            side,
            getattr(frames, f"power_port_facing_{side}"),
            getattr(frames, f"rotation_{side}_shoulder_to_{side}_arm_base_rpy_deg"),
        )

    return replace(config, startup=startup, frames=frames)


def save_settings(
    data: dict,
    path: Path | None = None,
    config: ControlConfig | None = None,
) -> ControlConfig:
    path = SETTINGS_PATH if path is None else path
    config = apply_settings(config or ControlConfig(), data)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    return config


def settings_data(config: ControlConfig) -> dict:
    return {
        "startup": {field: list(getattr(config.startup, field)) for field in STARTUP_FIELDS},
        "frames": {field: list(getattr(config.frames, field)) for field in FRAME_FIELDS},
        MOUNTS_KEY: {
            side: getattr(config.frames, f"power_port_facing_{side}") for side in mounting.SIDES
        },
    }


def reset_settings(path: Path | None = None) -> None:
    path = SETTINGS_PATH if path is None else path
    path.unlink(missing_ok=True)


def read_settings_text(path: Path | None = None) -> str | None:
    """Raw settings text for the UI's persistence-rollback snapshot."""
    path = SETTINGS_PATH if path is None else path
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None


def restore_settings_text(text: str | None, path: Path | None = None) -> None:
    """Atomically restore a previously snapshotted settings file (None = absent)."""
    path = SETTINGS_PATH if path is None else path
    temporary = path.with_suffix(".json.tmp")
    if text is None:
        path.unlink(missing_ok=True)
        temporary.unlink(missing_ok=True)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def _floats(value, length: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{name} must contain only numbers")
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{name} must contain finite numbers")
        result.append(number)
    return result
