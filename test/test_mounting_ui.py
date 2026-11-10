"""GUI integration tests for the power-port mounting presets.

These tests drive the real ``ui.app.build_page`` layout through NiceGUI's
``user_simulation`` together with a *real* ``ArmController`` control thread.
Only the real hardware backend is replaced by ``FakeArm``; the
APPLY_SETTINGS acknowledgement path, the control thread's accept/reject
policy, ``ui/settings.py`` form logic, the persisted settings file and
``TeleopScene.set_bases`` are the production implementations.

Persistence is isolated with
``monkeypatch.setattr(settings_store, "SETTINGS_PATH", tmp_path / "settings.json")``
so the repository-root ``settings.json`` is never written.
"""
from __future__ import annotations

import json
import threading
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

import mounting
import settings_store
from config import ControlConfig
from control_loop import ArmController
from controller_state import ControlCommand, ControlSnapshot, SnapshotStore
from frames import arm_base_in_horizon
from logging_utils import LogBuffer
from nicegui import ui
from nicegui.testing.user_simulation import user_simulation
from ui.app import build_page
from ui.scene import SCENE_FROM_HORIZON
from test._settings_ui_support import make_config, make_controller, no_real_arm_factory, rpy_numbers, rpy_values, select_widget, wait_until

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TIMEOUT_S = 3.0
STARTUP_TIMEOUT_S = 8.0
SIDES = ("left", "right")
HORIZON_SCENE_ROTATION = np.asarray(SCENE_FROM_HORIZON, dtype=float)


# ---------------------------------------------------------------------------
# Fake hardware backend (same contract as test/test_controller.py's FakeArm)
# ---------------------------------------------------------------------------
class FakeArm:
    """Stand-in for the SocketCAN/agx_cando backend; no hardware is touched."""

    def __init__(self, side: str, config, motion, logger) -> None:
        self.side = side
        self.config = config
        self.firmware = "v120"
        self.q = np.zeros(7, dtype=float)
        self.moves: list[np.ndarray] = []
        self.holds: list[np.ndarray] = []
        self.sent: list[tuple[Any, Any]] = []
        self.closed = False
        self.transport_dead = None

    def transport_error(self):
        return self.transport_dead

    def move_j_startup(self, side, q, stop_event=None) -> bool:
        if stop_event is not None and stop_event.is_set():
            return False
        self.moves.append(np.asarray(q, dtype=float).copy())
        self.q = np.asarray(q, dtype=float).copy()
        return True

    def prepare_teleop(self, side, stop_event=None):
        if stop_event is not None and stop_event.is_set():
            return None
        self.holds.append(self.q.copy())
        return self.q.copy()

    def current_joints(self, side):
        return self.q.copy()

    def refresh_feedback(self) -> bool:
        return True

    def update(self, left_q, right_q, frames, status, dt) -> None:
        self.sent.append(
            (
                None if left_q is None else np.asarray(left_q, dtype=float).copy(),
                None if right_q is None else np.asarray(right_q, dtype=float).copy(),
            )
        )

    def close(self) -> None:
        self.closed = True


def _isolate_settings(monkeypatch, tmp_path: Path) -> Path:
    """Make ui/settings.py persist into tmp_path instead of the repository root."""
    path = tmp_path / "settings.json"
    monkeypatch.setattr(settings_store, "SETTINGS_PATH", path)
    return path


# ---------------------------------------------------------------------------
# Generic async helpers (poll, never fixed-sleep)
# ---------------------------------------------------------------------------
def _elements(user, **filters) -> list:
    return list(user.find(**filters).elements)


def _button(user, label: str) -> ui.button:
    elements = _elements(user, kind=ui.button, content=label)
    assert len(elements) == 1, f"expected exactly one {label!r} button, got {len(elements)}"
    return elements[0]


def _click_button(user, label: str) -> None:
    """Dispatch a real click event on the (unique) button with this label."""
    interaction = user.find(kind=ui.button, content=label)
    assert len(interaction.elements) == 1, (
        f"expected exactly one {label!r} button, got {len(interaction.elements)}"
    )
    assert any(
        not isinstance(element, ui.button) or element.enabled
        for element in interaction.elements
    ), f"{label!r} button is disabled"
    interaction.click()


def _labels(user, text: str) -> list:
    """Labels whose content contains ``text``; empty list when there are none."""
    try:
        elements = _elements(user, kind=ui.label, content=text)
    except AssertionError:
        return []
    return sorted(elements, key=lambda element: element.id)


def _rotation(obj) -> np.ndarray:
    return np.asarray(obj.R, dtype=float)


def _scene_rotation(base: np.ndarray) -> np.ndarray:
    return HORIZON_SCENE_ROTATION @ base[:3, :3]


def _assert_base_group(arm, base: np.ndarray, name: str) -> None:
    group = arm.base_group
    assert np.allclose(_rotation(group), _scene_rotation(base), atol=1e-9), (
        f"{name} base_group rotation does not match the horizon->scene mapping of the new base"
    )
    scene_translation = HORIZON_SCENE_ROTATION @ base[:3, 3]
    assert np.allclose([group.x, group.y, group.z], scene_translation, atol=1e-9), (
        f"{name} base_group translation does not match the new base: "
        f"{[group.x, group.y, group.z]} != {scene_translation.tolist()}"
    )


def _assert_descends_from(obj, ancestor) -> None:
    assert obj is not None, "expected a scene object, got None"
    assert ancestor in obj.ancestors, "scene object is not a descendant of the arm base group"


# ---------------------------------------------------------------------------
# Scene capture: build_page calls the module-global build_scene; spy on it so
# the tests can assert on the *real* TeleopScene the page uses.
# ---------------------------------------------------------------------------
@contextmanager
def _capture_scene(monkeypatch):
    import ui.app as app_module
    from ui.scene import build_scene as real_build_scene

    captured: dict[str, Any] = {}

    def spy(page_config: ControlConfig):
        refresh, visual = real_build_scene(page_config)
        captured["visual"] = visual
        captured["refresh"] = refresh
        return refresh, visual

    monkeypatch.setattr(app_module, "build_scene", spy)
    yield captured


# ---------------------------------------------------------------------------
# Control-loop snapshot recording: capture every published tick, so the
# "no joint teleport" assertion sees adjacent ticks instead of racing an
# async sampler that could skip a tick.
# ---------------------------------------------------------------------------
def _record_published_states(store: SnapshotStore, monkeypatch) -> list[dict[str, np.ndarray]]:
    samples: list[dict[str, np.ndarray]] = []
    original_publish = store.publish

    def recording_publish(snapshot: ControlSnapshot) -> None:
        original_publish(snapshot)
        samples.append(
            {side: np.asarray(snapshot.sim_q[side], dtype=float) for side in SIDES}
        )

    monkeypatch.setattr(store, "publish", recording_publish)
    return samples


def _assert_no_joint_teleport(
    samples: list[dict[str, np.ndarray]],
    first_index: int,
    max_joint_step_rad: float,
) -> None:
    window = list(samples)[first_index:]
    assert len(window) >= 25, f"expected at least 25 published control ticks, got {len(window)}"
    for side in SIDES:
        stacked = np.vstack([sample[side] for sample in window])
        deltas = np.max(np.abs(np.diff(stacked, axis=0)), axis=1)
        worst = float(deltas.max())
        limit = 1.5 * max_joint_step_rad
        assert worst <= limit + 1e-12, (
            f"{side} simulation joints jumped {worst:.5f} rad between two adjacent published "
            f"snapshots; the planner may step at most {max_joint_step_rad} rad per tick"
        )


async def _wait_for_scene_bases(visual, expected: dict[str, np.ndarray], description: str) -> None:
    def matches() -> bool:
        return all(np.allclose(getattr(visual.bases, side), base, atol=1e-9) for side, base in expected.items())

    await wait_until(matches, description)


# ---------------------------------------------------------------------------
# A) Accept path: Down preset applied from the real Settings panel
# ---------------------------------------------------------------------------
async def test_apply_down_updates_settings_file_controller_and_scene(tmp_path, monkeypatch):
    settings_path = _isolate_settings(monkeypatch, tmp_path)
    with _capture_scene(monkeypatch) as captured:
        config = make_config()
        store = SnapshotStore()
        controller = make_controller(config, store, no_real_arm_factory)
        published = _record_published_states(store, monkeypatch)
        controller.start()
        try:
            await wait_until(
                lambda: store.read().phase == "TELEOP",
                "controller reaches TELEOP in pure simulation",
            )
            buffer = LogBuffer()
            async with user_simulation(
                root=lambda: build_page(controller, store, config, buffer)
            ) as user:
                await user.open("/")
                await wait_until(lambda: "visual" in captured, "page built the 3D scene")
                visual = captured["visual"]
                scene_refresh = captured["refresh"]

                # -- initial state: both arms are Front, preset RPY auto-filled
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT
                assert select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT
                assert np.allclose(rpy_values(user, 0), mounting.FRONT_RPY_DEG["left"])
                assert np.allclose(rpy_values(user, 1), mounting.FRONT_RPY_DEG["right"])
                assert len(_labels(user, "Front preset: power port faces forward")) == 2
                await user.should_see("effective RPY (-90.0, -90.0, 90.0)")
                await user.should_see("effective RPY (0.0, -90.0, 180.0)")

                front_bases = arm_base_in_horizon(config.frames)
                assert np.allclose(visual.bases.left, front_bases.left)
                assert np.allclose(visual.bases.right, front_bases.right)
                _assert_base_group(visual.left, front_bases.left, "left arm")
                _assert_base_group(visual.right, front_bases.right, "right arm")
                _assert_descends_from(visual.left.axes, visual.left.base_group)
                _assert_descends_from(visual.left.port_marker, visual.left.base_group)
                _assert_descends_from(visual.right.axes, visual.right.base_group)
                _assert_descends_from(visual.right.port_marker, visual.right.base_group)

                # Ghost arms exist before the remap (real-feedback snapshot fed
                # through the real scene refresh callback), so the accepted
                # config must rotate them too.
                ghost_snapshot = replace(
                    store.read(),
                    real_q={"left": (0.1,) * 7, "right": (-0.1,) * 7},
                )
                with visual.scene:
                    scene_refresh(ghost_snapshot)
                left_ghost = visual.left_ghost
                right_ghost = visual.right_ghost
                assert left_ghost is not None and right_ghost is not None
                _assert_base_group(left_ghost, front_bases.left, "left ghost")
                _assert_base_group(right_ghost, front_bases.right, "right ghost")

                # -- selecting Down refills the preset RPY and updates the note
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN
                assert np.allclose(rpy_values(user, 0), mounting.DOWN_RPY_DEG["left"])
                await user.should_see("Down preset: power port faces down (horizon -Y)")
                await user.should_see("effective RPY (-90.0, 0.0, 90.0)")
                # the right arm row must not be affected by the left selection
                assert select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT
                assert np.allclose(rpy_values(user, 1), mounting.FRONT_RPY_DEG["right"])
                assert len(_labels(user, "Front preset: power port faces forward")) == 1
                assert len(_labels(user, "Down preset: power port faces down")) == 1

                # -- editing an RPY number of a preset row switches that row to
                #    Custom (right Front RPY[0] is already 0.0, so edit to 10.0
                #    to prove the value change drives the mode switch)
                rpy_numbers(user, 1)[0].set_value(10.0)
                assert np.allclose(rpy_values(user, 1), [10.0, -90.0, 180.0])
                assert select_widget(user, "right").value == mounting.POWER_PORT_FACING_CUSTOM
                assert len(_labels(user, "Custom: the shoulder-to-base RPY is edited manually.")) == 1
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN
                assert np.allclose(rpy_values(user, 0), mounting.DOWN_RPY_DEG["left"])
                # -- re-selecting Front restores the preset RPY
                select_widget(user, "right").set_value(mounting.POWER_PORT_FACING_FRONT)
                assert select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT
                assert np.allclose(rpy_values(user, 1), mounting.FRONT_RPY_DEG["right"])
                assert len(_labels(user, "Front preset: power port faces forward")) == 1
                assert len(_labels(user, "Custom: the shoulder-to-base RPY is edited manually.")) == 0

                # -- click the real Apply button: the controller decides first,
                #    only an accepted change is persisted and painted.
                revision_before = int(store.read().settings["revision"])
                published_before = len(published)
                before_sim_q = np.asarray(store.read().sim_q["left"], dtype=float)
                assert _button(user, "Apply").enabled, "Apply button is disabled before any request"
                _click_button(user, "Apply")

                await wait_until(
                    lambda: (
                        controller.config.frames.power_port_facing_left
                        == mounting.POWER_PORT_FACING_DOWN
                        and store.read().settings.get("revision") == revision_before + 1
                        and settings_path.is_file()
                    ),
                    "controller accepts the Down preset and settings.json is written",
                )
                await user.should_see("Settings applied.")
                assert store.read().settings.get("error") is None

                # -- persisted settings (tmp_path, never the repo root file)
                data = json.loads(settings_path.read_text(encoding="utf-8"))
                assert data["mounts"] == {"left": "down", "right": "front"}
                assert np.allclose(
                    data["frames"]["rotation_left_shoulder_to_left_arm_base_rpy_deg"],
                    mounting.DOWN_RPY_DEG["left"],
                )
                assert np.allclose(
                    data["frames"]["rotation_right_shoulder_to_right_arm_base_rpy_deg"],
                    mounting.FRONT_RPY_DEG["right"],
                )
                assert not (REPO_ROOT / "settings.json").exists(), (
                    "the UI test must never write the repository-root settings.json"
                )

                # -- controller bases
                accepted_config = controller.config
                assert accepted_config.frames.power_port_facing_left == mounting.POWER_PORT_FACING_DOWN
                assert accepted_config.frames.power_port_facing_right == mounting.POWER_PORT_FACING_FRONT
                accepted_bases = arm_base_in_horizon(accepted_config.frames)
                assert np.allclose(controller.bases.left, accepted_bases.left)
                assert np.allclose(controller.bases.right, accepted_bases.right)
                assert np.allclose(
                    controller.bases.left[:3, :3], mounting.DOWN_ROTATION["left"], atol=1e-12
                )
                assert np.allclose(
                    controller.bases.right[:3, :3], mounting.FRONT_ROTATION["right"], atol=1e-12
                )

                # -- scene: only the accepted new_config is rotated
                await _wait_for_scene_bases(
                    visual,
                    {"left": accepted_bases.left, "right": accepted_bases.right},
                    "scene bases follow the accepted configuration",
                )
                _assert_base_group(visual.left, accepted_bases.left, "left arm")
                _assert_base_group(visual.right, accepted_bases.right, "right arm")
                assert np.allclose(
                    _rotation(visual.left.base_group),
                    HORIZON_SCENE_ROTATION @ mounting.DOWN_ROTATION["left"],
                    atol=1e-9,
                )
                assert visual.left_ghost is left_ghost
                assert visual.right_ghost is right_ghost
                _assert_base_group(left_ghost, accepted_bases.left, "left ghost")
                _assert_base_group(right_ghost, accepted_bases.right, "right ghost")
                _assert_descends_from(visual.left.axes, visual.left.base_group)
                _assert_descends_from(visual.left.port_marker, visual.left.base_group)
                _assert_descends_from(visual.right.axes, visual.right.base_group)
                _assert_descends_from(visual.right.port_marker, visual.right.base_group)

                # -- the remap must not teleport the simulation joints: compare
                #    every adjacent *published* control tick after the Apply click.
                await wait_until(
                    lambda: len(published) >= published_before + 40,
                    "control loop publishes 40 ticks after the settings apply",
                    timeout=5.0,
                )
                max_step = float(accepted_config.motion.max_joint_step_rad)
                window = list(published)[published_before:]
                assert any(
                    np.max(np.abs(sample["left"] - before_sim_q)) > 0.0 for sample in window
                ), "the simulation joints never moved after the remap; the check would be vacuous"
                _assert_no_joint_teleport(published, published_before, max_step)
        finally:
            controller.stop()
            controller.wait(timeout=5.0)


# ---------------------------------------------------------------------------
# B) Rejected while a real (fake) arm is following
# ---------------------------------------------------------------------------
async def test_apply_rejected_while_real_arm_follows_keeps_file_controller_and_scene(
    tmp_path, monkeypatch
):
    settings_path = _isolate_settings(monkeypatch, tmp_path)
    with _capture_scene(monkeypatch) as captured:
        config = make_config()
        store = SnapshotStore()
        arms: dict[str, FakeArm] = {}

        def factory(side, arm_config, motion, logger):
            arms[side] = FakeArm(side, arm_config, motion, logger)
            return arms[side]

        controller = make_controller(config, store, factory)
        controller.start()
        try:
            await wait_until(
                lambda: store.read().phase == "TELEOP",
                "controller reaches TELEOP in pure simulation",
            )
            buffer = LogBuffer()
            async with user_simulation(
                root=lambda: build_page(controller, store, config, buffer)
            ) as user:
                await user.open("/")
                await wait_until(lambda: "visual" in captured, "page built the 3D scene")
                visual = captured["visual"]

                # -- 1) a successful apply in pure simulation creates settings.json
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                _click_button(user, "Apply")
                await wait_until(
                    lambda: (
                        controller.config.frames.power_port_facing_left
                        == mounting.POWER_PORT_FACING_DOWN
                        and store.read().settings.get("revision") == 1
                        and settings_path.is_file()
                    ),
                    "baseline Down preset accepted and persisted",
                )
                await _wait_for_scene_bases(
                    visual,
                    {"left": arm_base_in_horizon(controller.config.frames).left},
                    "scene rotates to the baseline Down preset",
                )
                file_before = settings_path.read_text(encoding="utf-8")
                before_config = controller.config
                before_revision = int(store.read().settings["revision"])
                before_bases = {
                    side: np.asarray(getattr(controller.bases, side), dtype=float).copy()
                    for side in SIDES
                }
                before_visual_bases = {
                    side: np.asarray(getattr(visual.bases, side), dtype=float).copy()
                    for side in SIDES
                }
                before_scene_rotation = _rotation(visual.left.base_group)
                await wait_until(lambda: _button(user, "Apply").enabled, "Apply button re-enabled")

                # -- 2) connect the fake arm and let it follow in TELEOP
                controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
                await wait_until(
                    lambda: store.read().phase == "STOPPED" and store.read().can_reset,
                    "STOP latched after connecting the fake left arm",
                )
                controller.submit(ControlCommand("RESET"))
                await wait_until(
                    lambda: store.read().phase == "TELEOP" and "left" in controller.following,
                    "fake left arm follows in TELEOP",
                    timeout=STARTUP_TIMEOUT_S,
                )

                # -- 3) the GUI must be refused while the arm is following
                select_widget(user, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                await user.should_see("Down preset: power port faces down (horizon -Y)")
                assert _button(user, "Apply").enabled
                _click_button(user, "Apply")

                await wait_until(
                    lambda: bool(store.read().settings.get("error")),
                    "controller reports the settings rejection",
                )
                await user.should_see("Apply rejected")
                await wait_until(
                    lambda: select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT,
                    "settings panel rolls the right select back to the last accepted value",
                )
                await wait_until(lambda: _button(user, "Apply").enabled, "Apply button re-enabled")

                error = str(store.read().settings.get("error") or "")
                assert "following" in error, f"unexpected rejection reason: {error!r}"
                assert int(store.read().settings["revision"]) == before_revision
                assert controller.config is before_config, "a rejected apply must not replace the config"
                for side in SIDES:
                    assert np.array_equal(
                        getattr(controller.bases, side), before_bases[side]
                    ), f"{side} controller base changed on a rejected apply"
                    assert np.array_equal(
                        getattr(visual.bases, side), before_visual_bases[side]
                    ), f"{side} scene base changed on a rejected apply"
                assert np.allclose(_rotation(visual.left.base_group), before_scene_rotation)
                assert settings_path.read_text(encoding="utf-8") == file_before, (
                    "a rejected apply rewrote settings.json"
                )
                assert json.loads(file_before)["mounts"] == {"left": "down", "right": "front"}
                # -- UI state rolled back to the last accepted configuration
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN
                assert select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT
                assert np.allclose(rpy_values(user, 0), mounting.DOWN_RPY_DEG["left"])
                assert np.allclose(rpy_values(user, 1), mounting.FRONT_RPY_DEG["right"])
        finally:
            controller.stop()
            controller.wait(timeout=5.0)


# ---------------------------------------------------------------------------
# C) Rejected while a hardware connect is in flight
# ---------------------------------------------------------------------------
async def test_apply_rejected_while_hardware_connect_is_busy(tmp_path, monkeypatch):
    settings_path = _isolate_settings(monkeypatch, tmp_path)
    with _capture_scene(monkeypatch) as captured:
        config = make_config()
        store = SnapshotStore()
        gate = threading.Event()
        arms: dict[str, FakeArm] = {}

        def factory(side, arm_config, motion, logger):
            gate.wait(timeout=5.0)
            arms[side] = FakeArm(side, arm_config, motion, logger)
            return arms[side]

        controller = make_controller(config, store, factory)
        controller.start()
        try:
            await wait_until(
                lambda: store.read().phase == "TELEOP",
                "controller reaches TELEOP in pure simulation",
            )
            buffer = LogBuffer()
            async with user_simulation(
                root=lambda: build_page(controller, store, config, buffer)
            ) as user:
                await user.open("/")
                await wait_until(lambda: "visual" in captured, "page built the 3D scene")
                visual = captured["visual"]
                before_config = controller.config
                before_bases = {
                    side: np.asarray(getattr(controller.bases, side), dtype=float).copy()
                    for side in SIDES
                }
                before_visual_bases = {
                    side: np.asarray(getattr(visual.bases, side), dtype=float).copy()
                    for side in SIDES
                }
                assert not settings_path.exists()

                # -- start a connect whose backend factory is still blocked
                controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
                await wait_until(
                    lambda: store.read().arm_status["left"].state == "CONNECTING",
                    "the hardware connect is in flight",
                )

                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                await user.should_see("Down preset: power port faces down (horizon -Y)")
                assert _button(user, "Apply").enabled
                _click_button(user, "Apply")

                await wait_until(
                    lambda: bool(store.read().settings.get("error")),
                    "controller reports the settings rejection",
                )
                await user.should_see("Apply rejected")
                await wait_until(
                    lambda: select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT,
                    "settings panel rolls the left select back to Front",
                )

                error = str(store.read().settings.get("error") or "")
                assert "connect" in error.lower() or "hardware" in error.lower(), (
                    f"unexpected rejection reason: {error!r}"
                )
                assert not settings_path.exists(), "a rejected apply must not write settings.json"
                assert controller.config is before_config
                assert int(store.read().settings["revision"]) == 0
                for side in SIDES:
                    assert np.array_equal(getattr(controller.bases, side), before_bases[side])
                    assert np.array_equal(getattr(visual.bases, side), before_visual_bases[side])
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT
                assert np.allclose(rpy_values(user, 0), mounting.FRONT_RPY_DEG["left"])
                assert len(_labels(user, "Front preset: power port faces forward")) == 2
        finally:
            gate.set()
            controller.stop()
            controller.wait(timeout=5.0)


# ---------------------------------------------------------------------------
# D) Rejected while real arms are aligning
# ---------------------------------------------------------------------------
async def test_apply_rejected_while_aligning(tmp_path, monkeypatch):
    settings_path = _isolate_settings(monkeypatch, tmp_path)
    with _capture_scene(monkeypatch) as captured:
        config = make_config()
        store = SnapshotStore()
        gate = threading.Event()
        arms: dict[str, FakeArm] = {}

        class BlockingStartupArm(FakeArm):
            def move_j_startup(self, side, q, stop_event=None):
                gate.wait(timeout=5.0)
                return super().move_j_startup(side, q, stop_event)

        def factory(side, arm_config, motion, logger):
            arms[side] = BlockingStartupArm(side, arm_config, motion, logger)
            return arms[side]

        controller = make_controller(config, store, factory)
        controller.start()
        try:
            await wait_until(
                lambda: store.read().phase == "TELEOP",
                "controller reaches TELEOP in pure simulation",
            )
            buffer = LogBuffer()
            async with user_simulation(
                root=lambda: build_page(controller, store, config, buffer)
            ) as user:
                await user.open("/")
                await wait_until(lambda: "visual" in captured, "page built the 3D scene")
                visual = captured["visual"]
                before_config = controller.config
                before_bases = {
                    side: np.asarray(getattr(controller.bases, side), dtype=float).copy()
                    for side in SIDES
                }
                before_visual_bases = {
                    side: np.asarray(getattr(visual.bases, side), dtype=float).copy()
                    for side in SIDES
                }
                assert not settings_path.exists()

                # -- connect, then RESET: the blocking startup move keeps the
                #    controller in ALIGNING while we try to apply.
                controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
                await wait_until(
                    lambda: store.read().phase == "STOPPED" and store.read().can_reset,
                    "STOP latched after connecting the fake left arm",
                )
                controller.submit(ControlCommand("RESET"))
                await wait_until(
                    lambda: store.read().phase == "ALIGNING",
                    "controller stays in ALIGNING while the startup move is blocked",
                )

                select_widget(user, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                await user.should_see("Down preset: power port faces down (horizon -Y)")
                assert _button(user, "Apply").enabled
                _click_button(user, "Apply")

                await wait_until(
                    lambda: bool(store.read().settings.get("error")),
                    "controller reports the settings rejection",
                )
                await user.should_see("Apply rejected")
                await wait_until(
                    lambda: select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT,
                    "settings panel rolls the right select back to Front",
                )

                error = str(store.read().settings.get("error") or "")
                assert "align" in error.lower(), f"unexpected rejection reason: {error!r}"
                assert not settings_path.exists(), "a rejected apply must not write settings.json"
                assert controller.config is before_config
                assert int(store.read().settings["revision"]) == 0
                for side in SIDES:
                    assert np.array_equal(getattr(controller.bases, side), before_bases[side])
                    assert np.array_equal(getattr(visual.bases, side), before_visual_bases[side])
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT
                assert select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT

                # -- releasing the startup move must finish alignment normally
                gate.set()
                await wait_until(
                    lambda: store.read().phase == "TELEOP" and "left" in controller.following,
                    "fake left arm finishes alignment and follows in TELEOP",
                    timeout=STARTUP_TIMEOUT_S,
                )
        finally:
            gate.set()
            controller.stop()
            controller.wait(timeout=5.0)


# ---------------------------------------------------------------------------
# E) Restore Defaults returns an applied Down configuration to Front
# ---------------------------------------------------------------------------
async def test_restore_defaults_returns_scene_and_config_to_front(tmp_path, monkeypatch):
    settings_path = _isolate_settings(monkeypatch, tmp_path)
    with _capture_scene(monkeypatch) as captured:
        config = make_config()
        store = SnapshotStore()
        controller = make_controller(config, store, no_real_arm_factory)
        controller.start()
        try:
            await wait_until(
                lambda: store.read().phase == "TELEOP",
                "controller reaches TELEOP in pure simulation",
            )
            buffer = LogBuffer()
            async with user_simulation(
                root=lambda: build_page(controller, store, config, buffer)
            ) as user:
                await user.open("/")
                await wait_until(lambda: "visual" in captured, "page built the 3D scene")
                visual = captured["visual"]

                # -- apply Down on both arms
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                select_widget(user, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                assert np.allclose(rpy_values(user, 0), mounting.DOWN_RPY_DEG["left"])
                assert np.allclose(rpy_values(user, 1), mounting.DOWN_RPY_DEG["right"])
                await user.should_see("effective RPY (-90.0, 0.0, 90.0)")
                await user.should_see("effective RPY (90.0, 0.0, 90.0)")
                _click_button(user, "Apply")
                await wait_until(
                    lambda: (
                        store.read().settings.get("revision") == 1
                        and controller.config.frames.power_port_facing_right
                        == mounting.POWER_PORT_FACING_DOWN
                        and settings_path.is_file()
                    ),
                    "both Down presets accepted and persisted",
                )
                down_bases = arm_base_in_horizon(controller.config.frames)
                await wait_until(
                    lambda: np.allclose(visual.bases.left, down_bases.left)
                    and np.allclose(visual.bases.right, down_bases.right),
                    "scene rotates to the Down presets",
                )
                data = json.loads(settings_path.read_text(encoding="utf-8"))
                assert data["mounts"] == {"left": "down", "right": "down"}
                revision_before_restore = int(store.read().settings["revision"])
                await wait_until(
                    lambda: _button(user, "Restore Defaults").enabled,
                    "Restore Defaults button enabled",
                )

                # -- Restore Defaults: the controller accepts Front again and
                #    the override file is removed.
                _click_button(user, "Restore Defaults")
                await wait_until(
                    lambda: (
                        store.read().settings.get("revision") == revision_before_restore + 1
                        and not settings_path.exists()
                        and controller.config.frames.power_port_facing_left
                        == mounting.POWER_PORT_FACING_FRONT
                        and controller.config.frames.power_port_facing_right
                        == mounting.POWER_PORT_FACING_FRONT
                    ),
                    "Restore Defaults accepted, config back to Front and settings.json removed",
                )
                await user.should_see("Defaults restored and applied.")

                defaults = ControlConfig()
                assert controller.config.frames == defaults.frames
                front_bases = arm_base_in_horizon(defaults.frames)
                assert np.allclose(controller.bases.left, front_bases.left)
                assert np.allclose(controller.bases.right, front_bases.right)
                await _wait_for_scene_bases(
                    visual,
                    {"left": front_bases.left, "right": front_bases.right},
                    "scene returns to the Front preset bases",
                )
                _assert_base_group(visual.left, front_bases.left, "left arm")
                _assert_base_group(visual.right, front_bases.right, "right arm")
                _assert_descends_from(visual.left.axes, visual.left.base_group)
                _assert_descends_from(visual.left.port_marker, visual.left.base_group)
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT
                assert select_widget(user, "right").value == mounting.POWER_PORT_FACING_FRONT
                assert np.allclose(rpy_values(user, 0), mounting.FRONT_RPY_DEG["left"])
                assert np.allclose(rpy_values(user, 1), mounting.FRONT_RPY_DEG["right"])
                assert len(_labels(user, "Front preset: power port faces forward")) == 2
                assert not (REPO_ROOT / "settings.json").exists(), (
                    "the UI test must never write the repository-root settings.json"
                )
        finally:
            controller.stop()
            controller.wait(timeout=5.0)
