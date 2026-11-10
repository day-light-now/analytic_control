"""Two real pages/clients: effective config + revision are the single truth.

Page A applies a change; a page B that was already open must mirror the same
actual/target/ghost bases and Settings values. A stale page may not silently
overwrite a newer revision, and concurrent Apply/Restore from two pages is
serialised: exactly one commit wins, the other is rejected as stale.
"""
from __future__ import annotations

import json
from contextlib import contextmanager
from typing import Any

import httpx
import numpy as np
from nicegui import core, ui
from nicegui.testing import User
from nicegui.testing.user_simulation import user_simulation

import mounting
import settings_store
from config import ControlConfig
from controller_state import SnapshotStore
from frames import arm_base_in_horizon
from logging_utils import LogBuffer
from ui.app import build_page
from ui.scene import SCENE_FROM_HORIZON
from test._settings_ui_support import click_apply, click_restore, make_config, make_controller, mounts_from_file, no_real_arm_factory, rpy_numbers, select_widget, wait_until

SIDES = ("left", "right")
HORIZON_SCENE = np.asarray(SCENE_FROM_HORIZON, dtype=float)
TIMEOUT_S = 4.0


@contextmanager
def _capture_pages(monkeypatch):
    """Spy on the real build_scene and collect page handles in creation order.

    The NiceGUI refresh timers are replaced by inert handles: the tests drive
    each page's real ``refresh`` callback explicitly, which makes "this page did
    not see the newer revision yet" deterministic instead of racing a timer.
    """
    import ui.app as app_module
    from ui.scene import build_scene as real_build_scene

    captured: dict[str, list[Any]] = {"visuals": [], "handles": []}

    def spy(page_config: ControlConfig):
        refresh, visual = real_build_scene(page_config)
        captured["visuals"].append(visual)
        return refresh, visual

    class _InertTimer:
        def cancel(self) -> None:
            return None

    monkeypatch.setattr(app_module.ui, "timer", lambda *args, **kwargs: _InertTimer())
    monkeypatch.setattr(app_module, "build_scene", spy)
    yield captured


async def _open_second_user() -> User:
    client = httpx.AsyncClient(transport=httpx.ASGITransport(core.app), base_url="http://test")
    user = User(client)
    await user.open("/")
    return user


def _base_rotation(arm) -> np.ndarray:
    return np.asarray(arm.base_group.R, dtype=float)


async def _wait_visuals_synced(pages, visuals, expected, description: str) -> None:
    """Drive real page refreshes until both scenes mirror the expected bases."""

    def matches() -> bool:
        for page in pages:
            page["refresh"]()
        for visual in visuals:
            for side in SIDES:
                if not np.allclose(getattr(visual.bases, side), getattr(expected, side)):
                    return False
        return True

    await wait_until(matches, description)


def _effective_mounts(controller) -> dict:
    frames = controller.config.frames
    return {
        "left": frames.power_port_facing_left,
        "right": frames.power_port_facing_right,
    }


async def wait_settings_idle(store) -> None:
    """Wait until no page owns the settings transaction lifecycle."""
    await wait_until(
        lambda: ((store.read().settings or {}).get("transaction") or {}).get("state")
        == "idle",
        "settings ownership is released",
    )


def _page_setup(monkeypatch, tmp_path):
    """Create the controller and the root builder for two simulated clients."""
    settings_path = tmp_path / "settings.json"
    monkeypatch.setattr(settings_store, "SETTINGS_PATH", settings_path)
    handles: list[dict] = []

    config = make_config()
    store = SnapshotStore()
    controller = make_controller(config, store, no_real_arm_factory)
    buffer = LogBuffer()

    def root():
        handles.append(build_page(controller, store, config, buffer))

    return settings_path, handles, root, controller, store


async def test_second_page_mirrors_new_revision_and_stale_submit_is_rejected(
    tmp_path, monkeypatch
):
    with _capture_pages(monkeypatch) as captured:
        settings_path, handles, root, controller, store = _page_setup(monkeypatch, tmp_path)
        controller.start()
        user_b: User | None = None
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "controller TELEOP")
            async with user_simulation(root=root) as user_a:
                await user_a.open("/")
                user_b = await _open_second_user()
                await wait_until(
                    lambda: len(captured["visuals"]) == 2 and len(handles) == 2,
                    "two pages were built",
                )
                page_a, page_b = handles
                visual_a, visual_b = captured["visuals"]
                # Timers are driven explicitly in the simulation harness.
                page_a["refresh"]()
                page_b["refresh"]()
                front = arm_base_in_horizon(controller.config.frames)
                assert np.allclose(visual_a.bases.left, front.left)
                assert np.allclose(visual_b.bases.left, front.left)

                # --- page A commits Down on the left arm
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user_a)
                await wait_until(
                    lambda: store.read().settings.get("revision") == 1
                    and mounts_from_file(settings_path) == {"left": "down", "right": "front"}
                    and controller.config.frames.power_port_facing_left == "down",
                    "page A committed Down and persisted it",
                )
                await wait_settings_idle(store)
                page_a["refresh"]()
                page_b["refresh"]()
                expected_after_a = arm_base_in_horizon(controller.config.frames)
                # Page B (opened before the change) mirrors the new base and panel.
                assert np.allclose(visual_b.bases.left, expected_after_a.left)
                assert np.allclose(visual_b.bases.right, expected_after_a.right)
                assert np.allclose(
                    _base_rotation(visual_b.left), HORIZON_SCENE @ mounting.DOWN_ROTATION["left"]
                )
                assert select_widget(user_b, "left").value == mounting.POWER_PORT_FACING_DOWN
                assert "another page" in (page_b["settings"]["status"]() or "")

                # --- an unsaved edit on page B is discarded when another page commits
                rpy_numbers(user_b, 1)[0].set_value(10.0)
                assert select_widget(user_b, "right").value == mounting.POWER_PORT_FACING_CUSTOM
                select_widget(user_a, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user_a)
                await wait_until(
                    lambda: store.read().settings.get("revision") == 2
                    and controller.config.frames.power_port_facing_right == "down"
                    and mounts_from_file(settings_path)
                    == {"left": "down", "right": "down"},
                    "page A committed the right arm Down",
                )
                await wait_settings_idle(store)
                page_b["refresh"]()
                assert select_widget(user_b, "right").value == mounting.POWER_PORT_FACING_DOWN
                assert np.allclose(
                    [float(number.value) for number in rpy_numbers(user_b, 1)],
                    mounting.DOWN_RPY_DEG["right"],
                )
                assert "another page" in (page_b["settings"]["status"]() or "")

                # --- page B did not refresh after A's next change: its submit is stale
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_FRONT)
                click_apply(user_a)
                await wait_until(
                    lambda: store.read().settings.get("revision") == 3
                    and controller.config.frames.power_port_facing_left == "front"
                    and mounts_from_file(settings_path) == {"left": "front", "right": "down"},
                    "page A committed the left arm back to Front",
                )
                await wait_settings_idle(store)
                file_before = settings_path.read_text(encoding="utf-8")
                select_widget(user_b, "right").set_value(mounting.POWER_PORT_FACING_FRONT)
                click_apply(user_b)
                await wait_until(
                    lambda: "rejected" in (page_b["settings"]["status"]() or ""),
                    "page B stale submit is rejected",
                )
                assert "newer settings revision" in (page_b["settings"]["status"]() or "")
                assert store.read().settings.get("revision") == 3
                assert controller.config.frames.power_port_facing_left == "front"
                assert controller.config.frames.power_port_facing_right == "down"
                assert settings_path.read_text(encoding="utf-8") == file_before, (
                    "a stale page submission overwrote the newer settings file"
                )
                data = json.loads(file_before)
                assert data["mounts"] == {"left": "front", "right": "down"}
                # The stale page is resynchronised to the newer effective config.
                assert select_widget(user_b, "right").value == mounting.POWER_PORT_FACING_DOWN
                page_b["refresh"]()
                assert np.allclose(
                    _base_rotation(visual_b.right), HORIZON_SCENE @ mounting.DOWN_ROTATION["right"]
                )
        finally:
            if user_b is not None:
                await user_b.http_client.aclose()
            controller.stop()
            controller.wait(timeout=5.0)


async def test_concurrent_apply_and_restore_serialise_and_converge(tmp_path, monkeypatch):
    """Apply from page A racing Restore Defaults from page B: one wins, no mixed file."""
    with _capture_pages(monkeypatch) as captured:
        settings_path, handles, root, controller, store = _page_setup(monkeypatch, tmp_path)
        controller.start()
        user_b: User | None = None
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "controller TELEOP")
            async with user_simulation(root=root) as user_a:
                await user_a.open("/")
                user_b = await _open_second_user()
                await wait_until(
                    lambda: len(captured["visuals"]) == 2 and len(handles) == 2,
                    "two pages were built",
                )
                page_a, page_b = handles
                visual_a, visual_b = captured["visuals"]
                page_a["refresh"]()
                page_b["refresh"]()
                # Common starting point: both pages see a persisted Down/Down.
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                select_widget(user_a, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user_a)
                await wait_until(
                    lambda: store.read().settings.get("revision") == 1
                    and mounts_from_file(settings_path) == {"left": "down", "right": "down"},
                    "baseline Down/Down is persisted",
                )
                page_a["refresh"]()
                page_b["refresh"]()
                # A applies Front/Right-down edit while B restores defaults from
                # the same base revision: exactly one may commit.
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_FRONT)
                click_apply(user_a)
                click_restore(user_b)
                await wait_until(
                    lambda: controller.settings_revision >= 2,
                    "one of Apply/Restore commits",
                )
                # Wait for the global settings ownership to be released so both
                # outcomes are settled before inspecting UI status / files.
                await wait_until(
                    lambda: ((store.read().settings or {}).get("transaction") or {}).get("state")
                    == "idle",
                    "settings ownership is released",
                )
                await wait_until(
                    lambda: any(
                        status.startswith("Applied.")
                        or "Defaults restored and applied." in status
                        for status in (
                            page_a["settings"]["status"]() or "",
                            page_b["settings"]["status"]() or "",
                        )
                    ),
                    "the winning operation reported success",
                )
                statuses = {
                    index: page["settings"]["status"]() or ""
                    for index, page in enumerate((page_a, page_b))
                }
                applied_pages = {
                    index: status.startswith("Applied.")
                    or "Defaults restored and applied." in status
                    for index, status in statuses.items()
                }
                rejected_pages = {
                    index: "rejected" in status for index, status in statuses.items()
                }
                assert sum(applied_pages.values()) == 1, statuses
                await wait_until(
                    lambda: store.read().settings.get("revision") == 2,
                    "the winning revision is published",
                )
                effective = _effective_mounts(controller)
                expected = arm_base_in_horizon(controller.config.frames)
                await _wait_visuals_synced(
                    (page_a, page_b), (visual_a, visual_b), expected, "both scenes converge"
                )
                if applied_pages[1]:
                    # Restore won: defaults are active, the override file is gone,
                    # and the losing Apply must not have written anything.
                    assert effective == {
                        "left": mounting.POWER_PORT_FACING_FRONT,
                        "right": mounting.POWER_PORT_FACING_FRONT,
                    }
                    assert not settings_path.exists()
                    assert rejected_pages[0], "the losing Apply was not rejected"
                    assert select_widget(user_a, "left").value == mounting.POWER_PORT_FACING_FRONT
                    assert select_widget(user_b, "left").value == mounting.POWER_PORT_FACING_FRONT
                else:
                    # Apply won: the file matches the controller exactly; the
                    # racing Restore loses on ownership or stale revision.
                    assert applied_pages[0]
                    assert effective == {
                        "left": mounting.POWER_PORT_FACING_FRONT,
                        "right": mounting.POWER_PORT_FACING_DOWN,
                    }
                    assert mounts_from_file(settings_path) == effective
                    assert rejected_pages[1], "the racing Restore was not rejected"
                # Final file/controller agreement for whichever operation won.
                if settings_path.exists():
                    assert mounts_from_file(settings_path) == effective
                else:
                    assert effective == {
                        "left": mounting.POWER_PORT_FACING_FRONT,
                        "right": mounting.POWER_PORT_FACING_FRONT,
                    }
        finally:
            if user_b is not None:
                await user_b.http_client.aclose()
            controller.stop()
            controller.wait(timeout=5.0)


async def test_concurrent_submits_from_two_pages_serialise_on_revision(tmp_path, monkeypatch):
    with _capture_pages(monkeypatch) as captured:
        settings_path, handles, root, controller, store = _page_setup(monkeypatch, tmp_path)
        controller.start()
        user_b: User | None = None
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "controller TELEOP")
            async with user_simulation(root=root) as user_a:
                await user_a.open("/")
                user_b = await _open_second_user()
                await wait_until(
                    lambda: len(captured["visuals"]) == 2 and len(handles) == 2,
                    "two pages were built",
                )
                page_a, page_b = handles
                visual_a, visual_b = captured["visuals"]
                page_a["refresh"]()
                page_b["refresh"]()

                # Both pages hold revision 0 and submit different candidates before
                # either can refresh: exactly one may win.
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                select_widget(user_b, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user_a)
                click_apply(user_b)
                await wait_until(
                    lambda: store.read().settings.get("revision") == 1
                    and mounts_from_file(settings_path) == _effective_mounts(controller),
                    "one of the two concurrent submissions committed and persisted",
                )
                await wait_until(
                    lambda: any(
                        "rejected" in (page["settings"]["status"]() or "")
                        for page in (page_a, page_b)
                    ),
                    "the losing page reports a rejection",
                )
                assert controller.settings_revision == 1
                await wait_settings_idle(store)
                page_a["refresh"]()
                page_b["refresh"]()
                effective = controller.config
                data = json.loads(settings_path.read_text(encoding="utf-8"))
                assert data["mounts"] == {
                    "left": effective.frames.power_port_facing_left,
                    "right": effective.frames.power_port_facing_right,
                }
                # Exactly one side changed away from Front, matching the winner.
                down_sides = {
                    side
                    for side in SIDES
                    if getattr(effective.frames, f"power_port_facing_{side}") == "down"
                }
                assert len(down_sides) == 1, f"expected exactly one winner, got {down_sides}"
                expected = arm_base_in_horizon(effective.frames)
                for visual in (visual_a, visual_b):
                    assert np.allclose(visual.bases.left, expected.left)
                    assert np.allclose(visual.bases.right, expected.right)

                # Both pages now converge on revision 1 and a second round is also
                # serialised: the later submit is stale, never a silent overwrite.
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_FRONT)
                select_widget(user_b, "left").set_value(mounting.POWER_PORT_FACING_FRONT)
                click_apply(user_a)
                click_apply(user_b)
                await wait_until(
                    lambda: controller.settings_revision == 2
                    and mounts_from_file(settings_path) == _effective_mounts(controller),
                    "a second commit happens and is persisted",
                )
                await wait_until(
                    lambda: any(
                        "newer settings revision" in (page["settings"]["status"]() or "")
                        or "another page is applying settings"
                        in (page["settings"]["status"]() or "")
                        for page in (page_a, page_b)
                    ),
                    "the stale second-round page is rejected",
                )
                assert controller.settings_revision == 2
                await wait_settings_idle(store)
                page_a["refresh"]()
                page_b["refresh"]()
                data = json.loads(settings_path.read_text(encoding="utf-8"))
                assert data["mounts"] == _effective_mounts(controller)
        finally:
            if user_b is not None:
                await user_b.http_client.aclose()
            controller.stop()
            controller.wait(timeout=5.0)
