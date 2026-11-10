"""Persistence-failure and late-ack fault injection through the real UI callback.

These tests drive the real ``build_page`` Settings panel and the real
``ArmController`` control thread; only the injectable failure points are
replaced (``save_settings``/``reset_settings`` writing/delete exception, a
rollback verdict, or a delayed command delivery). They assert that a failed
persistence is never reported as success unless both the controller rollback
and the file restoration were confirmed, and that a request whose page-side
wait timed out can never take effect later.
"""
from __future__ import annotations

import logging
import math
from contextlib import contextmanager
from dataclasses import replace

import numpy as np
from nicegui import ui
from nicegui.testing.user_simulation import user_simulation

import mounting
import settings_store
import ui.app as app_module
import ui.settings as settings_module
from config import ControlConfig
from control_loop import ArmController
from controller_state import ControlCommand, SettingsTransaction, SnapshotStore
from frames import arm_base_in_horizon
from logging_utils import LogBuffer
from pose_source import DemoPoseSource
from ui.app import build_page
from test._settings_ui_support import click_apply, click_restore, make_config, mounts_from_file, no_real_arm_factory, select_widget, status_of, wait_until

TIMEOUT_S = 4.0


@contextmanager
def _page_harness(monkeypatch, tmp_path):
    """Isolated settings path + inert timers + captured real scene, one page."""
    settings_path = tmp_path / "settings.json"
    monkeypatch.setattr(settings_store, "SETTINGS_PATH", settings_path)
    import ui.app as app
    from ui.scene import build_scene as real_build_scene

    captured: dict = {}

    def spy(page_config: ControlConfig):
        refresh, visual = real_build_scene(page_config)
        captured["visual"] = visual
        return refresh, visual

    class _InertTimer:
        def cancel(self) -> None:
            return None

    monkeypatch.setattr(ui, "timer", lambda *args, **kwargs: _InertTimer())
    monkeypatch.setattr(app, "build_scene", spy)

    config = make_config()
    store = SnapshotStore()
    controller = ArmController(
        config,
        logging.getLogger("test"),
        DemoPoseSource(config.pose_input),
        store,
        arm_factory=no_real_arm_factory,
    )
    buffer = LogBuffer()
    handles: list = []

    def root() -> None:
        handles.append(build_page(controller, store, config, buffer))

    yield settings_path, captured, handles, controller, store, root


async def test_save_failure_with_confirmed_rollback_is_not_reported_as_success(
    monkeypatch, tmp_path
):
    with _page_harness(monkeypatch, tmp_path) as (settings_path, captured, handles, controller, store, root):
        controller.start()
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user:
                await user.open("/")
                await wait_until(lambda: handles and "visual" in captured, "page built")
                visual = captured["visual"]
                real_save = settings_module.save_settings

                def failing_save(*args, **kwargs):
                    raise OSError("injected disk full")

                monkeypatch.setattr(settings_module, "save_settings", failing_save)
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user)
                await wait_until(
                    lambda: "rolled back" in status_of(handles),
                    "the failed save reports a confirmed rollback",
                )
                message = status_of(handles)
                assert "disk full" in message
                assert "Applied." not in message
                # Controller, scene, panel and file all show the previous config.
                assert controller.config.frames.power_port_facing_left == "front"
                assert controller.settings_revision == 2  # apply + confirmed rollback
                front = arm_base_in_horizon(controller.config.frames)
                assert np.allclose(visual.bases.left, front.left)
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT
                assert not settings_path.exists()

                # Recovery: with persistence healthy again, the operator can
                # retry and the panel/file/controller converge.
                monkeypatch.setattr(settings_module, "save_settings", real_save)
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user)
                await wait_until(
                    lambda: mounts_from_file(settings_path) == {"left": "down", "right": "front"}
                    and "Applied." in status_of(handles),
                    "retry persists the change",
                )
                await wait_until(
                    lambda: controller.config.frames.power_port_facing_left == "down",
                    "controller committed the retried change",
                )
        finally:
            controller.stop()
            controller.wait(timeout=5.0)


async def test_save_failure_with_unconfirmed_rollback_shows_actual_config(
    monkeypatch, tmp_path
):
    with _page_harness(monkeypatch, tmp_path) as (settings_path, captured, handles, controller, store, root):
        controller.start()
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user:
                await user.open("/")
                await wait_until(lambda: handles and "visual" in captured, "page built")
                visual = captured["visual"]
                real_save = settings_module.save_settings
                real_request = app_module.request_settings_apply
                calls = {"count": 0}

                async def request_with_rejected_rollback(controller_, config_, base_revision=None):
                    calls["count"] += 1
                    if calls["count"] == 1:
                        return await real_request(controller_, config_, base_revision)
                    return False, "simulated rollback rejection: real arm started following", -1

                def failing_save(*args, **kwargs):
                    raise OSError("injected disk full")

                monkeypatch.setattr(settings_module, "save_settings", failing_save)
                monkeypatch.setattr(app_module, "request_settings_apply", request_with_rejected_rollback)
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user)
                await wait_until(
                    lambda: "rollback failed" in status_of(handles),
                    "the unconfirmed rollback is reported",
                )
                message = status_of(handles)
                assert "rolled back" not in message
                assert "Applied." not in message
                assert "Apply again" in message
                # The *actual* active config (the candidate) is shown, not the
                # previous values as a fake rollback claim.
                assert controller.config.frames.power_port_facing_left == "down"
                down = arm_base_in_horizon(controller.config.frames)
                assert np.allclose(visual.bases.left, down.left)
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN
                assert mounts_from_file(settings_path) is None

                # Recovery: apply again with healthy persistence.
                monkeypatch.setattr(settings_module, "save_settings", real_save)
                monkeypatch.setattr(app_module, "request_settings_apply", real_request)
                click_apply(user)
                await wait_until(
                    lambda: mounts_from_file(settings_path) == {"left": "down", "right": "front"}
                    and "Applied." in status_of(handles),
                    "re-apply persists the active configuration",
                )
                assert controller.config.frames.power_port_facing_left == "down"
        finally:
            controller.stop()
            controller.wait(timeout=5.0)


async def test_restore_delete_failure_with_unconfirmed_rollback_is_not_success(
    monkeypatch, tmp_path
):
    with _page_harness(monkeypatch, tmp_path) as (settings_path, captured, handles, controller, store, root):
        controller.start()
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user:
                await user.open("/")
                await wait_until(lambda: handles and "visual" in captured, "page built")
                # Establish a persisted Down configuration first.
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user)
                await wait_until(
                    lambda: mounts_from_file(settings_path) == {"left": "down", "right": "front"},
                    "baseline Down is persisted",
                )
                real_reset = settings_module.reset_settings
                real_request = app_module.request_settings_apply
                calls = {"count": 0}

                async def request_with_timed_out_rollback(controller_, config_, base_revision=None):
                    calls["count"] += 1
                    if calls["count"] == 1:
                        return await real_request(controller_, config_, base_revision)
                    return False, "controller did not acknowledge the rollback: timeout", -1

                def failing_reset(*args, **kwargs):
                    raise OSError("injected delete failure")

                monkeypatch.setattr(settings_module, "reset_settings", failing_reset)
                monkeypatch.setattr(app_module, "request_settings_apply", request_with_timed_out_rollback)
                click_restore(user)
                await wait_until(
                    lambda: "Restore Defaults failed" in status_of(handles),
                    "the failed delete is reported",
                )
                message = status_of(handles)
                assert "Defaults restored and applied." not in message
                assert "rolled back" not in message
                assert "Apply again" in message
                # Actual active config (defaults) is displayed; the old file is
                # still on disk, so the message must warn about the mismatch.
                assert controller.config.frames == ControlConfig().frames
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT
                assert mounts_from_file(settings_path) == {"left": "down", "right": "front"}

                # Recovery: deleting works again; the retry resolves the mismatch.
                monkeypatch.setattr(settings_module, "reset_settings", real_reset)
                monkeypatch.setattr(app_module, "request_settings_apply", real_request)
                click_restore(user)
                await wait_until(
                    lambda: not settings_path.exists()
                    and "Defaults restored and applied." in status_of(handles),
                    "retry removes the file",
                )
                assert controller.config.frames == ControlConfig().frames
        finally:
            controller.stop()
            controller.wait(timeout=5.0)


async def test_restore_delete_failure_with_confirmed_rollback_restores_down(
    monkeypatch, tmp_path
):
    with _page_harness(monkeypatch, tmp_path) as (settings_path, captured, handles, controller, store, root):
        controller.start()
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user:
                await user.open("/")
                await wait_until(lambda: handles and "visual" in captured, "page built")
                visual = captured["visual"]
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user)
                await wait_until(
                    lambda: mounts_from_file(settings_path) == {"left": "down", "right": "front"},
                    "baseline Down is persisted",
                )
                real_reset = settings_module.reset_settings

                def failing_reset(*args, **kwargs):
                    raise OSError("injected delete failure")

                monkeypatch.setattr(settings_module, "reset_settings", failing_reset)
                click_restore(user)
                await wait_until(
                    lambda: "rolled back" in status_of(handles),
                    "the confirmed rollback is reported",
                )
                message = status_of(handles)
                assert "Defaults restored and applied." not in message
                # Confirmed rollback: controller/scene/panel/file are all Down again.
                assert controller.config.frames.power_port_facing_left == "down"
                down = arm_base_in_horizon(controller.config.frames)
                assert np.allclose(visual.bases.left, down.left)
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN
                assert mounts_from_file(settings_path) == {"left": "down", "right": "front"}

                monkeypatch.setattr(settings_module, "reset_settings", real_reset)
                click_restore(user)
                await wait_until(
                    lambda: not settings_path.exists()
                    and "Defaults restored and applied." in status_of(handles),
                    "retry restores defaults",
                )
        finally:
            controller.stop()
            controller.wait(timeout=5.0)


async def test_gui_timed_out_request_never_applies_when_delivered_late(
    monkeypatch, tmp_path
):
    with _page_harness(monkeypatch, tmp_path) as (settings_path, captured, handles, controller, store, root):
        controller.start()
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user:
                await user.open("/")
                await wait_until(lambda: handles and "visual" in captured, "page built")
                visual = captured["visual"]
                monkeypatch.setattr(app_module, "SETTINGS_ACK_TIMEOUT_S", 0.05)
                real_submit = controller.submit
                withheld = []

                def delaying_submit(command) -> None:
                    if command.kind == "APPLY_SETTINGS":
                        withheld.append(command)
                    else:
                        real_submit(command)

                monkeypatch.setattr(controller, "submit", delaying_submit)
                select_widget(user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user)
                await wait_until(
                    lambda: "timed out" in status_of(handles),
                    "the page reports the timeout",
                )
                assert not settings_path.exists()
                assert controller.config.frames.power_port_facing_left == "front"
                assert controller.settings_revision == 0
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_FRONT
                assert np.allclose(visual.bases.left, arm_base_in_horizon(controller.config.frames).left)

                # Deliver the expired request late: it must be refused, not applied.
                monkeypatch.setattr(controller, "submit", real_submit)
                assert withheld, "the request was not withheld"
                for command in withheld:
                    controller.commands.put(command)
                controller._drain_commands()
                assert controller.config.frames.power_port_facing_left == "front"
                assert controller.settings_revision == 0
                assert not settings_path.exists()
                assert "expired" in str(controller._settings_error or "")
        finally:
            controller.stop()
            controller.wait(timeout=5.0)

async def test_stale_rejection_then_validation_error_does_not_revert_newer_config(
    monkeypatch, tmp_path
):
    """Independent review finding: a stale page must keep showing the newer config.

    Sequence: another (already open) page commits Down; this page's stale Apply
    is rejected and its panel is refreshed to Down; a later local validation
    error must repaint the *active* Down config (not the stale pre-refresh one),
    and fixing only the typo must not silently revert the newer Down config.
    """
    with _page_harness(monkeypatch, tmp_path) as (settings_path, captured, handles, controller, store, root):
        controller.start()
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user:
                await user.open("/")
                await wait_until(lambda: handles and "visual" in captured, "page built")
                # Another page commits Down first; this page has not refreshed.
                down = replace(
                    controller.config,
                    frames=replace(
                        controller.config.frames,
                        power_port_facing_left="down",
                        rotation_left_shoulder_to_left_arm_base_rpy_deg=mounting.DOWN_RPY_DEG["left"],
                    ),
                )
                transaction = SettingsTransaction(down, controller.settings_revision)
                controller.submit(
                    ControlCommand("APPLY_SETTINGS", config=down, transaction=transaction)
                )
                assert transaction.future.result(2.0) == (True, "")
                rev_after_remote = controller.settings_revision
                await wait_until(
                    lambda: controller.config.frames.power_port_facing_left == "down",
                    "the other page's Down became active",
                )

                # Stale Apply: rejected, panel syncs to the newer active config.
                click_apply(user)
                await wait_until(
                    lambda: "rejected" in status_of(handles)
                    and select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN,
                    "the stale Apply is rejected and the panel syncs to Down",
                )

                # A local validation error must repaint the panel from the
                # active config, not from the stale pre-refresh config.
                numbers = list(user.find(kind=ui.number).elements)
                j1 = next(
                    number
                    for number in numbers
                    if abs(float(number.value or 0.0) - (-2.61)) < 0.02
                )
                j1.set_value(999.0)
                click_apply(user)
                await wait_until(lambda: "Apply failed" in status_of(handles), "validation error")
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN, (
                    "the validation-error path must not repaint stale values"
                )

                # Fixing only the typo must apply the *displayed* Down config,
                # never silently revert the newer revision back to Front.
                j1.set_value(math.degrees(ControlConfig().startup.left_pose_rad[0]))
                click_apply(user)
                await wait_until(
                    lambda: "Applied." in status_of(handles)
                    and controller.settings_revision > rev_after_remote,
                    "the corrected Apply commits",
                )
                assert controller.config.frames.power_port_facing_left == "down"
                assert select_widget(user, "left").value == mounting.POWER_PORT_FACING_DOWN
                assert mounts_from_file(settings_path) == {"left": "down", "right": "front"}
        finally:
            controller.stop()
            controller.wait(timeout=5.0)
