"""Global settings transaction ownership: commit -> persist -> recover.

These are the green regressions for the Codex second-review race
(docs/power_port_mounting_20261107/codex_commit_persist_race.py). They use the
real ArmController, two real simulated browser pages, the real Apply/Restore
callbacks and a tmp_path settings file. The old repro file itself is kept
unchanged as the red artifact.
"""
from __future__ import annotations

import asyncio
from contextlib import contextmanager

import pytest
from nicegui.testing.user_simulation import user_simulation

import mounting
import ui.app as app_module
import ui.settings as settings_module
from controller_state import ControlCommand
from test._settings_ui_support import (
    click_apply,
    click_restore,
    mounts_from_file,
    select_widget,
    wait_until,
)
from test.test_mounting_multipage import (
    _capture_pages,
    _effective_mounts,
    _open_second_user,
    _page_setup,
    wait_settings_idle,
)


@contextmanager
def _two_page_env(monkeypatch, tmp_path):
    with _capture_pages(monkeypatch):
        settings_path, pages, root, controller, store = _page_setup(monkeypatch, tmp_path)
        controller.start()
        try:
            yield settings_path, pages, root, controller, store
        finally:
            controller.stop()
            controller.wait(timeout=5.0)


def _delay_first_reply(monkeypatch):
    """Pause the first accepted reply after the controller commit (before persist)."""
    committed = asyncio.Event()
    release = asyncio.Event()
    real_request = app_module.request_settings_apply
    calls = [0]

    async def delayed(controller, config, base_revision=None):
        calls[0] += 1
        ordinal = calls[0]
        result = await real_request(controller, config, base_revision)
        if ordinal == 1 and result[0]:
            committed.set()
            await release.wait()
        return result

    monkeypatch.setattr(app_module, "request_settings_apply", delayed)
    return committed, release


def _manager_state(store) -> str:
    return str(((store.read().settings or {}).get("transaction") or {}).get("state"))


async def wait_panels(pages, users, left: str, right: str) -> None:
    """Refresh the real pages until both Settings panels mirror the active config."""

    def converged() -> bool:
        for page in pages:
            page["refresh"]()
        return all(
            select_widget(user, "left").value == left
            and select_widget(user, "right").value == right
            for user in users
        )

    await wait_until(converged, f"both pages show {left}/{right}")


@contextmanager
def _settings_file_failure(monkeypatch):
    real_save = settings_module.save_settings

    def failing_save(*args, **kwargs):
        raise OSError("injected disk full")

    monkeypatch.setattr(settings_module, "save_settings", failing_save)
    try:
        yield real_save
    finally:
        monkeypatch.setattr(settings_module, "save_settings", real_save)


@pytest.mark.parametrize("first", ["a", "b"])
async def test_late_persist_cannot_overwrite_newer_revision_and_retry_succeeds(
    tmp_path, monkeypatch, first
):
    """Green variant of codex_commit_persist_race.py (both completion orders)."""
    committed, release = _delay_first_reply(monkeypatch)
    with _two_page_env(monkeypatch, tmp_path) as (path, pages, root, controller, store):
        user_b = None
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user_a:
                await user_a.open("/")
                user_b = await _open_second_user()
                users = {"a": user_a, "b": user_b}
                first_user, second_user = users[first], users["b" if first == "a" else "a"]
                pages[0]["refresh"]()
                pages[1]["refresh"]()

                # Page 1 commits revision 1, then is paused before persistence.
                select_widget(first_user, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(first_user)
                await asyncio.wait_for(committed.wait(), 4)
                await wait_until(
                    lambda: _manager_state(store) == "in_flight",
                    "the first page owns the settings transaction",
                )
                assert controller.settings_revision == 1

                # Page 2 cannot interleave commit/persist: explicit ownership rejection.
                select_widget(second_user, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(second_user)
                await wait_until(
                    lambda: any(
                        "another page is applying settings" in (page["settings"]["status"]() or "")
                        for page in pages
                    ),
                    "the second page is rejected while ownership is held",
                )
                assert controller.settings_revision == 1
                assert not (path.exists() and mounts_from_file(path) == {
                    "left": "down", "right": "down"
                })

                # Release page 1: revision 1 is persisted and remains consistent.
                release.set()
                await wait_until(
                    lambda: _manager_state(store) == "idle"
                    and mounts_from_file(path) == {"left": "down", "right": "front"},
                    "revision 1 is persisted under ownership",
                )
                assert _effective_mounts(controller) == mounts_from_file(path)
                assert controller.settings_revision == 1

                # Page 2 retries on top of revision 1: no lost update, no stale file.
                pages[0]["refresh"]()
                pages[1]["refresh"]()
                select_widget(second_user, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(second_user)
                await wait_until(
                    lambda: controller.settings_revision == 2
                    and _manager_state(store) == "idle"
                    and mounts_from_file(path) == {"left": "down", "right": "down"},
                    "the retried revision 2 is committed and persisted",
                )
                assert _effective_mounts(controller) == mounts_from_file(path) == {
                    "left": "down",
                    "right": "down",
                }
                # Both scenes converge on the same authoritative revision.
                await wait_until(
                    lambda: store.read().settings.get("revision") == 2,
                    "revision 2 is published to the pages",
                )
                await wait_panels(
                    pages,
                    (user_a, user_b),
                    mounting.POWER_PORT_FACING_DOWN,
                    mounting.POWER_PORT_FACING_DOWN,
                )
        finally:
            release.set()
            if user_b is not None:
                await user_b.http_client.aclose()


@pytest.mark.parametrize("retry_action", ["apply", "restore"])
async def test_failed_retry_preserves_unconverged_state_until_persistence_succeeds(
    tmp_path, monkeypatch, retry_action
):
    """Rolling back a failed retry restores the old mismatch, not consistency."""
    real_request = app_module.request_settings_apply
    calls = 0

    async def reject_first_rollback(controller, config, base_revision=None):
        nonlocal calls
        calls += 1
        if calls == 2:
            return False, "injected first rollback rejection", -1
        return await real_request(controller, config, base_revision)

    monkeypatch.setattr(app_module, "request_settings_apply", reject_first_rollback)
    with _settings_file_failure(monkeypatch) as real_save:
        real_reset = settings_module.reset_settings

        def failing_reset(*args, **kwargs):
            raise OSError("injected persistent delete failure")

        monkeypatch.setattr(settings_module, "reset_settings", failing_reset)
        with _two_page_env(monkeypatch, tmp_path) as (path, pages, root, controller, store):
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user:
                await user.open("/")
                controller.submit(ControlCommand("STOP"))
                await wait_until(lambda: store.read().phase == "STOPPED", "STOPPED")
                select_widget(user, "left").set_value("down")
                click_apply(user)
                await wait_until(lambda: _manager_state(store) == "unconverged"
                    and not pages[0]["settings"]["busy"](), "first failure left a mismatch")
                assert not path.exists()
                assert _effective_mounts(controller) == {"left": "down", "right": "front"}

                retry = click_apply if retry_action == "apply" else click_restore
                retry(user)
                await wait_until(lambda: controller.settings_revision == 3
                    and not pages[0]["settings"]["busy"](), "failed retry rolled back")
                await wait_until(lambda: _manager_state(store) == "unconverged", "mismatch stays latched")
                assert not path.exists()
                assert _effective_mounts(controller) == {"left": "down", "right": "front"}
                assert "already not persisted" in pages[0]["settings"]["status"]()
                assert not store.read().can_reset
                controller.submit(ControlCommand("RESET"))
                await wait_until(lambda: "Start is blocked" in str(store.read().settings.get("error")),
                    "direct RESET is still refused")
                assert store.read().phase == "STOPPED"

                monkeypatch.setattr(settings_module, "save_settings", real_save)
                monkeypatch.setattr(settings_module, "reset_settings", real_reset)
                retry(user)
                await wait_until(lambda: controller.settings_revision == 4
                    and _manager_state(store) == "idle" and store.read().can_reset,
                    "healthy persistence releases START")
                expected = {"left": "down" if retry_action == "apply" else "front", "right": "front"}
                assert _effective_mounts(controller) == expected
                if retry_action == "apply":
                    assert mounts_from_file(path) == expected
                else:
                    assert not path.exists()
                assert select_widget(user, "left").value == expected["left"]
                controller.submit(ControlCommand("RESET"))
                await wait_until(lambda: store.read().phase == "TELEOP", "START accepted after resolution")


async def test_apply_and_restore_are_serialised_by_ownership(tmp_path, monkeypatch):
    """A running Apply blocks a Restore from another page until ownership is released."""
    committed, release = _delay_first_reply(monkeypatch)
    with _two_page_env(monkeypatch, tmp_path) as (path, pages, root, controller, store):
        user_b = None
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user_a:
                await user_a.open("/")
                user_b = await _open_second_user()
                pages[0]["refresh"]()
                pages[1]["refresh"]()
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user_a)
                await asyncio.wait_for(committed.wait(), 4)
                await wait_until(
                    lambda: _manager_state(store) == "in_flight", "Apply owns the transaction"
                )

                click_restore(user_b)
                await wait_until(
                    lambda: "another page is applying settings"
                    in (pages[1]["settings"]["status"]() or ""),
                    "the Restore is rejected while Apply owns the transaction",
                )
                assert not path.exists(), "the rejected Restore must not delete the file"

                release.set()
                await wait_until(
                    lambda: _manager_state(store) == "idle"
                    and mounts_from_file(path) == {"left": "down", "right": "front"},
                    "the Apply is persisted under ownership",
                )
                assert controller.settings_revision == 1

                pages[0]["refresh"]()
                pages[1]["refresh"]()
                click_restore(user_b)
                await wait_until(
                    lambda: controller.settings_revision == 2
                    and _manager_state(store) == "idle"
                    and not path.exists(),
                    "the retried Restore commits and removes the file",
                )
                assert _effective_mounts(controller) == {
                    "left": mounting.POWER_PORT_FACING_FRONT,
                    "right": mounting.POWER_PORT_FACING_FRONT,
                }
                await wait_until(
                    lambda: store.read().settings.get("revision") == 2,
                    "revision 2 is published to the pages",
                )
                await wait_panels(
                    pages,
                    (user_a, user_b),
                    mounting.POWER_PORT_FACING_FRONT,
                    mounting.POWER_PORT_FACING_FRONT,
                )
        finally:
            release.set()
            if user_b is not None:
                await user_b.http_client.aclose()


async def test_write_failure_recovery_blocks_other_page_commit(tmp_path, monkeypatch):
    """During persist-failure recovery the ownership still blocks a new-page Apply."""
    rollback_started = asyncio.Event()
    release_rollback = asyncio.Event()
    real_request = app_module.request_settings_apply
    calls = [0]

    async def delayed_rollback(controller, config, base_revision=None):
        calls[0] += 1
        if calls[0] == 2:
            rollback_started.set()
            await release_rollback.wait()
        return await real_request(controller, config, base_revision)

    monkeypatch.setattr(app_module, "request_settings_apply", delayed_rollback)
    with _settings_file_failure(monkeypatch):
        with _two_page_env(monkeypatch, tmp_path) as (path, pages, root, controller, store):
            user_b = None
            try:
                await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
                async with user_simulation(root=root) as user_a:
                    await user_a.open("/")
                    user_b = await _open_second_user()
                    pages[0]["refresh"]()
                    pages[1]["refresh"]()
                    select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                    click_apply(user_a)
                    await asyncio.wait_for(rollback_started.wait(), 4)
                    await wait_until(
                        lambda: _manager_state(store) == "in_flight",
                        "recovery holds the transaction ownership",
                    )

                    select_widget(user_b, "right").set_value(mounting.POWER_PORT_FACING_DOWN)
                    click_apply(user_b)
                    await wait_until(
                        lambda: "another page is applying settings"
                        in (pages[1]["settings"]["status"]() or ""),
                        "the new-page Apply is rejected during recovery",
                    )
                    assert controller.settings_revision == 1

                    release_rollback.set()
                    await wait_until(
                        lambda: _manager_state(store) == "idle"
                        and "rolled back" in (pages[0]["settings"]["status"]() or ""),
                        "the confirmed rollback finishes and releases ownership",
                    )
                    assert controller.settings_revision == 2
                    assert _effective_mounts(controller) == {
                        "left": mounting.POWER_PORT_FACING_FRONT,
                        "right": mounting.POWER_PORT_FACING_FRONT,
                    }
                    assert not path.exists()
            finally:
                release_rollback.set()
                if user_b is not None:
                    await user_b.http_client.aclose()


async def test_start_is_blocked_during_transaction_and_resumes_after(tmp_path, monkeypatch):
    committed, release = _delay_first_reply(monkeypatch)
    with _two_page_env(monkeypatch, tmp_path) as (path, pages, root, controller, store):
        user_b = None
        try:
            await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
            async with user_simulation(root=root) as user_a:
                await user_a.open("/")
                user_b = await _open_second_user()
                pages[0]["refresh"]()
                pages[1]["refresh"]()
                controller.submit(ControlCommand("STOP"))
                await wait_until(lambda: store.read().phase == "STOPPED", "STOPPED")
                select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                click_apply(user_a)
                await asyncio.wait_for(committed.wait(), 4)
                await wait_until(
                    lambda: _manager_state(store) == "in_flight", "Apply owns the transaction"
                )
                assert store.read().can_reset is False

                controller.submit(ControlCommand("RESET"))
                await wait_until(
                    lambda: "in progress" in str(store.read().settings.get("error") or ""),
                    "RESET is refused while the settings transaction is in flight",
                )
                assert store.read().phase == "STOPPED"

                release.set()
                await wait_until(
                    lambda: _manager_state(store) == "idle"
                    and mounts_from_file(path) == {"left": "down", "right": "front"},
                    "Apply persists and releases ownership",
                )
                await wait_until(lambda: store.read().can_reset is True, "START becomes available")
                controller.submit(ControlCommand("RESET"))
                await wait_until(lambda: store.read().phase == "TELEOP", "START accepted")
        finally:
            release.set()
            if user_b is not None:
                await user_b.http_client.aclose()


async def test_start_blocked_after_unconverged_persist_and_recovers(tmp_path, monkeypatch):
    """Unconfirmed recovery blocks START until a later Apply persists successfully."""
    real_request = app_module.request_settings_apply
    calls = [0]

    async def rejected_rollback(controller, config, base_revision=None):
        calls[0] += 1
        if calls[0] == 2:
            return False, "simulated rollback rejection", -1
        return await real_request(controller, config, base_revision)

    monkeypatch.setattr(app_module, "request_settings_apply", rejected_rollback)
    with _settings_file_failure(monkeypatch) as real_save:
        with _two_page_env(monkeypatch, tmp_path) as (path, pages, root, controller, store):
            user_b = None
            try:
                await wait_until(lambda: store.read().phase == "TELEOP", "TELEOP")
                async with user_simulation(root=root) as user_a:
                    await user_a.open("/")
                    user_b = await _open_second_user()
                    pages[0]["refresh"]()
                    pages[1]["refresh"]()
                    controller.submit(ControlCommand("STOP"))
                    await wait_until(lambda: store.read().phase == "STOPPED", "STOPPED")
                    select_widget(user_a, "left").set_value(mounting.POWER_PORT_FACING_DOWN)
                    click_apply(user_a)
                    await wait_until(
                        lambda: _manager_state(store) == "unconverged",
                        "the failed persistence is marked unconverged",
                    )
                    assert store.read().can_reset is False
                    blocker = str(store.read().settings.get("error") or "")
                    assert blocker == ""  # set when START is actually attempted

                    controller.submit(ControlCommand("RESET"))
                    await wait_until(
                        lambda: "Start is blocked" in str(store.read().settings.get("error") or ""),
                        "START is refused while settings are unconverged",
                    )
                    assert store.read().phase == "STOPPED"

                    # A later Apply with healthy persistence resolves the state.
                    monkeypatch.setattr(app_module, "request_settings_apply", real_request)
                    monkeypatch.setattr(settings_module, "save_settings", real_save)
                    click_apply(user_a)
                    await wait_until(
                        lambda: _manager_state(store) == "idle"
                        and store.read().can_reset is True,
                        "a successful Apply converges the settings",
                    )
                    controller.submit(ControlCommand("RESET"))
                    await wait_until(lambda: store.read().phase == "TELEOP", "START accepted")
            finally:
                if user_b is not None:
                    await user_b.http_client.aclose()
