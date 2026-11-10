from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import replace

import ui.app as app
from config import ControlConfig
from control_loop import ArmController
from controller_state import ControlCommand, SettingsTransaction, SnapshotStore
from pose_source import DemoPoseSource


def _config() -> ControlConfig:
    config = ControlConfig()
    return replace(config, startup=replace(config.startup, enable_startup_pose=False))


def _candidate(config: ControlConfig, facing: str = "down") -> ControlConfig:
    return replace(
        config,
        frames=replace(
            config.frames,
            power_port_facing_left=facing,
            rotation_left_shoulder_to_left_arm_base_rpy_deg=(-90.0, 0.0, 90.0)
            if facing == "down"
            else config.frames.rotation_left_shoulder_to_left_arm_base_rpy_deg,
        ),
    )


def _controller() -> ArmController:
    config = _config()
    controller = ArmController(
        config,
        logging.getLogger("test"),
        DemoPoseSource(config.pose_input),
        SnapshotStore(),
    )
    controller.phase = "STOPPED"
    return controller


def _drain(controller: ArmController) -> None:
    controller._drain_commands()
    controller._executor.shutdown(wait=True)


def _accepted(controller: ArmController, config: ControlConfig, base_revision: int) -> SettingsTransaction:
    transaction = SettingsTransaction(config, base_revision)
    controller.submit(ControlCommand("APPLY_SETTINGS", config=config, transaction=transaction))
    controller._drain_commands()
    return transaction


def test_timed_out_request_never_applies_later(monkeypatch):
    """Green regression for docs/.../codex_late_ack_repro.py (late-ack window)."""
    monkeypatch.setattr(app, "SETTINGS_ACK_TIMEOUT_S", 0.02)
    controller = _controller()
    candidate = _candidate(controller.config)
    try:
        accepted, reason, revision = asyncio.run(
            app.request_settings_apply(controller, candidate)
        )
        assert accepted is False
        assert "timed out" in reason and "cancelled" in reason
        assert revision == -1
        # The page has given up; the queued command must not take effect even
        # though the control thread consumes it afterwards.
        assert controller.config.frames.power_port_facing_left == "front"
        controller._drain_commands()
        assert controller.config.frames.power_port_facing_left == "front"
        assert controller.settings_revision == 0
        assert "expired" in str(controller._settings_error)
        controller._drain_commands()
        assert controller.config.frames.power_port_facing_left == "front"
    finally:
        _drain(controller)


def test_controller_commit_before_timeout_is_reported_accepted(monkeypatch):
    monkeypatch.setattr(app, "SETTINGS_ACK_TIMEOUT_S", 0.5)
    controller = _controller()
    candidate = _candidate(controller.config)

    def drain_soon() -> None:
        time.sleep(0.01)
        controller._drain_commands()

    thread = threading.Thread(target=drain_soon, daemon=True)
    thread.start()
    try:
        accepted, reason, revision = asyncio.run(
            app.request_settings_apply(controller, candidate)
        )
        thread.join(timeout=2.0)
        assert accepted is True, reason
        assert revision == 1
        assert controller.config.frames.power_port_facing_left == "down"
        assert controller.settings_revision == revision
    finally:
        _drain(controller)


def test_timeout_and_commit_race_never_desynchronise(monkeypatch):
    """Whatever the timing, the page verdict and controller state must agree."""
    monkeypatch.setattr(app, "SETTINGS_ACK_TIMEOUT_S", 0.02)
    for delay in (0.0, 0.004, 0.008, 0.012, 0.016, 0.02, 0.024, 0.03):
        controller = _controller()
        candidate = _candidate(controller.config)

        def drain_soon(delay=delay, controller=controller) -> None:
            time.sleep(delay)
            controller._drain_commands()

        thread = threading.Thread(target=drain_soon, daemon=True)
        thread.start()
        try:
            accepted, reason, revision = asyncio.run(
                app.request_settings_apply(controller, candidate)
            )
            thread.join(timeout=2.0)
            if accepted:
                assert controller.config.frames.power_port_facing_left == "down"
                assert controller.settings_revision == revision == 1
            else:
                assert controller.config.frames.power_port_facing_left == "front"
                assert controller.settings_revision == 0
                assert revision == -1
                assert reason
            controller._drain_commands()
            expected = "down" if accepted else "front"
            assert controller.config.frames.power_port_facing_left == expected
        finally:
            _drain(controller)


def test_expired_request_is_rejected_on_consumption_and_stays_dead(monkeypatch):
    """Expiry is sticky: draining repeatedly (or after a later request) never revives it."""
    monkeypatch.setattr(app, "SETTINGS_ACK_TIMEOUT_S", 0.02)
    controller = _controller()
    candidate = _candidate(controller.config)
    try:
        accepted, _reason, _revision = asyncio.run(
            app.request_settings_apply(controller, candidate)
        )
        assert accepted is False
        for _ in range(3):
            controller._drain_commands()
            assert controller.config.frames.power_port_facing_left == "front"
            assert controller.settings_revision == 0
        # A later, fresh request is still allowed to apply normally.
        transaction = _accepted(controller, candidate, controller.settings_revision)
        assert transaction.state == SettingsTransaction.ACCEPTED
        assert controller.config.frames.power_port_facing_left == "down"
        assert controller.settings_revision == 1
    finally:
        _drain(controller)


def test_stale_page_revision_is_rejected_without_overwriting(monkeypatch):
    controller = _controller()
    first = _candidate(controller.config, "down")
    try:
        transaction = _accepted(controller, first, 0)
        assert transaction.state == SettingsTransaction.ACCEPTED
        assert controller.settings_revision == 1
        # A second page that still believes it is on revision 0 submits an old
        # edit: it must be refused, not silently overwrite the newer config.
        stale_pending = _candidate(controller.config, "front")
        stale = SettingsTransaction(stale_pending, 0)
        controller.submit(
            ControlCommand("APPLY_SETTINGS", config=stale_pending, transaction=stale)
        )
        controller._drain_commands()
        assert stale.state == SettingsTransaction.REJECTED
        assert "newer settings revision" in stale.reason
        assert "reload the page" in stale.reason
        assert controller.config.frames.power_port_facing_left == "down"
        assert controller.settings_revision == 1
    finally:
        _drain(controller)


def test_legacy_future_command_still_acknowledged():
    """The old ControlCommand.future path remains supported for compatibility."""
    from concurrent.futures import Future

    controller = _controller()
    candidate = _candidate(controller.config)
    future: Future = Future()
    try:
        controller.submit(ControlCommand("APPLY_SETTINGS", config=candidate, future=future))
        controller._drain_commands()
        accepted, reason = future.result(timeout=2.0)
        assert accepted is True, reason
        assert controller.config.frames.power_port_facing_left == "down"
        assert controller.settings_revision == 1
    finally:
        _drain(controller)


def test_snapshot_exposes_effective_config_and_revision():
    controller = _controller()
    candidate = _candidate(controller.config)
    try:
        _accepted(controller, candidate, 0)
        store = controller.store
        controller._publish_snapshot()
        snapshot = store.read()
        assert snapshot.settings["revision"] == 1
        assert snapshot.settings_config is candidate
    finally:
        _drain(controller)
