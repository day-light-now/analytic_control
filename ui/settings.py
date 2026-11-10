from __future__ import annotations

import math
from typing import Awaitable, Callable

from nicegui import ui

import mounting
from config import ControlConfig
from settings_store import (
    apply_settings,
    read_settings_text,
    reset_settings,
    restore_settings_text,
    save_settings,
    settings_data,
)

SIDES = mounting.SIDES
ApplyHandler = Callable[[ControlConfig, int], Awaitable[tuple[bool, str, int]]]
EffectiveConfigGetter = Callable[[], ControlConfig]
EffectiveRevisionGetter = Callable[[], int]
SYNC_NOTE = "Settings changed in another page; inputs refreshed to the currently active configuration."


def build_settings(
    config: ControlConfig,
    on_apply: ApplyHandler,
    *,
    revision: int = 0,
    effective_config: EffectiveConfigGetter | None = None,
    effective_revision: EffectiveRevisionGetter | None = None,
    transaction_owner=None,
) -> dict:
    """Settings panel.

    The whole ``commit -> persist -> recover`` lifecycle is owned by
    ``transaction_owner`` (the controller's SettingsTransactionManager) when it
    is provided: a second page cannot commit/persist in the middle of this
    page's file write, so an older revision can never overwrite a newer one.
    The controller commits the candidate (with a version check) first, then the
    file is written; write/delete failures are recovered and recorded as
    converged/unconverged for START gating.
    """
    values = settings_data(config)
    startup_inputs = {}
    frame_inputs = {}
    mount_inputs = {}
    mount_notes = {}
    syncing = {side: False for side in SIDES}
    state = {"revision": int(revision), "busy": False}
    transaction_note = {"active": False}

    def effective() -> tuple[ControlConfig, int]:
        try:
            cfg = effective_config() if effective_config is not None else config
            rev = (
                int(effective_revision())
                if effective_revision is not None
                else state["revision"]
            )
            return cfg, rev
        except Exception:
            return config, state["revision"]

    ui.label("The controller commits the change, then it is persisted.").classes(
        "text-sm text-slate-600"
    )

    with ui.card().classes("w-full"):
        ui.label("Startup Pose").classes("font-semibold text-base")
        for side, title in (("left", "Left Arm"), ("right", "Right Arm")):
            ui.label(f"{title} (deg)").classes("text-xs font-semibold text-slate-600")
            with ui.row().classes("w-full items-center gap-2"):
                inputs = []
                for index, value in enumerate(values["startup"][f"{side}_pose_rad"]):
                    ui.label(f"J{index + 1}").classes("text-xs")
                    inputs.append(
                        ui.number(value=math.degrees(value), step=0.1, format="%.2f")
                        .props("dense outlined")
                        .classes("w-24")
                    )
            startup_inputs[side] = inputs

    def rpy_key(side: str) -> str:
        return f"rotation_{side}_shoulder_to_{side}_arm_base_rpy_deg"

    def set_startup_inputs(source: dict) -> None:
        for side in SIDES:
            for index, number in enumerate(startup_inputs[side]):
                number.set_value(math.degrees(source["startup"][f"{side}_pose_rad"][index]))

    def set_frame_inputs(source: dict) -> None:
        for side in SIDES:
            for key, inputs in frame_inputs[side].items():
                for index, number in enumerate(inputs):
                    number.set_value(source["frames"][key][index])

    def set_rpy(side: str, rpy_deg: tuple[float, float, float]) -> None:
        for index, number in enumerate(frame_inputs[side][rpy_key(side)]):
            number.set_value(rpy_deg[index])

    def set_note(side: str) -> None:
        mount_notes[side].set_text(
            mounting.describe_facing(
                side, mount_inputs[side].value or mounting.POWER_PORT_FACING_CUSTOM
            )
        )

    def apply_facing_to_inputs(side: str, facing: str) -> None:
        # Front/Down auto-fill the preset RPY; the fields stay editable so a
        # manual correction is possible, and any such edit flips the row to
        # Custom via on_manual_rpy to keep the label and the values consistent.
        if facing in mounting.PRESET_RPY_DEG:
            set_rpy(side, mounting.PRESET_RPY_DEG[facing][side])
        set_note(side)

    def on_facing_change(side: str) -> None:
        if syncing[side]:
            return
        syncing[side] = True
        try:
            apply_facing_to_inputs(side, mount_inputs[side].value)
        finally:
            syncing[side] = False

    def on_manual_rpy(side: str) -> None:
        if syncing[side] or state["busy"]:
            return
        if mount_inputs[side].value == mounting.POWER_PORT_FACING_CUSTOM:
            return
        # Editing a number must never leave a preset label contradicting the
        # values, so the row switches itself to Custom.
        syncing[side] = True
        try:
            mount_inputs[side].set_value(mounting.POWER_PORT_FACING_CUSTOM)
            set_note(side)
        finally:
            syncing[side] = False

    def sync_ui(source: ControlConfig) -> None:
        """Mirror ``source`` into the panel; it becomes the last known effective config."""
        nonlocal config
        config = source
        data = settings_data(source)
        set_startup_inputs(data)
        set_frame_inputs(data)
        for side in SIDES:
            syncing[side] = True
            try:
                mount_inputs[side].set_value(data["mounts"][side])
                apply_facing_to_inputs(side, data["mounts"][side])
            finally:
                syncing[side] = False

    def collect_data() -> dict:
        return {
            "startup": {
                f"{side}_pose_rad": [math.radians(number.value) for number in startup_inputs[side]]
                for side in SIDES
            },
            "frames": {
                key: [number.value for number in inputs]
                for side in SIDES
                for key, inputs in frame_inputs[side].items()
            },
            "mounts": {side: mount_inputs[side].value for side in SIDES},
        }

    def set_panel_editable(editable: bool) -> None:
        """Freeze all editable widgets while a settings transaction is in flight."""
        for side in SIDES:
            for number in startup_inputs[side]:
                number.set_enabled(editable)
            for inputs in frame_inputs[side].values():
                for number in inputs:
                    number.set_enabled(editable)
            mount_inputs[side].set_enabled(editable)

    with ui.card().classes("w-full"):
        ui.label("Base Frames").classes("font-semibold text-base")
        for side, title in (("left", "Left Arm"), ("right", "Right Arm")):
            ui.label(title).classes("text-xs font-semibold text-slate-600")
            frame_inputs[side] = {}
            for key, label, step, fmt in (
                (f"translation_horizon_to_{side}_shoulder_m", "Horizon to Shoulder (m)", 0.001, "%.4f"),
                (f"translation_{side}_shoulder_to_{side}_arm_base_m", "Shoulder to Base (m)", 0.001, "%.4f"),
                (rpy_key(side), "Shoulder to Base RPY (deg)", 0.1, "%.2f"),
            ):
                with ui.row().classes("w-full items-center gap-2"):
                    ui.label(label).classes("w-52 text-xs")
                    inputs = [
                        ui.number(value=value, step=step, format=fmt)
                        .props("dense outlined")
                        .classes("w-24")
                        for value in values["frames"][key]
                    ]
                frame_inputs[side][key] = inputs
            with ui.row().classes("w-full items-center gap-2"):
                ui.label("Power Port Facing").classes("w-52 text-xs")
                mount_inputs[side] = (
                    ui.select(mounting.POWER_PORT_FACING_LABELS, value=values["mounts"][side])
                    .props("dense outlined")
                    .classes("w-40")
                    .mark(f"power-port-facing-{side}")
                )
            mount_notes[side] = ui.label("").classes("text-xs text-slate-500")
            mount_inputs[side].on_value_change(lambda _=None, side=side: on_facing_change(side))
            for number in frame_inputs[side][rpy_key(side)]:
                number.on_value_change(lambda _=None, side=side: on_manual_rpy(side))
            syncing[side] = True
            try:
                apply_facing_to_inputs(side, values["mounts"][side])
            finally:
                syncing[side] = False

    status = ui.label("").classes("text-sm text-slate-700")

    def set_status(text: str) -> None:
        status.set_text(text)

    def settle_rejection(reason: str, prefix: str) -> None:
        """Show the actual active configuration after a rejected request."""
        cfg, rev = effective()
        if rev > state["revision"]:
            state["revision"] = rev
            sync_ui(cfg)
            message = f"{prefix} rejected: {reason or 'controller declined'} (inputs refreshed to the newer active configuration)"
        else:
            sync_ui(config)
            message = f"{prefix} rejected: {reason or 'controller declined'}"
        set_status(message)
        ui.notify(message, type="negative")

    def snapshot_settings_file() -> tuple[bool, str | None]:
        try:
            return True, read_settings_text()
        except Exception:
            return False, None

    async def recover_failed_persistence(
        prefix: str,
        failure: Exception,
        previous_config: ControlConfig,
        file_snapshot: tuple[bool, str | None],
    ) -> tuple[bool, str]:
        """Confirm the rollback of controller + file; never claim an unconfirmed one.

        Returns ``(converged, reason)``: a confirmed rollback is converged only
        if the restored starting state was already consistent. Otherwise the
        active config is displayed and START stays blocked until persistence
        succeeds on a later Apply/Restore.
        """
        file_error: Exception | str | None = None
        known, previous_text = file_snapshot
        if known:
            try:
                restore_settings_text(previous_text)
            except Exception as exc:
                file_error = exc
        else:
            file_error = "could not snapshot settings.json before the write"
        rollback_ok = False
        rollback_reason = "rollback was not requested"
        rollback_revision = -1
        try:
            rollback_ok, rollback_reason, rollback_revision = await on_apply(
                previous_config, state["revision"]
            )
        except Exception as exc:
            rollback_reason = f"rollback request failed: {exc}"
        previous_was_converged = (
            transaction_owner is None or transaction_owner.was_converged_before_begin()
        )
        if rollback_ok and file_error is None and previous_was_converged:
            state["revision"] = int(rollback_revision)
            sync_ui(previous_config)
            message = (
                f"{prefix}; the change was rolled back to the previous configuration ({failure})."
            )
            set_status(message)
            ui.notify(message, type="warning")
            return True, ""
        # Unconfirmed: show the *actual* active configuration and ask for a retry;
        # never present the previous values as if they were active.
        active_config, active_revision = effective()
        state["revision"] = int(active_revision)
        sync_ui(active_config)
        if not rollback_ok:
            details = f"rollback failed: {rollback_reason}"
        elif file_error is not None:
            details = f"settings file could not be restored: {file_error}"
        else:
            details = "rollback restored a configuration that was already not persisted"
        message = (
            f"{prefix}: {failure}. The currently active configuration is shown, but settings.json "
            f"was not updated ({details}). Apply again to persist the active configuration."
        )
        set_status(message)
        ui.notify(message, type="negative")
        return False, message

    def transaction_outcome(
        acquired: bool, committed: bool, submitted: bool, outcome: dict
    ) -> None:
        """Release the global settings ownership with its convergence verdict.

        A cancelled/interrupted page-side wait is conservatively treated as
        unconverged (START stays blocked until a later Apply/Restore succeeds):
        the controller may already have committed even though this page never
        received the verdict.
        """
        if not acquired or transaction_owner is None:
            return
        if outcome["decided"]:
            transaction_owner.finish(outcome["converged"], outcome["reason"])
        elif outcome.get("abort") or not submitted:
            transaction_owner.abort()
        elif committed:
            transaction_owner.finish(
                False, "settings transaction was interrupted before persistence"
            )
        else:
            transaction_owner.finish(
                False, "settings transaction was interrupted before its outcome was known"
            )

    async def save() -> None:
        if state["busy"]:
            return
        try:
            data = collect_data()
            candidate = apply_settings(config, data)
        except Exception as exc:
            sync_ui(config)
            message = f"Apply failed: {exc}"
            set_status(message)
            ui.notify(message, type="negative")
            return
        acquired = False
        committed = False
        submitted = False
        outcome = {"decided": False, "converged": True, "reason": "", "abort": False}
        if transaction_owner is not None:
            acquired, blocked_reason = transaction_owner.begin()
            if not acquired:
                settle_rejection(blocked_reason, "Apply")
                return
        state["busy"] = True
        apply_button.set_enabled(False)
        restore_button.set_enabled(False)
        set_panel_editable(False)
        try:
            previous_config = config
            file_snapshot = snapshot_settings_file()
            submitted = True
            accepted, reason, revision = await on_apply(candidate, state["revision"])
            if not accepted:
                outcome["abort"] = True
                settle_rejection(reason, "Apply")
                return
            committed = True
            state["revision"] = int(revision)
            try:
                save_settings(data)
            except Exception as exc:
                converged, message = await recover_failed_persistence(
                    "Save failed", exc, previous_config, file_snapshot
                )
                outcome.update(decided=True, converged=converged, reason=message)
                return
            # The operator may have typed while the transaction was in flight;
            # the panel always mirrors the configuration that was committed and
            # persisted, never a half-applied edit.
            sync_ui(candidate)
            set_status("Applied.")
            ui.notify("Settings applied.", type="positive")
            outcome.update(decided=True, converged=True, reason="")
        finally:
            # Release the cross-page ownership before un-marking this page busy,
            # so this page's own refresh never reports "another page" for itself.
            transaction_outcome(acquired, committed, submitted, outcome)
            state["busy"] = False
            set_panel_editable(True)
            apply_button.set_enabled(True)
            restore_button.set_enabled(True)

    async def restore() -> None:
        if state["busy"]:
            return
        acquired = False
        committed = False
        submitted = False
        outcome = {"decided": False, "converged": True, "reason": "", "abort": False}
        if transaction_owner is not None:
            acquired, blocked_reason = transaction_owner.begin()
            if not acquired:
                settle_rejection(blocked_reason, "Restore Defaults")
                return
        state["busy"] = True
        apply_button.set_enabled(False)
        restore_button.set_enabled(False)
        set_panel_editable(False)
        try:
            previous_config = config
            file_snapshot = snapshot_settings_file()
            defaults = ControlConfig()
            submitted = True
            accepted, reason, revision = await on_apply(defaults, state["revision"])
            if not accepted:
                outcome["abort"] = True
                settle_rejection(reason, "Restore Defaults")
                return
            committed = True
            state["revision"] = int(revision)
            try:
                reset_settings()
            except Exception as exc:
                converged, message = await recover_failed_persistence(
                    "Restore Defaults failed", exc, previous_config, file_snapshot
                )
                outcome.update(decided=True, converged=converged, reason=message)
                return
            sync_ui(defaults)
            set_status("Defaults restored and applied.")
            ui.notify("Defaults restored and applied.", type="positive")
            outcome.update(decided=True, converged=True, reason="")
        finally:
            transaction_outcome(acquired, committed, submitted, outcome)
            state["busy"] = False
            set_panel_editable(True)
            apply_button.set_enabled(True)
            restore_button.set_enabled(True)

    def note_transaction(transaction) -> None:
        """Surface another page's in-flight/unconverged settings state."""
        if state["busy"] or not isinstance(transaction, dict):
            return
        tx_state = transaction.get("state")
        if tx_state == "in_flight":
            transaction_note["active"] = True
            set_status("A settings transaction is in progress on another page; wait for it to finish.")
        elif tx_state == "unconverged":
            transaction_note["active"] = True
            reason = transaction.get("reason") or "settings are not persisted"
            set_status(f"Settings are not persisted yet: {reason} Apply again to resolve.")
        elif transaction_note["active"]:
            transaction_note["active"] = False
            set_status("")

    def sync_if_behind(source: ControlConfig, source_revision: int) -> None:
        """Mirror a newer effective configuration into this page's panel."""
        if state["busy"]:
            return
        if int(source_revision) <= state["revision"]:
            return
        state["revision"] = int(source_revision)
        sync_ui(source)
        set_status(SYNC_NOTE)

    with ui.row().classes("w-full items-center gap-2"):
        apply_button = ui.button("Apply", on_click=save).props("unelevated")
        restore_button = ui.button("Restore Defaults", on_click=restore).props("outline")

    return {
        "sync_if_behind": sync_if_behind,
        "note_transaction": note_transaction,
        "revision": lambda: state["revision"],
        "busy": lambda: state["busy"],
        "status": lambda: status.text,
    }
