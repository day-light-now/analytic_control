"""Shared helpers for the Settings-panel GUI tests (real page + real controller).

Only page/controller scaffolding and widget lookup helpers live here; the tests
keep their own scenario-specific wiring and fault injection.
"""
from __future__ import annotations

import asyncio
import time
from dataclasses import replace

from nicegui import ui
from nicegui.testing.user_simulation import user_simulation

from config import ControlConfig
from control_loop import ArmController
from controller_state import SnapshotStore
from logging_utils import LogBuffer
from pose_source import DemoPoseSource
from ui.app import build_page

TIMEOUT_S = 4.0


def make_config() -> ControlConfig:
    config = ControlConfig()
    return replace(
        config,
        startup=replace(config.startup, enable_startup_pose=False),
        motion=replace(config.motion, command_hz=100.0),
    )


def no_real_arm_factory(side, arm_config, motion, logger):
    raise AssertionError(f"test only covers pure simulation, got {side} arm backend")


def make_controller(config: ControlConfig, store: SnapshotStore, arm_factory) -> ArmController:
    return ArmController(
        config,
        __import__("logging").getLogger("test"),
        DemoPoseSource(config.pose_input),
        store,
        arm_factory=arm_factory,
    )


def build_page_root(handles: list, controller, store, config) -> None:
    """user_simulation root that records the real build_page handles."""
    handles.append(build_page(controller, store, config, LogBuffer()))


async def wait_until(predicate, description: str, timeout: float = TIMEOUT_S) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.01)
    raise AssertionError(f"timed out waiting for: {description}")


def select_widget(user, side: str) -> ui.select:
    elements = list(user.find(marker=f"power-port-facing-{side}").elements)
    assert len(elements) == 1, f"expected one power-port select for {side}, got {len(elements)}"
    element = elements[0]
    assert isinstance(element, ui.select)
    return element


def click_button(user, label: str) -> None:
    interaction = user.find(kind=ui.button, content=label)
    assert len(interaction.elements) == 1, f"expected exactly one {label!r} button"
    interaction.click()


def click_apply(user) -> None:
    click_button(user, "Apply")


def click_restore(user) -> None:
    click_button(user, "Restore Defaults")


def rpy_numbers(user, side_index: int) -> list:
    labels = sorted(
        list(user.find(kind=ui.label, content="Shoulder to Base RPY (deg)").elements),
        key=lambda element: element.id,
    )
    assert len(labels) == 2, f"expected two RPY rows, got {len(labels)}"
    row = labels[side_index].parent_slot.parent
    numbers = [child for child in row.default_slot.children if isinstance(child, ui.number)]
    assert len(numbers) == 3, f"expected three RPY numbers, got {len(numbers)}"
    return numbers


def rpy_values(user, side_index: int) -> list[float]:
    return [float(number.value) for number in rpy_numbers(user, side_index)]


def mounts_from_file(settings_path) -> dict | None:
    import json

    try:
        return json.loads(settings_path.read_text(encoding="utf-8"))["mounts"]
    except (OSError, ValueError, KeyError):
        return None


def inert_timers(monkeypatch) -> None:
    """Replace the real page refresh timer with an inert handle.

    The tests drive the real page refresh callback explicitly, which keeps
    cross-page/timing assertions deterministic under the simulation harness.
    """

    class _InertTimer:
        def cancel(self) -> None:
            return None

    monkeypatch.setattr(ui, "timer", lambda *args, **kwargs: _InertTimer())


def status_of(handles: list) -> str:
    return str(handles[0]["settings"]["status"]() or "")


__all__ = [
    "TIMEOUT_S",
    "build_page_root",
    "click_apply",
    "click_button",
    "click_restore",
    "inert_timers",
    "make_config",
    "make_controller",
    "mounts_from_file",
    "no_real_arm_factory",
    "rpy_numbers",
    "rpy_values",
    "select_widget",
    "status_of",
    "user_simulation",
    "wait_until",
]
