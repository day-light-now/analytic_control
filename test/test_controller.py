from __future__ import annotations

import logging
import threading
import time
from concurrent.futures import Future
from contextlib import contextmanager
from dataclasses import replace

import numpy as np

import mounting
from config import ControlConfig
from control_loop import ArmController
from controller_state import ControlCommand, SettingsTransaction, SnapshotStore
from pose_source import DemoPoseSource


class FakeArm:
    def __init__(self, side: str, config, motion, logger) -> None:
        self.side = side
        self.config = config
        self.firmware = "v120"
        self.q = np.zeros(7, dtype=float)
        self.moves = []
        self.holds = []
        self.sent = []
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
        self.sent.append((None if left_q is None else np.asarray(left_q).copy(), None if right_q is None else np.asarray(right_q).copy()))

    def close(self) -> None:
        self.closed = True


def _config() -> ControlConfig:
    config = ControlConfig()
    return replace(
        config,
        startup=replace(config.startup, enable_startup_pose=False),
        motion=replace(config.motion, command_hz=100.0),
    )


def _wait(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition not reached")


@contextmanager
def _controller():
    config = _config()
    store = SnapshotStore()
    pose = DemoPoseSource(config.pose_input)
    arms = {}

    def factory(side, arm_config, motion, logger):
        arms[side] = FakeArm(side, arm_config, motion, logger)
        return arms[side]

    controller = ArmController(config, logging.getLogger("test"), pose, store, arm_factory=factory)
    controller.start()
    try:
        yield controller, store, arms
    finally:
        controller.stop()
        controller.wait(timeout=3.0)


def test_pure_simulation_enters_teleop():
    with _controller() as (_controller_obj, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: not np.allclose(store.read().sim_q["left"], np.zeros(7)))
        assert not arms


def test_connect_latches_stop_without_motion():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        assert store.read().arm_status["left"].state == "STOPPED"
        assert not arms["left"].moves


def test_connect_second_arm_while_stopped():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can1"))
        _wait(lambda: store.read().arm_status["right"].state == "STOPPED")
        assert store.read().phase == "STOPPED"
        assert set(arms) == {"left", "right"}


def test_start_returns_simulation_and_arm_to_startup():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        assert len(arms["left"].moves) == 1
        assert np.allclose(arms["left"].moves[0], np.asarray(ControlConfig().startup.left_pose_rad))
        assert store.read().arm_status["left"].state == "TELEOP"


def test_start_moves_both_arms():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can1"))
        _wait(lambda: store.read().arm_status["right"].state == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        assert len(arms["left"].moves) == 1
        assert len(arms["right"].moves) == 1
        assert store.read().arm_status["left"].state == "TELEOP"
        assert store.read().arm_status["right"].state == "TELEOP"
        _wait(lambda: bool(arms["left"].sent) and bool(arms["right"].sent))
        left_q, right_q = arms["left"].sent[-1]
        assert left_q is not None and right_q is None
        left_q, right_q = arms["right"].sent[-1]
        assert left_q is None and right_q is not None


def test_duplicate_channel_is_rejected():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("CONNECT", side="right", channel="can0"))
        _wait(lambda: store.read().arm_status["right"].state == "ERROR")
        assert "channel" in (store.read().arm_status["right"].error or "")


def test_stop_holds_simulation_and_following_arm():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: bool(arms["left"].sent))
        controller.submit(ControlCommand("STOP"))
        _wait(lambda: store.read().phase == "STOPPED")
        before = store.read().sim_q["left"]
        sent_before = len(arms["left"].sent)
        time.sleep(0.2)
        assert store.read().sim_q["left"] == before
        assert len(arms["left"].sent) > sent_before


def test_start_without_connected_arm_returns_to_teleop():
    with _controller() as (controller, store, _arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("STOP"))
        _wait(lambda: store.read().phase == "STOPPED")
        _wait(lambda: store.read().can_reset)
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")


def test_snapshot_reports_tightest_limit_margin():
    with _controller() as (controller, store, _arms):
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: store.read().joint_margin_deg is not None)
        snapshot = store.read()
        assert snapshot.joint_margin_deg is not None and snapshot.joint_margin_deg >= 0.0
        assert snapshot.joint_margin_index is not None
        assert 0 <= snapshot.joint_margin_index <= 6


def test_dead_can_link_disconnects_arm_and_latches_stop():
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: bool(arms["left"].sent))
        arms["left"].transport_dead = "CAN transport on can0 closed"
        _wait(lambda: store.read().arm_status["left"].state == "ERROR")
        _wait(lambda: store.read().phase == "STOPPED")
        assert "CAN transport" in (store.read().arm_status["left"].error or "")
        _wait(lambda: arms["left"].closed)
        assert store.read().real_q["left"] is None


# ---------------------------------------------------------------- settings apply
def _down_config(controller: ArmController, sides=("left", "right")) -> ControlConfig:
    config = controller.config
    for side in sides:
        config = replace(
            config,
            frames=replace(
                config.frames,
                **{
                    f"power_port_facing_{side}": mounting.POWER_PORT_FACING_DOWN,
                    f"rotation_{side}_shoulder_to_{side}_arm_base_rpy_deg": mounting.DOWN_RPY_DEG[side],
                },
            ),
        )
    return config


def _apply(controller: ArmController, config: ControlConfig, timeout: float = 2.0):
    transaction = SettingsTransaction(config, controller.settings_revision)
    controller.submit(
        ControlCommand("APPLY_SETTINGS", config=config, transaction=transaction)
    )
    return transaction.future.result(timeout=timeout)


def test_apply_settings_is_accepted_in_pure_simulation_teleop():
    with _controller() as (controller, store, _arms):
        _wait(lambda: store.read().phase == "TELEOP")
        _wait(lambda: store.read().settings.get("revision") == 0)
        before_q = np.asarray(store.read().sim_q["left"], dtype=float)
        old_filters = {side: id(controller._pose_filters[side]) for side in ("left", "right")}
        old_planners = {side: id(controller._planners[side]) for side in ("left", "right")}
        old_ik_state = {side: controller.iks[side]._state for side in ("left", "right")}
        candidate = _down_config(controller)

        accepted, reason = _apply(controller, candidate)
        assert accepted, reason
        assert controller.config is candidate
        for side, expected in mounting.DOWN_ROTATION.items():
            assert np.allclose(controller.bases.__getattribute__(side)[:3, :3], expected, atol=1e-12)
        # Stale anchors, targets and filter residuals were cleared for both arms.
        assert set(controller.following) == set()
        for side in ("left", "right"):
            assert id(controller._pose_filters[side]) != old_filters[side]
            assert id(controller._planners[side]) != old_planners[side]
            assert controller.iks[side]._state is not old_ik_state[side]
            assert np.allclose(controller._planners[side].q, controller.sim_q[side])
            assert np.allclose(controller._last_hold[side], controller.sim_q[side])
        # Snapshot reports the accepted revision and no error.
        _wait(lambda: store.read().settings.get("revision") == 1)
        assert store.read().settings.get("error") is None
        # The change must not teleport the simulation joints: no tick may step
        # more than the planner's per-tick joint limit.
        _wait(lambda: np.max(np.abs(np.asarray(store.read().sim_q["left"]) - before_q)) > 0.0)
        max_step = controller.config.motion.max_joint_step_rad
        deltas = []
        previous = np.asarray(store.read().sim_q["left"], dtype=float)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            current = np.asarray(store.read().sim_q["left"], dtype=float)
            deltas.append(float(np.max(np.abs(current - previous))))
            previous = current
            time.sleep(0.002)
        assert max(deltas) <= 3.0 * max_step + 1e-9


def test_apply_settings_is_accepted_after_stop_with_a_connected_arm():
    """STOP keeps `following` for the hold stream; Apply must work in STOPPED.

    This is the documented operator workflow: follow -> STOP -> Apply -> Start.
    """
    with _controller() as (controller, store, arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP" and "left" in controller.following)
        controller.submit(ControlCommand("STOP"))
        _wait(lambda: store.read().phase == "STOPPED")
        assert "left" in controller.following  # hold stream still keeps the servo alive
        holds_before = len(arms["left"].sent)
        candidate = _down_config(controller)
        accepted, reason = _apply(controller, candidate)
        assert accepted, reason
        assert controller.config is candidate
        assert np.allclose(controller.bases.left[:3, :3], mounting.DOWN_ROTATION["left"], atol=1e-12)
        assert store.read().phase == "STOPPED"
        # The arm keeps receiving hold commands (no jump, no new motion target).
        _wait(lambda: len(arms["left"].sent) > holds_before)
        # Explicit Start re-aligns and resumes following with the new mapping.
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP" and "left" in controller.following)
        assert np.allclose(controller.bases.left[:3, :3], mounting.DOWN_ROTATION["left"], atol=1e-12)


def test_apply_settings_is_rejected_while_a_real_arm_is_following():
    with _controller() as (controller, store, _arms):
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "TELEOP" and "left" in controller.following)
        before_config = controller.config
        before_bases = {side: controller.bases.__getattribute__(side).copy() for side in ("left", "right")}
        candidate = _down_config(controller)

        accepted, reason = _apply(controller, candidate)
        assert not accepted
        assert "following" in reason and "STOP" in reason
        assert controller.config is before_config
        for side in ("left", "right"):
            assert np.array_equal(controller.bases.__getattribute__(side), before_bases[side])
        assert store.read().settings.get("revision") == 0
        _wait(lambda: bool(store.read().settings.get("error")))
        assert "following" in str(store.read().settings.get("error"))


def test_apply_settings_is_rejected_while_a_connection_is_busy():
    config = _config()
    store = SnapshotStore()
    pose = DemoPoseSource(config.pose_input)
    gate = threading.Event()
    arms = {}

    def factory(side, arm_config, motion, logger):
        gate.wait(timeout=5.0)
        arms[side] = FakeArm(side, arm_config, motion, logger)
        return arms[side]

    controller = ArmController(config, logging.getLogger("test"), pose, store, arm_factory=factory)
    controller.start()
    try:
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().arm_status["left"].state == "CONNECTING")
        candidate = _down_config(controller)
        accepted, reason = _apply(controller, candidate)
        assert not accepted
        assert "connect" in reason.lower() or "hardware" in reason.lower()
        assert controller.config is not candidate
        _wait(lambda: bool(store.read().settings.get("error")))
    finally:
        gate.set()
        controller.stop()
        controller.wait(timeout=3.0)


def test_apply_settings_is_rejected_during_startup_motion():
    config = ControlConfig()
    config = replace(config, motion=replace(config.motion, command_hz=100.0))
    store = SnapshotStore()
    pose = DemoPoseSource(config.pose_input)
    controller = ArmController(config, logging.getLogger("test"), pose, store)
    candidate = replace(
        config,
        frames=replace(
            config.frames,
            power_port_facing_left=mounting.POWER_PORT_FACING_DOWN,
            rotation_left_shoulder_to_left_arm_base_rpy_deg=mounting.DOWN_RPY_DEG["left"],
        ),
    )
    future: Future = Future()
    controller.submit(ControlCommand("APPLY_SETTINGS", config=candidate, future=future))
    controller.start()
    try:
        accepted, reason = future.result(timeout=3.0)
        assert not accepted
        assert "startup" in reason
        assert controller.config is not candidate
    finally:
        controller.stop()
        controller.wait(timeout=5.0)


def test_apply_settings_is_rejected_while_aligning():
    config = _config()
    store = SnapshotStore()
    pose = DemoPoseSource(config.pose_input)
    gate = threading.Event()
    arms = {}

    class BlockingStartupArm(FakeArm):
        def move_j_startup(self, side, q, stop_event=None):
            gate.wait(timeout=5.0)
            return super().move_j_startup(side, q, stop_event)

    def factory(side, arm_config, motion, logger):
        arms[side] = BlockingStartupArm(side, arm_config, motion, logger)
        return arms[side]

    controller = ArmController(config, logging.getLogger("test"), pose, store, arm_factory=factory)
    controller.start()
    try:
        _wait(lambda: store.read().phase == "TELEOP")
        controller.submit(ControlCommand("CONNECT", side="left", channel="can0"))
        _wait(lambda: store.read().phase == "STOPPED")
        controller.submit(ControlCommand("RESET"))
        _wait(lambda: store.read().phase == "ALIGNING")
        candidate = _down_config(controller)
        accepted, reason = _apply(controller, candidate)
        assert not accepted
        assert "align" in reason.lower()
        assert controller.config is not candidate
        gate.set()
        _wait(lambda: store.read().phase == "TELEOP")
    finally:
        gate.set()
        controller.stop()
        controller.wait(timeout=5.0)

