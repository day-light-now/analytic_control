from __future__ import annotations

import threading
from concurrent.futures import Future
from dataclasses import dataclass, field


SIDES = ("left", "right")


@dataclass(frozen=True)
class ControllerSnapshot:
    valid: bool
    xyz: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    buttons: int = 0


@dataclass(frozen=True)
class ArmStatus:
    state: str = "SIM"
    channel: str = ""
    error: str | None = None


@dataclass(frozen=True)
class ControlSnapshot:
    phase: str = "STARTING"
    pico: dict = field(default_factory=dict)
    controllers: dict[str, ControllerSnapshot | None] = field(
        default_factory=lambda: {side: None for side in SIDES}
    )
    sim_q: dict[str, tuple[float, ...]] = field(
        default_factory=lambda: {side: (0.0,) * 7 for side in SIDES}
    )
    real_q: dict[str, tuple[float, ...] | None] = field(
        default_factory=lambda: {side: None for side in SIDES}
    )
    arm_status: dict[str, ArmStatus] = field(
        default_factory=lambda: {side: ArmStatus() for side in SIDES}
    )
    ros: dict = field(default_factory=dict)
    # Outcome of the last APPLY_SETTINGS request: {"revision": n, "error": str|None}.
    settings: dict = field(default_factory=dict)
    # The one authoritative effective ControlConfig (immutable) for this revision.
    # Every open page mirrors its actual/target/ghost models and its Settings panel
    # from this value, so a page that applied a change and a page that did not
    # cannot drift apart.
    settings_config: object = None
    error: str | None = None
    can_reset: bool = False
    joint_margin_deg: float | None = None
    joint_margin_index: int | None = None


class SettingsTransaction:
    """One settings commit request with explicit expiry/commit semantics.

    The page-side timeout and the control-thread commit share this object's
    lock, so exactly one of two outcomes happens: the control thread commits
    while the request is still pending (the page learns accepted), or the page
    expires the request before it is consumed (the control thread must not
    apply it). The control thread therefore never applies a request whose
    waiter already reported a timeout, closing the late-ack window.
    """

    PENDING = "pending"
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    EXPIRED = "expired"

    def __init__(self, config: object, base_revision: int, future: Future | None = None) -> None:
        self.config = config
        self.base_revision = int(base_revision)
        self.future: Future = future or Future()
        self.lock = threading.Lock()
        self.state = self.PENDING
        self.reason = ""
        self.revision = -1

    def expire_if_pending(self, reason: str) -> None:
        """Called by the page when its wait timed out; later commits are refused."""
        with self.lock:
            if self.state == self.PENDING:
                self.state = self.EXPIRED
                self.reason = reason

    def settle(self, state: str, reason: str, revision: int) -> None:
        """Called by the control thread while holding the lock."""
        self.state = state
        self.reason = reason
        self.revision = int(revision)
        if not self.future.done():
            self.future.set_result((state == self.ACCEPTED, reason))


class SettingsTransactionManager:
    """Per-controller ownership of the whole settings lifecycle across pages.

    The controller commit is version-checked, but the file persistence and a
    possible rollback happen on the page side afterwards. Without a global
    owner, page A can commit revision 1, page B can commit revision 2 and
    persist, and then A's late persistence can overwrite B's file with revision
    1. This manager serialises the whole commit -> persist -> recover span: a
    second page gets an explicit rejection in the meantime.

    ``start_block_reason()`` gates START while a settings change is in flight or
    while the active configuration/file are not converged. The state is
    recoverable by applying again (never a permanent lock).
    """

    IDLE = "idle"
    IN_FLIGHT = "in_flight"
    UNCONVERGED = "unconverged"

    def __init__(self) -> None:
        self._state_lock = threading.Lock()
        self._ownership = threading.Lock()
        self._state = self.IDLE
        self._reason = ""
        self._previous: tuple[str, str] = (self.IDLE, "")
        self._held = False
        self._owner_thread: int | None = None

    def begin(self) -> tuple[bool, str]:
        """Try to become the single settings-transaction owner (non-blocking)."""
        if not self._ownership.acquire(blocking=False):
            return (
                False,
                "another page is applying settings; wait for it to finish and retry",
            )
        with self._state_lock:
            self._held = True
            self._owner_thread = threading.get_ident()
            self._previous = (self._state, self._reason)
            self._state = self.IN_FLIGHT
            self._reason = ""
        return True, ""

    def abort(self) -> None:
        """Release ownership when no commit happened; keep the previous convergence state."""
        with self._state_lock:
            if not self._held:
                return
            self._state, self._reason = self._previous
            self._held = False
            self._owner_thread = None
        self._ownership.release()

    def finish(self, converged: bool, reason: str = "") -> None:
        """Release ownership after persistence/recovery; records converged state."""
        with self._state_lock:
            if not self._held:
                return
            self._state = self.IDLE if converged else self.UNCONVERGED
            self._reason = (
                "" if converged else (reason or "settings persistence did not converge")
            )
            self._held = False
            self._owner_thread = None
        self._ownership.release()

    def was_converged_before_begin(self) -> bool:
        """Whether restoring the owner's starting state restores consistency.

        A confirmed rollback can only restore the consistency that existed
        before begin(); retrying an unconverged state must not unlock START
        merely because that same state was successfully restored.
        """
        with self._state_lock:
            return self._held and self._previous[0] == self.IDLE

    def start_block_reason(self) -> str | None:
        with self._state_lock:
            if self._state == self.IN_FLIGHT:
                return "Start is blocked while a settings transaction is in progress"
            if self._state == self.UNCONVERGED:
                return (
                    "Start is blocked: the settings change is not persisted "
                    f"({self._reason}); apply again to resolve it"
                )
            return None

    def start_blocked(self) -> bool:
        return self.start_block_reason() is not None

    def snapshot(self) -> dict:
        with self._state_lock:
            return {"state": self._state, "reason": self._reason}


@dataclass(frozen=True)
class ControlCommand:
    kind: str
    side: str = ""
    channel: str = ""
    enabled: bool = False
    config: object = None
    # Transaction for APPLY_SETTINGS: expiry-aware commit request (see
    # SettingsTransaction). The GUI only persists/rotates after accepted.
    transaction: object = None
    # Legacy acknowledgement without expiry semantics. Kept only for internal
    # callers/tests that predate the transaction; the GUI never sets this.
    future: object = None


class SnapshotStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._snapshot = ControlSnapshot()

    def publish(self, snapshot: ControlSnapshot) -> None:
        with self._lock:
            self._snapshot = snapshot

    def read(self) -> ControlSnapshot:
        with self._lock:
            return self._snapshot
