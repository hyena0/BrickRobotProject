"""Motor hardware abstraction for the EV3 bricks.

The brick programs stay robot-agnostic: they only know port names
(e.g. ``outA``), never joint names or gear ratios. Real hardware access is
isolated here so the command executor and the safety layer can be tested
with :class:`FakeMotor` on a PC.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class Motor(ABC):
    """Blocking-capable motor interface used by the brick core."""

    port: str

    @abstractmethod
    def position(self) -> float:
        """Return the current encoder angle in degrees."""

    @abstractmethod
    def start_move(self, target: float, speed: float) -> None:
        """Start a non-blocking absolute move (degrees, deg/s)."""

    @abstractmethod
    def is_moving(self) -> bool:
        """Return True while a commanded move is still running."""

    @abstractmethod
    def wait_done(self, timeout_s: float) -> bool:
        """Block until the move finishes. Return False on timeout."""

    @abstractmethod
    def stop(self) -> None:
        """Stop the motor immediately."""

    @abstractmethod
    def reset_offset(self) -> None:
        """Zero the encoder offset on this motor."""


class FakeMotor(Motor):
    """In-memory motor for PC-side tests. Moves complete instantly."""

    def __init__(self, port: str, initial: float = 0.0) -> None:
        self.port = port
        self._pos = float(initial)
        self._moving = False
        self.stopped = False

    def position(self) -> float:
        return self._pos

    def start_move(self, target: float, speed: float) -> None:
        self._moving = True
        self._target = float(target)
        self.stopped = False

    def is_moving(self) -> bool:
        return self._moving

    def wait_done(self, timeout_s: float) -> bool:
        self._pos = self._target
        self._moving = False
        return True

    def stop(self) -> None:
        self._moving = False
        self.stopped = True

    def reset_offset(self) -> None:
        self._pos = 0.0


def open_motor(port: str, motor_type: str = "medium") -> Motor:
    """Open a real ev3dev motor. Imported lazily so PCs can use fakes.

    ``motor_type`` is ``"medium"`` or ``"large"`` and only selects the
    ev3dev driver class; no robot semantics are attached.
    """
    from ev3dev2.motor import LargeMotor, MediumMotor  # type: ignore

    cls = MediumMotor if motor_type == "medium" else LargeMotor
    return _Ev3DevMotor(port, cls(port))


class _Ev3DevMotor(Motor):
    def __init__(self, port: str, dev: object) -> None:
        self.port = port
        self._dev = dev

    def position(self) -> float:
        return float(self._dev.position)

    def start_move(self, target: float, speed: float) -> None:
        self._dev.on_to_position(speed, target, block=False)

    def is_moving(self) -> bool:
        state = getattr(self._dev, "state", [])
        return "running" in state

    def wait_done(self, timeout_s: float) -> bool:
        import time

        deadline = time.monotonic() + timeout_s
        while self.is_moving():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return True

    def stop(self) -> None:
        self._dev.stop(stop_action="brake")

    def reset_offset(self) -> None:
        self._dev.reset()
