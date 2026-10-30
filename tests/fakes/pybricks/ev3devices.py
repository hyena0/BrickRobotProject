"""Fake ``pybricks.ev3devices`` with scriptable motion.

``Motor`` advances toward its target on every ``angle()`` poll so the
real supervised-wait loop observes motion and settles. Set ``jammed``
to emulate a stalled motor (no motion, zero speed).
"""

from __future__ import annotations

_MISSING: set = set()


def set_missing(*ports):
    """Mark ports as physically empty (OSError ENODEV on open)."""
    _MISSING.update(ports)


def reset():
    _MISSING.clear()


class _FakeControl:
    def __init__(self):
        self.stall_tol = None

    def stall_tolerances(self, speed, time):
        self.stall_tol = (speed, time)


class Motor:
    def __init__(self, port, positive_direction=None, gears=None):
        if port in _MISSING:
            raise OSError(19, "ENODEV")
        self.port = port
        self._angle = 0.0
        self._target = 0.0
        self._speed = 0.0
        self._running = False
        self.jammed = False
        self.braked = False
        self.last_then = None
        self.run_target_calls = 0
        self.fake_stalled = False
        self.control = _FakeControl()

    def _advance(self):
        if not self._running or self.jammed:
            return
        err = self._target - self._angle
        if abs(err) < 0.5:
            self._angle = self._target
            self._running = False
            return
        step = max(2.0, abs(self._speed) * 0.05)
        move = step if err > 0 else -step
        if abs(move) > abs(err):
            move = err
        self._angle += move

    def angle(self):
        self._advance()
        return self._angle

    def speed(self):
        if not self._running or self.jammed:
            return 0.0
        if abs(self._target - self._angle) < 0.5:
            return 0.0
        return self._speed

    def run_target(self, speed, target_angle, then=None, wait=True):
        self._speed = float(speed)
        self._target = float(target_angle)
        self.last_then = then
        self._running = True
        self.braked = False
        self.run_target_calls += 1
        if wait:
            for _ in range(10000):
                self._advance()
                if not self._running:
                    break

    def run_angle(self, speed, rotation_angle, then=None, wait=True):
        self._speed = float(speed)
        self._target = self._angle + float(rotation_angle)
        self.last_then = then
        self._running = True
        self.braked = False
        if wait:
            for _ in range(10000):
                self._advance()
                if not self._running:
                    break

    def done(self):
        self._advance()
        return not self._running

    def stalled(self):
        return bool(self.jammed or self.fake_stalled)

    def run(self, speed):
        self._speed = float(speed)
        self._running = True
        self.braked = False

    def stop(self):
        self._running = False

    def brake(self):
        self._running = False
        self.braked = True

    def hold(self):
        self._running = False
        self.braked = True

    def reset_angle(self, angle=0):
        self._angle = float(angle)
        self._target = float(angle)
        self._running = False


class TouchSensor:
    def __init__(self, port):
        if port in _MISSING:
            raise OSError(19, "ENODEV")
        self.port = port
        self._pressed = False

    def pressed(self):
        return bool(self._pressed)

    def set_pressed(self, value):
        self._pressed = bool(value)


class ColorSensor:
    def __init__(self, port):
        if port in _MISSING:
            raise OSError(19, "ENODEV")
        self.port = port
        self._reflection = 0

    def reflection(self):
        return self._reflection

    def set_reflection(self, value):
        self._reflection = value
