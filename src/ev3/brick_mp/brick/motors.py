# brick-sw-version: 5B.17
"""Motor hardware abstraction for the Pybricks bricks.

Ports only (outA and friends). Real hardware uses pybricks motors;
FakeMotor is used by PC side tests. motor_type is kept for wiring
compat (pybricks drives large and medium motors with one class).
"""

import clock
import guards

OUT_LETTERS = {"outA": "A", "outB": "B", "outC": "C", "outD": "D"}
SETTLE_SPEED_DEG_S = 3.0
SETTLE_ERR_DEG = 2.0


class Motor:
    def __init__(self, port):
        self.port = port

    def position(self):
        raise NotImplementedError("Motor.position")

    def start_move(self, target, speed):
        raise NotImplementedError("Motor.start_move")

    def run_angle(self, angle, speed, pump):
        raise NotImplementedError("Motor.run_angle")

    def is_moving(self):
        raise NotImplementedError("Motor.is_moving")

    def firmware_stalled(self):
        raise NotImplementedError("Motor.firmware_stalled")

    def wait_done(self, timeout_s):
        raise NotImplementedError("Motor.wait_done")

    def stop(self):
        raise NotImplementedError("Motor.stop")

    def reset_offset(self):
        raise NotImplementedError("Motor.reset_offset")


class FakeMotor(Motor):
    """In-memory motor for PC side tests. Moves complete instantly."""

    def __init__(self, port, initial=0.0):
        Motor.__init__(self, port)
        self._pos = float(initial)
        self._moving = False
        self._target = float(initial)
        self.stopped = False
        self.hold_mode = "hold"

    def set_hold(self, mode):
        self.hold_mode = str(mode)

    def position(self):
        return self._pos

    def start_move(self, target, speed):
        self._moving = True
        self._target = float(target)
        self.stopped = False

    def is_moving(self):
        return self._moving

    def firmware_stalled(self):
        return False

    def wait_done(self, timeout_s):
        if self._moving:
            self._pos = self._target
            self._moving = False
        return True

    def stop(self):
        self._moving = False
        self.stopped = True

    def run_angle(self, angle, speed, pump):
        before = self._pos
        self._pos = before + float(angle)
        self._target = self._pos
        self._moving = False
        self.stopped = False
        return (before, self._pos)

    def reset_offset(self):
        self._pos = 0.0
        self._target = 0.0
        self._moving = False


class PybricksMotor(Motor):
    def __init__(self, port, dev, hold):
        Motor.__init__(self, port)
        self._dev = dev
        self._hold = hold
        self._target = 0.0
        self._active = False

    def position(self):
        return float(self._dev.angle())

    def start_move(self, target, speed):
        self._target = float(target)
        self._active = True
        self._dev.run_target(float(speed), float(target), self._hold, False)

    def is_moving(self):
        if not self._active:
            return False
        err = abs(float(self._dev.angle()) - self._target)
        spd = abs(float(self._dev.speed()))
        if spd < SETTLE_SPEED_DEG_S and err < SETTLE_ERR_DEG:
            self._active = False
            return False
        return True

    def firmware_stalled(self):
        try:
            query = self._dev.stalled
        except AttributeError:
            try:
                query = self._dev.control.stalled
            except AttributeError:
                return False
        return bool(query())

    def wait_done(self, timeout_s):
        if timeout_s <= 0:
            return not self.is_moving()
        t0 = clock.now_ms()
        while self.is_moving():
            if clock.diff_ms(clock.now_ms(), t0) > timeout_s * 1000.0:
                return False
            clock.sleep_ms(10)
        return True

    def stop(self):
        self._active = False
        self._dev.brake()

    def set_hold(self, mode):
        """Relax the arrival hold: brake (passive) or coast.

        Unknown modes keep the firmware hold (safe default).
        """
        if mode == "brake":
            self._dev.brake()
        elif mode == "coast":
            self._dev.stop()

    def run_angle(self, angle, speed, pump):
        before = float(self._dev.angle())
        self._target = before + float(angle)
        self._active = True
        self._dev.run_angle(float(speed), float(angle), self._hold, False)
        t0 = clock.now_ms()
        while not self._dev.done():
            if pump is not None:
                pump()
            if clock.diff_ms(clock.now_ms(), t0) > 30000:
                self._dev.brake()
                break
            clock.sleep_ms(10)
        after = float(self._dev.angle())
        self._target = after
        self._active = False
        return (before, after)

    def reset_offset(self):
        self._dev.reset_angle(0)
        self._target = 0.0
        self._active = False


def open_motor(port, motor_type="medium"):
    """Open a real pybricks motor. Imported lazily for PC side tests."""
    from pybricks.ev3devices import Motor as PybricksDevMotor
    from pybricks.parameters import Port, Stop
    if port not in OUT_LETTERS:
        raise ValueError("unknown motor port: " + str(port))
    # reset_angle=False: keep the accumulated encoder value so the
    # program knows where it left off (Pybricks ev3devices docs).
    # Default True would reset every motor to its absolute marker
    # (-180..179) at each program start and force a full recalibration.
    try:
        dev = PybricksDevMotor(getattr(Port, OUT_LETTERS[port]),
                               reset_angle=False)
    except TypeError:
        # Old firmware without the reset_angle kwarg: encoders reset
        # to the absolute marker at boot, so recalibrate afterwards.
        dev = PybricksDevMotor(getattr(Port, OUT_LETTERS[port]))
    # Presence probe: raises OSError on an empty port so build_core
    # can report and skip it instead of crashing later mid-move.
    dev.angle()
    # Align the firmware stall detector with ours (missing on old
    # firmware: our own speed-based detector still covers that).
    try:
        dev.control.stall_tolerances(int(guards.STALL_EPS_DEG_S),
                                     int(guards.STALL_TIMEOUT_MS))
    except Exception:
        pass
    return PybricksMotor(port, dev, Stop.HOLD)
