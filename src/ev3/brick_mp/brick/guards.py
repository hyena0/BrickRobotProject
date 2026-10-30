# brick-sw-version: 5B.14
"""On-brick safety layer for the Pybricks brick programs.

Same rules as the legacy brick code: sensor guards polled about every
10 ms, homing exception, stall guard, per-port motor angle limits,
heartbeat watchdog, link watchdog and latch. Plain classes only so this
file runs on MicroPython. Timing uses clock milliseconds.
"""

import clock

MAX_SPEED_DEG_S = 300.0
SENSOR_POLL_MS = 10
STALL_TIMEOUT_MS = 300
STALL_EPS_DEG_S = 3.0
STALL_GRACE_MS = 300
FW_STALL_MS = 200
ARRIVE_BAND_MOTOR_DEG = 5.0
HEARTBEAT_TIMEOUT_MS = 2000


def stall_detail(target, position, speed, elapsed_ms):
    """Structured stall context for events and latch reasons."""
    return {"target": target, "position": position,
            "speed": speed, "elapsed_ms": elapsed_ms}


def stall_reason(port, info):
    """Latch reason with port, target, actual, speed, elapsed time."""
    if not info:
        return "stall:" + str(port)
    return ("stall:" + str(port)
            + " target=" + str(info.get("target"))
            + " position=" + str(info.get("position"))
            + " speed=" + str(info.get("speed"))
            + " elapsed_ms=" + str(info.get("elapsed_ms")))


class SensorRule:
    def __init__(self, sensor, invert=False):
        self.sensor = sensor
        self.invert = invert


class GuardEvent:
    def __init__(self, event, detail=None):
        self.event = event
        if detail is None:
            self.detail = {}
        else:
            self.detail = detail

    def to_dict(self):
        return {"event": self.event, "detail": self.detail}


class SafetyController:
    """Pure-logic safety state. Hardware access stays with the caller."""

    def __init__(self):
        self.rules = []
        self.limits = {}
        self.latched = False
        self.latch_reason = ""
        self._last_heartbeat = clock.now_ms()

    def configure(self, rules):
        self.rules = list(rules)

    def set_limits(self, limits):
        out = {}
        for port in limits:
            pair = limits[port]
            out[port] = (float(pair[0]), float(pair[1]))
        self.limits = out

    def clear_limits(self):
        self.limits = {}

    def check_limit(self, port, target):
        """True when target is inside the port limits (or none set)."""
        if port not in self.limits:
            return True
        pair = self.limits[port]
        t = float(target)
        return pair[0] <= t <= pair[1]

    def clear_latch(self):
        self.latched = False
        self.latch_reason = ""

    def note_heartbeat(self):
        self._last_heartbeat = clock.now_ms()

    def heartbeat_expired(self, now=None):
        if now is None:
            now = clock.now_ms()
        return clock.diff_ms(now, self._last_heartbeat) > HEARTBEAT_TIMEOUT_MS

    def check_sensors(self, snapshot, homing=None):
        """Evaluate one sensor snapshot.

        homing is {"sensor": ..., "motor": ...} during a homing move. A
        trigger on the homing sensor is reported as a homed event (not a
        guard stop); every other trigger latches.
        """
        events = []
        for rule in self.rules:
            raw = int(snapshot.get(rule.sensor, 0))
            if rule.invert:
                active = (raw == 0)
            else:
                active = (raw == 1)
            if not active:
                continue
            if homing is not None and rule.sensor == homing.get("sensor"):
                events.append(GuardEvent("homed", {
                    "sensor": rule.sensor,
                    "motor": homing.get("motor", ""),
                }))
            else:
                self.latched = True
                self.latch_reason = "sensor_guard:" + str(rule.sensor)
                events.append(GuardEvent("sensor_guard", {
                    "sensor": rule.sensor,
                }))
        return events

    def check_stall(self, commanded, actual_speed, stalled_ms, dt_ms,
                    detail=None):
        """Track motors commanded but barely moving. Pure bookkeeping.

        commanded maps port to commanded speed. actual_speed maps port to
        measured speed. stalled_ms is mutated in place. Returns one stall
        event per newly stalled motor and latches the controller.
        detail optionally maps port to a stall_detail dict, which is
        merged into the event and the latch reason.
        """
        events = []
        for port in commanded:
            cmd = commanded[port]
            if abs(cmd) < 0.000000001:
                if port in stalled_ms:
                    del stalled_ms[port]
                continue
            speed = abs(actual_speed.get(port, 0.0))
            if speed < STALL_EPS_DEG_S:
                stalled_ms[port] = stalled_ms.get(port, 0) + dt_ms
                if stalled_ms[port] > STALL_TIMEOUT_MS:
                    info = None
                    if detail is not None:
                        info = detail.get(port)
                    self.latched = True
                    self.latch_reason = stall_reason(port, info)
                    evt_detail = {"port": port}
                    if info is not None:
                        for key in ("target", "position", "speed",
                                    "elapsed_ms"):
                            if key in info:
                                evt_detail[key] = info[key]
                    events.append(GuardEvent("stall", evt_detail))
                    del stalled_ms[port]
            else:
                if port in stalled_ms:
                    del stalled_ms[port]
        return events


def clamp_speed(speed):
    if speed > MAX_SPEED_DEG_S:
        return MAX_SPEED_DEG_S
    if speed < -MAX_SPEED_DEG_S:
        return -MAX_SPEED_DEG_S
    return speed
