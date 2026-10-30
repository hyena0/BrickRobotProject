"""Shared command executor for both bricks.

Pure logic over the HAL interfaces: validates the ``{"id", "cmd",
"args"}`` envelope, drives motors, polls the safety layer, and returns
ack payloads plus outbound events. MQTT and Bluetooth live in
``master.py`` / ``slave.py``; this module never touches the network.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .guards import (
    HEARTBEAT_TIMEOUT_S,
    SENSOR_POLL_S,
    GuardEvent,
    SafetyController,
    SensorRule,
)
from .motors import Motor
from .sensors import Sensor

COMMANDS = (
    "ping", "info", "run_target", "run_targets", "read",
    "reset", "stop", "set_guards", "clear_latch",
)


@dataclass
class Ack:
    mid: str
    ok: bool
    result: dict = field(default_factory=dict)
    error: str = ""

    def to_dict(self, brick: str) -> dict:
        payload: dict = {"id": self.mid, "ok": self.ok}
        if self.ok:
            payload["result"] = self.result
        else:
            payload["error"] = self.error
        payload["brick"] = brick
        return payload


class BrickCore:
    """Executes protocol commands for one brick (ports only)."""

    def __init__(
        self,
        brick: str,
        motors: dict[str, Motor],
        sensors: dict[str, Sensor],
        motor_types: dict[str, str] | None = None,
        sensor_types: dict[str, str] | None = None,
    ) -> None:
        self.brick = brick
        self.motors = motors
        self.sensors = sensors
        self.motor_types = motor_types or {p: "medium" for p in motors}
        self.sensor_types = sensor_types or {p: "touch" for p in sensors}
        self.guards = SafetyController()
        self.pending_events: list[GuardEvent] = []

    # -- snapshots -----------------------------------------------------

    def snapshot(self) -> dict:
        return {
            "brick": self.brick,
            "encoders": {p: m.position() for p, m in self.motors.items()},
            "sensors": {p: s.read() for p, s in self.sensors.items()},
            "latched": self.guards.latched,
        }

    def info(self) -> dict:
        return {
            "brick": self.brick,
            "motors": [
                {"port": p, "type": self.motor_types.get(p, "medium")}
                for p in sorted(self.motors)
            ],
            "sensors": [
                {"port": p, "type": self.sensor_types.get(p, "touch")}
                for p in sorted(self.sensors)
            ],
        }

    # -- entry point ---------------------------------------------------

    def handle(self, msg: dict) -> Ack:
        mid = str(msg.get("id", ""))
        cmd = msg.get("cmd", "")
        args = msg.get("args", {}) or {}
        if not mid or cmd not in COMMANDS:
            return Ack(mid or "?", False, error=f"unknown command: {cmd!r}")
        try:
            if cmd == "ping":
                return Ack(mid, True, {"brick": self.brick})
            if cmd == "info":
                return Ack(mid, True, self.info())
            if cmd == "read":
                return Ack(mid, True, self.snapshot())
            if cmd == "reset":
                for m in self.motors.values():
                    m.reset_offset()
                return Ack(mid, True, {})
            if cmd == "stop":
                self._stop_all()
                return Ack(mid, True, {})
            if cmd == "set_guards":
                rules = [SensorRule(r["sensor"], bool(r.get("invert", False)))
                         for r in args.get("rules", [])]
                self.guards.configure(rules)
                if "limits" in args:
                    raw_limits = args.get("limits") or []
                    if isinstance(raw_limits, dict):
                        items = raw_limits.items()
                        parsed = {p: (float(v[0]), float(v[1]))
                                  for p, v in items}
                    else:
                        parsed = {e["port"]: (float(e["min"]), float(e["max"]))
                                  for e in raw_limits}
                    if parsed:
                        self.guards.set_limits(parsed)
                    else:
                        self.guards.clear_limits()
                return Ack(mid, True, {"rules": len(rules),
                                       "limits": len(self.guards.limits)})
            if cmd == "clear_latch":
                self.guards.clear_latch()
                return Ack(mid, True, {})
            if cmd == "run_target":
                return self._run_target(args, mid)
            if cmd == "run_targets":
                return self._run_targets(args, mid)
        except KeyError as exc:
            return Ack(mid, False, error=f"missing arg: {exc}")
        raise AssertionError("unreachable")

    # -- moves ----------------------------------------------------------

    def _refuse_if_latched(self, mid: str) -> Ack | None:
        if self.guards.latched:
            return Ack(mid, False,
                       error=f"latched: {self.guards.latch_reason}; "
                             "send clear_latch first")
        return None

    def _limit_reject(self, mid: str, port: str, target: float) -> Ack:
        self.pending_events.append(
            GuardEvent("limit_reject", {"port": port, "target": target}))
        lo, hi = self.guards.limits[port]
        return Ack(mid, False,
                   error=f"limit_reject: {port} target {target} "
                         f"outside [{lo}, {hi}]")

    def _run_target(self, args: dict, mid: str) -> Ack:
        refused = self._refuse_if_latched(mid)
        if refused:
            return refused
        port = args["port"]
        target = float(args["target"])
        speed = self.guards.clamp_speed(float(args.get("speed", 60.0)))
        until_sensor = args.get("until_sensor")
        if port not in self.motors:
            return Ack(mid, False, error=f"unknown port: {port!r}")
        if not self.guards.check_limit(port, target):
            return self._limit_reject(mid, port, target)
        homing = None
        if until_sensor:
            if until_sensor not in self.sensors:
                return Ack(mid, False, error=f"unknown sensor: {until_sensor!r}")
            homing = {"sensor": until_sensor, "motor": port}
        motor = self.motors[port]
        motor.start_move(target, speed)
        homed = self._supervised_wait([(port, target)], homing)
        if homed:
            return Ack(mid, True, {"homed": True, "port": port,
                                   "sensor": until_sensor})
        if self.guards.latched:
            return Ack(mid, False,
                       error=f"guard stop: {self.guards.latch_reason}")
        return Ack(mid, True, {"port": port, "position": motor.position()})

    def _run_targets(self, args: dict, mid: str) -> Ack:
        refused = self._refuse_if_latched(mid)
        if refused:
            return refused
        moves = args["moves"]
        if not moves:
            return Ack(mid, False, error="moves must be non-empty")
        for mv in moves:
            if mv["port"] not in self.motors:
                return Ack(mid, False, error=f"unknown port: {mv['port']!r}")
            if not self.guards.check_limit(mv["port"],
                                           float(mv["target"])):
                return self._limit_reject(mid, mv["port"],
                                          float(mv["target"]))
        started: list[tuple[str, float]] = []
        for mv in moves:
            port = mv["port"]
            target = float(mv["target"])
            speed = self.guards.clamp_speed(float(mv.get("speed", 60.0)))
            self.motors[port].start_move(target, speed)
            started.append((port, target))
        self._supervised_wait(started, None)
        if self.guards.latched:
            return Ack(mid, False,
                       error=f"guard stop: {self.guards.latch_reason}")
        return Ack(mid, True, {"positions": {
            p: self.motors[p].position() for p, _ in started}})

    def _supervised_wait(
        self, active: list[tuple[str, float]],
        homing: dict[str, str] | None,
    ) -> bool:
        """Poll guards while moves run. Returns True if homing finished."""
        deadline = time.monotonic() + HEARTBEAT_TIMEOUT_S * 30
        prev = {p: self.motors[p].position() for p, _ in active}
        stalled: dict[str, float] = {}
        last = time.monotonic()
        while True:
            time.sleep(SENSOR_POLL_S)
            now = time.monotonic()
            dt = now - last
            last = now
            snapshot = {p: s.read() for p, s in self.sensors.items()}
            for evt in self.guards.check_sensors(snapshot, homing):
                if evt.event == "homed" and homing:
                    self.motors[homing["motor"]].stop()
                    for port, _ in active:
                        if port != homing["motor"]:
                            self.motors[port].stop()
                    return True
                self.pending_events.append(evt)
            if self.guards.latched:
                self._stop_all()
                return False
            cur = {p: self.motors[p].position() for p, _ in active}
            speeds = {p: ((cur[p] - prev[p]) / dt if dt > 0 else 0.0)
                      for p in cur}
            prev = cur
            for evt in self.guards.check_stall(
                    {p: 1.0 for p, _ in active}, speeds, stalled, dt):
                self.pending_events.append(evt)
            if self.guards.latched:
                self._stop_all()
                return False
            for m in self.motors.values():
                m.wait_done(0)
            if not any(self.motors[p].is_moving() for p, _ in active):
                return False
            if now > deadline:
                self._stop_all()
                self.guards.latched = True
                self.guards.latch_reason = "move_timeout"
                self.pending_events.append(GuardEvent("stall", {"port": "timeout"}))
                return False

    def poll_idle_guards(self) -> list[GuardEvent]:
        """Poll sensor guards outside any move (call from main loops).

        Any trigger latches, stops all motors and queues the event for
        publishing. Returns the newly queued events.
        """
        if self.guards.latched:
            return []
        snapshot = {p: s.read() for p, s in self.sensors.items()}
        fresh = self.guards.check_sensors(snapshot, None)
        if fresh:
            self._stop_all()
            self.pending_events.extend(fresh)
        return fresh

    def _stop_all(self) -> None:
        for m in self.motors.values():
            m.stop()
