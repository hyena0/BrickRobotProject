# brick-sw-version: 5B.14
"""Shared command executor for the Pybricks bricks.

Pure logic over the motor and sensor objects: validates the id, cmd and
args envelope, drives motors, polls the safety layer, and returns ack
payloads plus outbound events. MQTT and mailbox code live in the main
files; this module never touches the network.

Moves run as a supervised wait that calls pump each poll so the single
main loop keeps serving stop and heartbeat traffic while a move runs.
"""

import clock
import guards

# Same-goal skip band. Matches motors.SETTLE_ERR_DEG: a new command
# whose target is already reached never re-triggers run_target.
SAME_GOAL_ERR_DEG = 2.0

COMMANDS = (
    "ping", "info", "run_target", "run_targets", "run_angle", "read",
    "reset", "stop", "set_guards", "clear_latch",
)
MOVE_TIMEOUT_MS = 60000


class Ack:
    def __init__(self, mid, ok, result=None, error=""):
        self.mid = mid
        self.ok = ok
        if result is None:
            self.result = {}
        else:
            self.result = result
        self.error = error

    def to_dict(self, brick):
        payload = {"id": self.mid, "ok": self.ok}
        if self.ok:
            payload["result"] = self.result
        else:
            payload["error"] = self.error
        payload["brick"] = brick
        return payload


class BrickCore:
    """Executes protocol commands for one brick (ports only)."""

    def __init__(self, brick, motors, sensors, motor_types=None,
                 sensor_types=None):
        self.brick = brick
        self.motors = motors
        self.sensors = sensors
        if motor_types is None:
            motor_types = {}
            for p in motors:
                motor_types[p] = "medium"
        if sensor_types is None:
            sensor_types = {}
            for p in sensors:
                sensor_types[p] = "touch"
        self.motor_types = motor_types
        self.sensor_types = sensor_types
        self.guards = guards.SafetyController()
        self.pending_events = []
        self.missing = []

    def snapshot(self):
        encoders = {}
        for p in self.motors:
            encoders[p] = self.motors[p].position()
        values = {}
        for p in self.sensors:
            values[p] = self.sensors[p].read()
        return {
            "brick": self.brick,
            "encoders": encoders,
            "sensors": values,
            "latched": self.guards.latched,
            "latch_reason": self.guards.latch_reason,
            "missing": list(self.missing),
        }

    def info(self):
        motors = []
        for p in sorted(self.motors):
            motors.append({"port": p,
                           "type": self.motor_types.get(p, "medium")})
        sensors = []
        for p in sorted(self.sensors):
            sensors.append({"port": p,
                            "type": self.sensor_types.get(p, "touch")})
        return {
            "brick": self.brick,
            "motors": motors,
            "sensors": sensors,
        }

    def handle(self, msg, pump=None):
        mid = str(msg.get("id", ""))
        cmd = msg.get("cmd", "")
        args = msg.get("args")
        if args is None:
            args = {}
        if not mid or cmd not in COMMANDS:
            if not mid:
                mid = "?"
            return Ack(mid, False, error="unknown command: " + str(cmd))
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
                return self._set_guards(args, mid)
            if cmd == "clear_latch":
                self.guards.clear_latch()
                return Ack(mid, True, {})
            if cmd == "run_target":
                return self._run_target(args, mid, pump)
            if cmd == "run_targets":
                return self._run_targets(args, mid, pump)
            if cmd == "run_angle":
                return self._run_angle(args, mid, pump)
        except KeyError as exc:
            return Ack(mid, False, error="missing arg: " + str(exc))
        except OSError as exc:
            self._stop_all()
            if not self.guards.latched:
                self.guards.latched = True
                self.guards.latch_reason = "motor_io:" + str(cmd)
            self.pending_events.append(
                guards.GuardEvent("motor_error",
                                  {"cmd": str(cmd),
                                   "error": str(exc)}))
            return Ack(mid, False,
                       error="motor io error: " + str(exc))
        raise AssertionError("unreachable")

    def _set_guards(self, args, mid):
        rules = []
        for r in args.get("rules", []):
            rules.append(guards.SensorRule(r["sensor"],
                                           bool(r.get("invert", False))))
        self.guards.configure(rules)
        if "limits" in args:
            raw = args.get("limits")
            if raw is None:
                raw = []
            parsed = {}
            if isinstance(raw, dict):
                for p in raw:
                    v = raw[p]
                    parsed[p] = (float(v[0]), float(v[1]))
            else:
                for e in raw:
                    parsed[e["port"]] = (float(e["min"]), float(e["max"]))
            if parsed:
                self.guards.set_limits(parsed)
            else:
                self.guards.clear_limits()
        return Ack(mid, True, {"rules": len(rules),
                               "limits": len(self.guards.limits)})

    def _refuse_if_latched(self, mid):
        if self.guards.latched:
            return Ack(mid, False,
                       error="latched: " + str(self.guards.latch_reason)
                       + "; send clear_latch first")
        return None

    def _limit_reject(self, mid, port, target):
        self.pending_events.append(
            guards.GuardEvent("limit_reject",
                              {"port": port, "target": target}))
        pair = self.guards.limits[port]
        return Ack(mid, False,
                   error="limit_reject: " + str(port) + " target "
                   + str(target) + " outside [" + str(pair[0]) + ", "
                   + str(pair[1]) + "]")

    def _run_target(self, args, mid, pump):
        refused = self._refuse_if_latched(mid)
        if refused is not None:
            return refused
        port = args["port"]
        target = float(args["target"])
        speed = guards.clamp_speed(float(args.get("speed", 60.0)))
        until_sensor = args.get("until_sensor")
        if port not in self.motors:
            return Ack(mid, False, error="unknown port: " + str(port))
        if not self.guards.check_limit(port, target):
            return self._limit_reject(mid, port, target)
        homing = None
        if until_sensor:
            if until_sensor not in self.sensors:
                return Ack(mid, False,
                           error="unknown sensor: " + str(until_sensor))
            homing = {"sensor": until_sensor, "motor": port}
        motor = self.motors[port]
        if homing is None and not motor.is_moving():
            settled = abs(motor.position() - target)
            if settled < SAME_GOAL_ERR_DEG:
                return Ack(mid, True, {"port": port,
                                       "position": motor.position(),
                                       "residual": motor.position() - target,
                                       "skipped": True})
        motor.start_move(target, speed)
        homed = self._supervised_wait([(port, target)], homing, pump)
        if homed:
            return Ack(mid, True, {"homed": True, "port": port,
                                   "sensor": until_sensor})
        if self.guards.latched:
            return Ack(mid, False,
                       error="guard stop: " + str(self.guards.latch_reason))
        return Ack(mid, True, {"port": port, "position": motor.position(),
                               "residual": motor.position() - target})

    def _run_targets(self, args, mid, pump):
        refused = self._refuse_if_latched(mid)
        if refused is not None:
            return refused
        moves = args["moves"]
        if not moves:
            return Ack(mid, False, error="moves must be non-empty")
        for mv in moves:
            if mv["port"] not in self.motors:
                return Ack(mid, False,
                           error="unknown port: " + str(mv["port"]))
            if not self.guards.check_limit(mv["port"],
                                           float(mv["target"])):
                return self._limit_reject(mid, mv["port"],
                                          float(mv["target"]))
        started = []
        for mv in moves:
            port = mv["port"]
            target = float(mv["target"])
            speed = guards.clamp_speed(float(mv.get("speed", 60.0)))
            motor = self.motors[port]
            if not motor.is_moving():
                settled = abs(motor.position() - target)
                if settled < SAME_GOAL_ERR_DEG:
                    continue
            motor.start_move(target, speed)
            started.append((port, target))
        if not started:
            positions = {}
            for mv in moves:
                positions[mv["port"]] = self.motors[mv["port"]].position()
            return Ack(mid, True, {"positions": positions,
                                   "skipped": True})
        self._supervised_wait(started, None, pump)
        if self.guards.latched:
            return Ack(mid, False,
                       error="guard stop: " + str(self.guards.latch_reason))
        positions = {}
        residuals = {}
        for p, tgt in started:
            positions[p] = self.motors[p].position()
            residuals[p] = self.motors[p].position() - tgt
        return Ack(mid, True, {"positions": positions,
                               "residuals": residuals})

    def _run_angle(self, args, mid, pump):
        refused = self._refuse_if_latched(mid)
        if refused is not None:
            return refused
        port = args["port"]
        angle = float(args["angle"])
        speed = guards.clamp_speed(float(args.get("speed", 90.0)))
        if port not in self.motors:
            return Ack(mid, False, error="unknown port: " + str(port))
        motor = self.motors[port]
        projected = motor.position() + angle
        if not self.guards.check_limit(port, projected):
            return self._limit_reject(mid, port, projected)
        before, after = motor.run_angle(angle, speed, pump)
        return Ack(mid, True, {"port": port, "before": before,
                               "after": after,
                               "moved": after - before})

    def _supervised_wait(self, active, homing, pump):
        """Poll guards while moves run. True when homing finished."""
        t0 = clock.now_ms()
        prev = {}
        for p, tgt in active:
            prev[p] = self.motors[p].position()
        stalled = {}
        fw_stalled_ms = {}
        last = clock.now_ms()
        while True:
            clock.sleep_ms(guards.SENSOR_POLL_MS)
            if pump is not None:
                pump()
            now = clock.now_ms()
            dt_ms = clock.diff_ms(now, last)
            if dt_ms <= 0:
                dt_ms = 1
            last = now
            snapshot = {}
            for p in self.sensors:
                snapshot[p] = self.sensors[p].read()
            for evt in self.guards.check_sensors(snapshot, homing):
                if evt.event == "homed" and homing is not None:
                    self.motors[homing["motor"]].stop()
                    for port, tgt in active:
                        if port != homing["motor"]:
                            self.motors[port].stop()
                    return True
                self.pending_events.append(evt)
            if self.guards.latched:
                self._stop_all()
                return False
            cur = {}
            for p, tgt in active:
                cur[p] = self.motors[p].position()
            speeds = {}
            dt_s = dt_ms / 1000.0
            for p in cur:
                speeds[p] = (cur[p] - prev[p]) / dt_s
            prev = cur
            commanded = {}
            judged = {}
            for p, tgt in active:
                # Within the arrival band the motor counts as
                # reached: it is never judged, so stopping there is
                # arrival, not stall.
                if abs(cur[p] - tgt) > guards.ARRIVE_BAND_MOTOR_DEG:
                    commanded[p] = 1.0
                    judged[p] = tgt
            elapsed = clock.diff_ms(now, t0)
            # Stall verdicts only while a move runs and only after the
            # start grace. Holding at the target never reaches this
            # loop (same-goal moves are skipped, finished moves exit),
            # so idle HOLD is never judged.
            if elapsed >= guards.STALL_GRACE_MS:
                info = {}
                for p, tgt in judged.items():
                    info[p] = guards.stall_detail(tgt, cur[p],
                                                  speeds[p], elapsed)
                for evt in self.guards.check_stall(commanded, speeds,
                                                  stalled, dt_ms, info):
                    self.pending_events.append(evt)
            if not self.guards.latched:
                for p, tgt in judged.items():
                    if elapsed >= guards.STALL_GRACE_MS \
                            and self._fw_stalled(p):
                        fw = fw_stalled_ms.get(p, 0) + dt_ms
                        fw_stalled_ms[p] = fw
                        if fw >= guards.FW_STALL_MS:
                            if not self.guards.latched:
                                self.guards.latched = True
                                self.guards.latch_reason = (
                                    guards.stall_reason(
                                        p, guards.stall_detail(
                                            tgt, cur[p], speeds[p],
                                            elapsed)))
                            self.pending_events.append(
                                guards.GuardEvent(
                                    "stall",
                                    {"port": p, "source": "firmware",
                                     "target": tgt, "position": cur[p],
                                     "speed": speeds[p],
                                     "elapsed_ms": elapsed}))
                    else:
                        if p in fw_stalled_ms:
                            del fw_stalled_ms[p]
            if self.guards.latched:
                self._stop_all()
                return False
            for m in self.motors.values():
                m.wait_done(0)
            moving = False
            for p, tgt in active:
                if self.motors[p].is_moving():
                    moving = True
            if not moving:
                return False
            if clock.diff_ms(now, t0) > MOVE_TIMEOUT_MS:
                self._stop_all()
                self.guards.latched = True
                self.guards.latch_reason = "move_timeout"
                self.pending_events.append(
                    guards.GuardEvent("stall", {"port": "timeout"}))
                return False

    def poll_idle_guards(self):
        """Poll sensor guards outside any move (call from main loops)."""
        if self.guards.latched:
            return []
        snapshot = {}
        for p in self.sensors:
            snapshot[p] = self.sensors[p].read()
        fresh = self.guards.check_sensors(snapshot, None)
        if fresh:
            self._stop_all()
            for evt in fresh:
                self.pending_events.append(evt)
        return fresh

    def _fw_stalled(self, port):
        """Firmware stall signal. Missing API reads False; a failed
        query reads True so a faulted driver cuts drive at once."""
        motor = self.motors[port]
        try:
            query = motor.firmware_stalled
        except AttributeError:
            return False
        try:
            return bool(query())
        except Exception:
            return True

    def _stop_all(self):
        """Stop every motor. Never raises: per-motor errors latch."""
        for port in sorted(self.motors):
            try:
                self.motors[port].stop()
            except Exception as exc:
                if not self.guards.latched:
                    self.guards.latched = True
                    self.guards.latch_reason = "motor_error:" + str(port)
                self.pending_events.append(
                    guards.GuardEvent("motor_error",
                                      {"port": port,
                                       "error": str(exc)}))


def spin_once(rt):
    """Run one ``rt.step()`` pass. Never raises.

    Unexpected motor or link errors stop the brick and latch it
    instead of killing the program: a dead brick would leave motors
    driving with no supervisor and no e-stop.
    """
    try:
        rt.step()
    except Exception as exc:
        bk = rt.core
        bk._stop_all()
        if not bk.guards.latched:
            bk.guards.latched = True
            bk.guards.latch_reason = "loop_error"
        bk.pending_events.append(
            guards.GuardEvent("loop_error", {"error": str(exc)}))
