"""On-brick safety layer.

Runs locally on each brick with no network in the loop. Rules:

- Sensor guards are polled every ~10 ms. Any configured sensor triggering
  outside a homing move stops all motors on that brick, latches, and emits
  an event. The master forwards the stop to the other brick.
- Homing exception: a move carrying ``{sensor, motor}`` stops that motor
  when the sensor triggers. This ends homing and is not an error.
- Stall guard: a motor commanded but near zero speed for > 300 ms stops
  and reports (for joints without a sensor).
- Heartbeat watchdog: no heartbeat for 2 s stops everything.
- Bluetooth watchdog: slave losing the master link stops everything.
- Latch: after a guard stop, moves are refused until ``clear_latch``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time

# Stage 0 caps until joint limits are known.
MAX_SPEED_DEG_S = 100.0
SENSOR_POLL_S = 0.01
STALL_TIMEOUT_S = 0.30
STALL_EPS_DEG_S = 3.0
HEARTBEAT_TIMEOUT_S = 2.0


@dataclass
class SensorRule:
    sensor: str
    invert: bool = False


@dataclass
class GuardEvent:
    event: str
    detail: dict = field(default_factory=dict)


class SafetyController:
    """Pure-logic safety state. Hardware access stays with the caller."""

    def __init__(self) -> None:
        self.rules: list[SensorRule] = []
        self.latched = False
        self.latch_reason: str = ""
        self._last_heartbeat = time.monotonic()

    def configure(self, rules: list[SensorRule]) -> None:
        self.rules = list(rules)

    def clear_latch(self) -> None:
        self.latched = False
        self.latch_reason = ""

    def note_heartbeat(self) -> None:
        self._last_heartbeat = time.monotonic()

    def heartbeat_expired(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return (now - self._last_heartbeat) > HEARTBEAT_TIMEOUT_S

    def check_sensors(
        self,
        snapshot: dict[str, int],
        homing: dict[str, str] | None = None,
    ) -> list[GuardEvent]:
        """Evaluate one sensor snapshot.

        ``homing`` is ``{"sensor": ..., "motor": ...}`` during a homing
        move. A trigger on the homing sensor is reported as a
        ``homed`` event (not a guard stop); every other trigger latches.
        """
        events: list[GuardEvent] = []
        for rule in self.rules:
            raw = int(snapshot.get(rule.sensor, 0))
            active = (raw == 0) if rule.invert else (raw == 1)
            if not active:
                continue
            if homing and rule.sensor == homing.get("sensor"):
                events.append(
                    GuardEvent("homed", {"sensor": rule.sensor,
                                        "motor": homing.get("motor", "")})
                )
            else:
                self.latched = True
                self.latch_reason = f"sensor_guard:{rule.sensor}"
                events.append(
                    GuardEvent("sensor_guard", {"sensor": rule.sensor})
                )
        return events

    def check_stall(
        self, commanded: dict[str, float], actual_speed: dict[str, float],
        stalled_for_s: dict[str, float], dt_s: float,
    ) -> list[GuardEvent]:
        """Track motors commanded but barely moving. Pure bookkeeping.

        ``commanded`` maps port to commanded speed (deg/s, signed).
        ``actual_speed`` maps port to measured speed. ``stalled_for_s``
        is mutated in place. Returns one ``stall`` event per newly
        stalled motor and latches the controller.
        """
        events: list[GuardEvent] = []
        for port, cmd in commanded.items():
            if abs(cmd) < 1e-9:
                stalled_for_s.pop(port, None)
                continue
            speed = abs(actual_speed.get(port, 0.0))
            if speed < STALL_EPS_DEG_S:
                stalled_for_s[port] = stalled_for_s.get(port, 0.0) + dt_s
                if stalled_for_s[port] > STALL_TIMEOUT_S:
                    self.latched = True
                    self.latch_reason = f"stall:{port}"
                    events.append(GuardEvent("stall", {"port": port}))
                    stalled_for_s.pop(port, None)
            else:
                stalled_for_s.pop(port, None)
        return events

    @staticmethod
    def clamp_speed(speed: float) -> float:
        if speed > MAX_SPEED_DEG_S:
            return MAX_SPEED_DEG_S
        if speed < -MAX_SPEED_DEG_S:
            return -MAX_SPEED_DEG_S
        return speed
