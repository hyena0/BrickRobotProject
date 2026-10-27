"""PC-side host driver for the two EV3 bricks.

Reads ``config/hardware.local.yaml`` (git-ignored): broker address and
brick names (``left`` = master, ``right`` = slave). Bricks know ports
only; this driver also stays joint-agnostic.

API: ``move_motors({(brick, port): angle}, speed)``, ``read_state()``,
``stop()``, ``home_motor(brick, port, sensor_port, direction, speed)``,
``on_event(callback)``. Heartbeats run in a background thread; ``estop()``
publishes ``robot/estop`` from any script, and ``install_keyboard_estop()``
binds a terminal key to it.
"""

from __future__ import annotations

import itertools
import json
import queue
import threading
import time
import uuid

HEARTBEAT_PERIOD_S = 0.5
DEFAULT_TIMEOUT_S = 10.0


class DriverError(RuntimeError):
    pass


class Driver:
    def __init__(
        self,
        broker: str = "localhost",
        broker_port: int = 1883,
        client_factory=None,
        bricks: tuple[str, str] = ("left", "right"),
    ) -> None:
        self.bricks = bricks
        self._pending: dict[str, queue.Queue] = {}
        self._pending_lock = threading.Lock()
        self._events: list = []
        self._event_cb = None
        self._states: dict[str, dict] = {}
        self._ids = (f"pc-{i}" for i in itertools.count())
        try:
            if client_factory is None:
                import paho.mqtt.client as mqtt  # type: ignore
                client = mqtt.Client()
                client.connect(broker, broker_port, 60)
                client.loop_start()
            else:
                client = client_factory()
        except ImportError as exc:
            raise DriverError(
                "paho-mqtt is required on the PC: pip install paho-mqtt"
            ) from exc
        self._client = client
        self._client.on_message = self._on_message
        for brick in bricks:
            self._client.subscribe(f"robot/ack/{brick}", qos=1)
            self._client.subscribe(f"robot/state/{brick}", qos=0)
            self._client.subscribe(f"robot/event/{brick}", qos=1)
        self._stop_hb = threading.Event()
        self._hb = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._hb.start()

    # -- config ---------------------------------------------------------

    @classmethod
    def from_config(cls, path: str, **kwargs) -> "Driver":
        import yaml  # type: ignore

        with open(path, encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)
        return cls(broker=cfg.get("broker", "localhost"),
                   broker_port=int(cfg.get("broker_port", 1883)), **kwargs)

    # -- public API ------------------------------------------------------

    def on_event(self, callback) -> None:
        """Register ``callback(event_dict)`` for brick events."""
        self._event_cb = callback

    def request(self, brick: str, cmd: str, args: dict | None = None,
                timeout_s: float = DEFAULT_TIMEOUT_S) -> dict:
        mid = f"{next(self._ids)}-{uuid.uuid4().hex[:8]}"
        box: queue.Queue = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[mid] = box
        try:
            self._client.publish(
                f"robot/cmd/{brick}",
                json.dumps({"id": mid, "cmd": cmd, "args": args or {}}),
                qos=1)
            try:
                ack = box.get(timeout=timeout_s)
            except queue.Empty:
                raise DriverError(f"timeout waiting for {brick}/{cmd}")
        finally:
            with self._pending_lock:
                self._pending.pop(mid, None)
        if not ack.get("ok"):
            raise DriverError(f"{brick}/{cmd} failed: {ack.get('error')}")
        return ack.get("result", {})

    def move_motors(self, targets: dict[tuple[str, str], float],
                    speed: float = 60.0,
                    timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        """Move motors to absolute angles. Keys are ``(brick, port)``."""
        by_brick: dict[str, list] = {}
        for (brick, port), angle in targets.items():
            by_brick.setdefault(brick, []).append(
                {"port": port, "target": float(angle),
                 "speed": min(float(speed), 100.0)})
        for brick, moves in by_brick.items():
            if len(moves) == 1:
                mv = moves[0]
                self.request(brick, "run_target", mv, timeout_s)
            else:
                self.request(brick, "run_targets", {"moves": moves},
                             timeout_s)

    def read_state(self) -> dict[str, dict]:
        self.request("left", "read")
        if "right" in self.bricks:
            self.request("right", "read")
        return dict(self._states)

    def stop(self) -> None:
        for brick in self.bricks:
            try:
                self.request(brick, "stop", timeout_s=3.0)
            except DriverError:
                pass

    def estop(self) -> None:
        self._client.publish("robot/estop", json.dumps({"stop": True}), qos=1)

    def home_motor(self, brick: str, port: str, sensor_port: str,
                   direction: float = -30.0, speed: float = 30.0,
                   timeout_s: float = DEFAULT_TIMEOUT_S) -> dict:
        """Home one motor until its sensor triggers (homing exception).

        ``direction`` gives the search target angle; the brick stops that
        motor when the sensor triggers instead of latching a guard stop.
        """
        return self.request(brick, "run_target",
                            {"port": port, "target": float(direction),
                             "speed": min(float(speed), 100.0),
                             "until_sensor": sensor_port}, timeout_s)

    def set_guards(self, brick: str, rules: list[dict]) -> dict:
        return self.request(brick, "set_guards", {"rules": rules})

    def clear_latch(self, brick: str | None = None) -> None:
        for b in [brick] if brick else list(self.bricks):
            self.request(b, "clear_latch")

    def close(self) -> None:
        self._stop_hb.set()
        self._hb.join(timeout=2.0)
        try:
            self._client.loop_stop()
        except Exception:
            pass

    # -- internals ---------------------------------------------------------

    def _heartbeat_loop(self) -> None:
        while not self._stop_hb.is_set():
            try:
                self._client.publish("robot/heartbeat",
                                    json.dumps({"t": time.time()}), qos=0)
            except Exception:
                pass
            self._stop_hb.wait(HEARTBEAT_PERIOD_S)

    def _on_message(self, _client, _userdata, message) -> None:
        topic = message.topic
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        if topic.startswith("robot/ack/"):
            mid = payload.get("id")
            if mid:
                with self._pending_lock:
                    box = self._pending.get(mid)
                if box is not None:
                    box.put(payload)
        elif topic.startswith("robot/state/"):
            brick = topic.rsplit("/", 1)[-1]
            self._states[brick] = payload
        elif topic.startswith("robot/event/"):
            self._events.append(payload)
            if self._event_cb is not None:
                try:
                    self._event_cb(payload)
                except Exception:
                    pass


def install_keyboard_estop(driver: Driver, key: str = "e") -> threading.Thread:
    """Publish ``robot/estop`` when ``key`` + Enter is typed on stdin.

    Call from any script so a terminal key always stops both bricks.
    Returns the background thread.
    """

    def watch() -> None:
        print(f"Press '{key}' + Enter for emergency stop.")
        while True:
            try:
                line = input()
            except EOFError:
                return
            if line.strip() == key:
                driver.estop()
                print("ESTOP published.")

    thread = threading.Thread(target=watch, daemon=True)
    thread.start()
    return thread
