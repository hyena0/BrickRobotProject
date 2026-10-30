"""PC Transport adapter for Stage 5B (hub = PC for the hackathon).

Implements the controller package's ``Transport`` interface over MQTT:

``send(brick, cmd)``, ``on_state(cb)``, ``on_event(cb)``, ``estop()``,
plus a heartbeat thread. Convenience wrapper for tools and checks:
``move_motors``, ``read_state``, ``stop``, ``home_motor``,
``set_guards``. Keyboard e-stop from any PC script via
:func:`install_keyboard_estop`.

Holds no robot knowledge: bricks and ports only. The core logic (Stage
5C) uses it only through the Transport interface. Connects to the local
broker anonymously (``allow_anonymous true`` on the hub LAN IP only);
there are no accounts or passwords.
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
# Mirror of the brick-side cap (guards.MAX_SPEED_DEG_S). PC clamps to
# the same value so callers see the effective speed before sending.
MAX_BRICK_SPEED_DEG_S = 300.0


class TransportError(RuntimeError):
    pass


class MqttTransport:
    """MQTT Transport to the two EV3 bricks (ports only, no robot data)."""

    def __init__(
        self,
        broker: str = "localhost",
        broker_port: int = 1883,
        client_factory=None,
        bricks: tuple[str, ...] = ("left", "right"),
        heartbeat_period_s: float = HEARTBEAT_PERIOD_S,
        auto_heartbeat: bool = True,
    ) -> None:
        self.broker = broker
        self.broker_port = broker_port
        self.bricks = tuple(bricks)
        self.heartbeat_period_s = heartbeat_period_s
        self._pending: dict[str, queue.Queue] = {}
        self._pending_lock = threading.Lock()
        self._events: list = []
        self._event_cb = None
        self._state_cb = None
        self._states: dict[str, dict] = {}
        self._ids = (f"hub-{i}" for i in itertools.count())
        try:
            if client_factory is None:
                import paho.mqtt.client as mqtt  # type: ignore

                client = mqtt.Client()
                client.connect(broker, broker_port, 60)
                client.loop_start()
            else:
                client = client_factory()
        except ImportError as exc:
            raise TransportError(
                "paho-mqtt is required on the PC: pip install paho-mqtt"
            ) from exc
        self._client = client
        self._client.on_message = self._on_message
        for brick in self.bricks:
            self._client.subscribe(f"robot/ack/{brick}", qos=1)
            self._client.subscribe(f"robot/state/{brick}", qos=0)
            self._client.subscribe(f"robot/event/{brick}", qos=1)
        self._stop_hb = threading.Event()
        if auto_heartbeat:
            self._hb = threading.Thread(target=self._heartbeat_loop,
                                        daemon=True)
            self._hb.start()
        else:
            self._hb = None

    # -- config ---------------------------------------------------------

    @classmethod
    def from_config(cls, path: str, **kwargs) -> "MqttTransport":
        """Build from ``config/hardware.local.yaml`` (git-ignored).

        Reads ``broker``, ``broker_port`` and ``bricks``. Explicit kwargs
        override file values.
        """
        import yaml  # type: ignore

        with open(path, encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh) or {}
        bricks = tuple(kwargs.pop("bricks", None)
                       or cfg.get("bricks", ["left", "right"]))
        return cls(
            broker=kwargs.pop("broker", cfg.get("broker", "localhost")),
            broker_port=int(kwargs.pop("broker_port",
                                       cfg.get("broker_port", 1883))),
            bricks=bricks,
            **kwargs)

    # -- Transport interface ---------------------------------------------

    def on_state(self, callback) -> None:
        """Register ``callback(state_dict)`` for brick state messages."""
        self._state_cb = callback

    def on_event(self, callback) -> None:
        """Register ``callback(event_dict)`` for brick events."""
        self._event_cb = callback

    def send(self, brick: str, cmd: str | dict,
             args: dict | None = None,
             timeout_s: float = DEFAULT_TIMEOUT_S) -> dict:
        """Send one command envelope and wait for its ack.

        ``cmd`` is a command name (with ``args``) or a full envelope
        dict ``{"cmd", "args", "id"?}``. Returns the ack ``result`` dict;
        raises :class:`TransportError` on timeout or ``ok: false``.
        """
        if isinstance(cmd, dict):
            name = cmd["cmd"]
            cmd_args = dict(cmd.get("args", {}) or {})
            mid = str(cmd.get("id") or self._next_id())
        else:
            name = cmd
            cmd_args = dict(args or {})
            mid = self._next_id()
        box: queue.Queue = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[mid] = box
        try:
            self._client.publish(
                f"robot/cmd/{brick}",
                json.dumps({"id": mid, "cmd": name, "args": cmd_args}),
                qos=1)
            try:
                ack = box.get(timeout=timeout_s)
            except queue.Empty:
                raise TransportError(f"timeout waiting for {brick}/{name}")
        finally:
            with self._pending_lock:
                self._pending.pop(mid, None)
        if not ack.get("ok"):
            raise TransportError(f"{brick}/{name} failed: {ack.get('error')}")
        return ack.get("result", {})

    # Backwards-compatible alias for the Stage 0 driver API.
    def request(self, brick: str, cmd: str, args: dict | None = None,
                timeout_s: float = DEFAULT_TIMEOUT_S) -> dict:
        return self.send(brick, cmd, args, timeout_s)

    def estop(self) -> None:
        self._client.publish("robot/estop", json.dumps({"stop": True}),
                             qos=1)

    def heartbeat(self) -> None:
        """Publish one heartbeat (the background thread calls this)."""
        self._client.publish("robot/heartbeat",
                             json.dumps({"t": time.time()}), qos=0)

    # -- convenience wrapper (tools and checks) ---------------------------

    def move_motors(self, targets: dict[tuple[str, str], float],
                    speed: float = 60.0,
                    timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        """Move motors to absolute angles. Keys are ``(brick, port)``."""
        by_brick: dict[str, list] = {}
        for (brick, port), angle in targets.items():
            by_brick.setdefault(brick, []).append(
                {"port": port, "target": float(angle),
                 "speed": min(float(speed), MAX_BRICK_SPEED_DEG_S)})
        for brick, moves in by_brick.items():
            if len(moves) == 1:
                mv = moves[0]
                self.send(brick, "run_target", mv, timeout_s)
            else:
                self.send(brick, "run_targets", {"moves": moves},
                          timeout_s)

    def read_state(self) -> dict[str, dict]:
        for brick in self.bricks:
            self.send(brick, "read")
        return dict(self._states)

    def stop(self) -> None:
        for brick in self.bricks:
            try:
                self.send(brick, "stop", timeout_s=3.0)
            except TransportError:
                pass

    def home_motor(self, brick: str, port: str, sensor_port: str,
                   direction: float = -30.0, speed: float = 30.0,
                   timeout_s: float = DEFAULT_TIMEOUT_S) -> dict:
        """Home one motor until its sensor triggers (homing exception).

        ``direction`` gives the search target angle; the brick stops that
        motor when the sensor triggers instead of latching a guard stop.
        """
        return self.send(brick, "run_target",
                         {"port": port, "target": float(direction),
                          "speed": min(float(speed),
                                       MAX_BRICK_SPEED_DEG_S),
                          "until_sensor": sensor_port}, timeout_s)

    def set_guards(self, brick: str, rules: list[dict],
                   limits: list[dict] | dict | None = None) -> dict:
        args: dict = {"rules": rules}
        if limits is not None:
            args["limits"] = limits
        return self.send(brick, "set_guards", args)

    def clear_latch(self, brick: str | None = None) -> None:
        for b in [brick] if brick else list(self.bricks):
            self.send(b, "clear_latch")

    def close(self) -> None:
        self._stop_hb.set()
        if self._hb is not None:
            self._hb.join(timeout=2.0)
        try:
            self._client.loop_stop()
        except Exception:
            pass

    # -- internals --------------------------------------------------------

    def _next_id(self) -> str:
        return f"{next(self._ids)}-{uuid.uuid4().hex[:8]}"

    def _heartbeat_loop(self) -> None:
        while not self._stop_hb.is_set():
            try:
                self.heartbeat()
            except Exception:
                pass
            self._stop_hb.wait(self.heartbeat_period_s)

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
            if self._state_cb is not None:
                try:
                    self._state_cb(payload)
                except Exception:
                    pass
        elif topic.startswith("robot/event/"):
            self._events.append(payload)
            if self._event_cb is not None:
                try:
                    self._event_cb(payload)
                except Exception:
                    pass


def install_keyboard_estop(transport: MqttTransport,
                           key: str = "e") -> threading.Thread:
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
                transport.estop()
                print("ESTOP published.")

    thread = threading.Thread(target=watch, daemon=True)
    thread.start()
    return thread
