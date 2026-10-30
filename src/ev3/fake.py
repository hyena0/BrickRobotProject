"""In-memory fakes for PC-side Stage 5B tests and ``--fake`` checks.

Two builders live here. :func:`build_mp_transport` (used by
``check --fake``) drives the real Pybricks runtimes
(:mod:`src.ev3.brick_mp`) sharing an in-memory mailbox link. The legacy
:class:`FakeMqttClient` path reproduces the old framed Bluetooth relay
for the legacy brick tests. No network, no bricks needed either way.
"""

from __future__ import annotations

import json
import re

from .brick.core import BrickCore
from .brick.link import Framer, encode
from .brick.motors import FakeMotor
from .brick.sensors import FakeSensor

FULL_WIRING = {
    "left": {"motors": ["outA", "outB", "outC", "outD"],
             "sensors": ["in1", "in2", "in3"]},
    "right": {"motors": ["outA", "outB"],
              "sensors": ["in1", "in2"]},
}


def build_fake_cores(full: bool = True):
    """Create ``(left_core, right_core)`` with fake motors/sensors."""
    if full:
        wiring = FULL_WIRING
    else:
        wiring = {"left": {"motors": ["outA"], "sensors": ["in1"]},
                  "right": {"motors": ["outB"], "sensors": ["in1"]}}
    cores = {}
    for brick, spec in wiring.items():
        motors = {p: FakeMotor(p, 0.0) for p in spec["motors"]}
        sensors = {p: FakeSensor(p, 0) for p in spec["sensors"]}
        cores[brick] = BrickCore(brick, motors, sensors)
    return cores["left"], cores["right"]


class FakeBtLink:
    """In-memory Bluetooth socket pair speaking ``link`` framing.

    ``master_send(msg)`` transmits one relayed command dict to the slave
    side; ``slave_recv()`` returns decoded frames. ``slave_send(kind,
    body)`` transmits back; ``master_recv()`` decodes. Feeds can be split
    to exercise the incremental :class:`Framer`.
    """

    def __init__(self) -> None:
        self._to_slave = bytearray()
        self._to_master = bytearray()
        self._slave_framer = Framer()
        self._master_framer = Framer()

    def master_send(self, msg: dict) -> None:
        self._to_slave.extend(encode({"kind": "cmd", "body": msg}))

    def slave_recv(self) -> list[dict]:
        data = bytes(self._to_slave)
        self._to_slave.clear()
        return self._slave_framer.feed(data)

    def slave_send(self, kind: str, body: dict) -> None:
        self._to_master.extend(encode({"kind": kind, "body": body}))

    def master_recv(self) -> list[dict]:
        data = bytes(self._to_master)
        self._to_master.clear()
        return self._master_framer.feed(data)


class FakeMqttClient:
    """Fake MQTT client routing through the master relay + fake BT link."""

    def __init__(self, left: BrickCore, right: BrickCore) -> None:
        from .brick import master as master_mod
        from .brick import slave as slave_mod

        self._master_mod = master_mod
        self._slave_mod = slave_mod
        self.left = left
        self.right = right
        self.bt = FakeBtLink()
        self.on_message = None
        self.published: list = []

    def subscribe(self, *_a, **_k) -> None:
        pass

    def loop_start(self) -> None:
        pass

    def loop_stop(self) -> None:
        pass

    # -- helpers ------------------------------------------------------

    def _emit(self, topic: str, payload: dict, qos: int = 1) -> None:
        body = json.dumps(payload)
        self.published.append((topic, body, qos))
        if self.on_message is not None:
            self.on_message(
                self, None,
                type("M", (), {"topic": topic,
                               "payload": body.encode()})())

    def _publish(self, topic: str, body: str, qos: int = 1) -> None:
        self.published.append((topic, body, qos))
        if self.on_message is not None:
            self.on_message(
                self, None,
                type("M", (), {"topic": topic,
                               "payload": body.encode()})())

    def _slave_send(self, msg: dict) -> None:
        """Master-side relay: frame to slave, execute, frame back."""
        self.bt.master_send(msg)
        for frame in self.bt.slave_recv():
            if frame.get("kind") == "cmd":
                self._slave_mod.handle_relayed_command(
                    self.right, self.bt.slave_send, frame["body"])
        for frame in self.bt.master_recv():
            self._master_mod.on_slave_frame(
                self.left, self._publish, frame)

    # -- MQTT surface ---------------------------------------------------

    def publish(self, topic: str, payload: str, qos: int = 0):
        if topic in ("robot/estop", "robot/heartbeat") or re.match(
                r"robot/cmd/(left|right)$", topic):
            raw = payload if isinstance(payload, (bytes, bytearray)) \
                else payload.encode()
            if isinstance(raw, str):  # pragma: no cover - defensive
                raw = raw.encode()
            self._master_mod.on_mqtt_command(
                self.left, self._publish, self._slave_send, topic, raw)
            # Left state snapshot after each command (the real master
            # publishes state at 15 Hz; the fake emits on demand).
            if re.match(r"robot/cmd/left$", topic):
                self._emit("robot/state/left", self.left.snapshot(), qos=0)
            elif re.match(r"robot/cmd/(right)$", topic):
                self._emit("robot/state/right", self.right.snapshot(),
                           qos=0)
        else:
            self.published.append((topic, payload, qos))
        return None


def build_fake_transport(full: bool = True):
    """Build an :class:`MqttTransport` over the legacy fake relay path.

    Returns ``(transport, left_core, right_core)`` so tests can press
    fake sensors and inspect fake motors directly.
    """
    from src.adapters.pc.mqtt_transport import MqttTransport

    left, right = build_fake_cores(full=full)

    def factory():
        return FakeMqttClient(left, right)

    transport = MqttTransport(client_factory=factory,
                             bricks=("left", "right"),
                             auto_heartbeat=False)
    return transport, left, right


def _import_mp():
    """Import brick_mp modules the way the brick sees them (flat)."""
    import sys
    from pathlib import Path
    from types import SimpleNamespace

    mp_dir = str(Path(__file__).resolve().parent / "brick_mp")
    if mp_dir not in sys.path:
        sys.path.insert(0, mp_dir)
    import core as mp_core
    import left_main as mp_left
    import link as mp_link
    import motors as mp_motors
    import right_main as mp_right
    import sensors as mp_sensors

    return SimpleNamespace(core=mp_core, left=mp_left, link=mp_link,
                           motors=mp_motors, right=mp_right,
                           sensors=mp_sensors)


class MpFakeClient:
    """Fake MQTT client driving the real Pybricks runtimes.

    Owns a ``MasterRuntime`` + ``SlaveRuntime`` pair sharing an
    in-memory mailbox link. Hub publishes route through the real master
    code; settle rounds pump the slave and relay back.
    """

    def __init__(self, master, slave) -> None:
        self.master = master
        self.slave = slave
        self.left = master.core
        self.right = slave.core
        self.on_message = None
        self.published: list = []
        master.publish = self._hub_publish

    def subscribe(self, *_a, **_k) -> None:
        pass

    def loop_start(self) -> None:
        pass

    def loop_stop(self) -> None:
        pass

    def _hub_publish(self, topic, obj, qos):
        self._emit(topic, obj, qos)

    def _emit(self, topic: str, payload, qos: int = 1) -> None:
        if isinstance(payload, bytes):
            body = payload
        elif isinstance(payload, str):
            body = payload.encode()
        else:
            body = json.dumps(payload).encode()
        self.published.append((topic, body, qos))
        if self.on_message is not None:
            self.on_message(
                self, None,
                type("M", (), {"topic": topic, "payload": body})())

    def _settle(self, rounds: int = 4) -> None:
        for _ in range(rounds):
            self.master.service_slave()
            self.slave.tick()
        self.master.service_slave()

    def publish(self, topic: str, payload, qos: int = 0):
        if topic in ("robot/estop", "robot/heartbeat") or re.match(
                r"robot/cmd/(left|right)$", topic):
            raw = payload if isinstance(payload, (bytes, bytearray)) \
                else str(payload).encode()
            self.master.on_mqtt(topic, bytes(raw))
            self._settle()
            # On-demand state snapshots (the real bricks publish state
            # at 15 Hz; the fake emits after each command instead).
            if topic == "robot/cmd/left":
                self._emit("robot/state/left", self.left.snapshot(), 0)
            elif topic == "robot/cmd/right":
                self._emit("robot/state/right", self.right.snapshot(), 0)
        else:
            self.published.append((topic, payload, qos))
        return None


def build_mp_cores(full: bool = True):
    """Create ``(left_core, right_core)`` from brick_mp with its fakes."""
    mp = _import_mp()
    if full:
        wiring = FULL_WIRING
    else:
        wiring = {"left": {"motors": ["outA"], "sensors": ["in1"]},
                  "right": {"motors": ["outB"], "sensors": ["in1"]}}
    cores = {}
    for brick, spec in wiring.items():
        motors = {p: mp.motors.FakeMotor(p, 0.0)
                  for p in spec["motors"]}
        sensors = {p: mp.sensors.FakeSensor(p, 0)
                   for p in spec["sensors"]}
        cores[brick] = mp.core.BrickCore(brick, motors, sensors)
    return cores["left"], cores["right"]


def build_mp_transport(full: bool = True):
    """Build an :class:`MqttTransport` over the real brick_mp runtimes.

    Returns ``(transport, left_core, right_core)``, same shape as the
    legacy builder. Used by ``check --fake`` and the check-level tests.
    """
    from src.adapters.pc.mqtt_transport import MqttTransport

    mp = _import_mp()
    left, right = build_mp_cores(full=full)
    mbox = mp.link.FakeLink()
    master = mp.left.MasterRuntime(left, None, mbox)
    slave = mp.right.SlaveRuntime(right, mbox)

    def factory():
        return MpFakeClient(master, slave)

    transport = MqttTransport(client_factory=factory,
                             bricks=("left", "right"),
                             auto_heartbeat=False)
    return transport, left, right
