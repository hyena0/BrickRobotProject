"""Master brick program (left brick).

ev3dev + ``ev3dev2`` (Stage 5B decision): MQTT client to the hub plus
Bluetooth relay to the slave. Executes its own commands via
:class:`BrickCore` and relays ``robot/cmd/right`` to the slave over
Bluetooth, forwarding acks, states and events back to MQTT.

The hub address comes from the brick-local config
(``config/brick.local.yaml``, never committed); nothing is hard-coded.
The broker takes anonymous clients. The brick knows ports only.

Hardware imports are lazy so the module imports on a PC for inspection
(``main()`` raises a clear error there).
"""

from __future__ import annotations

import json
import threading
import time
import uuid

STATE_HZ = 15.0


def build_core_from_config(wiring: dict):
    """Create a :class:`BrickCore` from a wiring dict (ports only)."""
    from .config import build_core_from_wiring

    return build_core_from_wiring("left", wiring)


def _drain_events(core, publish) -> None:
    for evt in core.pending_events:
        publish("robot/event/left", json.dumps(
            {"brick": "left", "event": evt.event,
             "detail": evt.detail}))
    core.pending_events.clear()


def _forward_stop_to_slave(core, slave_send) -> None:
    """Forward a local guard stop to the slave (cross-brick stop)."""
    slave_send({"id": f"cross-{uuid.uuid4().hex[:8]}",
                "cmd": "__estop__", "args": {}})


def on_mqtt_command(core, publish, slave_send, topic: str, payload: bytes):
    """Route one incoming MQTT message. Testable without hardware."""
    msg = json.loads(payload.decode("utf-8"))
    if topic == "robot/estop":
        core._stop_all()
        slave_send({"id": f"estop-{uuid.uuid4().hex[:8]}",
                    "cmd": "__estop__", "args": {}})
        publish("robot/event/left", json.dumps(
            {"brick": "left", "event": "estop", "detail": {}}))
        return None
    if topic == "robot/heartbeat":
        core.guards.note_heartbeat()
        slave_send({"id": f"hb-{uuid.uuid4().hex[:8]}",
                    "cmd": "__heartbeat__", "args": {}})
        return None
    if topic == "robot/cmd/left":
        ack = core.handle(msg)
        publish("robot/ack/left", json.dumps(ack.to_dict("left")))
        latched_before_forward = core.guards.latched
        _drain_events(core, publish)
        if latched_before_forward:
            _forward_stop_to_slave(core, slave_send)
        return None
    if topic == "robot/cmd/right":
        slave_send(msg)  # relayed; ack comes back async over Bluetooth
        return None
    return None


def on_slave_frame(core, publish, frame: dict):
    """Route one Bluetooth frame from the slave. Testable without HW.

    Acks/states/events are published to MQTT. A slave guard event
    (``sensor_guard``/``stall``) also stops this brick's motors and
    latches: the cross-brick stop path (ports only; the hub decides
    which sensor/motor pair it belongs to).
    """
    kind = frame.get("kind")
    body = frame.get("body", {})
    if kind == "ack":
        publish("robot/ack/right", json.dumps(body))
    elif kind == "state":
        publish("robot/state/right", json.dumps(body))
    elif kind == "event":
        publish("robot/event/right", json.dumps(body))
        if body.get("event") in ("sensor_guard", "stall"):
            core._stop_all()
            if not core.guards.latched:
                core.guards.latched = True
                core.guards.latch_reason = (
                    f"cross:{body.get('event')}")
            publish("robot/event/left", json.dumps(
                {"brick": "left", "event": "estop",
                 "detail": {"reason": "cross-brick stop"}}))
    return None


def main() -> None:  # pragma: no cover - needs brick hardware
    from .config import load_brick_config

    cfg = load_brick_config()
    core = build_core_from_config(
        {"motors": cfg["motors"], "sensors": cfg["sensors"]})
    core.guards.note_heartbeat()

    try:
        import paho.mqtt.client as mqtt  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "paho-mqtt not installed on brick (ev3dev: pip install "
            "paho-mqtt)") from exc

    import bluetooth  # type: ignore  # pybluez on ev3dev

    slave_sock = None

    def slave_send(msg: dict) -> None:
        from .link import encode

        if slave_sock is None:
            if msg.get("cmd") not in ("__estop__", "__heartbeat__"):
                publish("robot/ack/right", json.dumps(
                    {"id": msg.get("id", "?"), "ok": False,
                     "brick": "right", "error": "slave not connected"}))
            return
        slave_sock.sendall(encode({"kind": "cmd", "body": msg}))

    client = mqtt.Client()
    client.will_set("robot/status/left", json.dumps("offline"),
                    qos=1, retain=True)

    def publish(topic: str, body: str, qos: int = 1) -> None:
        client.publish(topic, body, qos=qos)

    def on_message(_c, _u, message) -> None:
        on_mqtt_command(core, publish, slave_send, message.topic,
                        message.payload)

    def on_connect(c, _u, _f, rc, _p=None) -> None:
        c.subscribe("robot/cmd/left", qos=1)
        c.subscribe("robot/cmd/right", qos=1)
        c.subscribe("robot/estop", qos=1)
        c.subscribe("robot/heartbeat", qos=0)
        c.publish("robot/status/left", json.dumps("online"),
                  qos=1, retain=True)

    client.on_message = on_message
    client.on_connect = on_connect
    client.connect(cfg["hub_address"], int(cfg["hub_port"]), 60)
    client.loop_start()

    # Bluetooth relay server for the slave (runs in background).
    def bt_server() -> None:
        nonlocal slave_sock
        from .link import Framer

        srv = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        srv.bind(("", 1))
        srv.listen(1)
        while True:
            sock, _info = srv.accept()
            slave_sock = sock
            framer = Framer()
            try:
                while True:
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    for frame in framer.feed(chunk):
                        on_slave_frame(core, publish, frame)
            finally:
                try:
                    sock.close()
                finally:
                    slave_sock = None
                    publish("robot/event/left", json.dumps(
                        {"brick": "left", "event": "bluetooth_loss",
                         "detail": {"link": "slave"}}))

    threading.Thread(target=bt_server, daemon=True).start()

    # Main loop: idle guard polling, heartbeat watchdog, state publishing.
    period = 1.0 / STATE_HZ
    while True:
        time.sleep(period)
        fresh = core.poll_idle_guards()
        if fresh:
            _drain_events(core, publish)
            _forward_stop_to_slave(core, slave_send)
        if core.guards.heartbeat_expired():
            core._stop_all()
            publish("robot/event/left", json.dumps(
                {"brick": "left", "event": "heartbeat_timeout",
                 "detail": {}}))
            core.guards.note_heartbeat()  # avoid repeat spam; latch stops moves
        publish("robot/state/left",
                json.dumps(core.snapshot()), qos=0)


if __name__ == "__main__":  # pragma: no cover
    main()
