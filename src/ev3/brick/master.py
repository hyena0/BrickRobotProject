"""Master brick program (left brick).

MQTT client plus Bluetooth relay to the slave. Executes its own commands
via :class:`BrickCore` and relays ``robot/cmd/right`` to the slave over
Bluetooth, forwarding acks, states and events back to MQTT.

Runtime: ev3dev with ``ev3dev2`` + ``paho-mqtt``, or Pybricks with
``umqtt``. Hardware/bluetooth imports are lazy so the module imports on a
PC for inspection (``main()`` raises a clear error there).
"""

from __future__ import annotations

import json
import threading
import time

STATE_HZ = 15.0


def build_core_from_config(wiring: dict):
    """Create a :class:`BrickCore` from a wiring dict (ports only)."""
    from .core import BrickCore
    from .motors import open_motor
    from .sensors import open_sensor

    motors = {m["port"]: open_motor(m["port"], m.get("type", "medium"))
              for m in wiring.get("motors", [])}
    sensors = {s["port"]: open_sensor(s["port"], s.get("type", "touch"))
               for s in wiring.get("sensors", [])}
    core = BrickCore("left", motors, sensors,
                     {m["port"]: m.get("type", "medium")
                      for m in wiring.get("motors", [])},
                     {s["port"]: s.get("type", "touch")
                      for s in wiring.get("sensors", [])})
    return core


def on_mqtt_command(core, publish, slave_send, topic: str, payload: bytes):
    """Route one incoming MQTT message. Testable without hardware."""
    from .core import Ack

    msg = json.loads(payload.decode("utf-8"))
    if topic == "robot/estop":
        core._stop_all()
        ack: Ack | None = None
    elif topic == "robot/heartbeat":
        core.guards.note_heartbeat()
        return None
    elif topic == "robot/cmd/left":
        ack = core.handle(msg)
    elif topic == "robot/cmd/right":
        slave_send(msg)  # relayed; ack comes back async over Bluetooth
        return None
    else:
        return None
    if ack is not None:
        publish("robot/ack/left", json.dumps(ack.to_dict("left")))
        for evt in core.pending_events:
            publish("robot/event/left", json.dumps(
                {"brick": "left", "event": evt.event,
                 "detail": evt.detail}))
        core.pending_events.clear()
    return None


def main() -> None:  # pragma: no cover - needs brick hardware
    import yaml  # type: ignore

    with open("config/hardware.local.yaml", encoding="utf-8") as fh:
        hw = yaml.safe_load(fh)
    wiring = hw.get("left", {})
    core = build_core_from_config(wiring)
    core.guards.note_heartbeat()

    try:
        import paho.mqtt.client as mqtt  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "paho-mqtt not installed on brick; "
            "install it (ev3dev) or use the Pybricks/umqtt variant") from exc

    import bluetooth  # type: ignore  # pybluez on ev3dev

    slave_sock = None

    def slave_send(msg: dict) -> None:
        from .link import encode

        if slave_sock is None:
            publish("robot/ack/right", json.dumps(
                {"id": msg.get("id", "?"), "ok": False,
                 "brick": "right", "error": "slave not connected"}))
            return
        slave_sock.sendall(encode(msg))

    client = mqtt.Client()
    last_will = json.dumps("offline")
    client.will_set("robot/status/left", last_will, qos=1, retain=True)

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
    client.connect(hw.get("broker", "localhost"),
                   int(hw.get("broker_port", 1883)), 60)
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
                        kind = frame.get("kind")
                        body = frame.get("body", {})
                        if kind == "ack":
                            publish("robot/ack/right", json.dumps(body))
                        elif kind in ("state", "event"):
                            publish(f"robot/{kind}/right",
                                    json.dumps(body),
                                    qos=0 if kind == "state" else 1)
            finally:
                try:
                    sock.close()
                finally:
                    slave_sock = None

    threading.Thread(target=bt_server, daemon=True).start()

    # Main loop: guard polling, heartbeat watchdog, state publishing.
    period = 1.0 / STATE_HZ
    while True:
        time.sleep(period)
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
