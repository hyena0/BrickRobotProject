"""Slave brick program (right brick).

Bluetooth client of the master. Receives relayed command envelopes,
executes them via :class:`BrickCore`, and returns acks, states and events
over the same link. Local guard loops run here too: losing the master
link stops everything on the slave.
"""

from __future__ import annotations

import json
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
    return BrickCore("right", motors, sensors)


def main() -> None:  # pragma: no cover - needs brick hardware
    import yaml  # type: ignore

    with open("config/hardware.local.yaml", encoding="utf-8") as fh:
        hw = yaml.safe_load(fh)
    core = build_core_from_config(hw.get("right", {}))
    core.guards.note_heartbeat()

    import bluetooth  # type: ignore  # pybluez on ev3dev

    from .link import Framer, encode

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((hw["master_bt_addr"], 1))
    framer = Framer()
    last_state = 0.0

    def send(kind: str, body: dict) -> None:
        try:
            sock.sendall(encode({"kind": kind, "body": body}))
        except OSError:
            pass  # link loss is handled by the watchdog below

    try:
        while True:
            try:
                chunk = sock.recv(4096)
            except OSError:
                chunk = b""
            if not chunk:
                core._stop_all()  # Bluetooth watchdog
                send("event", {"brick": "right",
                               "event": "bluetooth_loss", "detail": {}})
                time.sleep(1.0)
                continue
            for frame in framer.feed(chunk):
                if frame.get("kind") == "cmd":
                    msg = frame["body"]
                    if msg.get("cmd") == "__estop__":
                        core._stop_all()
                    elif msg.get("cmd") == "__heartbeat__":
                        core.guards.note_heartbeat()
                    else:
                        ack = core.handle(msg)
                        send("ack", ack.to_dict("right"))
                    for evt in core.pending_events:
                        send("event", {"brick": "right",
                                       "event": evt.event,
                                       "detail": evt.detail})
                    core.pending_events.clear()
            if core.guards.heartbeat_expired():
                core._stop_all()
                send("event", {"brick": "right",
                               "event": "heartbeat_timeout", "detail": {}})
                core.guards.note_heartbeat()
            now = time.monotonic()
            if now - last_state > 1.0 / STATE_HZ:
                send("state", core.snapshot())
                last_state = now
    finally:
        try:
            core._stop_all()
        finally:
            sock.close()


if __name__ == "__main__":  # pragma: no cover
    main()
