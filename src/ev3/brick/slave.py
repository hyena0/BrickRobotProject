"""Slave brick program (right brick).

ev3dev + ``ev3dev2`` (Stage 5B decision): Bluetooth client of the master.
Receives relayed command envelopes, executes them via :class:`BrickCore`,
and returns acks, states and events over the same link. Local guard loops
run here too: losing the master link stops everything on the slave.

The brick knows ports only. Connection details (master Bluetooth address)
come from the brick-local config, never hard-coded.
"""

from __future__ import annotations

import time

STATE_HZ = 15.0


def build_core_from_config(wiring: dict):
    """Create a :class:`BrickCore` from a wiring dict (ports only)."""
    from .config import build_core_from_wiring

    return build_core_from_wiring("right", wiring)


def handle_relayed_command(core, send, msg: dict):
    """Execute one relayed command. Testable without hardware.

    ``send`` takes ``(kind, body)`` and transmits one framed message to
    the master. Returns nothing; acks and pending events are sent.
    """
    cmd = msg.get("cmd")
    if cmd == "__estop__":
        core._stop_all()
        send("event", {"brick": "right", "event": "estop",
                       "detail": {}})
    elif cmd == "__heartbeat__":
        core.guards.note_heartbeat()
    else:
        ack = core.handle(msg)
        send("ack", ack.to_dict("right"))
    for evt in core.pending_events:
        send("event", {"brick": "right", "event": evt.event,
                       "detail": evt.detail})
    core.pending_events.clear()


def main() -> None:  # pragma: no cover - needs brick hardware
    from .config import load_brick_config

    cfg = load_brick_config()
    core = build_core_from_config(
        {"motors": cfg["motors"], "sensors": cfg["sensors"]})
    core.guards.note_heartbeat()

    if not cfg["master_bt_addr"]:
        raise SystemExit("brick config needs `master_bt_addr` (slave)")

    import bluetooth  # type: ignore  # pybluez on ev3dev

    from .link import Framer, encode

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((cfg["master_bt_addr"], 1))
    sock.settimeout(1.0 / STATE_HZ)
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
            except OSError as exc:
                import socket as _socket

                if isinstance(exc, _socket.timeout):
                    chunk = None  # idle tick: poll guards below
                else:
                    chunk = b""
            if chunk == b"":
                core._stop_all()  # Bluetooth watchdog
                send("event", {"brick": "right",
                               "event": "bluetooth_loss", "detail": {}})
                time.sleep(1.0)
                continue
            if chunk:
                for frame in framer.feed(chunk):
                    if frame.get("kind") == "cmd":
                        handle_relayed_command(core, send, frame["body"])
            fresh = core.poll_idle_guards()
            for evt in fresh:
                send("event", {"brick": "right", "event": evt.event,
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
