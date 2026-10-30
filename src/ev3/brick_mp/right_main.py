#!/usr/bin/env pybricks-micropython
# brick-sw-version: 5B.14
"""Right brick program for Pybricks MicroPython (slave).

Mailbox client of the left brick. Runs relayed commands through the
shared core and returns acks, states and events over mailboxes. One
loop serves mailboxes, guards and watchdogs in turn; a move in
progress keeps polling the heartbeat box so a stop from the left brick
lands mid-move. The brick knows ports only.
"""

import clock
import config
import core
import display
import link
import log
import motors
import sensors
import version

try:
    import ujson as json
except ImportError:
    import json

TICK_MS = 10
STATE_MS = 66
MASTER_LINK_MS = 3000
CONFIG_PATH = "config/brick.local.json"
MASTER_BT_NAME = "ev3dev"
DEFAULT_CONFIG = {
    "brick": "right",
    "motors": [{"port": "outA", "type": "medium"},
               {"port": "outB", "type": "medium"}],
    "sensors": [{"port": "in1", "type": "touch"},
                {"port": "in2", "type": "color"}],
}


class SlaveRuntime:
    """Slave state machine. Hardware enters only through the link."""

    def __init__(self, bk, mbox):
        self.core = bk
        self.mbox = mbox
        self.cmd_queue = []
        self._busy = False
        self.link_lost = False
        self._last_cmd_id = ""
        self._last_hb_t = -1
        self._last_estop_seen = False
        self._evt_seq = 0
        self._async = []
        now = clock.now_ms()
        self._last_master_contact = now
        self._last_state_send = 0
        self._last_reconnect_try = 0

    def step(self):
        self.tick()

    def tick(self):
        self._read_hb()
        self._read_cmd()
        self._drain()
        if not self._busy:
            fresh = self.core.poll_idle_guards()
            for evt in fresh:
                self._async.append({"event": evt.event,
                                    "detail": evt.detail})
            self.core.pending_events = []
        self._watchdogs()
        self._send_state()
        self._send_async()

    def _read_hb(self):
        raw = self._mbox_read(self.mbox.hb)
        if not raw:
            return
        try:
            obj = json.loads(raw)
        except Exception:
            return
        t = obj.get("t", 0)
        if t != self._last_hb_t:
            self._last_hb_t = t
            self._note_master()
        self.core.guards.note_heartbeat()
        if obj.get("estop", False):
            if not self._last_estop_seen:
                self.core._stop_all()
                self._async.append({"event": "estop", "detail": {}})
            self._last_estop_seen = True
        else:
            self._last_estop_seen = False

    def _read_cmd(self):
        raw = self._mbox_read(self.mbox.cmd)
        if not raw:
            return
        try:
            obj = json.loads(raw)
        except Exception:
            return
        mid = str(obj.get("id", ""))
        if not mid or mid == self._last_cmd_id:
            return
        self._last_cmd_id = mid
        self._note_master()
        self.cmd_queue.append(obj)

    def _drain(self):
        while len(self.cmd_queue) > 0 and not self._busy:
            msg = self.cmd_queue.pop(0)
            try:
                self._execute(msg)
            except Exception as exc:
                try:
                    ack = core.Ack(str(msg.get("id", "?")), False,
                                   error="brick error: " + str(exc))
                    self.mbox.reply.send(json.dumps({
                        "ack": ack.to_dict("right"),
                        "events": [],
                    }))
                except Exception:
                    pass
                self._busy = False

    def _execute(self, msg):
        self._busy = True
        try:
            cmd = msg.get("cmd", "")
            if cmd == "__estop__":
                self.core._stop_all()
                ack = core.Ack(str(msg.get("id", "?")), True, {})
                self._async.append({"event": "estop", "Detail": {}})
            elif cmd == "__heartbeat__":
                self.core.guards.note_heartbeat()
                ack = core.Ack(str(msg.get("id", "?")), True, {})
            else:
                ack = self.core.handle(msg, self._pump)
        finally:
            self._busy = False
        evts = []
        for e in self.core.pending_events:
            evts.append(e.to_dict())
        self.core.pending_events = []
        try:
            self.mbox.reply.send(json.dumps({
                "ack": ack.to_dict("right"),
                "events": evts,
            }))
        except Exception:
            pass

    def _pump(self):
        """Heartbeat watch kept alive while a move runs."""
        raw = self._mbox_read(self.mbox.hb)
        if not raw:
            return
        try:
            obj = json.loads(raw)
        except Exception:
            return
        t = obj.get("t", 0)
        if t != self._last_hb_t:
            self._last_hb_t = t
            self._note_master()
        self.core.guards.note_heartbeat()
        if obj.get("estop", False):
            if not self._last_estop_seen:
                self._async.append({"event": "estop", "Detail": {}})
            self._last_estop_seen = True
            self.core._stop_all()
        else:
            self._last_estop_seen = False

    def _watchdogs(self):
        now = clock.now_ms()
        if self.core.guards.heartbeat_expired(now):
            self.core._stop_all()
            self._async.append({"event": "heartbeat_timeout",
                                "detail": {}})
            self.core.guards.note_heartbeat()
        if clock.diff_ms(now, self._last_master_contact) > MASTER_LINK_MS:
            if not self.link_lost:
                self.link_lost = True
                self.core._stop_all()
                self._last_reconnect_try = 0
            if clock.diff_ms(now, self._last_reconnect_try) > 5000:
                self._last_reconnect_try = now
                try:
                    link.reconnect_slave_link(self.mbox)
                except Exception:
                    pass
        else:
            if self.link_lost:
                self.link_lost = False
                self._async.append({"event": "bluetooth_loss",
                                    "detail": {"link": "master"}})

    def _send_state(self):
        now = clock.now_ms()
        if clock.diff_ms(now, self._last_state_send) < STATE_MS:
            return
        self._last_state_send = now
        snap = self.core.snapshot()
        snap["t"] = now
        try:
            self.mbox.state.send(json.dumps(snap))
        except Exception:
            pass

    def _send_async(self):
        if len(self._async) == 0:
            return
        e = self._async[0]
        body = {"seq": self._evt_seq,
                "brick": "right",
                "event": e["event"],
                "detail": e["detail"]}
        try:
            self.mbox.evt.send(json.dumps(body))
        except Exception:
            return
        self._async.pop(0)
        self._evt_seq = self._evt_seq + 1

    def _note_master(self):
        self._last_master_contact = clock.now_ms()

    def _mbox_read(self, box):
        try:
            return box.read()
        except Exception:
            return None


def build_core(cfg):
    mot = {}
    mtypes = {}
    missing = []
    for m in cfg["motors"]:
        try:
            dev = motors.open_motor(m["port"], m.get("type", "medium"))
        except OSError:
            missing.append(m["port"])
        else:
            mot[m["port"]] = dev
            mtypes[m["port"]] = m.get("type", "medium")
    sen = {}
    stypes = {}
    for s in cfg["sensors"]:
        try:
            dev = sensors.open_sensor(s["port"], s.get("type", "touch"))
        except OSError:
            missing.append(s["port"])
        else:
            sen[s["port"]] = dev
            stypes[s["port"]] = s.get("type", "touch")
    log.log("right: motors " + " ".join(sorted(mot))
            + "; sensors " + " ".join(sorted(sen)))
    if missing:
        log.log("right: NOT CONNECTED: " + " ".join(sorted(missing))
                + " (check cables)")
    bk = core.BrickCore("right", mot, sen, mtypes, stypes)
    bk.missing = missing
    return bk


def main():
    log.log("brick sw version: " + version.BRICK_SW_VERSION)
    try:
        cfg = config.load_brick_config([config.RIGHT_PATH, CONFIG_PATH],
                                       "right")
    except OSError:
        log.log("no brick_right.json found, using built-in wiring")
        cfg = DEFAULT_CONFIG
    bk = build_core(cfg)
    print("right: waiting for left brick mailbox server")
    display.show(["RIGHT " + version.BRICK_SW_VERSION, "wait master..."])
    mbox = None
    while mbox is None:
        try:
            mbox = link.open_slave_link(MASTER_BT_NAME)
        except Exception:
            clock.sleep_s(2)
    print("right: mailbox link up, entering main loop")
    bk.guards.note_heartbeat()
    rt = SlaveRuntime(bk, mbox)
    last_disp = 0
    while True:
        core.spin_once(rt)
        now = clock.now_ms()
        if clock.diff_ms(now, last_disp) >= 1000:
            last_disp = now
            display.show(display.render_slave(rt, MASTER_BT_NAME))
        clock.sleep_ms(TICK_MS)


if __name__ == "__main__":
    main()
