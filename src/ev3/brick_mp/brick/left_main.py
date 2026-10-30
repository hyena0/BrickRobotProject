#!/usr/bin/env pybricks-micropython
# brick-sw-version: 5B.14
"""Left brick program for Pybricks MicroPython (master).

Connects to the hub broker over WiFi with umqtt, serves the left brick
commands itself and relays right brick traffic over Bluetooth mailboxes
(the left brick is the mailbox server). One loop serves MQTT, mailboxes,
guards and watchdogs in turn; a move in progress keeps pumping that
same service set so stop and heartbeat traffic stays live. The brick
knows ports only.
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
HB_MS = 500
SLAVE_CMD_TIMEOUT_MS = 65000
SLAVE_LINK_MS = 5000
CONFIG_PATH = "config/brick.local.json"
HUB_ADDRESS = "192.168.35.151"
HUB_PORT = 1883
DEFAULT_CONFIG = {
    "brick": "left",
    "motors": [{"port": "outA", "type": "large"},
               {"port": "outB", "type": "large"},
               {"port": "outC", "type": "medium"},
               {"port": "outD", "type": "medium"}],
    "sensors": [{"port": "in1", "type": "touch"},
                {"port": "in2", "type": "touch"},
                {"port": "in3", "type": "touch"}],
}
SUB_TOPICS = ("robot/cmd/left", "robot/cmd/right", "robot/estop",
              "robot/heartbeat")
STATUS_TOPIC = "robot/status/left"


class MasterRuntime:
    """Master state machine. Hardware enters only through callables."""

    def __init__(self, core, publish, mbox):
        self.core = core
        self.publish = publish
        self.mbox = mbox
        self.check_inbox = None
        self.inbox = []
        self.cmd_queue = []
        self.outbox = []
        self.in_flight = None
        self._busy = False
        self.hb_estop = False
        self.slave_link_lost = False
        self._last_reply_id = ""
        self._last_evt_seq = -1
        self._last_state_t = -1
        now = clock.now_ms()
        self._last_slave_contact = now
        self._last_hb_send = 0
        self._last_state_pub = 0
        self._id_counter = 0

    def _new_id(self, prefix):
        self._id_counter = self._id_counter + 1
        return prefix + "-" + str(self._id_counter)

    def on_mqtt(self, topic, payload):
        if isinstance(payload, bytes):
            text = payload.decode()
        else:
            text = str(payload)
        if topic == "robot/estop":
            self._estop()
            return
        if topic == "robot/heartbeat":
            self.core.guards.note_heartbeat()
            self._send_hb()
            return
        if topic == "robot/cmd/left" or topic == "robot/cmd/right":
            try:
                msg = json.loads(text)
            except Exception:
                return
            if self._busy:
                self.cmd_queue.append((topic, msg))
            else:
                self._route_cmd(topic, msg)
            return

    def _route_cmd(self, topic, msg):
        if topic == "robot/cmd/right":
            self.outbox.append(msg)
            return
        self._busy = True
        try:
            ack = self.core.handle(msg, self.pump)
        finally:
            self._busy = False
        was_latched = self.core.guards.latched
        self.publish("robot/ack/left", ack.to_dict("left"), 1)
        self._drain_events()
        if was_latched:
            self._forward_stop_to_slave()

    def drain(self):
        while len(self.cmd_queue) > 0:
            item = self.cmd_queue.pop(0)
            self._route_cmd(item[0], item[1])

    def step(self):
        self.pump()
        self.drain()

    def pump(self):
        """One non-blocking service pass (also used during moves).

        Socket reads only queue into inbox here; on_mqtt runs outside
        any socket call, since a qos 1 publish waits for its ack while
        still running the mqtt callback.
        """
        if self.check_inbox is not None:
            self.check_inbox()
        while len(self.inbox) > 0:
            item = self.inbox.pop(0)
            self.on_mqtt(item[0], item[1])
        self.service_slave()
        self.periodic()

    def periodic(self):
        now = clock.now_ms()
        if clock.diff_ms(now, self._last_hb_send) >= HB_MS:
            self._send_hb()
        if clock.diff_ms(now, self._last_state_pub) >= STATE_MS:
            self._last_state_pub = now
            self.publish("robot/state/left", self.core.snapshot(), 0)
        if self.core.guards.heartbeat_expired(now):
            self.core._stop_all()
            self._forward_stop_to_slave()
            self.publish("robot/event/left", {
                "brick": "left",
                "event": "heartbeat_timeout",
                "detail": {},
            }, 1)
            self.core.guards.note_heartbeat()
        if not self.slave_link_lost and clock.diff_ms(
                now, self._last_slave_contact) > SLAVE_LINK_MS:
            self.slave_link_lost = True
            self.publish("robot/event/left", {
                "brick": "left",
                "event": "bluetooth_loss",
                "detail": {"link": "slave"},
            }, 1)
        if not self._busy:
            fresh = self.core.poll_idle_guards()
            if len(fresh) > 0:
                self._drain_events()
                self._forward_stop_to_slave()

    def service_slave(self):
        try:
            self._service_outbox()
        except Exception:
            pass
        try:
            self._service_reply()
        except Exception:
            pass
        try:
            self._service_evt()
        except Exception:
            pass
        try:
            self._service_state()
        except Exception:
            pass

    def _service_outbox(self):
        now = clock.now_ms()
        if self.in_flight is not None:
            t0 = self.in_flight["t0"]
            if clock.diff_ms(now, t0) > SLAVE_CMD_TIMEOUT_MS:
                mid = self.in_flight["id"]
                self.in_flight = None
                self.publish("robot/ack/right", {
                    "id": mid,
                    "ok": False,
                    "error": "slave timeout",
                    "brick": "right",
                }, 1)
            return
        if len(self.outbox) == 0:
            return
        if self.slave_link_lost:
            msg = self.outbox.pop(0)
            self.publish("robot/ack/right", {
                "id": str(msg.get("id", "?")),
                "ok": False,
                "error": "slave not connected",
                "brick": "right",
            }, 1)
            return
        msg = self.outbox[0]
        self.mbox.cmd.send(json.dumps(msg))
        self.outbox.pop(0)
        self.in_flight = {"id": str(msg.get("id", "")),
                          "t0": clock.now_ms()}

    def _service_reply(self):
        raw = self._mbox_read(self.mbox.reply)
        if not raw:
            return
        obj = json.loads(raw)
        ack = obj.get("ack", {})
        mid = str(ack.get("id", ""))
        if not mid or mid == self._last_reply_id:
            return
        if self.in_flight is None or mid != self.in_flight["id"]:
            return
        self._last_reply_id = mid
        self.in_flight = None
        self._note_slave()
        self.publish("robot/ack/right", ack, 1)
        for e in obj.get("events", []):
            self._on_slave_evt(e)

    def _service_evt(self):
        raw = self._mbox_read(self.mbox.evt)
        if not raw:
            return
        obj = json.loads(raw)
        seq = obj.get("seq", -1)
        if seq == self._last_evt_seq:
            return
        self._last_evt_seq = seq
        self._note_slave()
        self._on_slave_evt({"event": obj.get("event"),
                            "detail": obj.get("detail", {})})

    def _service_state(self):
        raw = self._mbox_read(self.mbox.state)
        if not raw:
            return
        obj = json.loads(raw)
        t = obj.get("t", 0)
        if t == self._last_state_t:
            return
        self._last_state_t = t
        self._note_slave()
        self.publish("robot/state/right", {
            "brick": "right",
            "encoders": obj.get("encoders", {}),
            "sensors": obj.get("sensors", {}),
            "latched": obj.get("latched", False),
        }, 0)

    def _on_slave_evt(self, e):
        body = {"brick": "right",
                "event": e.get("event"),
                "detail": e.get("detail", {})}
        self.publish("robot/event/right", body, 1)
        if body["event"] == "sensor_guard" or body["event"] == "stall":
            self.core._stop_all()
            if not self.core.guards.latched:
                self.core.guards.latched = True
                self.core.guards.latch_reason = "cross:" + str(body["event"])
            self.publish("robot/event/left", {
                "brick": "left",
                "event": "estop",
                "detail": {"reason": "cross-brick stop"},
            }, 1)

    def _note_slave(self):
        self._last_slave_contact = clock.now_ms()
        self.slave_link_lost = False

    def _mbox_read(self, box):
        try:
            return box.read()
        except Exception:
            return None

    def _estop(self):
        self.core._stop_all()
        self._forward_stop_to_slave()
        self.publish("robot/event/left", {
            "brick": "left",
            "event": "estop",
            "detail": {},
        }, 1)

    def _forward_stop_to_slave(self):
        self.hb_estop = True
        self._send_hb()

    def _send_hb(self):
        now = clock.now_ms()
        try:
            self.mbox.hb.send(json.dumps({"t": now,
                                          "estop": self.hb_estop}))
        except Exception:
            pass
        self._last_hb_send = now
        self.hb_estop = False

    def _drain_events(self):
        for evt in self.core.pending_events:
            self.publish("robot/event/left", {
                "brick": "left",
                "event": evt.event,
                "detail": evt.detail,
            }, 1)
        self.core.pending_events = []


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
    log.log("left: motors " + " ".join(sorted(mot))
            + "; sensors " + " ".join(sorted(sen)))
    if missing:
        log.log("left: NOT CONNECTED: " + " ".join(sorted(missing))
                + " (check cables)")
    bk = core.BrickCore("left", mot, sen, mtypes, stypes)
    bk.missing = missing
    return bk


def main():
    log.log("brick sw version: " + version.BRICK_SW_VERSION)
    try:
        cfg = config.load_brick_config([config.LEFT_PATH, CONFIG_PATH],
                                       "left")
    except OSError:
        log.log("no brick_left.json found, using built-in wiring")
        cfg = DEFAULT_CONFIG
    bk = build_core(cfg)
    print("left: waiting for slave mailbox connection")
    display.show(["LEFT " + version.BRICK_SW_VERSION, "wait slave..."])
    mbox = link.open_master_link()
    print("left: mailbox link up")
    from umqtt.simple import MQTTClient
    client = MQTTClient("brick_left".encode(), HUB_ADDRESS,
                        HUB_PORT, None, None, 60)
    state = {"up": False, "last_try": 0}

    def publish(topic, obj, qos):
        try:
            payload = json.dumps(obj)
            client.publish(topic.encode(), payload.encode(), False, qos)
        except Exception:
            state["up"] = False

    rt = MasterRuntime(bk, publish, mbox)

    def route(topic_b, msg_b):
        if isinstance(topic_b, bytes):
            topic = topic_b.decode()
        else:
            topic = str(topic_b)
        rt.inbox.append((topic, msg_b))

    def check_inbox():
        if state["up"]:
            try:
                for _ in range(8):
                    if client.check_msg() is None:
                        break
            except Exception:
                state["up"] = False
                state["last_try"] = clock.now_ms()
        else:
            now = clock.now_ms()
            if clock.diff_ms(now, state["last_try"]) > 2000:
                state["last_try"] = now
                try:
                    client.connect()
                    for t in SUB_TOPICS:
                        client.subscribe(t.encode(), 1)
                    client.publish(STATUS_TOPIC.encode(),
                                   '"online"'.encode(), True, 1)
                    state["up"] = True
                except Exception:
                    pass

    rt.check_inbox = check_inbox
    client.set_callback(route)
    client.set_last_will(STATUS_TOPIC.encode(), '"offline"'.encode(),
                         True, 1)
    print("left: connecting to hub at " + HUB_ADDRESS)
    display.show(["LEFT " + version.BRICK_SW_VERSION, "wait hub..."])
    while not state["up"]:
        check_inbox()
        if not state["up"]:
            clock.sleep_s(2)
    print("left: hub link up, entering main loop")
    bk.guards.note_heartbeat()
    last_disp = 0
    while True:
        core.spin_once(rt)
        now = clock.now_ms()
        if clock.diff_ms(now, last_disp) >= 1000:
            last_disp = now
            display.show(display.render_master(rt, state["up"],
                                               HUB_ADDRESS))
        clock.sleep_ms(TICK_MS)


if __name__ == "__main__":
    main()
