"""PC-side tests for Stage 5B hardware connectivity.

Uses FakeMotor/FakeSensor and the fake Bluetooth relay
(:mod:`src.ev3.fake`): no bricks or broker needed.
"""

from __future__ import annotations

import json

import pytest

from src.adapters.pc.mqtt_transport import MqttTransport, TransportError
from src.ev3.brick import master as master_mod
from src.ev3.brick import slave as slave_mod
from src.ev3.brick.config import load_brick_config
from src.ev3.brick.core import BrickCore
from src.ev3.brick.guards import SafetyController
from src.ev3.brick.link import Framer, encode
from src.ev3.brick.motors import FakeMotor
from src.ev3.brick.sensors import FakeSensor
from src.ev3.fake import (
    FakeBtLink,
    build_fake_cores,
    build_fake_transport,
)


def make_core() -> BrickCore:
    return BrickCore("left", {"outA": FakeMotor("outA", 0.0)},
                     {"in1": FakeSensor("in1", 0)})


# -- 5B.1 brick config: hub address from file, never hard-coded -----------


def test_brick_config_loads_hub_address(tmp_path):
    cfg = tmp_path / "brick.local.yaml"
    cfg.write_text(
        "brick: left\nhub_address: 192.168.1.10\nhub_port: 1883\n"
        "master_bt_addr: '00:16:53:AA:BB:CC'\n"
        "motors: [{port: outA, type: large}]\n"
        "sensors: [{port: in1, type: touch}]\n")
    loaded = load_brick_config(str(cfg))
    assert loaded["hub_address"] == "192.168.1.10"
    assert loaded["hub_port"] == 1883
    assert "account" not in loaded
    assert "password" not in loaded
    assert loaded["motors"] == [{"port": "outA", "type": "large"}]


def test_brick_config_defaults_and_missing_file(tmp_path):
    cfg = tmp_path / "brick.local.yaml"
    cfg.write_text("brick: right\nhub_address: hub.local\n")
    loaded = load_brick_config(str(cfg))
    assert loaded["hub_port"] == 1883
    with pytest.raises(FileNotFoundError):
        load_brick_config(str(tmp_path / "nope.yaml"))
    bad = tmp_path / "bad.yaml"
    bad.write_text("brick: left\n")
    with pytest.raises(KeyError):
        load_brick_config(str(bad))


def test_hub_config_from_file(tmp_path):
    cfg = tmp_path / "hardware.local.yaml"
    cfg.write_text("broker: 192.168.1.10\nbroker_port: 1883\n"
                   "bricks: [left, right]\n")
    t = MqttTransport.from_config(
        str(cfg),
        client_factory=lambda: _NullClient())
    try:
        assert t.broker == "192.168.1.10"
        assert t.bricks == ("left", "right")
    finally:
        t.close()


class _NullClient:
    def __init__(self) -> None:
        self.on_message = None

    def subscribe(self, *_a, **_k) -> None:
        pass

    def publish(self, *_a, **_k) -> None:
        pass

    def loop_start(self) -> None:
        pass

    def loop_stop(self) -> None:
        pass


# -- 5B.4 motor-angle limits ------------------------------------------------


def test_set_guards_limits_reject_out_of_range_without_latch():
    core = make_core()
    ack = core.handle({"id": "1", "cmd": "set_guards",
                       "args": {"rules": [],
                                "limits": [{"port": "outA", "min": -90.0,
                                            "max": 90.0}]}})
    assert ack.ok and ack.result["limits"] == 1
    assert core.handle({"id": "2", "cmd": "run_target",
                        "args": {"port": "outA", "target": 45.0}}).ok
    bad = core.handle({"id": "3", "cmd": "run_target",
                       "args": {"port": "outA", "target": 120.0}})
    assert not bad.ok and "limit_reject" in bad.error
    assert not core.guards.latched  # rejections do not latch
    assert any(e.event == "limit_reject" for e in core.pending_events)
    # run_targets rejects when any move is out of range.
    bad2 = core.handle({"id": "4", "cmd": "run_targets",
                        "args": {"moves": [{"port": "outA",
                                            "target": -120.0}]}})
    assert not bad2.ok
    # Empty limits list clears; the same target now passes.
    core.handle({"id": "5", "cmd": "set_guards",
                 "args": {"rules": [], "limits": []}})
    assert core.handle({"id": "6", "cmd": "run_target",
                        "args": {"port": "outA", "target": 120.0}}).ok


def test_limits_dict_form_and_clamp():
    ctl = SafetyController()
    assert ctl.check_limit("outA", 999.0)  # no limits: always pass
    ctl.set_limits({"outA": (-10.0, 10.0)})
    assert ctl.check_limit("outA", 10.0)
    assert not ctl.check_limit("outA", 10.1)
    core = make_core()
    assert core.handle({"id": "1", "cmd": "set_guards",
                        "args": {"rules": [],
                                 "limits": {"outA": [-5.0, 5.0]}}}).ok
    assert not core.handle({"id": "2", "cmd": "run_target",
                            "args": {"port": "outA",
                                     "target": 6.0}}).ok


def test_idle_guard_poll_latches_outside_moves():
    core = make_core()
    core.handle({"id": "1", "cmd": "set_guards",
                 "args": {"rules": [{"sensor": "in1"}]}})
    assert core.poll_idle_guards() == []
    core.sensors["in1"].set(1)
    fresh = core.poll_idle_guards()
    assert [e.event for e in fresh] == ["sensor_guard"]
    assert core.guards.latched
    assert not core.handle({"id": "2", "cmd": "run_target",
                            "args": {"port": "outA",
                                     "target": 5.0}}).ok


# -- 5B.3 fake Bluetooth link + master relay -------------------------------


def test_fake_bt_link_framing_roundtrip():
    link = FakeBtLink()
    link.master_send({"id": "7", "cmd": "ping", "args": {}})
    frames = link.slave_recv()
    assert frames == [{"kind": "cmd",
                       "body": {"id": "7", "cmd": "ping", "args": {}}}]
    link.slave_send("ack", {"id": "7", "ok": True})
    assert link.master_recv() == [{"kind": "ack",
                                   "body": {"id": "7", "ok": True}}]


def test_link_frame_split_feed():
    raw = encode({"kind": "cmd", "body": {"id": "1"}})
    framer = Framer()
    assert framer.feed(raw[:2]) == []
    assert framer.feed(raw[2:]) == [{"kind": "cmd", "body": {"id": "1"}}]
    with pytest.raises(ValueError):
        encode({"big": "x" * (2 ** 20 + 1)})


def test_slave_handler_commands_and_specials():
    core = BrickCore("right", {"outA": FakeMotor("outA", 0.0)},
                     {"in1": FakeSensor("in1", 0)})
    sent: list = []
    slave_mod.handle_relayed_command(
        core, lambda k, b: sent.append((k, b)),
        {"id": "1", "cmd": "ping", "args": {}})
    assert ("ack", {"id": "1", "ok": True, "result": {"brick": "right"},
                    "brick": "right"}) in sent
    slave_mod.handle_relayed_command(
        core, lambda k, b: sent.append((k, b)),
        {"id": "2", "cmd": "run_target",
         "args": {"port": "outA", "target": 20.0}})
    assert abs(core.motors["outA"].position() - 20.0) < 1e-9
    slave_mod.handle_relayed_command(
        core, lambda k, b: sent.append((k, b)),
        {"id": "3", "cmd": "__heartbeat__", "args": {}})
    assert not core.guards.heartbeat_expired()
    slave_mod.handle_relayed_command(
        core, lambda k, b: sent.append((k, b)),
        {"id": "4", "cmd": "__estop__", "args": {}})
    assert core.motors["outA"].stopped
    assert any(k == "event" and b["event"] == "estop" for k, b in sent)


def test_master_estop_and_heartbeat_forward_to_slave():
    core = make_core()
    pubs: list = []
    sent: list = []
    master_mod.on_mqtt_command(
        core, lambda t, b, qos=1: pubs.append((t, json.loads(b))),
        sent.append, "robot/estop", b'{"stop": true}')
    assert sent and sent[0]["cmd"] == "__estop__"
    assert any(t == "robot/event/left" for t, _ in pubs)
    pubs.clear()
    sent.clear()
    master_mod.on_mqtt_command(
        core, lambda t, b, qos=1: pubs.append((t, json.loads(b))),
        sent.append, "robot/heartbeat", b'{"t": 1.0}')
    assert sent and sent[0]["cmd"] == "__heartbeat__"
    assert not core.guards.heartbeat_expired()


def test_master_forwards_local_guard_stop_to_slave():
    core = make_core()
    core.handle({"id": "0", "cmd": "set_guards",
                 "args": {"rules": [{"sensor": "in1"}]}})
    core.sensors["in1"].set(1)
    pubs: list = []
    sent: list = []
    master_mod.on_mqtt_command(
        core, lambda t, b, qos=1: pubs.append((t, json.loads(b))),
        sent.append, "robot/cmd/left",
        json.dumps({"id": "1", "cmd": "run_target",
                    "args": {"port": "outA", "target": 30.0}}).encode())
    acks = [b for t, b in pubs if t == "robot/ack/left"]
    assert acks and not acks[0]["ok"]
    assert any(m.get("cmd") == "__estop__" for m in sent)


def test_slave_guard_event_stops_master():
    core = make_core()
    pubs: list = []
    master_mod.on_slave_frame(
        core, lambda t, b, qos=1: pubs.append((t, json.loads(b))),
        {"kind": "event",
         "body": {"brick": "right", "event": "sensor_guard",
                  "detail": {"sensor": "in1"}}})
    assert core.motors["outA"].stopped
    assert core.guards.latched
    assert any(t == "robot/event/right" for t, _ in pubs)


# -- 5B.5 Transport interface over the fake relay ---------------------------


def test_transport_send_state_event_estop_heartbeat():
    transport, left, right = build_fake_transport(full=False)
    try:
        states: list = []
        events: list = []
        transport.on_state(states.append)
        transport.on_event(events.append)
        assert transport.send("left", "ping") == {"brick": "left"}
        # Slave path goes through the fake Bluetooth relay.
        assert transport.send("right", "ping") == {"brick": "right"}
        assert transport.send("left", {"cmd": "ping", "id": "custom-1"}) \
            == {"brick": "left"}
        transport.move_motors({("right", "outB"): 25.0}, speed=30.0)
        assert abs(right.motors["outB"].position() - 25.0) < 1e-9
        assert states  # state callbacks fire
        transport.estop()
        transport.heartbeat()  # single heartbeat must not raise
        assert not left.guards.heartbeat_expired()
    finally:
        transport.close()


def test_transport_convenience_and_errors():
    transport, left, right = build_fake_transport(full=False)
    try:
        transport.set_guards("left", [{"sensor": "in1"}])
        left.sensors["in1"].set(1)  # homing ends at once
        res = transport.home_motor("left", "outA", "in1",
                                   direction=-30.0, speed=20.0)
        assert res.get("homed")
        left.sensors["in1"].set(0)
        state = transport.read_state()
        assert set(state) == {"left", "right"}
        transport.stop()
        transport.clear_latch()
        with pytest.raises(TransportError):
            transport.send("left", "run_target", {"port": "outZ",
                                                 "target": 1.0},
                           timeout_s=2.0)
        # request() is the backwards-compatible alias of send().
        assert transport.request("left", "ping") == {"brick": "left"}
    finally:
        transport.close()


def test_transport_estop_reaches_slave_over_fake_bt():
    transport, left, right = build_fake_transport(full=False)
    try:
        transport.estop()
        assert right.motors["outB"].stopped
    finally:
        transport.close()


def test_fake_cores_full_wiring_counts():
    left, right = build_fake_cores(full=True)
    assert len(left.motors) + len(right.motors) == 6
    assert len(left.sensors) + len(right.sensors) == 5
    assert left.info()["brick"] == "left"


# -- 5B.6 fake end-to-end ----------------------------------------------------


def test_fake_cross_brick_relay_ms():
    from src.ev3.check import cross_brick_check
    from src.ev3.fake import build_mp_transport

    transport, _, _ = build_mp_transport(full=True)
    try:
        ms = cross_brick_check(transport, auto_press=lambda b, s: None)
        assert ms is not None and ms >= 0.0
    finally:
        transport.close()


def test_check_driver_fake_full_sequence():
    import queue

    from src.ev3.check import build_fake_setup, check_driver

    driver, left, right = build_fake_setup()
    try:
        cores = {"left": left, "right": right}

        def auto_press(brick, sensor):
            for s in cores[brick].sensors.values():
                s.set(0)
            if sensor is not None:
                cores[brick].sensors[sensor].set(1)

        results = check_driver(driver, auto_press=auto_press)
        assert results["total_motors"] == 6
        assert results["total_sensors"] == 5
        for brick_res in results["bricks"].values():
            for m in brick_res["motors"].values():
                assert m["ok"]
        assert results["sensor_events"]
        assert all(v["ok"] for v in results["sensor_events"].values())
        assert results["cross_brick_ms"] is not None
    finally:
        driver.close()
        for s in list(left.sensors.values()) + list(right.sensors.values()):
            s.set(0)


def test_brick_knows_ports_only():
    """No joint names, gear ratios or sensor meanings in brick sources."""
    import pathlib

    root = pathlib.Path(__file__).resolve().parents[1] / "src" / "ev3" / "brick"
    text = "\n".join(p.read_text() for p in root.glob("*.py"))
    for token in ("J1", "J2", "J3", "J4", "J5", "J6", "joint_name",
                  "gear_ratio", "ratio:", "shoulder", "elbow", "wrist"):
        assert token not in text
