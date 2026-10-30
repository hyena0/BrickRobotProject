"""Hardware-level tests: unmodified brick code against fake devices.

``tests/fakes`` on ``sys.path`` makes the brick modules' lazy
``pybricks.*`` / ``umqtt.simple`` imports resolve to scriptable fakes,
so ``open_motor``, ``open_sensor``, ``open_master_link``,
``open_slave_link``, the runtimes and the MQTT call shapes all execute
exactly as written for the brick. Re-verifies the 5B safety scenarios
through that real path.
"""

import json
import pathlib
import sys

import pytest

FAKES = pathlib.Path(__file__).resolve().parent / "fakes"
BRICK_MP = pathlib.Path(__file__).resolve().parents[1] / "src" / "ev3" / "brick_mp"
for _mod in [m for m in sys.modules if m == "pybricks" or m.startswith("pybricks.")
              or m == "umqtt" or m.startswith("umqtt.")]:
    del sys.modules[_mod]
sys.path.insert(0, str(FAKES))
sys.path.insert(0, str(BRICK_MP))

import clock  # noqa: E402
import core  # noqa: E402
import display  # noqa: E402
import guards  # noqa: E402
import left_main  # noqa: E402
import link  # noqa: E402
import motors  # noqa: E402
import right_main  # noqa: E402
import sensors  # noqa: E402
from pybricks import ev3devices as pb_ev3  # noqa: E402
from pybricks import messaging as pb_messaging  # noqa: E402
from umqtt import simple as umqtt_simple  # noqa: E402


@pytest.fixture(autouse=True)
def _reset_fakes():
    pb_ev3.reset()
    pb_messaging.reset()
    umqtt_simple.reset()
    yield
    pb_ev3.reset()
    pb_messaging.reset()
    umqtt_simple.reset()


def _make_core(name="left"):
    return core.BrickCore(
        name,
        {"outA": motors.open_motor("outA", "large")},
        {"in1": sensors.open_sensor("in1", "touch")},
    )


def _make_runtimes():
    left = _make_core("left")
    right = _make_core("right")
    master_end = link.open_master_link()
    slave_end = link.open_slave_link("00:16:53:AA:BB:CC")
    pubs = []
    master = left_main.MasterRuntime(
        left, lambda t, o, q: pubs.append((t, o, q)), master_end)
    slave = right_main.SlaveRuntime(right, slave_end)
    return master, slave, pubs


def _acked(pubs, topic):
    return [o for t, o, q in pubs if t == topic]


# -- real HAL construction ----------------------------------------------------


def test_builtin_default_wiring_builds_both_bricks(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    left = left_main.build_core(left_main.DEFAULT_CONFIG)
    assert set(left.motors) == {"outA", "outB", "outC", "outD"}
    assert set(left.sensors) == {"in1", "in2", "in3"}
    assert left.motor_types == {"outA": "large", "outB": "large",
                                "outC": "medium", "outD": "medium"}
    right = right_main.build_core(right_main.DEFAULT_CONFIG)
    assert set(right.motors) == {"outA", "outB"}
    assert set(right.sensors) == {"in1", "in2"}
    assert set(right.motor_types.values()) == {"medium"}
    assert right.sensor_types == {"in1": "touch", "in2": "color"}
    assert len(left.motors) + len(right.motors) == 6
    assert len(left.sensors) + len(right.sensors) == 5


def test_build_core_skips_unconnected_ports(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    pb_ev3.set_missing("C", "S3")
    left = left_main.build_core(left_main.DEFAULT_CONFIG)
    assert set(left.motors) == {"outA", "outB", "outD"}
    assert set(left.sensors) == {"in1", "in2"}
    out = capsys.readouterr().out
    assert "NOT CONNECTED" in out and "outC" in out and "in3" in out
    assert left.info()["motors"] == [
        {"port": "outA", "type": "large"},
        {"port": "outB", "type": "large"},
        {"port": "outD", "type": "medium"},
    ]
    missing_move = left.handle({"id": "1", "cmd": "run_target",
                                "args": {"port": "outC", "target": 10.0}})
    assert not missing_move.ok and "unknown port" in missing_move.error
    assert set(left.snapshot()["missing"]) == {"outC", "in3"}
    logged = (tmp_path / "brick.log").read_text()
    assert "NOT CONNECTED" in logged and "outC" in logged


def test_open_motor_sensor_use_pybricks_api():
    motor = motors.open_motor("outA", "large")
    assert motor.port == "outA"
    assert motor._dev.port == "A"
    sensor = sensors.open_sensor("in1", "touch")
    assert sensor.port == "in1"
    assert sensor._dev.port == "S1"
    with pytest.raises(ValueError):
        motors.open_motor("outZ", "large")
    with pytest.raises(ValueError):
        sensors.open_sensor("in9", "touch")
    with pytest.raises(ValueError):
        sensors.open_sensor("in1", "sonar")


def test_color_sensor_reads_binary_via_threshold():
    sensor = sensors.open_sensor("in2", "color")
    sensor._dev.set_reflection(10)
    assert sensor.read() == 0
    sensor._dev.set_reflection(90)
    assert sensor.read() == 1


def test_move_and_readback_through_real_motor():
    brick = _make_core()
    ack = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": 20.0,
                                 "speed": 60.0}})
    assert ack.ok
    assert abs(brick.motors["outA"].position() - 20.0) < 3.0
    snap = brick.handle({"id": "2", "cmd": "read"}).result
    assert abs(snap["encoders"]["outA"] - 20.0) < 3.0


def test_stop_cancels_real_motor():
    motor = motors.open_motor("outA", "medium")
    motor.start_move(90.0, 30.0)
    assert motor.is_moving()
    motor.stop()
    assert not motor.is_moving()
    assert motor._dev.braked


# -- 5B safety scenarios through the real path --------------------------------


def test_sensor_guard_latch_refuse_clear():
    brick = _make_core()
    brick.handle({"id": "0", "cmd": "set_guards",
                  "args": {"rules": [{"sensor": "in1"}]}})
    brick.sensors["in1"]._dev.set_pressed(True)
    ack = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": 10.0}})
    assert not ack.ok and brick.guards.latched
    brick.sensors["in1"]._dev.set_pressed(False)
    assert brick.handle({"id": "3", "cmd": "clear_latch",
                         "args": {}}).ok
    assert brick.handle({"id": "4", "cmd": "run_target",
                         "args": {"port": "outA", "target": 10.0}}).ok


def test_homing_ends_without_latch():
    brick = _make_core()
    brick.handle({"id": "0", "cmd": "set_guards",
                  "args": {"rules": [{"sensor": "in1"}]}})
    brick.sensors["in1"]._dev.set_pressed(True)
    ack = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": -30.0,
                                 "speed": 20.0, "until_sensor": "in1"}})
    assert ack.ok and ack.result.get("homed")
    assert not brick.guards.latched


def test_speed_cap_reaches_motor():
    brick = _make_core()
    assert brick.handle({"id": "1", "cmd": "run_target",
                         "args": {"port": "outA", "target": 5.0,
                                  "speed": 400.0}}).ok
    assert brick.motors["outA"]._dev._speed == 300.0


def test_same_goal_calls_dev_run_target_once():
    brick = _make_core()
    args = {"port": "outA", "target": 20.0, "speed": 250.0}
    first = brick.handle({"id": "1", "cmd": "run_target",
                          "args": dict(args)})
    assert first.ok and not first.result.get("skipped")
    second = brick.handle({"id": "2", "cmd": "run_target",
                           "args": dict(args)})
    assert second.ok and second.result.get("skipped")
    assert brick.motors["outA"]._dev.run_target_calls == 1


def test_run_angle_diag_moves_relative():
    brick = _make_core()
    ack = brick.handle({"id": "1", "cmd": "run_angle",
                        "args": {"port": "outA", "angle": 360.0,
                                 "speed": 90.0}})
    assert ack.ok
    assert abs(ack.result["before"]) < 1e-9
    assert abs(ack.result["after"] - 360.0) < 3.0
    assert abs(ack.result["moved"] - 360.0) < 3.0


def test_open_sets_firmware_stall_tolerances():
    brick = _make_core()
    assert brick.motors["outA"]._dev.control.stall_tol == (3, 300)


def test_open_motor_without_control_api(monkeypatch):
    real_motor = pb_ev3.Motor

    class _NoControlMotor(real_motor):
        def __init__(self, port, **kwargs):
            real_motor.__init__(self, port, **kwargs)
            del self.control

    monkeypatch.setattr(pb_ev3, "Motor", _NoControlMotor)
    motor = motors.open_motor("outA", "medium")
    assert motor.position() == 0.0


def test_firmware_stalled_missing_api_reads_false():
    class _LegacyDev:
        def angle(self):
            return 0.0

    motor = motors.PybricksMotor("outA", _LegacyDev(), None)
    assert motor.firmware_stalled() is False


def test_firmware_stall_signal_latches():
    brick = _make_core()
    brick.motors["outA"]._dev.fake_stalled = True
    ack = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": 40.0,
                                 "speed": 250.0}})
    assert not ack.ok
    assert brick.guards.latched
    assert brick.guards.latch_reason == "stall:outA"


def test_stall_with_brake_eio_survives():
    brick = _make_core()
    dev = brick.motors["outA"]._dev
    dev.jammed = True

    def _boom():
        raise OSError(5, "EIO")

    dev.brake = _boom
    ack = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": 40.0,
                                 "speed": 30.0}})
    assert not ack.ok and "guard stop" in ack.error
    assert brick.guards.latched
    assert brick.guards.latch_reason == "stall:outA"
    assert any(e.event == "motor_error"
               for e in brick.pending_events)


def test_stall_guard_trips_on_jammed_motor():
    brick = _make_core()
    brick.motors["outA"]._dev.jammed = True
    ack = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": 40.0,
                                 "speed": 30.0}})
    assert not ack.ok and "guard stop" in ack.error
    assert brick.guards.latched
    assert any(e.event == "stall" for e in brick.pending_events)


def test_motor_limits_reject_without_latch():
    brick = _make_core()
    brick.handle({"id": "0", "cmd": "set_guards",
                  "args": {"rules": [],
                           "limits": [{"port": "outA", "min": -10.0,
                                       "max": 10.0}]}})
    bad = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": 30.0}})
    assert not bad.ok and "limit_reject" in bad.error
    assert not brick.guards.latched


def test_idle_poll_and_unknown_inputs():
    brick = _make_core()
    brick.handle({"id": "0", "cmd": "set_guards",
                  "args": {"rules": [{"sensor": "in1"}]}})
    brick.sensors["in1"]._dev.set_pressed(True)
    assert brick.poll_idle_guards()
    assert brick.guards.latched
    assert not brick.handle({"id": "1", "cmd": "fly",
                             "args": {}}).ok
    assert not brick.handle({"id": "2", "cmd": "run_target",
                             "args": {"port": "outZ",
                                      "target": 1.0}}).ok


# -- real mailbox link + runtimes ----------------------------------------------


def _settle(master, slave, rounds=3):
    for _ in range(rounds):
        master.service_slave()
        slave.tick()
    master.service_slave()


def test_relay_ping_move_state_over_real_mailboxes():
    master, slave, pubs = _make_runtimes()
    assert master.mbox._conn.waited == 1
    master.on_mqtt("robot/cmd/right",
                   b'{"id": "1", "cmd": "ping", "args": {}}')
    _settle(master, slave)
    acks = _acked(pubs, "robot/ack/right")
    assert len(acks) == 1 and acks[0]["ok"] and acks[0]["id"] == "1"
    master.on_mqtt("robot/cmd/right",
                   b'{"id": "2", "cmd": "run_target", "args": '
                   b'{"port": "outA", "target": 25.0, "speed": 60.0}}')
    _settle(master, slave)
    assert len(_acked(pubs, "robot/ack/right")) == 2
    assert abs(slave.core.motors["outA"].position() - 25.0) < 3.0
    assert _acked(pubs, "robot/state/right")


def test_slave_event_published_once():
    master, slave, pubs = _make_runtimes()
    slave.core.handle({"id": "0", "cmd": "set_guards",
                       "args": {"rules": [{"sensor": "in1"}]}})
    slave.core.sensors["in1"]._dev.set_pressed(True)
    slave.tick()
    master.service_slave()
    master.service_slave()  # same box value twice: no duplicate
    evts = [e for e in _acked(pubs, "robot/event/right")
            if e["event"] == "sensor_guard"]
    assert len(evts) == 1


def test_estop_and_heartbeat_forwarding():
    master, slave, pubs = _make_runtimes()
    master.on_mqtt("robot/estop", b'{"stop": true}')
    assert master.core.motors["outA"]._dev.braked
    slave.tick()
    assert slave.core.motors["outA"]._dev.braked
    master.service_slave()
    assert any(e["event"] == "estop"
               for e in _acked(pubs, "robot/event/right"))
    master.on_mqtt("robot/heartbeat", b'{"t": 1}')
    assert not master.core.guards.heartbeat_expired()
    slave.tick()
    assert not slave.core.guards.heartbeat_expired()


def test_cross_brick_slave_to_master():
    master, slave, pubs = _make_runtimes()
    slave.core.handle({"id": "0", "cmd": "set_guards",
                       "args": {"rules": [{"sensor": "in1"}]}})
    slave.core.sensors["in1"]._dev.set_pressed(True)
    slave.tick()
    master.service_slave()
    assert master.core.motors["outA"]._dev.braked
    assert master.core.guards.latched


def test_cross_brick_master_to_slave():
    master, slave, pubs = _make_runtimes()
    master.core.handle({"id": "0", "cmd": "set_guards",
                        "args": {"rules": [{"sensor": "in1"}]}})
    master.core.sensors["in1"]._dev.set_pressed(True)
    master.on_mqtt("robot/cmd/left",
                   b'{"id": "1", "cmd": "run_target", "args": '
                   b'{"port": "outA", "target": 30.0, "speed": 30.0}}')
    assert master.core.guards.latched
    slave.tick()
    assert slave.core.motors["outA"]._dev.braked


def test_master_heartbeat_watchdog_stops_and_forwards():
    master, slave, pubs = _make_runtimes()
    master.core.guards._last_heartbeat -= 5000
    master.pump()
    assert master.core.motors["outA"]._dev.braked
    assert any(e["event"] == "heartbeat_timeout"
               for e in _acked(pubs, "robot/event/left"))
    slave.tick()
    assert slave.core.motors["outA"]._dev.braked


def test_master_slave_link_watchdog_and_fail_fast():
    master, slave, pubs = _make_runtimes()
    master._last_slave_contact -= 10000
    master.pump()
    assert master.slave_link_lost
    assert any(e["event"] == "bluetooth_loss"
               for e in _acked(pubs, "robot/event/left"))
    master.on_mqtt("robot/cmd/right",
                   b'{"id": "9", "cmd": "ping", "args": {}}')
    master.service_slave()
    acks = _acked(pubs, "robot/ack/right")
    assert len(acks) == 1 and not acks[0]["ok"]
    slave._last_state_send = 0
    slave.tick()
    master.service_slave()
    assert not master.slave_link_lost


def test_slave_link_watchdog_stops_and_recovers():
    master, slave, pubs = _make_runtimes()
    slave.mbox._conn.disconnect()
    slave._last_master_contact -= 10000
    slave._last_reconnect_try = 0
    slave.tick()
    assert slave.link_lost
    assert slave.core.motors["outA"]._dev.braked
    slave.mbox._conn.connect("00:16:53:AA:BB:CC")
    master._send_hb()
    slave.tick()
    assert not slave.link_lost
    raw = slave.mbox.evt.read()
    assert raw is not None
    assert json.loads(raw)["event"] == "bluetooth_loss"


# -- fake umqtt broker ----------------------------------------------------------


def test_display_renders_and_shows():
    from pybricks import hubs as pb_hubs

    master, slave, _pubs = _make_runtimes()
    lines = display.render_master(master, True, "192.168.35.151")
    assert lines[0].startswith("LEFT ")
    assert lines[0].endswith(" OK")
    assert lines[1] == "M:A S:1"
    assert lines[2] == "HUB:UP SLV:UP"
    assert lines[3] == "192.168.35.151"
    master.core.guards.latched = True
    master.core.guards.latch_reason = "sensor_guard:in1"
    master.core.missing = ["in2"]
    lines = display.render_master(master, False, "192.168.35.151")
    assert lines[0].endswith(" LATCH")
    assert lines[2] == "HUB:DOWN SLV:UP"
    assert "LATCH:sensor_guard:in1" in lines
    assert "MISS:in2" in lines
    slines = display.render_slave(slave, "ev3dev")
    assert slines[0].startswith("RIGHT ")
    assert slines[2] == "MST:UP ev3dev"
    display.show(lines)
    display.show(slines)
    assert display._screen is not None
    assert display._screen.clears == 2
    assert display._screen.lines == slines
    assert isinstance(display._screen, pb_hubs.FakeScreen)


def test_fake_mqtt_pub_sub_retained_will():
    hub = umqtt_simple.MQTTClient(b"hub", "localhost", 1883)
    brick = umqtt_simple.MQTTClient(b"brick", "localhost", 1883)
    hub.connect()
    brick.connect()
    got = []
    hub.set_callback(lambda t, m: got.append((t, m)))
    hub.subscribe(b"robot/ack/right", 1)
    hub.subscribe(b"robot/state/#", 1)
    brick.publish(b"robot/ack/right", b'{"id": "1"}', False, 1)
    brick.publish(b"robot/state/left", b'{}', False, 0)
    brick.publish(b"robot/cmd/left", b'{}', False, 1)  # not subscribed
    hub.check_msg()
    hub.check_msg()
    assert hub.check_msg() is None
    assert [t for t, m in got] == [b"robot/ack/right", b"robot/state/left"]
    brick.publish(b"robot/status/left", b'"online"', True, 1)
    late = umqtt_simple.MQTTClient(b"late", "localhost", 1883)
    late.connect()
    late.subscribe(b"robot/status/left", 1)
    assert late.inbox == [(b"robot/status/left", b'"online"')]
    brick.set_last_will(b"robot/status/left", b'"offline"', True, 1)
    hub.subscribe(b"robot/status/left", 1)
    hub.check_msg()  # drain retained online
    brick.drop()
    hub.check_msg()
    assert got[-1] == (b"robot/status/left", b'"offline"')
    umqtt_simple.set_broker_down(True)
    with pytest.raises(OSError):
        umqtt_simple.MQTTClient(b"x", "localhost", 1883).connect()


def _wire_left_over_fake_broker(master):
    """Mirror left_main.main()'s hub wiring against the fake broker."""
    left = umqtt_simple.MQTTClient(b"brick_left", "hub", 1883, None, None, 60)
    left.set_last_will(b"robot/status/left", b'"offline"', True, 1)
    left.connect()

    def publish(topic, obj, qos):
        left.publish(topic.encode(), json.dumps(obj).encode(), False, qos)

    master.publish = publish

    def route(topic_b, msg_b):
        master.inbox.append((topic_b.decode(), msg_b))

    left.set_callback(route)
    for t in left_main.SUB_TOPICS:
        left.subscribe(t.encode(), 1)
    left.publish(b"robot/status/left", b'"online"', True, 1)
    master.check_inbox = left.check_msg
    return left


def test_hub_to_brick_roundtrip_over_fake_mqtt():
    master, slave, _pubs = _make_runtimes()
    left = _wire_left_over_fake_broker(master)
    hub = umqtt_simple.MQTTClient(b"hub", "hub", 1883)
    hub.connect()
    got = []
    hub.set_callback(lambda t, m: got.append((t, json.loads(m.decode()))))
    hub.subscribe(b"robot/ack/left", 1)
    hub.subscribe(b"robot/ack/right", 1)
    hub.publish(b"robot/cmd/left", b'{"id": "h1", "cmd": "ping", "args": {}}',
                False, 1)
    left.check_msg()
    master.pump()
    hub.check_msg()
    assert got == [(b"robot/ack/left",
                    {"id": "h1", "ok": True, "result": {"brick": "left"},
                     "brick": "left"})]
    hub.publish(b"robot/cmd/right",
                b'{"id": "h2", "cmd": "ping", "args": {}}', False, 1)
    left.check_msg()
    master.pump()
    _settle(master, slave)
    hub.check_msg()
    assert got[-1][0] == b"robot/ack/right"
    assert got[-1][1]["ok"] and got[-1][1]["id"] == "h2"
