"""PC-side tests for the Stage 0 safety layer and command executor.

Uses FakeMotor/FakeSensor and a fake Bluetooth link: no bricks needed.
"""

from __future__ import annotations

import time

from src.ev3.brick.core import BrickCore
from src.ev3.brick.guards import SafetyController, SensorRule
from src.ev3.brick.link import Framer, encode
from src.ev3.brick.motors import FakeMotor
from src.ev3.brick.sensors import FakeSensor


def make_core() -> BrickCore:
    return BrickCore(
        "left",
        {"outA": FakeMotor("outA", 0.0)},
        {"in1": FakeSensor("in1", 0)},
    )


def test_sensor_guard_latches_and_refuses_moves():
    core = make_core()
    assert core.handle({"id": "1", "cmd": "set_guards",
                        "args": {"rules": [{"sensor": "in1"}]}}).ok
    assert core.handle({"id": "2", "cmd": "run_target",
                        "args": {"port": "outA", "target": 50.0,
                                 "speed": 30.0}}).ok
    core.sensors["in1"].set(1)
    ack = core.handle({"id": "3", "cmd": "run_target",
                       "args": {"port": "outA", "target": 60.0,
                                "speed": 30.0}})
    assert not ack.ok and ("latched" in ack.error or "guard stop" in ack.error)
    assert any(e.event == "sensor_guard" for e in core.pending_events)


def test_homing_exception_does_not_latch():
    core = make_core()
    core.handle({"id": "1", "cmd": "set_guards",
                 "args": {"rules": [{"sensor": "in1"}]}})
    core.sensors["in1"].set(1)  # sensor already pressed: homing ends at once
    ack = core.handle({"id": "2", "cmd": "run_target",
                       "args": {"port": "outA", "target": -30.0,
                                "speed": 20.0, "until_sensor": "in1"}})
    assert ack.ok and ack.result.get("homed")
    assert not core.guards.latched


def test_clear_latch_releases_moves():
    core = make_core()
    core.handle({"id": "1", "cmd": "set_guards",
                 "args": {"rules": [{"sensor": "in1"}]}})
    core.sensors["in1"].set(1)
    assert not core.handle(
        {"id": "2", "cmd": "run_target",
         "args": {"port": "outA", "target": 10.0}}).ok
    core.sensors["in1"].set(0)
    assert core.handle({"id": "3", "cmd": "clear_latch",
                        "args": {}}).ok
    assert core.handle({"id": "4", "cmd": "run_target",
                        "args": {"port": "outA", "target": 10.0}}).ok


def test_speed_is_capped_at_100():
    assert SafetyController.clamp_speed(250.0) == 100.0
    assert SafetyController.clamp_speed(-250.0) == -100.0


def test_heartbeat_expiry_detected():
    ctl = SafetyController()
    assert not ctl.heartbeat_expired()
    ctl._last_heartbeat -= 5.0
    assert ctl.heartbeat_expired()


def test_stall_latches_after_timeout():
    ctl = SafetyController()
    stalled: dict[str, float] = {}
    events = []
    for _ in range(40):  # 40 x 10 ms > 300 ms
        events += ctl.check_stall({"outA": 30.0}, {"outA": 0.0},
                                  stalled, 0.01)
    assert any(e.event == "stall" for e in events)
    assert ctl.latched


def test_unknown_command_and_port_rejected():
    core = make_core()
    assert not core.handle({"id": "1", "cmd": "fly",
                            "args": {}}).ok
    ack = core.handle({"id": "2", "cmd": "run_target",
                       "args": {"port": "outZ", "target": 5.0}})
    assert not ack.ok


def test_link_framing_roundtrip_and_split_feed():
    msg = {"kind": "cmd", "body": {"id": "7", "cmd": "ping"}}
    raw = encode(msg)
    framer = Framer()
    assert framer.feed(raw[:3]) == []
    assert framer.feed(raw[3:]) == [msg]


def test_driver_request_response_against_fake_brick():
    import json
    import re

    from src.ev3.driver import Driver

    core = make_core()

    class FakeClient:
        def __init__(self) -> None:
            self.on_message = None

        def subscribe(self, *_a, **_k) -> None:
            pass

        def publish(self, topic: str, payload: str, qos: int = 0) -> None:
            m = re.match(r"robot/cmd/(left|right)$", topic)
            if m:
                ack = core.handle(json.loads(payload))
                self.on_message(
                    self, None,
                    type("M", (), {
                        "topic": "robot/ack/left",
                        "payload": json.dumps(
                            ack.to_dict("left")).encode()})())

        def loop_start(self) -> None:
            pass

        def loop_stop(self) -> None:
            pass

    driver = Driver(client_factory=FakeClient, bricks=("left",))
    try:
        assert driver.request("left", "ping")["brick"] == "left"
        driver.move_motors({("left", "outA"): 20.0}, speed=30.0)
        assert abs(core.motors["outA"].position() - 20.0) < 1e-9
        driver.estop()  # must not raise without a broker
    finally:
        driver.close()
        time.sleep(0.05)
