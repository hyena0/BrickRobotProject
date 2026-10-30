"""PC-side tests for the Pybricks brick programs (`brick_mp`).

The brick modules use plain top-level imports so they also run flat on
the brick; tests add src/ev3/brick_mp to sys.path first. Fake motors,
sensors and mailbox link: no bricks needed. Also enforces the
MicroPython syntax constraints statically.
"""

import ast
import json
import pathlib
import sys

import pytest

BRICK_MP = pathlib.Path(__file__).resolve().parents[1] / "src" / "ev3" / "brick_mp"
sys.path.insert(0, str(BRICK_MP))

import clock  # noqa: E402
import config as brick_config  # noqa: E402
import core  # noqa: E402
import guards  # noqa: E402
import left_main  # noqa: E402
import link  # noqa: E402
import log as brick_log  # noqa: E402
import motors  # noqa: E402
import right_main  # noqa: E402
import sensors  # noqa: E402
import version as brick_version  # noqa: E402


def make_core(name="left"):
    return core.BrickCore(
        name,
        {"outA": motors.FakeMotor("outA", 0.0)},
        {"in1": sensors.FakeSensor("in1", 0)},
    )


def make_pair():
    """Master + slave runtimes sharing one fake mailbox link."""
    left = make_core("left")
    right = make_core("right")
    mbox = link.FakeLink()
    pubs = []
    master = left_main.MasterRuntime(
        left, lambda t, o, q: pubs.append((t, o, q)), mbox)
    slave = right_main.SlaveRuntime(right, mbox)
    return master, slave, pubs


def ack_of(pubs, topic):
    return [o for t, o, q in pubs if t == topic]


# -- config -----------------------------------------------------------------


def test_config_json_load(tmp_path):
    cfg = tmp_path / "brick.local.json"
    cfg.write_text(json.dumps({
        "brick": "left",
        "motors": [{"port": "outA", "type": "large"}],
        "sensors": [{"port": "in1", "type": "touch"}],
    }))
    loaded = brick_config.load_brick_config(str(cfg))
    assert loaded == {
        "brick": "left",
        "motors": [{"port": "outA", "type": "large"}],
        "sensors": [{"port": "in1", "type": "touch"}],
    }


def test_connection_targets_hardcoded():
    assert left_main.HUB_ADDRESS == "192.168.35.151"
    assert left_main.HUB_PORT == 1883
    assert right_main.MASTER_BT_NAME == "ev3dev"


def test_config_errors(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "notes.txt").write_text("decoy")
    with pytest.raises(OSError) as exc:
        brick_config.load_brick_config(str(tmp_path / "nope.json"))
    assert "tried" in str(exc.value)
    assert "brick.local.json" in str(exc.value)
    assert "notes.txt" in str(exc.value)
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"brick": "middle"}))
    with pytest.raises(ValueError):
        brick_config.load_brick_config(str(bad))
    bad.write_text(json.dumps({}))
    with pytest.raises(ValueError):
        brick_config.load_brick_config(str(bad))


def test_config_flat_fallback_next_to_program(tmp_path, monkeypatch):
    (tmp_path / "brick.local.json").write_text(json.dumps({
        "brick": "right"}))
    monkeypatch.chdir(tmp_path)
    loaded = brick_config.load_brick_config("config/brick.local.json")
    assert loaded["brick"] == "right"


# -- guards -----------------------------------------------------------------


def test_sensor_guard_latches_and_homing_passes():
    brick = make_core()
    assert brick.handle({"id": "1", "cmd": "set_guards",
                         "args": {"rules": [{"sensor": "in1"}]}}).ok
    brick.sensors["in1"].set(1)
    ack = brick.handle({"id": "2", "cmd": "run_target",
                        "args": {"port": "outA", "target": 50.0}})
    assert not ack.ok
    assert brick.guards.latched
    brick.sensors["in1"].set(0)
    assert brick.handle({"id": "3", "cmd": "clear_latch",
                         "args": {}}).ok
    brick.sensors["in1"].set(1)
    homed = brick.handle({"id": "4", "cmd": "run_target",
                          "args": {"port": "outA", "target": -30.0,
                                   "until_sensor": "in1"}})
    assert homed.ok and homed.result.get("homed")
    assert not brick.guards.latched


def test_motor_limits_reject_without_latch():
    brick = make_core()
    ack = brick.handle({"id": "1", "cmd": "set_guards",
                        "args": {"rules": [],
                                 "limits": [{"port": "outA", "min": -90.0,
                                             "max": 90.0}]}})
    assert ack.ok and ack.result["limits"] == 1
    assert brick.handle({"id": "2", "cmd": "run_target",
                         "args": {"port": "outA", "target": 45.0}}).ok
    bad = brick.handle({"id": "3", "cmd": "run_target",
                        "args": {"port": "outA", "target": 120.0}})
    assert not bad.ok and "limit_reject" in bad.error
    assert not brick.guards.latched
    brick.handle({"id": "4", "cmd": "set_guards",
                  "args": {"rules": [], "limits": []}})
    assert brick.handle({"id": "5", "cmd": "run_target",
                         "args": {"port": "outA", "target": 120.0}}).ok


def test_stall_and_heartbeat_ms():
    ctl = guards.SafetyController()
    stalled = {}
    events = []
    for _ in range(40):
        events += ctl.check_stall({"outA": 30.0}, {"outA": 0.0},
                                  stalled, 10)
    assert any(e.event == "stall" for e in events)
    assert ctl.latched
    assert guards.clamp_speed(250.0) == 250.0
    assert guards.clamp_speed(400.0) == 300.0
    assert guards.clamp_speed(-400.0) == -300.0
    assert not ctl.heartbeat_expired()
    ctl._last_heartbeat -= 5000
    assert ctl.heartbeat_expired()


def test_core_basics_and_pump():
    brick = make_core()
    assert brick.handle({"id": "1", "cmd": "ping"}).result == {"brick": "left"}
    assert brick.handle({"id": "2", "cmd": "info"}).ok
    assert brick.handle({"id": "3", "cmd": "read"}).ok
    assert not brick.handle({"id": "4", "cmd": "fly",
                             "args": {}}).ok
    assert not brick.handle({"id": "5", "cmd": "run_target",
                             "args": {"port": "outZ",
                                      "target": 1.0}}).ok
    calls = []
    brick.handle({"id": "6", "cmd": "run_target",
                  "args": {"port": "outA", "target": 20.0}},
                 pump=lambda: calls.append(1))
    assert calls  # supervised wait pumps the loop each poll
    assert abs(brick.motors["outA"].position() - 20.0) < 1e-9
    brick.sensors["in1"].set(1)
    brick.handle({"id": "7", "cmd": "set_guards",
                  "args": {"rules": [{"sensor": "in1"}]}})
    assert brick.poll_idle_guards()
    assert brick.guards.latched


def test_fake_motor_stop_cancels_move():
    motor = motors.FakeMotor("outA", 0.0)
    motor.start_move(50.0, 30.0)
    assert motor.is_moving()
    motor.stop()
    assert not motor.is_moving()
    assert motor.stopped


class _CountingMotor(motors.FakeMotor):
    def __init__(self, port, initial=0.0):
        motors.FakeMotor.__init__(self, port, initial)
        self.starts = 0

    def start_move(self, target, speed):
        self.starts += 1
        motors.FakeMotor.start_move(self, target, speed)


def _counting_core():
    return core.BrickCore(
        "left",
        {"outA": _CountingMotor("outA", 0.0)},
        {"in1": sensors.FakeSensor("in1", 0)},
    )


def test_same_goal_never_restarts_run_target():
    brick = _counting_core()
    first = brick.handle({"id": "1", "cmd": "run_target",
                          "args": {"port": "outA", "target": 20.0}})
    assert first.ok and not first.result.get("skipped")
    second = brick.handle({"id": "2", "cmd": "run_target",
                           "args": {"port": "outA", "target": 20.0}})
    assert second.ok and second.result.get("skipped")
    assert brick.motors["outA"].starts == 1


def test_same_goal_skips_run_targets_batch():
    brick = _counting_core()
    moves = {"moves": [{"port": "outA", "target": 20.0}]}
    assert brick.handle({"id": "1", "cmd": "run_targets",
                         "args": moves}).ok
    again = brick.handle({"id": "2", "cmd": "run_targets",
                          "args": moves})
    assert again.ok and again.result.get("skipped")
    assert brick.motors["outA"].starts == 1


def test_run_angle_diag_reports_before_after():
    brick = make_core()
    ack = brick.handle({"id": "1", "cmd": "run_angle",
                        "args": {"port": "outA", "angle": 360.0,
                                 "speed": 90.0}})
    assert ack.ok
    assert ack.result["before"] == 0.0
    assert ack.result["after"] == 360.0
    assert ack.result["moved"] == 360.0


class _EioStopMotor(motors.FakeMotor):
    def stop(self):
        raise OSError(5, "EIO")


class _EioAngleMotor(motors.FakeMotor):
    def position(self):
        raise OSError(5, "EIO")


def test_stop_all_survives_motor_error():
    brick = core.BrickCore(
        "left",
        {"outA": _EioStopMotor("outA", 0.0),
         "outB": motors.FakeMotor("outB", 0.0)},
        {"in1": sensors.FakeSensor("in1", 0)},
    )
    brick.motors["outB"].start_move(50.0, 30.0)
    brick._stop_all()  # must not raise
    assert not brick.motors["outB"].is_moving()
    assert brick.guards.latched
    assert brick.guards.latch_reason == "motor_error:outA"
    assert any(e.event == "motor_error"
               for e in brick.pending_events)


def test_handle_motor_io_error_latches():
    brick = core.BrickCore(
        "left",
        {"outA": _EioAngleMotor("outA", 0.0)},
        {"in1": sensors.FakeSensor("in1", 0)},
    )
    ack = brick.handle({"id": "1", "cmd": "run_target",
                        "args": {"port": "outA", "target": 20.0}})
    assert not ack.ok and "motor io error" in ack.error
    assert brick.guards.latched
    assert brick.guards.latch_reason == "motor_io:run_target"


class _FlakyRuntime:
    def __init__(self, brick, fail):
        self.core = brick
        self.calls = 0
        self._fail = fail

    def step(self):
        self.calls += 1
        if self._fail:
            raise OSError(5, "EIO")


def test_spin_once_survives_step_error():
    brick = make_core()
    brick.motors["outA"].start_move(50.0, 30.0)
    core.spin_once(_FlakyRuntime(brick, True))
    assert not brick.motors["outA"].is_moving()
    assert brick.guards.latched
    assert brick.guards.latch_reason == "loop_error"
    assert any(e.event == "loop_error"
               for e in brick.pending_events)


def test_spin_once_passes_clean_step():
    brick = make_core()
    rt = _FlakyRuntime(brick, False)
    core.spin_once(rt)
    assert rt.calls == 1
    assert not brick.guards.latched


def test_snapshot_reports_latch_reason():
    brick = make_core()
    brick.guards.latched = True
    brick.guards.latch_reason = "stall:outA"
    snap = brick.snapshot()
    assert snap["latched"] and snap["latch_reason"] == "stall:outA"


# -- mailbox relay ------------------------------------------------------------


def test_fake_link_boxes():
    mbox = link.FakeLink()
    assert mbox.cmd.read() is None
    mbox.cmd.send('{"id": "1"}')
    assert mbox.cmd.read() == '{"id": "1"}'


def test_left_right_relay_ping_and_move():
    master, slave, pubs = make_pair()
    master.on_mqtt("robot/cmd/right",
                   b'{"id": "1", "cmd": "ping", "args": {}}')
    master.service_slave()
    slave.tick()
    master.service_slave()
    acks = ack_of(pubs, "robot/ack/right")
    assert len(acks) == 1 and acks[0]["ok"]
    assert acks[0]["result"] == {"brick": "right"}
    assert acks[0]["id"] == "1"
    master.on_mqtt("robot/cmd/right",
                   b'{"id": "2", "cmd": "run_target", "args": '
                   b'{"port": "outA", "target": 25.0}}')
    master.service_slave()
    slave.tick()
    master.service_slave()
    acks = ack_of(pubs, "robot/ack/right")
    assert len(acks) == 2 and acks[1]["ok"]
    assert abs(slave.core.motors["outA"].position() - 25.0) < 1e-9
    assert ack_of(pubs, "robot/state/right")  # slave state flows back


def test_master_estop_reaches_slave_over_hb_box():
    master, slave, pubs = make_pair()
    master.on_mqtt("robot/estop", b'{"stop": true}')
    assert master.core.motors["outA"].stopped
    hb = json.loads(master.mbox.hb.read())
    assert hb["estop"] is True
    slave.tick()
    assert slave.core.motors["outA"].stopped
    master.service_slave()
    evts = ack_of(pubs, "robot/event/right")
    assert any(e["event"] == "estop" for e in evts)


def test_slave_guard_cross_stops_master():
    master, slave, pubs = make_pair()
    slave.core.handle({"id": "0", "cmd": "set_guards",
                       "args": {"rules": [{"sensor": "in1"}]}})
    slave.core.sensors["in1"].set(1)
    slave.tick()
    master.service_slave()
    assert master.core.motors["outA"].stopped
    assert master.core.guards.latched
    assert any(e["event"] == "sensor_guard"
               for e in ack_of(pubs, "robot/event/right"))


def test_master_guard_forwards_stop_to_slave():
    master, slave, pubs = make_pair()
    master.core.handle({"id": "0", "cmd": "set_guards",
                        "args": {"rules": [{"sensor": "in1"}]}})
    master.core.sensors["in1"].set(1)
    master.on_mqtt("robot/cmd/left",
                   b'{"id": "1", "cmd": "run_target", "args": '
                   b'{"port": "outA", "target": 30.0}}')
    assert master.core.guards.latched
    assert json.loads(master.mbox.hb.read())["estop"] is True
    slave.tick()
    assert slave.core.motors["outA"].stopped


def test_master_heartbeat_watchdog():
    master, slave, pubs = make_pair()
    master.core.guards._last_heartbeat -= 5000
    master.pump()
    assert master.core.motors["outA"].stopped
    assert any(e["event"] == "heartbeat_timeout"
               for e in ack_of(pubs, "robot/event/left"))
    assert json.loads(master.mbox.hb.read())["estop"] is True


def test_slave_link_watchdog_and_recovery():
    master, slave, pubs = make_pair()
    slave._last_master_contact -= 10000
    slave.tick()
    assert slave.link_lost
    assert slave.core.motors["outA"].stopped
    slave.mbox.hb.send(json.dumps({"t": clock.now_ms(), "estop": False}))
    slave.tick()
    assert not slave.link_lost
    assert any(e["event"] == "bluetooth_loss" for e in slave._async
               + ([json.loads(slave.mbox.evt.read())]
                  if slave.mbox.evt.read() else []))


def test_slave_outbox_timeout_ack():
    master, slave, pubs = make_pair()
    master.in_flight = {"id": "x", "t0": clock.now_ms() - 70000}
    master.service_slave()
    acks = ack_of(pubs, "robot/ack/right")
    assert len(acks) == 1 and not acks[0]["ok"]
    assert master.in_flight is None


# -- static MicroPython constraints -------------------------------------------


def _brick_files():
    return sorted(BRICK_MP.glob("*.py"))


def test_version_markers_match_and_printed():
    ver = brick_version.BRICK_SW_VERSION
    assert ver and '"' not in ver and "'" not in ver
    files = [p for p in _brick_files() if p.name != "__init__.py"]
    assert len(files) == 12
    marker = "# brick-sw-version: " + ver
    for path in files:
        assert marker in path.read_text(), path.name
    for name in ("left_main.py", "right_main.py"):
        src = (BRICK_MP / name).read_text()
        assert "version.BRICK_SW_VERSION" in src, name


def test_brick_log_appends_and_never_raises(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    brick_log.log("hello brick")
    brick_log.log_line("second line")
    assert (tmp_path / "brick.log").read_text().splitlines() == [
        "hello brick", "second line"]


def test_mains_shebang_and_guard():
    for name in ("left_main.py", "right_main.py"):
        text = (BRICK_MP / name).read_text()
        first = text.split("\n")[0]
        assert first == "#!/usr/bin/env pybricks-micropython", name
        assert 'if __name__ == "__main__":' in text, name


def test_no_banned_syntax_or_imports():
    banned_anywhere = {"typing", "dataclasses", "threading"}
    for path in _brick_files():
        text = path.read_text()
        assert "->" not in text, "%s: arrow found" % path.name
        assert "yaml" not in text, "%s: yaml token found" % path.name
        tree = ast.parse(text, str(path))
        parents = {}

        def visit(node, stack):
            parents[node] = list(stack)
            for child in ast.iter_child_nodes(node):
                visit(child, stack + [node])

        visit(tree, [])
        for node in ast.walk(tree):
            assert not isinstance(node, (ast.JoinedStr, ast.FormattedValue)), (
                "%s: f-string found" % path.name)
            assert not isinstance(node, ast.AnnAssign), (
                "%s: annotation found" % path.name)
            if isinstance(node, ast.arguments):
                all_args = (list(node.args) + list(node.kwonlyargs)
                            + list(node.posonlyargs))
                if node.vararg is not None:
                    all_args.append(node.vararg)
                if node.kwarg is not None:
                    all_args.append(node.kwarg)
                assert not any(a.annotation is not None for a in all_args), (
                    "%s: arg annotation found" % path.name)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                assert node.returns is None, (
                    "%s: return annotation found" % path.name)
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split(".")[0]
                    assert root not in banned_anywhere, (
                        "%s: banned import %s" % (path.name, alias.name))
                    assert "yaml" not in alias.name, (
                        "%s: yaml import" % path.name)
                    if root in ("pybricks", "umqtt"):
                        assert any(isinstance(p, (ast.FunctionDef,
                                                 ast.AsyncFunctionDef))
                                   for p in parents[node]), (
                            "%s: hw import outside function" % path.name)
                    if root == "ujson":
                        assert any(isinstance(p, ast.Try)
                                   for p in parents[node]), (
                            "%s: ujson outside try" % path.name)
            if isinstance(node, ast.ImportFrom):
                mod = node.module or ""
                root = mod.split(".")[0]
                assert root not in banned_anywhere, (
                    "%s: banned from-import %s" % (path.name, mod))
                assert "yaml" not in mod, "%s: yaml import" % path.name
                if root in ("pybricks", "umqtt"):
                    assert any(isinstance(p, (ast.FunctionDef,
                                             ast.AsyncFunctionDef))
                               for p in parents[node]), (
                        "%s: hw import outside function" % path.name)
