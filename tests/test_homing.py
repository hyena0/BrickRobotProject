"""Tests for the homing planner/runner (fake bricks, no sockets)."""

from __future__ import annotations

import json

import pytest
import yaml

from src.console.mini import load_jog_map
from src.ev3.fake import build_mp_transport
from src.homing import plan as hp


def _inputs():
    jog_map = load_jog_map()
    sensors = json.loads(open("runs/stage5/sensors.json").read())
    bounds = json.loads(open("runs/stage5a/sensor_bounds.json").read())
    with open("config/ports.local.yaml") as fh:
        ports = yaml.safe_load(fh)
    return jog_map, sensors, bounds, ports


def _cfg():
    return {"joints": {
        "J1": {"method": "sensor", "sensor": "in1",
               "direction": -1, "speed": 30.0},
        "J2": {"method": "sensor", "sensor": "in1",
               "direction": -1, "speed": 30.0},
        "J3": {"method": "sensor", "sensor": "in2",
               "direction": -1, "speed": 30.0},
        "J4": {"method": "sensor", "sensor": "in3",
               "direction": -1, "speed": 30.0},
        "J5": {"method": "stall", "direction": 1, "speed": 20.0},
        "J6": {"method": "manual"},
    }}


def test_build_plan_trip_angles_and_cross_brick_skip():
    jog_map, sensors, bounds, ports = _inputs()
    plan, errors, warnings = hp.build_plan(_cfg(), jog_map, sensors,
                                           bounds, ports)
    assert errors == []
    by_id = {item["joint"]: item for item in plan["joints"]}
    assert by_id["J1"]["trip_joint"] == 44.0
    assert by_id["J2"]["trip_joint"] == 51.0
    assert by_id["J3"]["trip_joint"] == 26.0
    assert by_id["J4"]["status"] == "skipped"
    assert any("J4" in warn for warn in warnings)
    assert by_id["J5"]["trip_joint"] is None


def test_build_plan_rejects_bad_config_before_any_motion():
    jog_map, sensors, bounds, ports = _inputs()
    cfg = _cfg()
    joints = cfg["joints"]
    joints["J2"] = {"method": "sensor", "sensor": "",
                    "direction": -1, "speed": 30.0}
    joints["J3"] = {"method": "sensor", "sensor": "in9",
                    "direction": -1, "speed": 30.0}
    joints["J5"] = {"method": "stall", "direction": 0, "speed": 20.0}
    joints["J6"] = {"method": "sensor", "sensor": "in1",
                    "direction": -1, "speed": 30.0}
    joints["J9"] = {"method": "manual"}
    del joints["J1"]
    _, errors, _ = hp.build_plan(cfg, jog_map, sensors, bounds, ports)
    text = " ".join(errors)
    assert "J1: missing" in text
    assert "J2" in text and "sensor" in text
    assert "J3" in text
    assert "J5" in text and "direction" in text
    assert "J6" in text
    assert "J9" in text and "unknown joint" in text


def test_build_plan_rejects_bad_speed():
    jog_map, sensors, bounds, ports = _inputs()
    cfg = _cfg()
    cfg["joints"]["J1"]["speed"] = 400.0
    _, errors, _ = hp.build_plan(cfg, jog_map, sensors, bounds, ports)
    assert any("J1" in err and "speed" in err for err in errors)


def test_execute_plan_on_fake_bricks(tmp_path):
    jog_map, sensors, bounds, ports = _inputs()
    plan, errors, _ = hp.build_plan(_cfg(), jog_map, sensors, bounds,
                                    ports)
    assert errors == []
    transport, left, right = build_mp_transport(full=True)
    try:
        hp.arm_fake_sensors(left, right)
        record = hp.execute_plan(plan, transport, confirm=lambda i: True)
    finally:
        transport.close()
    by_id = {res["joint"]: res for res in record["results"]}
    assert by_id["J1"]["status"] == "homed"
    assert by_id["J1"]["offset"] == pytest.approx(44.0)
    assert by_id["J2"]["offset"] == pytest.approx(51.0)
    assert by_id["J3"]["offset"] == pytest.approx(26.0)
    assert by_id["J4"]["status"] == "skipped"
    assert by_id["J5"]["status"] == "no_stall"
    assert by_id["J5"]["offset"] is None
    assert by_id["J6"]["status"] == "homed"
    # J5 and J6 share one motor: the stall run parked it at 360.
    assert by_id["J6"]["offset"] == pytest.approx(0.1429 * 360.0)
    out = hp.save_run(record, plan, [], str(tmp_path))
    assert (out / "homing.json").exists()
    assert (out / "homing.md").exists()
    saved = json.loads((out / "homing.json").read_text())
    assert saved["offsets"]["J2"] == pytest.approx(51.0)


def test_only_runs_selected_joints(tmp_path, capsys):
    probe = tmp_path / "homing.yaml"
    probe.write_text("joints:\n"
                     "  J1: {method: manual}\n"
                     "  J2: {method: manual}\n"
                     "  J3: {method: manual}\n"
                     "  J4: {method: manual}\n"
                     "  J5: {method: manual}\n"
                     "  J6: {method: manual}\n")
    out = tmp_path / "out"
    hp.main(["--homing", str(probe), "--auto", "--only", "J6",
             "--out", str(out)])
    saved = json.loads(next(out.glob("*/homing.json")).read_text())
    assert saved["mode"] == "fake"
    assert saved["only"] == ["J6"]
    assert [res["joint"] for res in saved["results"]] == ["J6"]
    with pytest.raises(SystemExit):
        hp.main(["--homing", str(probe), "--auto", "--only", "J9",
                 "--out", str(out)])
    assert "unknown joint" in capsys.readouterr().out


def _manual_only_plan():
    jog_map, sensors, bounds, ports = _inputs()
    plan, errors, _ = hp.build_plan({"joints": {
        "J1": {"method": "manual"}, "J2": {"method": "manual"},
        "J3": {"method": "manual"}, "J4": {"method": "manual"},
        "J5": {"method": "manual"}, "J6": {"method": "manual"}}},
        jog_map, sensors, bounds, ports)
    assert errors == []
    return {"joints": [item for item in plan["joints"]
                       if item["joint"] == "J6"]}


def _jog_closure(transport):
    return hp.make_jog_move(transport)


def test_jog_move_clears_stale_latch_first():
    plan = _manual_only_plan()
    transport, left, _ = build_mp_transport(full=True)
    try:
        left.guards.latched = True
        left.guards.latch_reason = "stall:outC"
        jog_move = hp.make_jog_move(transport)
        pos = jog_move(plan["joints"][0], 5.0)
        assert not left.guards.latched
        assert pos == pytest.approx(5.0 / -0.1429)
    finally:
        transport.close()


def test_manual_inline_jog_and_mark(monkeypatch):
    plan = _manual_only_plan()
    transport, _, _ = build_mp_transport(full=True)
    try:
        answers = iter(["+5", ""])
        monkeypatch.setattr("builtins.input",
                            lambda prompt="": next(answers))
        record = hp.execute_plan(plan, transport,
                                 jog_move=_jog_closure(transport))
    finally:
        transport.close()
    (res,) = record["results"]
    assert res["status"] == "homed"
    assert res["offset"] == pytest.approx(-5.0, abs=0.01)


def test_manual_skip_and_bad_input(monkeypatch):
    plan = _manual_only_plan()
    transport, _, _ = build_mp_transport(full=True)
    try:
        answers = iter(["q"])
        monkeypatch.setattr("builtins.input",
                            lambda prompt="": next(answers))
        record = hp.execute_plan(plan, transport,
                                 jog_move=_jog_closure(transport))
        assert record["results"][0]["status"] == "declined"
        answers = iter(["abc", ""])
        monkeypatch.setattr("builtins.input",
                            lambda prompt="": next(answers))
        record = hp.execute_plan(plan, transport,
                                 jog_move=_jog_closure(transport))
        assert record["results"][0]["status"] == "homed"
        assert record["results"][0]["offset"] == pytest.approx(0.0)
    finally:
        transport.close()


class _ClearingSensor:
    """Pressed until the motor moves away, like a real sensor.

    set(1)/set(0) holds/releases a pressed state like a finger on
    the sensor (used by the auto-trip hook during homing runs).
    """

    def __init__(self, motor=None):
        self.motor = motor
        self.start = None
        self.held = False

    def read(self):
        if self.held:
            return 1
        if self.motor is not None:
            pos = self.motor.position()
            if self.start is None:
                self.start = pos
            if abs(pos - self.start) > 1.0:
                return 0
        return 1

    def set(self, value):
        self.held = bool(value)


def test_backoff_clears_pressed_sensor():
    transport, left, _ = build_mp_transport(full=True)
    try:
        left.sensors["in1"] = _ClearingSensor(left.motors["outA"])
        assert hp._back_off(transport, "left", "outA", "in1", -1, -0.0238) is True
        left.sensors["in1"].set(1)
        assert hp._back_off(transport, "left", "outA", "in1", -1, -0.0238) is None
    finally:
        transport.close()


def test_pressed_sensor_backs_off_then_homes():
    jog_map, sensors, bounds, ports = _inputs()
    plan, errors, _ = hp.build_plan(_cfg(), jog_map, sensors, bounds,
                                    ports)
    assert errors == []
    j2 = {"joints": [item for item in plan["joints"]
                     if item["joint"] == "J2"]}
    transport, left, right = build_mp_transport(full=True)
    try:
        left.sensors["in1"] = _ClearingSensor(left.motors["outA"])
        hp.arm_fake_sensors(left, right)
        record = hp.execute_plan(j2, transport, confirm=lambda i: True)
    finally:
        transport.close()
    (res,) = record["results"]
    assert res["status"] == "homed"
    assert res["backed_off"] is True
    assert res["offset"] == pytest.approx(52.0, abs=2.0)


def test_stuck_sensor_fails_without_motion():
    jog_map, sensors, bounds, ports = _inputs()
    plan, errors, _ = hp.build_plan(_cfg(), jog_map, sensors, bounds,
                                    ports)
    assert errors == []
    j2 = {"joints": [item for item in plan["joints"]
                     if item["joint"] == "J2"]}
    transport, left, _ = build_mp_transport(full=True)
    try:
        left.sensors["in1"].set(1)
        record = hp.execute_plan(j2, transport, confirm=lambda i: True)
    finally:
        transport.close()
    assert record["results"][0]["status"] == "failed"
    assert "still pressed" in record["results"][0]["error"]


def test_stepping_stops_after_travel_cap():
    jog_map, sensors, bounds, ports = _inputs()
    cfg = {"joints": {
        "J1": {"method": "manual"}, "J2": {"method": "sensor",
                                           "sensor": "in1",
                                           "direction": -1,
                                           "speed": 30.0},
        "J3": {"method": "manual"}, "J4": {"method": "manual"},
        "J5": {"method": "manual"}, "J6": {"method": "manual"}}}
    full, errors, _ = hp.build_plan(cfg, jog_map, sensors, bounds,
                                    ports)
    assert errors == []
    j2 = {"joints": [item for item in full["joints"]
                     if item["joint"] == "J2"]}
    transport, _, _ = build_mp_transport(full=True)
    try:
        record = hp.execute_plan(j2, transport, confirm=lambda i: True)
    finally:
        transport.close()
    (res,) = record["results"]
    assert res["status"] == "failed"
    assert "no trip after 30" in res["error"]


def test_save_run_merges_same_day(tmp_path):
    out = tmp_path / "runs"
    hp.save_run({"results": [{"joint": "J2", "status": "homed",
                              "offset": 45.0}],
                 "offsets": {"J2": 45.0}}, {"joints": []}, [], str(out))
    hp.save_run({"results": [{"joint": "J1", "status": "homed",
                              "offset": 44.0}],
                 "offsets": {"J1": 44.0}}, {"joints": []}, ["w2"],
                str(out))
    saved = json.loads(next(out.glob("*/homing.json")).read_text())
    assert saved["offsets"] == {"J2": 45.0, "J1": 44.0}
    assert {res["joint"] for res in saved["results"]} == {"J1", "J2"}
    assert saved["warnings"] == ["w2"]


def _assisted_cfg():
    return {"joints": {
        "J1": {"method": "manual"}, "J2": {"method": "assisted",
                                           "sensor": "in1",
                                           "direction": -1, "speed": 30.0},
        "J3": {"method": "manual"}, "J4": {"method": "manual"},
        "J5": {"method": "manual"}, "J6": {"method": "manual"}}}


def _assisted_plan():
    jog_map, sensors, bounds, ports = _inputs()
    plan, errors, _ = hp.build_plan(_assisted_cfg(), jog_map,
                                    sensors, bounds, ports)
    assert errors == []
    return {"joints": [item for item in plan["joints"]
                       if item["joint"] == "J2"]}


def test_assisted_jog_onto_sensor_then_mark(monkeypatch):
    plan = _assisted_plan()
    transport, left, _ = build_mp_transport(full=True)
    try:
        answers = iter(["+2", ""])

        def fake_input(prompt=""):
            value = next(answers)
            if value == "":
                left.sensors["in1"].set(1)
            return value

        monkeypatch.setattr("builtins.input", fake_input)
        record = hp.execute_plan(plan, transport,
                                 jog_move=hp.make_jog_move(transport))
    finally:
        transport.close()
    (res,) = record["results"]
    assert res["status"] == "homed"
    assert res["method"] == "assisted"
    assert res["offset"] == pytest.approx(49.0, abs=0.5)


def test_assisted_pre_pressed_homes_without_prompt(monkeypatch):
    plan = _assisted_plan()
    transport, left, _ = build_mp_transport(full=True)
    try:
        left.sensors["in1"].set(1)

        def boom(prompt=""):
            raise AssertionError("should not prompt")

        monkeypatch.setattr("builtins.input", boom)
        record = hp.execute_plan(plan, transport,
                                 jog_move=hp.make_jog_move(transport))
    finally:
        transport.close()
    assert record["results"][0]["status"] == "homed"


def test_assisted_enter_requires_pressed_sensor(monkeypatch):
    plan = _assisted_plan()
    transport, _, _ = build_mp_transport(full=True)
    try:
        answers = iter(["", "q"])
        monkeypatch.setattr("builtins.input",
                            lambda prompt="": next(answers))
        record = hp.execute_plan(plan, transport,
                                 jog_move=hp.make_jog_move(transport))
    finally:
        transport.close()
    assert record["results"][0]["status"] == "declined"


def test_assisted_auto_steps_and_times_out():
    plan = _assisted_plan()
    transport, _, _ = build_mp_transport(full=True)
    try:
        record = hp.execute_plan(plan, transport)
    finally:
        transport.close()
    (res,) = record["results"]
    assert res["status"] == "failed"
    assert "direction" in res["error"]


def test_assisted_allows_cross_brick():
    jog_map, sensors, bounds, ports = _inputs()
    cfg = {"joints": {
        "J1": {"method": "manual"}, "J2": {"method": "manual"},
        "J3": {"method": "manual"},
        "J4": {"method": "assisted", "sensor": "in3",
               "direction": -1, "speed": 30.0},
        "J5": {"method": "manual"}, "J6": {"method": "manual"}}}
    plan, errors, _ = hp.build_plan(cfg, jog_map, sensors, bounds,
                                    ports)
    assert errors == []
    j4 = [item for item in plan["joints"] if item["joint"] == "J4"]
    assert j4[0]["status"] == "ready"
    assert j4[0]["sensor_brick"] == "left"


def test_markzero_records_current_as_zero(api=None):
    from src.console.mini import _Handler

    transport, _, _ = build_mp_transport(full=True)
    try:
        handler = _Handler.__new__(_Handler)
        handler.transport = transport
        handler.jog_map = load_jog_map()
        handler.jog_overrides = {}
        handler.zeros = None
        status, res = handler.dispatch_api("/api/markzero",
                                           {"joint": "J1"})
        assert status == 200 and res["ok"]
        ratio = load_jog_map()["J1"]["ratio"]
        assert res["offset"] == pytest.approx(-ratio * res[
            "motor_position"])
    finally:
        transport.close()
