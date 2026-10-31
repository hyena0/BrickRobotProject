"""Tests for the minimal test console (fake bricks, no sockets)."""

from __future__ import annotations

import json

import pytest

from src.console.mini import (PAGE, _Handler, load_jog_map,
                                read_jog_overrides)
from src.ev3.fake import build_mp_transport


@pytest.fixture()
def api(tmp_path):
    transport, left, right = build_mp_transport(full=True)
    handler = _Handler.__new__(_Handler)
    handler.transport = transport
    handler.jog_map = load_jog_map(overrides={})
    handler.jog_overrides = {}
    handler.override_path = str(tmp_path / "jogmap.local.json")
    handler.zeros = {}
    def fn(path, args):
        return handler.dispatch_api(path, args)

    fn.left_core = left
    fn.right_core = right
    fn.transport = transport
    yield fn
    transport.close()


def test_page_has_buttons():
    assert "Brick link test" in PAGE
    for marker in ("data-act=\"ping\"", "data-act=\"jog+\"",
                   "data-act=\"diag\"", "data-act=\"stop\"",
                   "data-act=\"estop\"", "id=\"joint\"",
                   "data-act=\"map-set\"", "data-act=\"map-show\"",
                   "data-act=\"map-reset\"", "id=\"ratio\"",
                   "data-act=\"clear\"", "data-act=\"home\"",
                   "id=\"sensor\"", "id=\"direction\"",
                   "data-act=\"markzero\"", "data-act=\"map-del\""):
        assert marker in PAGE


def test_ping_info_state(api):
    status, ping = api("/api/ping", {})
    assert status == 200 and ping["ok"]
    assert set(ping["ping"]) == {"left", "right"}
    status, info = api("/api/info", {})
    assert status == 200 and info["ok"]
    motors = sum(len(b["motors"]) for b in info["info"].values())
    assert motors == 6
    status, state = api("/api/state", {})
    assert status == 200 and state["ok"]
    assert set(state["state"]) == {"left", "right"}


def test_jog_clamps_to_joint_limit_and_shows_motor(api):
    status, res = api("/api/jog", {"joint": "J2", "delta": 40})
    assert status == 200 and res["ok"]
    assert res["joint"] == "J2" and res["joint_delta"] == 30.0
    assert res["brick"] == "left" and res["port"] == "outA"
    assert abs(res["ratio"] - 0.0238) < 1e-9
    assert abs(res["motor_delta"] - 30.0 / 0.0238) < 1e-6
    assert res["speed"] == 250.0
    assert res["executed"] is True
    assert abs(res["moved"] - res["motor_delta"]) < 3.0


def test_jog_applies_gear_ratio(api):
    status, res = api("/api/jog", {"joint": "J6", "delta": 5})
    assert status == 200 and res["ok"]
    assert res["brick"] == "left" and res["port"] == "outC"
    assert abs(res["ratio"] - 0.1429) < 1e-9
    assert abs(res["motor_delta"] - 5.0 / 0.1429) < 1e-6
    assert res["executed"] is True
    assert abs(res["moved"] - res["motor_delta"]) < 3.0


def test_jog_below_minimum_move_warns(api):
    status, res = api("/api/jog", {"joint": "J1", "delta": 0.5})
    assert status == 200 and res["ok"]
    assert res["executed"] is False
    assert abs(res["motor_delta"] - 0.5 / 0.0857) < 1e-6
    assert "warning" in res and "not executed" in res["warning"]


def test_jog_sends_to_exactly_one_motor(api, monkeypatch):
    calls = []
    orig = api.transport.send

    def rec(brick, cmd=None, args=None, timeout_s=None, **kw):
        name = cmd if isinstance(cmd, str) else cmd.get("cmd")
        calls.append((brick, name,
                      dict(args or ({} if isinstance(cmd, str)
                                    else cmd.get("args", {})))))
        if timeout_s is None:
            return orig(brick, cmd, args, **kw)
        return orig(brick, cmd, args, timeout_s, **kw)

    monkeypatch.setattr(api.transport, "send", rec)
    status, res = api("/api/jog", {"joint": "J1", "delta": 5})
    assert status == 200 and res["ok"] and res["executed"]
    status, res = api("/api/jog", {"joint": "J2", "delta": 5})
    assert status == 200 and res["ok"] and res["executed"]
    runs = [(brick, args["port"]) for brick, name, args in calls
            if name == "run_target"]
    assert runs == [("right", "outB"), ("left", "outA")]


def test_diag_runs_full_turn(api):
    status, res = api("/api/diag", {"brick": "left", "port": "outA"})
    assert status == 200 and res["ok"]
    assert abs(res["before"]) < 1e-9
    assert abs(res["after"] - 360.0) < 3.0
    assert abs(res["moved"] - 360.0) < 3.0


def test_wrist_coupling_roundtrip():
    from src.console.mini import (coupled_joint_readback,
                                  coupled_motor_targets)
    jog_map = {
        "J1": {"brick": "right", "port": "outB", "ratio": 0.0857},
        "J2": {"brick": "left", "port": "outA", "ratio": 0.0238},
        "J3": {"brick": "left", "port": "outB", "ratio": 0.0343},
        "J4": {"brick": "left", "port": "outC", "ratio": 0.2},
        "J5": {"brick": "left", "port": "outD", "ratio": 0.2},
        "J6": {"brick": "right", "port": "outA", "ratio": 0.1429}}
    q = {"J1": 10.0, "J2": -5.0, "J3": 3.0,
         "J4": 7.0, "J5": -4.0, "J6": 2.0}
    motors = coupled_motor_targets(q, jog_map)
    assert set(motors) == {("right", "outB"), ("left", "outA"),
                           ("left", "outB"), ("left", "outC"),
                           ("left", "outD"), ("right", "outA")}
    back = coupled_joint_readback(motors, jog_map)
    for j, v in q.items():
        assert back[j] == pytest.approx(v)
    # J4-only command holds J5/J6: compensation on their motors.
    solo = coupled_motor_targets(
        {"J1": 0.0, "J2": 0.0, "J3": 0.0,
         "J4": 5.0, "J5": 0.0, "J6": 0.0}, jog_map)
    assert solo[("left", "outC")] == pytest.approx(25.0)
    assert solo[("left", "outD")] == pytest.approx(5.0)
    assert solo[("right", "outA")] == pytest.approx(-25.0 / 7.0)


def test_j4_jog_drives_compensation_motors(api, monkeypatch):
    seen = []
    orig = api.transport.send

    def rec(brick, cmd=None, args=None, timeout_s=None, **kw):
        name = cmd if isinstance(cmd, str) else cmd.get("cmd")
        if name == "move_motors" or (isinstance(cmd, dict)
                                     and cmd.get("cmd") == "move_motors"):
            seen.append((brick, dict(args or {})))
        return orig(brick, cmd, args, **kw)

    monkeypatch.setattr(api.transport, "send", rec)
    status, res = api("/api/jog", {"joint": "J4", "delta": 30})
    assert status == 200 and res["ok"] and res["executed"]
    # Model map shares one motor for J5/J6: net compensation
    # (0.2 - 0.1427) * 150 = 8.6 deg falls below the min-move gate.
    assert res["motor_delta"] == pytest.approx(150.0)
    assert res["coupled"] == []
    # J1-J3 rows are identity: a J1 jog still moves exactly one motor.
    status, res = api("/api/jog", {"joint": "J1", "delta": 5})
    assert status == 200 and res["ok"] and res["coupled"] == []


def test_jogmap_get_and_set_ratio(api, tmp_path):
    status, res = api("/api/jogmap", {})
    assert status == 200 and res["ok"]
    assert set(res["jog_map"]) == {"J1", "J2", "J3", "J4", "J5", "J6"}
    assert abs(res["jog_map"]["J2"]["ratio"] - 0.0238) < 1e-9
    status, res = api("/api/jogmap/set",
                      {"joint": "J2", "ratio": 0.5})
    assert status == 200 and res["ok"]
    assert res["entry"]["ratio"] == 0.5
    status, res = api("/api/jog", {"joint": "J2", "delta": 5})
    assert status == 200 and res["ok"]
    assert abs(res["motor_delta"] - 10.0) < 1e-9
    assert abs(res["moved"] - 10.0) < 3.0
    saved = json.loads((tmp_path / "jogmap.local.json").read_text())
    assert saved == {"J2": {"ratio": 0.5}}


def test_jogmap_set_port_redirects_jog(api):
    status, res = api("/api/jogmap/set",
                      {"joint": "J2", "brick": "right",
                       "port": "outA", "ratio": 0.5})
    assert status == 200 and res["ok"]
    assert res["entry"]["brick"] == "right"
    assert res["entry"]["port"] == "outA"
    status, res = api("/api/jog", {"joint": "J2", "delta": 5})
    assert res["brick"] == "right" and res["port"] == "outA"
    assert abs(res["motor_delta"] - 10.0) < 1e-9
    assert abs(res["moved"] - 10.0) < 3.0


def test_jogmap_set_rejects_bad_values(api):
    for bad in ({"joint": "J9", "ratio": 1.0},
                {"joint": "J2", "ratio": 0.0},
                {"joint": "J2", "ratio": "x"},
                {"joint": "J2", "brick": "middle"},
                {"joint": "J2", "port": "outZ"}):
        status, obj = api("/api/jogmap/set", bad)
        assert status == 400 and not obj["ok"], bad
    status, res = api("/api/jogmap", {})
    assert abs(res["jog_map"]["J2"]["ratio"] - 0.0238) < 1e-9
    assert res["overrides"] == {}


def test_jogmap_delete_removes_single_override(api):
    status, res = api("/api/jogmap/set",
                      {"joint": "J1", "brick": "left",
                       "port": "outA", "ratio": 1.0})
    assert status == 200 and res["ok"]
    assert res["entry"]["brick"] == "left"
    status, res = api("/api/jogmap/delete", {"joint": "J1"})
    assert status == 200 and res["ok"]
    assert res["overrides"] == {}
    entry = res["jog_map"]["J1"]
    assert entry["brick"] == "right" and entry["port"] == "outB"
    assert abs(entry["ratio"] - 0.0857) < 1e-9


def test_jogmap_reset_restores_model(api, tmp_path):
    api("/api/jogmap/set", {"joint": "J2", "ratio": 0.5})
    status, res = api("/api/jogmap/reset", {})
    assert status == 200 and res["ok"]
    assert abs(res["jog_map"]["J2"]["ratio"] - 0.0238) < 1e-9
    assert res["overrides"] == {}
    assert not (tmp_path / "jogmap.local.json").exists()


def test_load_jog_map_skips_invalid_overrides():
    jog_map = load_jog_map(overrides={"J2": {"ratio": 0.0},
                                      "J9": {"ratio": 1.0},
                                      "J3": {"ratio": 2.0}})
    assert abs(jog_map["J2"]["ratio"] - 0.0238) < 1e-9
    assert jog_map["J3"]["ratio"] == 2.0
    assert "J9" not in jog_map


def test_jogmap_overrides_survive_reload(tmp_path):
    path = tmp_path / "jogmap.local.json"
    path.write_text(json.dumps({"J6": {"ratio": 2.0,
                                      "port": "outD"}}))
    jog_map = load_jog_map(overrides=read_jog_overrides(str(path)))
    assert jog_map["J6"]["ratio"] == 2.0
    assert jog_map["J6"]["port"] == "outD"
    assert abs(jog_map["J2"]["ratio"] - 0.0238) < 1e-9


def test_stop_and_estop(api):
    assert api("/api/stop", {}) == (200, {"ok": True})
    assert api("/api/estop", {}) == (200, {"ok": True})


def test_home_finds_pressed_sensor(api):
    api.left_core.sensors["in1"].set(1)
    status, res = api("/api/home", {"joint": "J2", "sensor": "in1",
                                    "direction": -1})
    assert status == 200 and res["ok"]
    assert res["homed"] is True
    assert res["brick"] == "left" and res["port"] == "outA"
    assert res["sensor"] == "in1" and res["direction"] == -1


def test_home_misses_without_sensor(api):
    status, res = api("/api/home", {"joint": "J2", "sensor": "in1",
                                    "direction": -1})
    assert status == 200 and res["ok"]
    assert res["homed"] is False
    assert abs(res["position"] - -360.0) < 3.0


def test_home_rejects_bad_args(api):
    for bad in ({"joint": "J2", "sensor": "in1", "direction": 0},
                {"joint": "J2", "sensor": "in1", "direction": "x"},
                {"joint": "J2", "direction": -1},
                {"sensor": "in1", "direction": -1}):
        status, obj = api("/api/home", bad)
        assert status == 400 and not obj["ok"], bad


def test_clear_latch_releases_brick(api):
    api.left_core.guards.latched = True
    api.left_core.guards.latch_reason = "stall:outA"
    assert api("/api/jog", {"joint": "J2", "delta": 5})[0] == 500
    status, res = api("/api/clear", {})
    assert status == 200 and res["ok"]
    assert not api.left_core.guards.latched
    status, res = api("/api/jog", {"joint": "J2", "delta": 5})
    assert status == 200 and res["ok"]
    assert abs(res["moved"] - res["motor_delta"]) < 3.0


def test_unknown_endpoint_and_bad_args(api):
    status, obj = api("/api/nope", {})
    assert status == 400 and not obj["ok"]
    status, obj = api("/api/jog", {"brick": "left"})
    assert status == 400 and not obj["ok"]
    status, obj = api("/api/jog", {"joint": "J9", "delta": 5})
    assert status == 400 and not obj["ok"]
