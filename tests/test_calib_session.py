"""Tests for session calibration + digital twin (fake bricks only)."""

from __future__ import annotations

import json

import pytest

from src.console import calib_session as cs
from src.console import control as ctl
from src.console.mini import default_override_path, load_jog_map
from src.console.mini import read_jog_overrides
from src.ev3.fake import build_mp_transport
from src.kin.model import load_model


@pytest.fixture()
def handler(tmp_path, monkeypatch):
    transport, _, _ = build_mp_transport(full=True)
    h = ctl._Handler.__new__(ctl._Handler)
    h.transport = transport
    # Explicit wrist ports: the no-override default maps J5 and J6
    # onto the same port, which no real session uses (local
    # config/jogmap.local.json remaps them; same values here so the
    # tests run without that git-ignored file).
    over = read_jog_overrides(default_override_path())
    over = dict(over, **{
        "J4": {"brick": "left", "port": "outC"},
        "J5": {"brick": "left", "port": "outD"},
        "J6": {"brick": "right", "port": "outA"}})
    h.jog_map = load_jog_map(overrides=over)
    h.jog_overrides = {}
    h.override_path = str(tmp_path / "jogmap.local.json")
    h.direction_path = str(tmp_path / "direction.local.json")
    h.model_doc = load_model("models/robot_inferred.json")
    h.zeros = {}
    h.console_path = str(tmp_path / "console.local.json")
    h.virtual = True
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "stage5c")
    monkeypatch.setattr(cs, "SESSION_PATH", tmp_path / "session.json")
    yield h
    transport.close()


def test_boot_status_same_rebooted_unknown():
    assert cs.boot_status({"left": 100, "right": 100},
                          {"left": 200, "right": 150}) == "same"
    assert cs.boot_status({"left": 100, "right": 100},
                          {"left": 50, "right": 150}) == "rebooted"
    assert cs.boot_status({"left": 100},
                          {"left": 200, "right": None}) == "rebooted"
    assert cs.boot_status({}, {"left": 1}) == "unknown"
    assert cs.boot_status({"left": 1}, {}) == "unknown"


def test_session_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setattr(cs, "SESSION_PATH", tmp_path / "session.json")
    rec = cs.save_session({"boot": {"left": 12, "right": 34},
                           "offsets": {"J1": 1.0}})
    assert rec["saved_at"]
    back = cs.load_session()
    assert back["boot"] == {"left": 12, "right": 34}


def test_twin_slots_validate():
    from src.vla import twin as T

    with pytest.raises(ValueError):
        T.validate_slots({})
    with pytest.raises(ValueError):
        T.validate_slots({"S9": "red"})
    with pytest.raises(ValueError):
        T.validate_slots({"S1": "purple"})
    assert T.validate_slots({"S1": "red", "S6": "blue"}) == \
        {"S1": "red", "S6": "blue"}
    with pytest.raises(ValueError):
        T.scene_from_slots({"S1": "red"}, "blue")


def test_twin_run_fallback_png_and_hold(tmp_path, monkeypatch):
    from src.vla import twin as T

    monkeypatch.setattr(T, "TWIN_SLOTS", tmp_path / "twin_slots.json")
    monkeypatch.setattr(T, "DEFAULT_CKPT", tmp_path / "no-ckpt")
    out = T.run_twin({"S4": "red", "S2": "blue"}, "red")
    assert out["png"][:8] == b"\x89PNG\r\n\x1a\n"
    assert out["policy"] == "expert-fallback"
    assert out["target"] == "red"
    assert len(out["hold_q"]) == 6
    assert TWIN_SAVED(tmp_path)


def TWIN_SAVED(tmp_path):
    return (tmp_path / "twin_slots.json").is_file()


def test_twin_png_decodes():
    import struct
    import zlib

    from src.vla import twin as T

    raw = T.png_bytes(__import__("numpy").zeros((4, 4, 3),
                                                dtype=__import__("numpy")
                                                .uint8))
    assert raw[:8] == b"\x89PNG\r\n\x1a\n"
    # IHDR width/height round-trip.
    w, h = struct.unpack(">II", raw[16:24])
    assert (w, h) == (4, 4)
    assert zlib  # module used by the encoder


def test_console_twin_endpoints(handler):
    status, res = handler.dispatch_api("/api/twin/slots", {})
    assert status == 200 and res["ok"]
    assert "S1" in res["ids"] and "red" in res["colours"]
    status, res = handler.dispatch_api(
        "/api/twin/slots/set", {"slots": {"S4": "red"}})
    assert status == 200 and res["slots"] == {"S4": "red"}
    status, res = handler.dispatch_api(
        "/api/twin/run", {"slots": {"S4": "red", "S2": "blue"},
                          "target": "red"})
    assert status == 200 and res["ok"]
    assert res["policy"] == "expert-fallback"
    assert len(res["hold_q"]) == 6
    assert res["png"][:8] != ""


def test_console_session_park_check_confirm(handler):
    status, res = handler.dispatch_api("/api/calib/session", {})
    assert status == 200 and res["continuity"] == "unknown"
    status, res = handler.dispatch_api("/api/calib/park", {})
    assert status == 200 and res["ok"] and res["saved_at"]
    status, res = handler.dispatch_api("/api/calib/check", {})
    assert status == 200 and res["deviation"]["max_abs_deg"] is not None
    assert res["deviation"]["max_abs_deg"] <= cs.CONFIRM_TOL_DEG
    status, res = handler.dispatch_api("/api/calib/confirm", {})
    assert status == 200 and res["confirmed_at"]
    saved = json.loads(cs.SESSION_PATH.read_text(encoding="utf-8"))
    assert saved["confirmed"] is True


def test_console_flow_and_restore(handler):
    status, res = handler.dispatch_api("/api/calib/park", {})
    assert status == 200
    status, res = handler.dispatch_api("/api/calib/flow", {})
    assert status == 200 and res["ok"]
    kinds = [s["step"] for s in res["steps"]]
    assert kinds[0] == "continuity"
    assert res["flow"] == "done"
    assert any(s["step"] == "wrist" and s["result"] == "restored"
               for s in res["steps"])
    status, res = handler.dispatch_api("/api/calib/restore", {})
    assert status == 200 and res["continuity"] == "same"


def test_restore_refuses_after_reboot(handler, monkeypatch):
    status, _ = handler.dispatch_api("/api/calib/park", {})
    assert status == 200
    saved = cs.load_session()
    saved["boot"] = {"left": 10 ** 12, "right": 10 ** 12}
    cs.save_session(saved)
    status, res = handler.dispatch_api("/api/calib/restore", {})
    assert status == 400 and "need-recal" in res["error"]
