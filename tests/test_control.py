"""Tests for the 5C control console (fake bricks, no sockets)."""

from __future__ import annotations

import json
import pathlib
import urllib.request

import pytest

from src.console import control as ctl
from src.console.mini import load_jog_map
from src.ev3.fake import build_mp_transport
from src.kin.model import load_model


@pytest.fixture()
def handler(tmp_path):
    transport, _, _ = build_mp_transport(full=True)
    h = ctl._Handler.__new__(ctl._Handler)
    h.transport = transport
    h.jog_map = load_jog_map(overrides={})
    h.jog_overrides = {}
    h.override_path = str(tmp_path / "jogmap.local.json")
    h.direction_path = str(tmp_path / "direction.local.json")
    h.model_doc = load_model("models/robot_inferred.json")
    h.zeros = {}
    yield h
    transport.close()


def test_overview_shape(handler):
    status, res = handler.dispatch_api("/api/overview", {})
    assert status == 200 and res["ok"]
    assert set(res["latched"]) == {"left", "right"}
    assert len(res["joints"]) == 6
    first = res["joints"][0]
    assert first["joint"] == "J1"
    assert isinstance(first["motor_deg"], float)
    assert first["direction"] == "unconfirmed"


def test_unconfirmed_jog_capped_at_5(handler):
    status, res = handler.dispatch_api("/api/jog",
                                       {"joint": "J1", "delta": 30})
    assert status == 200 and res["ok"]
    assert res["joint_delta"] == 5.0


def test_direction_correct_confirms(handler):
    status, res = handler.dispatch_api(
        "/api/direction", {"joint": "J1", "verdict": "correct"})
    assert status == 200 and res["ok"]
    assert res["ratio"] == pytest.approx(-0.0857)
    status, res = handler.dispatch_api("/api/jog",
                                       {"joint": "J1", "delta": 30})
    assert res["joint_delta"] == 30.0


def test_direction_reversed_flips_sign_and_saves(handler, tmp_path):
    before = handler.jog_map["J2"]["ratio"]
    status, res = handler.dispatch_api(
        "/api/direction", {"joint": "J2", "verdict": "reversed"})
    assert status == 200 and res["ok"]
    assert res["ratio"] == pytest.approx(-before)
    saved = json.loads(pathlib.Path(
        handler.override_path).read_text(encoding="utf-8"))
    assert saved["J2"]["ratio"] == pytest.approx(-before)
    status, res = handler.dispatch_api("/api/directions", {})
    assert res["directions"]["J2"] == "confirmed"


def test_model_payload(handler):
    status, res = handler.dispatch_api("/api/model", {})
    assert status == 200 and res["ok"]
    assert len(res["joints"]) == 6
    assert len(res["envelopes"]) == 7
    carriers = {e["link"]: e["carrier"] for e in res["envelopes"]}
    assert carriers["D0"] is None
    assert carriers["D1"] == "J1"


def test_create_server_rejects_non_loopback():
    with pytest.raises(ValueError):
        ctl.create_server(transport=None, host="0.0.0.0")


def test_http_serves_page_and_vendored_three(tmp_path):
    transport, _, _ = build_mp_transport(full=True)
    try:
        try:
            server = ctl.create_server(transport, port=0,
                                   override_path=str(
                                       tmp_path / "j.json"),
                                   direction_path=str(
                                       tmp_path / "d.json"))
        except OSError as exc:
            pytest.skip(f"socket bind blocked: {exc}")
        host, port = server.server_address
        assert host == "127.0.0.1"
        import threading

        thread = threading.Thread(target=server.serve_forever,
                                  daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{port}"
            with urllib.request.urlopen(base + "/") as r:
                assert r.status == 200
                assert "5C Console" in r.read().decode()
            with urllib.request.urlopen(
                    base + "/static/three/build/three.module.js") as r:
                assert r.status == 200
                assert len(r.read()) > 100000
        finally:
            server.shutdown()
            thread.join()
            server.server_close()
    finally:
        transport.close()
