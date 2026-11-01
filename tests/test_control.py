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
    h.console_path = str(tmp_path / "console.local.json")
    yield h
    transport.close()


def test_gt_frames_needs_private_visuals(handler):
    q = {j: 0.0 for j in
         ["J1", "J2", "J3", "J4", "J5", "J6"]}
    status, res = handler.dispatch_api("/api/gt_frames", {"q_deg": q})
    assert status == 200 and res["ok"] is False
    handler.private_visuals = "robot_inferred"
    status, res = handler.dispatch_api("/api/gt_frames", {"q_deg": q})
    assert status == 200 and res["ok"]
    assert len(res["points"]) == 8


def test_reachable_target_is_fk_consistent(handler):
    from src.kin.fk import fk_model
    from src.console.control import tip_doc
    status, res = handler.dispatch_api("/api/reachable_target",
                                       {"seed": 7})
    assert status == 200 and res["ok"]
    q = [float(v) for v in res["q_deg"]]
    pos = fk_model(q, tip_doc(handler.model_doc))["pos_mm"]
    assert all(abs(a - b) < 1.0
               for a, b in zip(pos, res["target"]))


def test_ik_preview_shows_motors_and_range(handler, tmp_path,
                                            monkeypatch):
    from src.console.mini import read_jog_overrides, default_override_path
    handler.jog_map = load_jog_map(
        overrides=read_jog_overrides(default_override_path()))
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    q = {j: 0.0 for j in
         ["J1", "J2", "J3", "J4", "J5", "J6"]}
    status, res = handler.dispatch_api("/api/preview", {"q_deg": q})
    assert status == 200 and res["ok"]
    assert res["range"] == "ok"
    assert len(res["motors"]) == 6
    status, res = handler.dispatch_api(
        "/api/preview", {"q_deg": dict(q, J1=100.0)})
    assert status == 200 and res["ok"] and res["range"] != "ok"


def test_viewer_background_contrast_for_dark_bricks():
    # The model is mostly black bricks (LDraw colour 0); a near-black
    # scene background makes them look transparent/floating.
    assert "0x111111" not in ctl.PAGE
    assert "HemisphereLight" in ctl.PAGE


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
    assert res["ratio"] == pytest.approx(0.0857)
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
    assert "Z" in res["frame"]


def test_page_initial_pose_and_up_axis():
    # Initial 3D view: assembled pose01 (io render reference),
    # Z-up model under a -90 deg X group, boxes rested on the grid.
    assert "6.595" in ctl.PAGE and "138.148" in ctl.PAGE
    assert "arm.rotation.x = -Math.PI / 2" in ctl.PAGE
    assert "setFromObject(st.arm)" in ctl.PAGE
    # Regression: the box-mesh era is over; every identifier the
    # script touches must be declared (glerr showed LINKS missing).
    assert "LINKS" not in ctl.PAGE


def test_create_server_rejects_non_loopback():
    with pytest.raises(ValueError):
        ctl.create_server(transport=None, host="0.0.0.0")


def test_overview_has_sensors(handler):
    status, res = handler.dispatch_api("/api/overview", {})
    by_joint = {j["joint"]: j for j in res["joints"]}
    assert by_joint["J1"]["sensor"] == "in1"
    assert by_joint["J5"]["sensor"] is None


def test_config_get_set(handler):
    status, res = handler.dispatch_api("/api/config", {})
    assert status == 200
    assert res["tunables"]["wizard_step_deg"] == 5.0
    assert res["package"]["operating_margin_deg"] == 5.0
    status, res = handler.dispatch_api(
        "/api/config/set", {"key": "measure_margin_deg",
                            "value": 3.0})
    assert status == 200
    assert res["tunables"]["measure_margin_deg"] == 3.0
    status, res = handler.dispatch_api(
        "/api/config/set", {"key": "nope", "value": 1.0})
    assert status == 400


def test_ranges_payload(handler, tmp_path, monkeypatch):
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    status, res = handler.dispatch_api("/api/ranges", {})
    assert status == 200
    assert len(res["joints"]) == 6
    j1 = next(j for j in res["joints"] if j["joint"] == "J1")
    assert j1["sim"] == [-50.0, 39.0]
    assert j1["adopted"] == [-50.0, 39.0]
    assert j1["adopted_source"] == "sim"


def test_record_range_adopted(handler, tmp_path, monkeypatch):
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    status, res = handler.dispatch_api(
        "/api/range", {"joint": "J1", "end": "lo",
                       "joint_deg": -40.0})
    assert status == 200
    j1 = next(j for j in res["joints"] if j["joint"] == "J1")
    assert j1["measured"] == [-40.0, None]
    assert j1["adopted"][0] == pytest.approx(-35.0)  # -40 + margin 5
    status, res = handler.dispatch_api(
        "/api/range", {"joint": "J9", "end": "lo",
                       "joint_deg": 0.0})
    assert status == 400


def test_poses_list(handler, tmp_path, monkeypatch):
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    status, res = handler.dispatch_api("/api/poses", {})
    assert status == 200
    assert len(res["poses"]) == 10
    assert res["poses"][0]["q_deg"] == [0.0] * 6


def test_pose_reproduce_fake(handler, tmp_path, monkeypatch):
    from src.console.mini import read_jog_overrides, default_override_path
    handler.jog_map = load_jog_map(
        overrides=read_jog_overrides(default_override_path()))
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    status, res = handler.dispatch_api("/api/pose",
                                       {"pose": "safe01"})
    assert status == 200 and res["ok"]
    assert res["max_abs_err"] == pytest.approx(0.0, abs=1e-6)
    status, res = handler.dispatch_api("/api/pose",
                                       {"pose": "nope"})
    assert status == 400


def test_mat_points_gated(handler):
    status, res = handler.dispatch_api("/api/mat_points", {})
    assert status == 200 and res["ok"] is False
    handler.private_visuals = "robot_inferred"
    status, res = handler.dispatch_api("/api/mat_points", {})
    assert status == 200 and res["ok"]
    assert len(res["mat"]["points"]) == 9


def test_goto_fake_and_refused(handler, tmp_path, monkeypatch):
    from src.console.mini import read_jog_overrides, default_override_path
    handler.jog_map = load_jog_map(
        overrides=read_jog_overrides(default_override_path()))
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    q = {j: 0.0 for j in handler.jog_map}
    status, res = handler.dispatch_api("/api/goto",
                                       {"q_deg": q, "speed_dps": 15,
                                        "label": "t"})
    assert status == 200 and res["ok"]
    bad = dict(q)
    bad["J1"] = 500.0
    status, res = handler.dispatch_api("/api/goto",
                                       {"q_deg": bad})
    assert status == 400


def test_table_matches_calibration(handler):
    import yaml, pathlib
    cfg = yaml.safe_load(
        (pathlib.Path("config/calibration.local.yaml")).read_text(
            encoding="utf-8"))["stage"]
    status, res = handler.dispatch_api("/api/table", {})
    assert status == 200 and res["ok"]
    for k in ("table_z_mm", "center_x_mm", "center_y_mm"):
        assert res["stage"][k] == cfg[k]


def test_base_bottom_is_table(handler):
    import glob
    if not glob.glob("assets/private/visual_pack/robot_inferred/D0.glb"):
        import pytest
        pytest.skip("no private visual pack")
    import numpy as np
    from src.verify.sim_check import link_frames, load_link_meshes
    doc = handler.model_doc
    frames = link_frames({j: 0.0 for j in handler.jog_map}, doc)
    tris, _ = load_link_meshes("robot_inferred", stride=3)["D0"]
    w = np.einsum("ij,...j->...i", frames[0][:3, :3],
                  tris.reshape(-1, 3)) + frames[0][:3, 3]
    from src.console.control import stage_table
    assert abs(float(w[:, 2].min()) - stage_table()["table_z_mm"]) < 2.0


def test_ik_clamps_below_table(handler):
    status, res = handler.dispatch_api("/api/ik", {
        "pos": [0.0, 100.0, -500.0], "mat": None,
        "q_init": [0.0] * 6, "position_only": True})
    assert status == 200 and res["clamped"] is True
    from src.console.control import stage_table
    assert res["pos_used"][2] == stage_table()["table_z_mm"]


def test_mat_points_level_and_reachable(handler):
    import json as _json
    import pathlib as _pl
    p = _pl.Path("assets/private/mat_points.json")
    if not p.exists():
        import pytest
        pytest.skip("no private mat")
    mat = _json.loads(p.read_text(encoding="utf-8"))
    zs = {pt["z"] for pt in mat["points"]}
    assert len(zs) == 1 and len(mat["points"]) >= 8
    for pt in mat["points"]:
        status, res = handler.dispatch_api("/api/ik", {
            "pos": [pt["x"], pt["y"], pt["z"]], "mat": None,
            "q_init": pt.get("seed") or [0.0] * 6,
            "position_only": True})
        assert status == 200 and res["ok"]
        assert res["ik"]["pos_err_mm"] <= 5.0


def test_ik_gripper_down_cold(handler):
    # Tip TCP + gripper-down orientation from a cold start.
    from src.console.control import gripper_down_mat
    status, res = handler.dispatch_api("/api/ik", {
        "pos": [-40.8, 324.5, 0.0], "mat": gripper_down_mat(),
        "q_init": [0.0] * 6, "position_only": False})
    assert status == 200 and res["ok"]
    assert res["ik"]["pos_err_mm"] <= 5.0
    assert res["ik"]["rot_err_deg"] <= 3.0


def test_ik_position_only_reaches_ee(handler, tmp_path, monkeypatch):
    from src.kin.fk import fk_model
    from src.console.control import tip_doc

    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    target = fk_model([0.0] * 6,
                      tip_doc(handler.model_doc))["pos_mm"]
    status, res = handler.dispatch_api("/api/ik", {
        "pos": target, "mat": None,
        "q_init": [0.0] * 6, "position_only": True})
    assert status == 200 and res["ik"]["success"]
    assert res["ik"]["pos_err_mm"] == pytest.approx(0.0, abs=0.2)
    assert res["joints"] == ["J1", "J2", "J3", "J4", "J5", "J6"]


def test_visual_primitives_only(handler):
    status, res = handler.dispatch_api("/api/visual", {})
    assert status == 200
    assert res["available"] == ["primitives"]
    assert res["links"] == {}


def test_visual_private_links(handler):
    handler.private_visuals = "robot_inferred"
    status, res = handler.dispatch_api("/api/visual", {})
    assert status == 200
    assert res["available"] == ["primitives", "ldraw"]
    assert len(res["links"]) == 7
    assert res["links"]["D0"].endswith("/D0.glb")


def test_ik_rejects_bad_shapes(handler, tmp_path, monkeypatch):
    monkeypatch.setattr(ctl, "runs_day_dir",
                        lambda base=None: tmp_path / "runs")
    status, res = handler.dispatch_api("/api/ik", {
        "pos": [1.0, 2.0], "mat": None,
        "q_init": [0.0] * 6, "position_only": True})
    assert status == 400
    status, res = handler.dispatch_api("/api/ik", {
        "pos": [1.0, 2.0, 3.0], "mat": [[1, 0], [0, 1]],
        "q_init": [0.0] * 6, "position_only": False})
    assert status == 400


def test_ldraw_subdir_refs_resolve(tmp_path):
    # LDraw subfile refs use backslash separators and subdirs
    # (parts/s/, p/48/); dropping them drops whole faces (ghost parts).
    from src.viz.build_link_meshes import _find_dat
    lib = tmp_path / "ldraw"
    (lib / "parts" / "s").mkdir(parents=True)
    (lib / "p" / "48").mkdir(parents=True)
    (lib / "parts" / "s" / "bar.dat").write_text("0 bar\n")
    (lib / "p" / "48" / "baz.dat").write_text("0 baz\n")
    assert _find_dat(str(lib), "s\\bar.dat") is not None
    assert _find_dat(str(lib), "48\\baz.dat") is not None


def test_ldraw_edges_parsed():
    from src.viz.build_link_meshes import _parse_dat
    import tempfile as _tf
    with _tf.NamedTemporaryFile("w", suffix=".dat",
                                 delete=False) as fh:
        fh.write("0 test\n2 24 -1 0 0 1 0 0\n3 4 0 0 0 1 0 0 0 1 0\n")
        path = fh.name
    tris, edges = _parse_dat(path, {"parts": {}, "p": {}}, {}, [])
    assert len(tris) == 1 and len(edges) == 1
    assert len(edges[0][1]) == 2


def test_glb_writer_roundtrip(tmp_path):
    from src.viz.build_link_meshes import _write_glb
    import json as _json
    import struct as _struct

    positions = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    normals = [0.0, 0.0, 1.0] * 6
    indices = [0, 1, 2, 3, 4, 5]
    path = str(tmp_path / "t.glb")
    _write_glb(path, positions, normals, indices,
               [(0, 0, 3, 4), (1, 3, 3, 4)], ["15", "4"])
    blob = open(path, "rb").read()
    magic, ver, ln = _struct.unpack("<III", blob[:12])
    assert magic == 0x46546C67 and len(blob) == ln
    jl, jt = _struct.unpack("<II", blob[12:20])
    doc = _json.loads(blob[20:20 + jl])
    assert jt == 0x4E4F534A
    assert len(doc["meshes"][0]["primitives"]) == 2
    assert [m["name"] for m in doc["materials"]] == ["c0", "c1"]
    prim = doc["meshes"][0]["primitives"][0]
    assert set(prim["attributes"]) == {"POSITION", "NORMAL"}
    assert "indices" in prim
    idx = doc["accessors"][prim["indices"]]
    assert idx["componentType"] == 5125 and idx["count"] == 3


def test_pose_refused_outside_range():
    adopted = {"J1": [-50.0, 39.0], "J2": [-116.25, 40.0],
               "J3": [-74.38, 21.0], "J4": [-115.0, 125.0],
               "J5": [-90.0, 124.38], "J6": [-60.0, 135.0]}
    q = {"J1": 0.0, "J2": 0.0, "J3": 0.0,
         "J4": 0.0, "J5": 0.0, "J6": 200.0}
    assert "J6" in ctl.check_pose_against_ranges(q, adopted)
    q_ok = dict(q, J6=0.0)
    assert ctl.check_pose_against_ranges(q_ok, adopted) == ""


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


def _tmp_runs(monkeypatch, tmp_path):
    day = tmp_path / "runs"
    monkeypatch.setattr(ctl, "runs_day_dir", lambda base=None: day)
    monkeypatch.setattr(ctl, "latest_homing_record",
                        lambda base=None: ctl.read_json(
                            day / "homing.json", {}))
    return day


def test_goto_refused_latched(handler, tmp_path, monkeypatch):
    _tmp_runs(monkeypatch, tmp_path)
    real_send = handler.transport.send

    def fake_send(brick, cmd, *a, **k):
        r = real_send(brick, cmd, *a, **k)
        if cmd == "read":
            r = dict(r)
            r["latched"] = True
        return r

    monkeypatch.setattr(handler.transport, "send", fake_send)
    q = {j: 0.0 for j in handler.jog_map}
    status, res = handler.dispatch_api("/api/goto", {"q_deg": q})
    assert status == 400 and "latched" in res["error"]


def test_offline_virtual_only_refuses_motion():
    from src.console import control as ctl2
    h = ctl._Handler.__new__(ctl._Handler)
    h.transport = ctl2.OfflineTransport()
    h.jog_map = load_jog_map(overrides={})
    h.jog_overrides = {}
    h.direction_path = "/tmp/never-direction.json"
    h.model_doc = load_model("models/robot_inferred.json")
    h.zeros = {}
    h.console_path = "/tmp/never-console.json"
    h.private_visuals = None
    h.virtual = True
    status, res = h.dispatch_api("/api/mode", {})
    assert status == 200 and res["connected"] is False
    assert res["virtual"] is True
    status, res = h.dispatch_api(
        "/api/overview", {})
    assert status == 200
    assert res["latched"] == {"left": None, "right": None}
    assert all(j["joint_deg"] is None for j in res["joints"])
    q = {j: 0.0 for j in h.jog_map}
    status, res = h.dispatch_api("/api/goto", {"q_deg": q})
    assert status == 500 and "offline" in res["error"]


def test_mode_virtual_and_connected(handler):
    handler.virtual = False
    status, res = handler.dispatch_api("/api/mode", {})
    assert status == 200 and res["connected"] is True
    assert res["virtual"] is False
    handler.virtual = True
    status, res = handler.dispatch_api("/api/mode", {})
    assert status == 200 and res["virtual"] is True
    assert res["connected"] is True


def test_home_joint_manual_and_sensor_simulated(handler, tmp_path,
                                               monkeypatch):
    _tmp_runs(monkeypatch, tmp_path)
    status, res = handler.dispatch_api("/api/home_joint",
                                       {"joint": "J5", "mark": True})
    assert status == 200 and res["ok"] and res["method"] == "manual"
    handler.virtual = True
    status, res = handler.dispatch_api("/api/home_joint",
                                       {"joint": "J1", "mark": False})
    assert status == 200 and res["ok"] and res.get("simulated") is True
    assert "J5" in ctl.latest_homing_record().get("offsets", {})


def test_verify_report_target_fields(handler):
    # Contract with renderVerify in control.py: each target needs
    # the keys the page reads without defaults.
    status, res = handler.dispatch_api("/api/verify_report", {})
    assert status == 200
    if not res.get("ok"):
        pytest.skip("no local ik_check_multi report")
    targets = res["report"].get("targets")
    assert isinstance(targets, list) and targets
    for t in targets:
        assert {"id", "tcp_err_mm", "pass"} <= set(t), t.get("id")


def test_homing_spec_shape(handler):
    status, res = handler.dispatch_api("/api/homing_spec", {})
    assert status == 200 and res["ok"]
    assert isinstance(res["joints"], dict)
    assert set(res["joints"]) >= {"J1", "J5", "J6"}


def test_pages_share_server_state(handler, tmp_path, monkeypatch):
    _tmp_runs(monkeypatch, tmp_path)
    handler.virtual = True
    status, res = handler.dispatch_api("/api/home_joint",
                                       {"joint": "J5", "mark": True})
    assert status == 200 and res["ok"]
    h2 = ctl._Handler.__new__(ctl._Handler)
    h2.transport = handler.transport
    h2.jog_map = handler.jog_map
    h2.jog_overrides = handler.jog_overrides
    h2.override_path = handler.override_path
    h2.direction_path = handler.direction_path
    h2.model_doc = handler.model_doc
    h2.zeros = {}
    h2.console_path = handler.console_path
    h2.private_visuals = None
    h2.virtual = True
    assert h2.transport is handler.transport
    rec = ctl.latest_homing_record()
    assert "J5" in rec.get("offsets", {})
    s1, m1 = handler.dispatch_api("/api/mode", {})
    s2, m2 = h2.dispatch_api("/api/mode", {})
    assert (s1, m1["connected"]) == (200, True)
    assert (s2, m2["connected"]) == (200, True)


def _gantry_doc():
    """Synthetic 3-prismatic gantry (no private files needed)."""
    return {
        "robot": "synthetic-gantry",
        "links": [{"id": l} for l in ("base", "a", "b", "c")],
        "joints": [
            {"id": "P1", "type": "prismatic", "parent": "base",
             "child": "a", "axis": [1, 0, 0], "point_mm": [0, 0, 0],
             "limits_mm": [-10.0, 10.0]},
            {"id": "P2", "type": "prismatic", "parent": "a",
             "child": "b", "axis": [0, 0, -1], "point_mm": [0, 0, 5],
             "limits_mm": [-8.0, 8.0]},
            {"id": "P3", "type": "prismatic", "parent": "b",
             "child": "c", "axis": [0, 1, 0], "point_mm": [0, 0, 5],
             "limits_mm": [-4.0, 4.0]},
        ],
        "actuators": [{"id": "A"}, {"id": "B"}, {"id": "C"}],
        "base": {"link": "base"},
        "end_effector": {"link": "c", "point_mm": [0, 4.0, 5.0]},
    }


def _gantry_handler(handler):
    handler.robot = "tbot"
    handler.model_doc = _gantry_doc()
    handler.jog_map = ctl.tbot_jog_map()
    return handler


def test_prismatic_model_payload_marks_mm(handler):
    _gantry_handler(handler)
    status, res = handler.dispatch_api("/api/model", {})
    assert status == 200 and res["ok"]
    assert [(j["id"], j["kind"], j["unit"])
            for j in res["joints"]] == [
        ("P1", "prismatic", "mm"), ("P2", "prismatic", "mm"),
        ("P3", "prismatic", "mm")]
    assert res["ee_mm"] == [0, 4.0, 5.0]


def test_prismatic_overview_unknown_encoders(handler):
    _gantry_handler(handler)
    status, res = handler.dispatch_api("/api/overview", {})
    assert status == 200 and res["ok"]
    assert [j["joint"] for j in res["joints"]] == ["P1", "P2", "P3"]
    assert all(j["joint_deg"] is None for j in res["joints"])


def test_tbot_mode_virtual_and_motion_blocked(handler):
    _gantry_handler(handler)
    status, res = handler.dispatch_api("/api/mode", {})
    assert status == 200 and res["virtual"] and not res["connected"]
    for path in ("/api/jog", "/api/goto", "/api/home_joint"):
        status, res = handler.dispatch_api(
            path, {"joint": "P1", "delta": 1.0,
                   "q_deg": {"P1": 0.0, "P2": 0.0, "P3": 0.0}})
        assert status == 400 and "virtual-only" in res["error"]


def test_visual_link_count_follows_chain(handler):
    _gantry_handler(handler)
    handler.private_visuals = "somepack"
    status, res = handler.dispatch_api("/api/visual", {})
    assert status == 200
    assert sorted(res["links"]) == ["D0", "D1", "D2", "D3"]


def test_console_js_has_no_hardcoded_joint_counts():
    import re

    src = pathlib.Path(ctl.__file__).read_text(encoding="utf-8")
    js = src.split('<script type="module">', 1)[1]
    assert not re.findall(r"\b[ik] <=? [56]\)", js)
    assert "m.joints[i + 1]" in js
    assert '"D0", "D1", "D2", "D3", "D4", "D5", "D6"' not in js
