"""Tests for src.gen.roundtrip (synthetic fixtures, fake inference)."""

import json

import pytest

from src.gen import roundtrip as rt
from src.render.modal_render import _fold_trailing_beam_segment as fold


def _exp_doc():
    return {
        "frame": "base link, Z up, mm",
        "links": [{"id": "L0"}, {"id": "L1"}, {"id": "L2"}],
        "joints": [
            {"id": "J1", "type": "revolute", "parent": "L0",
             "child": "L1", "axis": [0, 0, 1], "point_mm": [0, 0, 20],
             "limits_deg": [-150, 150], "zero": "pose in the source file"},
            {"id": "J2", "type": "revolute", "parent": "L1",
             "child": "L2", "axis": [0, 1, 0], "point_mm": [0, 0, 60],
             "limits_deg": [-30, 100], "zero": "pose in the source file"},
        ],
        "actuators": [
            {"id": "M1", "brick": "left", "port": "A", "motor": "medium"},
            {"id": "M2", "brick": "left", "port": "B", "motor": "large"},
        ],
        "transmission": {
            "coupling": "q = C m",
            "C": [[0.1, 0.0], [0.0, 0.05]],
        },
        "end_effector": {"link": "L2", "point_mm": [0, 0, 100]},
    }


def _pred_joint(jid, axis, point, parent="GA", child="GB"):
    return {"id": jid, "type": "revolute", "parent": parent,
            "child": child, "axis": axis, "point_mm": point,
            "limits_deg": [-180, 180], "confidence": 0.9,
            "reason": "fake"}


def _links_doc(joints):
    return {"links": [{"id": "GA", "members": ["p1"]},
                      {"id": "GB", "members": ["p2"]}],
            "joints": joints,
            "base": {"link": "GA"}, "end_effector": {"link": "GB"}}


def _trains():
    return {"C": [[0.1, 0.0], [0.0, 0.05]],
            "joint_ids": ["J1", "J2"], "motor_ids": ["m1", "m2"],
            "trains": {
                "J1": {"motor": "m1", "ratio": 0.1, "sign_known": True,
                       "path": ["external:8/80"]},
                "J2": {"motor": "m2", "ratio": 0.05, "sign_known": True,
                       "path": ["external:8/160"]}}}


def test_match_joints_reorders_and_tolerates_flip():
    pred = [_pred_joint("J2", [0, -1, 0], [0.5, 0, 60.5]),
            _pred_joint("J1", [0, 0, 1], [0.2, 0, 20.1])]
    pairs = rt.match_joints(pred, _exp_doc()["joints"])
    assert pairs == [(1, 0), (0, 1)]


def test_match_joints_rejects_mismatch():
    pred = [_pred_joint("J1", [0, 0, 1], [0, 0, 20])]
    with pytest.raises(ValueError, match="count differs"):
        rt.match_joints(pred, _exp_doc()["joints"])
    bad = [_pred_joint("J1", [0, 0, 1], [0, 0, 20]),
           _pred_joint("J2", [1, 0, 0], [0, 0, 60])]
    with pytest.raises(ValueError, match="could not pair"):
        rt.match_joints(bad, _exp_doc()["joints"])


def test_compare_joints_and_links():
    exp = _exp_doc()["joints"]
    good = [_pred_joint("J1", [0, 0, 1], [0.5, 0, 20.5]),
            _pred_joint("J2", [0, 1, 0], [0, 0.5, 60.5])]
    pairs = rt.match_joints(good, exp)
    assert rt.compare_joints(good, exp, pairs)["pass"] is True
    assert rt.compare_links(good, exp, pairs)["pass"] is True

    tilted = [_pred_joint("J1", [0, 0, 1], [0, 0, 20]),
              _pred_joint("J2", [0, 0.9, 0.4359], [0, 0, 60])]
    pairs = rt.match_joints(tilted, exp)
    assert rt.compare_joints(tilted, exp, pairs)["pass"] is False

    shifted = [_pred_joint("J1", [0, 0, 1], [0, 0, 20]),
               _pred_joint("J2", [0, 1, 0], [0, 0, 90])]
    pairs = rt.match_joints(shifted, exp)
    assert rt.compare_links(shifted, exp, pairs)["pass"] is False


def test_compare_ratios():
    exp = _exp_doc()
    pairs = [(0, 0), (1, 1)]
    ok = rt.compare_ratios(_trains()["trains"], exp, pairs, ["J1", "J2"])
    assert ok["pass"] is True
    assert ok["c_max_abs_diff"] == pytest.approx(0.0)
    bad = dict(_trains()["trains"])
    bad["J2"] = {**bad["J2"], "ratio": 0.06}
    nope = rt.compare_ratios(bad, exp, pairs, ["J1", "J2"])
    assert nope["pass"] is False


def test_compare_ratios_gates_sign():
    exp = _exp_doc()
    pairs = [(0, 0), (1, 1)]
    flipped = dict(_trains()["trains"])
    flipped["J1"] = {**flipped["J1"], "ratio": -0.1}
    nope = rt.compare_ratios(flipped, exp, pairs, ["J1", "J2"])
    assert nope["pass"] is False
    assert nope["joints"][0]["sign_ok"] is False
    unknown = dict(_trains()["trains"])
    unknown["J2"] = {**unknown["J2"], "sign_known": False}
    nope = rt.compare_ratios(unknown, exp, pairs, ["J1", "J2"])
    assert nope["pass"] is False


def _graph(edges):
    return {"nodes": [], "edges": edges, "motor_drive": {},
            "intra_meshes": []}


def _edge(a, b, ta, tb, kind="external"):
    return {"a": a, "b": b, "gear_a": f"{a}g", "gear_b": f"{b}g",
            "ta": ta, "tb": tb, "kind": kind}


def _joints(*ids):
    return [{"id": jid} for jid in ids]


def test_solve_trains_simple_and_idler():
    graph = _graph([_edge("A0", "A1", 8, 24)])
    out = rt.solve_trains(graph, _joints("J1"), ["m1"],
                          {"m1": "A0"}, {"J1": "A1"})
    assert out["C"] == [[pytest.approx(-8 / 24)]]
    assert out["trains"]["J1"]["sign_known"] is True

    graph = _graph([_edge("A0", "A1", 8, 16), _edge("A1", "A2", 16, 24)])
    out = rt.solve_trains(graph, _joints("J1"), ["m1"],
                          {"m1": "A0"}, {"J1": "A2"})
    assert out["C"] == [[pytest.approx(8 / 24)]]


def test_solve_trains_direct_bevel_and_internal():
    out = rt.solve_trains(_graph([]), _joints("J1"), ["m1"],
                          {"m1": "A0"}, {"J1": "A0"})
    assert out["C"] == [[1.0]]

    graph = _graph([_edge("A0", "A1", 12, 36, kind="bevel")])
    out = rt.solve_trains(graph, _joints("J1"), ["m1"],
                          {"m1": "A0"}, {"J1": "A1"})
    assert out["C"] == [[pytest.approx(12 / 36)]]
    assert out["trains"]["J1"]["sign_known"] is False

    graph = _graph([_edge("A0", "A1", 12, 36, kind="internal")])
    out = rt.solve_trains(graph, _joints("J1"), ["m1"],
                          {"m1": "A0"}, {"J1": "A1"})
    assert out["C"] == [[pytest.approx(12 / 36)]]
    assert out["trains"]["J1"]["sign_known"] is True


def test_solve_trains_rejects_bad_topology():
    graph = _graph([_edge("A0", "A1", 8, 24)])
    with pytest.raises(ValueError, match="undriven joints"):
        rt.solve_trains(graph, _joints("J1", "J2"), ["m1"],
                        {"m1": "A0"}, {"J1": "A1", "J2": "AX"})
    with pytest.raises(ValueError, match="reaches 0 joints"):
        rt.solve_trains(graph, _joints("J1"), ["m1"],
                        {"m1": "AZ"}, {"J1": "A1"})
    coupled = _graph([_edge("A0", "A1", 8, 24), _edge("A0", "A2", 8, 16)])
    with pytest.raises(ValueError, match="2 joints"):
        rt.solve_trains(coupled, _joints("J1", "J2"), ["m1"],
                        {"m1": "A0"}, {"J1": "A1", "J2": "A2"})
    with pytest.raises(ValueError, match="unknown teeth"):
        bad = _graph([_edge("A0", "A1", None, 24)])
        rt.solve_trains(bad, _joints("J1"), ["m1"],
                        {"m1": "A0"}, {"J1": "A1"})


def _robot_dir(tmp_path):
    robot = tmp_path / "gen1"
    robot.mkdir()
    (robot / "expected_kinematics.json").write_text(
        json.dumps(_exp_doc()), encoding="utf-8")
    (robot / "robot.ldr").write_text(
        "1 7 0 0 0 1 0 0 0 1 0 0 0 1 11478.dat\n", encoding="utf-8")
    return robot


def test_run_roundtrip_pass_with_fakes(tmp_path):
    robot = _robot_dir(tmp_path)
    joints = [_pred_joint("J1", [0, 0, 1], [0.3, 0, 20.2]),
              _pred_joint("J2", [0, 1, 0], [0, 0.3, 60.2])]
    infer = lambda parts, catalog: _links_doc(joints)  # noqa: E731
    trains = lambda parts, catalog, doc: _trains()  # noqa: E731
    report = rt.run_roundtrip(robot, infer, n_ik=4, seed=0,
                              mode="test", trains_fn=trains)
    assert report["overall_pass"] is True
    assert report["gates"]["ik"]["success_rate"] == 1.0
    path = rt.write_roundtrip(report, robot)
    assert path.is_file()
    assert json.loads(path.read_text())["overall_pass"] is True


def test_run_roundtrip_count_mismatch_verdict(tmp_path):
    robot = _robot_dir(tmp_path)
    joints = [_pred_joint("J1", [0, 0, 1], [0, 0, 20])]
    infer = lambda parts, catalog: _links_doc(joints)  # noqa: E731
    report = rt.run_roundtrip(robot, infer, n_ik=4, seed=0, mode="test")
    assert report["overall_pass"] is False
    assert report["gates"]["joint_count"] == {
        "pass": False, "pred": 1, "expected": 2}
    path = rt.write_roundtrip(report, robot)
    assert json.loads(path.read_text())["overall_pass"] is False


def test_run_roundtrip_fail_propagates(tmp_path):
    robot = _robot_dir(tmp_path)
    joints = [_pred_joint("J1", [0, 0, 1], [0, 0, 20]),
              _pred_joint("J2", [0, 0.9, 0.4359], [0, 0, 60])]
    infer = lambda parts, catalog: _links_doc(joints)  # noqa: E731
    trains = lambda parts, catalog, doc: _trains()  # noqa: E731
    report = rt.run_roundtrip(robot, infer, n_ik=4, seed=0,
                              mode="test", trains_fn=trains)
    assert report["overall_pass"] is False
    assert report["gates"]["axes"]["pass"] is False


def test_oracle_doc_from_assembly_truth(tmp_path):
    from src.agent.schema import validate_links_joints
    from src.parse import parse_file

    robot = tmp_path / "mini"
    robot.mkdir()
    (robot / "robot.ldr").write_text(
        "0 mini\n"
        "1 7 0 0 0 1 0 0 0 1 0 0 0 1 11478.dat\n"
        "1 7 1000 0 0 1 0 0 0 1 0 0 0 1 11478.dat\n",
        encoding="utf-8")
    parts = parse_file(str(robot / "robot.ldr"))
    segs = {p["uid"]: 0 for p in parts}
    (robot / "report.json").write_text(json.dumps({
        "segments": segs,
        "beams": [{"uid": parts[1]["uid"], "part_id": "11478.dat",
                   "segment": 0}]}), encoding="utf-8")
    (robot / "expected_kinematics.json").write_text(json.dumps({
        "frame": "base link, Z up, mm",
        "links": [{"id": "L0"}, {"id": "L1"}],
        "joints": [{"id": "J1", "type": "revolute", "parent": "L0",
                    "child": "L1", "axis": [0, 0, 1],
                    "point_mm": [0, 0, 20], "limits_deg": [-150, 150],
                    "zero": "pose in the source file"}],
        "actuators": [], "transmission": {"coupling": "q = C m", "C": []},
        "end_effector": {"link": "L1", "point_mm": [0, 0, 100]}}),
        encoding="utf-8")
    doc = rt.oracle_doc(robot)
    assert validate_links_joints(doc) == []
    assert [j["id"] for j in doc["joints"]] == ["J1"]
    assert sorted(u for link in doc["links"] for u in link["members"]) == \
        sorted(segs)


def test_append_log_format(tmp_path):
    log = tmp_path / "LOG.md"
    log.write_text("# log\n", encoding="utf-8")
    report = {"robot": "gen1", "mode": "live", "overall_pass": True,
              "gates": {
                  "joint_count": {"pred": 2, "expected": 2},
                  "axes": {"pass": True}, "links": {"pass": True},
                  "ratios": {"pass": True},
                  "ik": {"success_rate": 1.0, "median_pos_mm": 0.5}}}
    rt.append_log(report, log)
    lines = log.read_text().splitlines()
    assert len(lines) == 2
    assert "Roundtrip gen1 (live)" in lines[1]
    assert "PASS" in lines[1]


def test_module_roundtrip_both_templates_match():
    for name in ("M_base_yaw", "M_pitch"):
        got = rt.run_module_roundtrip(f"assets/modules/{name}")
        assert got["overall_pass"] is True, got
        assert got["axis"]["angle_deg"] < 1.0
        assert got["ratio"]["exact"] is True


def test_module_roundtrip_ratio_direction():
    got = rt.run_module_roundtrip("assets/modules/M_pitch")
    assert got["ratio"]["inferred_frac"] == "3/5"
    assert got["ratio"]["declared"] == 0.6


def test_infer_module_output_rejects_motorless():
    parts = [{"uid": "g1", "part_id": "32270.dat",
              "colour": "7", "position_mm": [0, 0, 0],
              "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}]
    with pytest.raises(ValueError, match="exactly 1 motor"):
        rt.infer_module_output(parts, {"32270.dat": {"teeth": 12}})


def test_fold_trailing_beam_segment_only_beams():
    segments = {"m0": 0, "m1": 1, "b0": 1, "m2": 2, "b2": 3}
    beams = [{"uid": "b0", "segment": 1}, {"uid": "b2", "segment": 3}]
    remapped, folded = fold(segments, beams)
    assert remapped["b2"] == 2
    assert remapped["m2"] == 2
    assert [b["segment"] for b in folded] == [1, 2]


def test_fold_trailing_beam_segment_keeps_modules():
    segments = {"m0": 0, "m1": 1}
    beams = [{"uid": "b0", "segment": 1}]
    remapped, folded = fold(segments, beams)
    assert remapped == segments
    assert [b["segment"] for b in folded] == [1]

