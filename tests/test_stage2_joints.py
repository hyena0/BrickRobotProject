"""Stage 2.9 joint re-identification tests (synthetic only)."""

from __future__ import annotations

import numpy as np

from src.agent import joints_spark
from src.tools import joints as jtools
from src.viz import overlay

CATALOG = {
    "axle.dat": {"category": "axle", "axis_local": [1, 0, 0],
                 "axle_length": 4, "holes": []},
    "beam.dat": {"category": "beam", "axis_local": None,
                 "holes": [{"kind": "round", "pos": [0, 0, 0]}]},
    "pin.dat": {"category": "pin", "axis_local": [1, 0, 0],
                "friction": False, "pin_length_studs": 2, "holes": []},
    "pin_rigid.dat": {"category": "pin", "axis_local": [1, 0, 0],
                      "friction": True, "pin_length_studs": 2, "holes": []},
    "gear.dat": {"category": "gear", "axis_local": [0, 0, 1], "teeth": 8,
                 "holes": [{"kind": "axle", "pos": [0, 0, 0]}]},
}

IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _part(uid, pid, pos):
    return {"uid": uid, "part_id": pid, "position_mm": list(pos),
            "rotation": [list(r) for r in IDENT]}


def hinge_parts():
    # Frictionless pin hinge: beamA - pin - beamB, all free.
    return [
        _part("beamA", "beam.dat", [0, 0, 0]),
        _part("beamB", "beam.dat", [16, 0, 0]),
        _part("pin", "pin.dat", [8, 0, 0]),
    ]


def test_fit_axis_line_recovers_axis_and_error():
    pts = [np.array([t, 2.0 * t + 1.0, -t]) for t in (-10.0, 0.0, 10.0)]
    fit = jtools.fit_axis_line(pts)
    assert fit["n_supports"] == 3
    assert fit["fit_error_mm"] < 1e-6
    d = np.asarray(fit["axis"])
    assert abs(abs(float(d @ np.array([1, 2, -1]) / np.sqrt(6)))) > 0.999


def test_candidates_find_free_hinge_not_rigid_pair():
    parts = hinge_parts() + [
        _part("beamC", "beam.dat", [0, 40, 0]),
        _part("beamD", "beam.dat", [16, 40, 0]),
        _part("pinR", "pin_rigid.dat", [8, 40, 0]),
    ]
    found = jtools.joint_candidates(parts, CATALOG)
    pairs = {tuple(sorted(c["groups"])) for c in found["candidates"]}
    # Exactly the two free hinge sides; the friction-pinned pair unites
    # into one group and yields no candidate.
    assert len(found["candidates"]) == 2
    assert found["counts"]["n_candidates"] == 2
    for cand in found["candidates"]:
        d = np.asarray(cand["fit"]["axis"])
        assert abs(abs(float(d[0]))) > 0.99  # hinge axis along X
        assert cand["fit"]["fit_error_mm"] <= 2.0


def test_candidates_exclude_powertrain_groups():
    parts = hinge_parts() + [
        _part("axle", "axle.dat", [8, 60, 0]),
        _part("gear", "gear.dat", [8, 60, 0]),
    ]
    found = jtools.joint_candidates(parts, CATALOG)
    assert len(found["powertrain"]) >= 1
    # Axle+gear unite rigidly but form a powertrain group: no candidate
    # may touch it, so only the 2 hinge sides remain.
    assert found["counts"]["n_candidates"] == 2
    for cand in found["candidates"]:
        for gid in cand["groups"]:
            assert gid not in found["powertrain"]


def test_sweep_reports_full_range_when_clear():
    parts = hinge_parts()
    found = jtools.joint_candidates(parts, CATALOG)
    cand = found["candidates"][0]
    sw = jtools.sweep_test(cand, parts, CATALOG, found["groups"])
    assert sw["pos"] is None and sw["neg"] is None
    assert sw["range_note"] == ("+180deg+", "-180deg+")


def test_consistency_accepts_bijection():
    doc = {"links": [{"id": "G0"}, {"id": "G1"}, {"id": "G2"}],
           "joints": [
               {"id": "J1", "parent": "G0", "child": "G1",
                "status": "selected"},
               {"id": "J2", "parent": "G1", "child": "G2",
                "status": "selected"}],
           "base": {"link": "G0"}, "end_effector": {"link": "G2"}}
    checked = jtools.check_kinematic_consistency(doc, {"m1": "J1",
                                                      "m2": "J2"})
    assert checked == {"errors": [], "gaps": []}


def test_consistency_reports_sentences():
    doc = {"links": [{"id": "G0"}, {"id": "G1"}],
           "joints": [{"id": "J1", "parent": "G0", "child": "G1",
                       "status": "selected"}],
           "base": {"link": "G0"}, "end_effector": {"link": "G9"}}
    checked = jtools.check_kinematic_consistency(doc, {"m1": "J1",
                                                      "m2": "J1",
                                                      "m3": "JX"})
    assert any("2 motors" in r for r in checked["gaps"])
    assert any("not a selected joint" in r for r in checked["errors"])
    assert any("not connected" in r for r in checked["gaps"])


class _FakeResponses:
    def __init__(self, texts):
        self.texts = list(texts)

    def create(self, **kwargs):
        text = self.texts.pop(0)
        return type("R", (), {"output_text": text})()


class _FakeClient:
    def __init__(self, texts):
        self.responses = _FakeResponses(texts)


def test_spark_retry_loop_stops_at_three():
    parts = hinge_parts()
    sel = {"selected": [{"candidate": 0, "id": "J1", "reason": "t"},
                        {"candidate": 1, "id": "J2", "reason": "t"}],
           "dropped": []}
    import json as _json
    client = _FakeClient([_json.dumps(sel)] * 3)
    result = joints_spark.run_selection(client, parts, CATALOG, {},
                                        max_attempts=3)
    # No motor terminals: the hinge joints report drive gaps, which are
    # recorded (not retried); the hinge chain is connected, so the loop
    # stops after the first attempt.
    assert result["attempts"] == 1
    assert result["errors"] == []
    assert any("no motor path" in r for r in result["gaps"])
    assert result["doc"]["method"] == "joints-spark-selection"


def test_overlay_marks_selected_and_dropped():
    pred = {"links": [{"id": "G0", "members": ["a"]}],
            "joints": [
                {"id": "J1", "axis": [1, 0, 0], "point_mm": [0, 0, 0],
                 "parent": "G0", "child": "G1", "status": "selected"},
                {"id": "X1", "axis": [0, 1, 0], "point_mm": [1, 1, 1],
                 "parent": "G0", "child": "G1", "status": "dropped"}],
            "base": {"link": "G0"}}
    import json as _json
    import tempfile as _tf
    with _tf.NamedTemporaryFile("w", suffix=".json",
                                delete=False) as fh:
        _json.dump(pred, fh)
        path = fh.name
    ov = overlay.build_overlay(path)
    styles = {a["id"]: a["style"] for a in ov["arrows"]}
    assert styles == {"J1": "selected", "X1": "dropped"}
