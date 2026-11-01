"""Stage 7 prismatic kinematics tests (additive; revolute path untouched).

Synthetic fixtures only. FK exact values, DLS convergence on a
3-prismatic gantry, closed-form agreement, MJCF slide output.
"""

from __future__ import annotations

import math

from src.kin import fk as fkmod
from src.kin import ik as ik_fn
from src.kin import mjcf as mjcfmod
from src.kin.ik import gantry_closed_form
from src.kin.model import load_model


def _doc3():
    return {
        "links": [{"id": "L%d" % i} for i in range(4)],
        "joints": [
            {"id": "P1", "type": "prismatic", "parent": "L0",
             "child": "L1", "axis": [1, 0, 0],
             "point_mm": [0.0, 0.0, 0.0], "limits_mm": [-100, 100]},
            {"id": "P2", "type": "prismatic", "parent": "L1",
             "child": "L2", "axis": [0, 1, 0],
             "point_mm": [0.0, 0.0, 0.0], "limits_mm": [-100, 100]},
            {"id": "P3", "type": "prismatic", "parent": "L2",
             "child": "L3", "axis": [0, 0, 1],
             "point_mm": [0.0, 0.0, 0.0], "limits_mm": [-100, 100]}],
        "base": {"link": "L0"},
        "end_effector": {"link": "L3", "point_mm": [10.0, 0.0, 0.0]}}


def test_fk_prismatic_translates():
    out = fkmod.fk_model([5.0, -3.0, 7.0], _doc3())
    assert [round(v, 9) for v in out["pos_mm"]] == [15.0, -3.0, 7.0]
    assert out["mat"] == [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def test_fk_mixed_carries_translation():
    doc = {
        "links": [{"id": "L0"}, {"id": "L1"}, {"id": "L2"}],
        "joints": [
            {"id": "J1", "type": "revolute", "parent": "L0",
             "child": "L1", "axis": [0, 0, 1],
             "point_mm": [0.0, 0.0, 0.0], "limits_deg": [-180, 180]},
            {"id": "P2", "type": "prismatic", "parent": "L1",
             "child": "L2", "axis": [1, 0, 0],
             "point_mm": [0.0, 0.0, 0.0], "limits_mm": [-100, 100]}],
        "base": {"link": "L0"},
        "end_effector": {"link": "L2", "point_mm": [0.0, 0.0, 0.0]}}
    out = fkmod.fk_model([90.0, 10.0], doc)
    assert [round(v, 9) for v in out["pos_mm"]] == [0.0, 10.0, 0.0]


def test_dls_and_closed_form_agree_on_gantry():
    doc = _doc3()
    tgt = [30.0, -20.0, 40.0]
    eye = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    cf = gantry_closed_form(tgt, doc)
    assert cf["success"] is True
    assert [round(v, 9) for v in cf["q"]] == [20.0, -20.0, 40.0]
    sol = ik_fn(tgt, eye, doc, position_only=True, seed=0)
    assert sol["success"] is True
    assert sol["pos_err_mm"] < 5.0
    for a, b in zip(sol["q"], cf["q"]):
        assert abs(a - b) < 5.0


def test_mjcf_slide_for_prismatic():
    xml = mjcfmod.build_mjcf(_doc3())
    assert 'type="slide"' in xml
    assert 'type="hinge"' not in xml


def test_load_model_prismatic_limits(tmp_path):
    import json
    doc = _doc3()
    path = str(tmp_path / "m.json")
    with open(path, "w") as fh:
        json.dump(doc, fh)
    loaded = load_model(path)
    assert loaded["joints"][0]["limits_mm"] == [-100, 100]
    bad = _doc3()
    del bad["joints"][0]["limits_mm"]
    with open(path, "w") as fh:
        json.dump(bad, fh)
    try:
        load_model(path)
    except ValueError as e:
        assert "limits_mm" in str(e)
    else:
        raise AssertionError("missing limits_mm must fail")
