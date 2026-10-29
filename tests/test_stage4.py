"""Stage 4 tests: synthetic fixtures only.

The real model (runs/, assets/private/) and ground truth
(eval/ground_truth/, gt/) are never read here.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from src.kin import (build_mjcf, fk_model, ik, joint_arrays, load_model,
                     joints_to_motors, load_zeros, motors_to_joints)
from src.tools.axes import axes_intersect
from src.verify.ik_eval import evaluate


def make_doc():
    axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0],
            [1, 0, 0], [0, 1, 0], [1, 0, 0]]
    points = [[0, 0, 0], [0, 0, 100], [0, 0, 200],
              [100, 0, 200], [100, 0, 300], [200, 0, 300]]
    joints = [{"id": f"J{i + 1}", "type": "revolute",
               "parent": f"D{i}", "child": f"D{i + 1}",
               "axis": axes[i], "point_mm": points[i],
               "limits_deg": [-170, 170],
               "zero": "pose in the source file"} for i in range(6)]
    links = [{"id": f"D{i}"} for i in range(7)]
    return {"frame": "base link, Z up, mm", "links": links,
            "joints": joints, "actuators": [],
            "transmission": {"coupling": "q = C m", "C": []},
            "end_effector": {"link": "D6",
                             "point_mm": list(points[-1])}}


def test_mjcf_has_joints_limits_and_ee(tmp_path):
    doc = make_doc()
    xml = build_mjcf(doc)
    root = ET.fromstring(xml)
    joints = root.findall(".//joint")
    assert len(joints) == 6
    assert [j.get("name") for j in joints] == \
        [f"J{i + 1}" for i in range(6)]
    assert joints[0].get("axis") == "0.0000 0.0000 1.0000"
    assert joints[0].get("range") == "-170 170"
    assert root.find(".//site[@name='ee']") is not None


def test_fk_matches_mujoco_synthetic():
    mujoco = pytest.importorskip("mujoco")
    doc = make_doc()
    model = mujoco.MjModel.from_xml_string(build_mjcf(doc))
    data = mujoco.MjData(model)
    arr = joint_arrays(doc)
    import random
    rng = random.Random(3)
    for _ in range(10):
        q_deg = [rng.uniform(-150, 150) for _ in range(6)]
        q = [math.radians(x) for x in q_deg]
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        got = fk_model(q_deg, doc)
        pos = [float(x) for x in data.site("ee").xpos]
        err = math.sqrt(sum((a - b) ** 2 for a, b in
                            zip(pos, got["pos_mm"])))
        assert err < 0.01


def test_axes_intersect_concurrent():
    out = axes_intersect([[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                         [[1, 0, 0], [0, 2, 0], [0, 0, 3]])
    assert out["intersect"] is True
    assert out["max_residual_mm"] < 1e-6


def test_axes_intersect_parallel_is_false():
    out = axes_intersect([[0, 0, 1], [0, 0, 1]],
                         [[0, 0, 0], [10, 0, 0]])
    assert out["intersect"] is False


def test_axes_intersect_skew_is_false():
    out = axes_intersect([[1, 0, 0], [0, 1, 0]],
                         [[0, 0, 0], [0, 0, 5]])
    assert out["intersect"] is False
    assert out["max_residual_mm"] == pytest.approx(2.5)


def test_axes_intersect_empty_raises():
    with pytest.raises(ValueError):
        axes_intersect([], [])


def test_ik_roundtrip_synthetic():
    doc = make_doc()
    q_known = [10.0, -20.0, 30.0, -15.0, 25.0, -10.0]
    tgt = fk_model(q_known, doc)
    sol = ik(tgt["pos_mm"], tgt["mat"], doc, seed=1)
    assert sol["success"] is True
    assert sol["pos_err_mm"] <= 5.0
    assert sol["rot_err_deg"] <= 3.0


def test_ik_position_only_synthetic():
    doc = make_doc()
    q_known = [-30.0, 40.0, -50.0, 20.0, -35.0, 15.0]
    tgt = fk_model(q_known, doc)
    sol = ik(tgt["pos_mm"], tgt["mat"], doc, position_only=True,
             seed=2)
    assert sol["success"] is True
    assert sol["pos_err_mm"] <= 5.0


def test_motors_roundtrip_diagonal():
    c = [[2.0, 0.0], [0.0, 0.5]]
    mids = ["m1", "m2"]
    lims = [[-180, 180], [-180, 180]]
    out = joints_to_motors([20.0, 10.0], c, mids, lims)
    assert out["motors_deg"] == pytest.approx({"m1": 10.0,
                                               "m2": 20.0})
    assert out["coupling_residual_deg"] == pytest.approx(0.0)
    back = motors_to_joints(out["motors_deg"], c, mids)
    assert back == pytest.approx([20.0, 10.0])


def test_motors_zero_column_lstsq_and_limits():
    c = [[1.0, 0.0], [0.0, 0.0]]
    mids = ["m1", "m2"]
    lims = [[-180, 180], [-180, 180]]
    out = joints_to_motors([30.0, 0.0], c, mids, lims)
    assert out["motors_deg"]["m1"] == pytest.approx(30.0)
    with pytest.raises(ValueError):
        joints_to_motors([200.0, 0.0], c, mids, lims)


def test_motors_zero_offsets(tmp_path):
    zeros = tmp_path / "zero.yaml"
    zeros.write_text("zeros:\n  m1: 5.0\n", encoding="utf-8")
    z = load_zeros(str(zeros))
    assert z == {"m1": 5.0}
    assert load_zeros(None) == {}
    assert load_zeros(str(tmp_path / "missing.yaml")) == {}
    c = [[1.0]]
    out = joints_to_motors([10.0], c, ["m1"], [[-180, 180]], z)
    assert out["motors_deg"]["m1"] == pytest.approx(15.0)
    back = motors_to_joints({"m1": 15.0}, c, ["m1"], z)
    assert back == pytest.approx([10.0])


def test_ik_eval_identical_models_pass(tmp_path):
    doc = make_doc()
    res = evaluate(doc, doc, n=20, seed=0, restarts=8)
    assert res["success_rate"] >= 0.95
    assert res["median_pos_mm"] <= 2.0
    assert res["gate_pass"] is True


def test_load_model_rejects_prismatic(tmp_path):
    doc = make_doc()
    doc["joints"][0]["type"] = "prismatic"
    path = tmp_path / "model.json"
    import json
    path.write_text(json.dumps(doc), encoding="utf-8")
    with pytest.raises(ValueError):
        load_model(str(path))
