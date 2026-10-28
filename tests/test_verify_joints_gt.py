"""Joint GT loader/scoring tests (Stage 2.5).

Synthetic only. The real ground truth (`gt/`, `eval/ground_truth/`)
is never read here: the GT-shaped document below is fabricated inline
and written to tmp_path.
"""

from __future__ import annotations

import json

import pytest

from src.verify import joints as joints_verify

GT_IDS = ["J1", "J2", "J3", "J4", "J5", "J6"]


def _synthetic_gt(path):
    doc = {
        "description": "synthetic GT-shaped fixture, not real data",
        "grading": {
            "position": ("distance from pred point to GT axis line "
                         "(point-along-axis is unobservable gauge)"),
            "axis": "|dot(pred_axis, gt_axis)| >= cos(tolerance_deg)",
        },
        "joints": {
            jid: {"point": [10.0 * i, 0.0, 0.0], "axis": [0, 0, 1]}
            for i, jid in enumerate(GT_IDS)
        },
    }
    path.write_text(json.dumps(doc), encoding="utf-8")
    return str(path)


def test_load_gt_joints_schema(tmp_path):
    gt = joints_verify.load_gt_joints(
        _synthetic_gt(tmp_path / "gt_joints.json"))
    assert sorted(gt.keys()) == GT_IDS
    assert gt["J3"]["point"] == [20.0, 0.0, 0.0]
    assert gt["J3"]["axis"] == pytest.approx([0, 0, 1])


def test_load_gt_joints_rejects_missing_joint(tmp_path):
    p = tmp_path / "gt_joints.json"
    doc = {"joints": {"J1": {"point": [0, 0, 0], "axis": [0, 0, 1]}}}
    p.write_text(json.dumps(doc), encoding="utf-8")
    with pytest.raises(ValueError):
        joints_verify.load_gt_joints(str(p))


def test_position_is_point_to_gt_line():
    # Pred point off the GT line laterally by 3mm, shifted 100mm
    # along the axis: along-axis shift must not count (gauge).
    d = joints_verify.point_to_axis_line([3.0, 0.0, 100.0],
                                          [0, 0, 1], [0, 0, 0])
    assert d == pytest.approx(3.0)


def test_axis_uses_abs_cosine():
    assert joints_verify.cosine_abs([0, 0, 1], [0, 0, -1]) == \
        pytest.approx(1.0)
    assert joints_verify.cosine_abs([1, 0, 0], [0, 1, 0]) == \
        pytest.approx(0.0)


def test_score_joint_pass_and_fail():
    gt = {"point": [0, 0, 0], "axis": [0, 0, 1]}
    ok = joints_verify.score_joint([0, 0, -1], [4.0, 0, 50.0],
                                   gt, pos_tol_mm=5.0, tol_deg=5.0)
    assert ok["pass"] is True  # sign-equivalent, 4mm lateral
    bad = joints_verify.score_joint([1, 0, 0], [4.0, 0, 50.0],
                                    gt, pos_tol_mm=5.0, tol_deg=5.0)
    assert bad["pass"] is False  # perpendicular axis
    far = joints_verify.score_joint([0, 0, 1], [6.0, 0, 0.0],
                                    gt, pos_tol_mm=5.0, tol_deg=5.0)
    assert far["pass"] is False  # 6mm lateral
