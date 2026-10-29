"""Convention-alignment tests: synthetic fixtures only.

Covers src/verify/conventions.py (per-joint sign mapping, EE roll
gauge) and its wiring in src/verify/ik_eval.py. The real model
(runs/, assets/private/) and ground truth (eval/ground_truth/, gt/)
are never read here.
"""

from __future__ import annotations

import copy
import math

import pytest

from src.verify.conventions import (align_ee_gauge, apply_signs,
                                    roll_joint_id, sign_map)
from src.verify.ik_eval import evaluate
from tests.test_stage4 import make_doc


def flip_wrist(doc):
    out = copy.deepcopy(doc)
    for j in out["joints"]:
        if j["id"] in ("J4", "J5", "J6"):
            j["axis"] = [-x for x in j["axis"]]
    return out


def test_sign_map_identity():
    doc = make_doc()
    assert sign_map(doc, doc) == {f"J{i + 1}": 1 for i in range(6)}


def test_sign_map_finds_flips():
    doc = make_doc()
    got = sign_map(doc, flip_wrist(doc))
    assert got == {"J1": 1, "J2": 1, "J3": 1,
                   "J4": -1, "J5": -1, "J6": -1}


def test_sign_map_rejects_id_mismatch():
    doc = make_doc()
    other = copy.deepcopy(doc)
    other["joints"][0]["id"] = "JX"
    with pytest.raises(ValueError):
        sign_map(doc, other)


def test_sign_map_rejects_perpendicular():
    doc = make_doc()
    other = copy.deepcopy(doc)
    other["joints"][2]["axis"] = [0, 0, 1]
    other["joints"][2]["point_mm"] = [0, 0, 200]
    with pytest.raises(ValueError):
        sign_map(doc, other)


def test_apply_signs_roundtrip():
    doc = make_doc()
    signs = sign_map(doc, flip_wrist(doc))
    assert apply_signs(flip_wrist(doc), signs)["joints"] == doc["joints"]


def test_roll_joint_is_last_child():
    doc = make_doc()
    doc["end_effector"] = {"link": "D6"}
    assert roll_joint_id(doc) == "J6"


def test_align_ee_gauge_only_along_axis():
    doc = make_doc()
    other = copy.deepcopy(doc)
    other["end_effector"]["point_mm"] = [206.0, 0.0, 300.0]
    aligned, offset = align_ee_gauge(doc, other)
    assert offset == pytest.approx(6.0)
    new = aligned["end_effector"]["point_mm"]
    assert new == pytest.approx([206.0, 0.0, 300.0])


def test_eval_recovers_flipped_conventions():
    pred = make_doc()
    gt = flip_wrist(pred)
    gt["end_effector"]["point_mm"] = [206.0, 0.0, 300.0]
    bad = evaluate(pred, gt, n=10, seed=0, restarts=8,
                   align_signs=False, align_ee=False)
    assert bad["success_rate"] < 0.95
    good = evaluate(pred, gt, n=10, seed=0, restarts=8)
    assert good["gate_pass"] is True
    assert good["signs"] == {"J1": 1, "J2": 1, "J3": 1,
                             "J4": -1, "J5": -1, "J6": -1}
    assert good["ee_roll_gauge_mm"] == pytest.approx(6.0)
