"""Stage 5 pose-check tests: synthetic fixtures only.

The real model (runs/, assets/private/) and ground truth are never
read here. Blender (bpy) is never imported.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from src.viz.pose_check import (check_gears_pose, check_supports_pose,
                                collision_range_summary,
                                contact_analysis, extract_candidates,
                                find_cross_link_meshes, lateral_to_axis,
                                link_transforms, pair_penetration, q_of_m,
                                sample_poses, soft_limits, within_limits)


def make_doc():
    joints = [
        {"id": "J1", "type": "revolute", "parent": "D0", "child": "D1",
         "axis": [0, 0, 1], "point_mm": [0, 0, 0],
         "limits_deg": [-90, 90], "zero": "pose in the source file"},
        {"id": "J2", "type": "revolute", "parent": "D1", "child": "D2",
         "axis": [0, 1, 0], "point_mm": [0, 0, 100],
         "limits_deg": [-90, 90], "zero": "pose in the source file"},
    ]
    return {"frame": "base link, Z up, mm",
            "links": [{"id": "D0"}, {"id": "D1"}, {"id": "D2"}],
            "joints": joints,
            "actuators": [{"id": "m1", "brick": "left", "port": "A",
                           "motor": "medium"},
                          {"id": "m2", "brick": "left", "port": "B",
                           "motor": "medium"}],
            "transmission": {"coupling": "q = C m",
                             "C": [[1.0, 0.0], [0.0, 1.0]]},
            "end_effector": {"link": "D2", "point_mm": [0, 0, 100]}}


def link_index(doc):
    from src.kin.model import chain_order
    ordered = chain_order(doc)
    ids = [ordered[0]["parent"]] + [j["child"] for j in ordered]
    return {lid: n for n, lid in enumerate(ids)}


def test_sample_poses_count_limits_determinism():
    doc = make_doc()
    poses = sample_poses(doc, seed=3)
    assert len(poses) == 7 + min(3, 2)  # random + edge (capped by joints)
    lims = soft_limits(doc)
    assert lims == [[-85.0, 85.0], [-85.0, 85.0]]
    for p in poses:
        assert within_limits(p["q_deg"], lims)
        assert p["q_deg"] == q_of_m(
            p["m_deg"], doc["transmission"]["C"],
            ["m1", "m2"])
    again = sample_poses(doc, seed=3)
    assert [p["q_deg"] for p in again] == [p["q_deg"] for p in poses]
    edge = poses[7:]
    assert any(max(abs(v) for v in p["q_deg"]) >= 0.90 * 85.0
               for p in edge)


def test_sample_poses_resample_rejects_violations():
    doc = make_doc()
    doc["joints"][0]["limits_deg"] = [-10, 10]
    poses = sample_poses(doc, seed=3)
    lims = soft_limits(doc)
    assert lims[0] == [-5.0, 5.0]
    assert all(within_limits(p["q_deg"], lims) for p in poses)


def test_link_transforms_identity_at_zero():
    doc = make_doc()
    for t in link_transforms(doc, [0.0, 0.0]):
        assert t == pytest.approx(np.eye(4))


def test_link_transforms_match_fk_on_diagonal_axis():
    # Regression: R[1][0] used x*z instead of x*y, invisible for
    # principal axes. A diagonal axis must agree with PoE FK.
    from src.kin.fk import fk_model
    doc = make_doc()
    doc["joints"][0]["axis"] = [0.7071, 0.7071, 0.0]
    doc["end_effector"] = {"link": "D2", "point_mm": [50, 50, 100]}
    q = [37.0, -23.0]
    t_mats = link_transforms(doc, q)
    ee0 = np.asarray(doc["end_effector"]["point_mm"], dtype=float)
    via_links = (t_mats[-1][:3, :3] @ ee0) + t_mats[-1][:3, 3]
    via_fk = np.asarray(fk_model(q, doc)["pos_mm"])
    assert via_links == pytest.approx(via_fk, abs=1e-6)


def test_link_transforms_child_rotates_about_axis():
    doc = make_doc()
    t_mats = link_transforms(doc, [90.0, 0.0])
    assert t_mats[0] == pytest.approx(np.eye(4))  # base never moves
    on_axis = (t_mats[1][:3, :3] @ np.array([0.0, 0.0, 7.0])) + \
        t_mats[1][:3, 3]
    assert on_axis == pytest.approx([0.0, 0.0, 7.0])
    off = (t_mats[1][:3, :3] @ np.array([10.0, 0.0, 0.0])) + \
        t_mats[1][:3, 3]
    assert off == pytest.approx([0.0, 10.0, 0.0])


def test_lateral_to_axis():
    assert lateral_to_axis([1.0, 0.0, 5.0], [0, 0, 0], [0, 0, 1]) == \
        pytest.approx(1.0)
    assert lateral_to_axis([0.0, 0.0, -3.0], [0, 0, 0], [0, 0, 1]) == \
        pytest.approx(0.0)


def test_check_supports_on_axis_stay_ok():
    doc = make_doc()
    li = link_index(doc)
    supports = {"J1": [{"uid": "p1", "point_mm": [0, 0, 5],
                        "link": "D1"},
                       {"uid": "p2", "point_mm": [0, 0, -2],
                        "link": "D0"}],
                "J2": []}
    rows = check_supports_pose(doc, supports, li,
                               link_transforms(doc, [90.0, 0.0]),
                               [90.0, 0.0])
    assert rows[0]["ok"] and rows[0]["worst_lateral_mm"] == \
        pytest.approx(0.0)


def test_check_supports_misattributed_flagged_with_uid():
    # On-axis at zero but owned by the wrong link: J2 motion carries it
    # off the J1 axis -> violation naming the part.
    doc = make_doc()
    li = link_index(doc)
    supports = {"J1": [{"uid": "bad-part", "point_mm": [0.3, 0.0, 5.0],
                        "link": "D2"}],
                "J2": []}
    rows = check_supports_pose(doc, supports, li,
                               link_transforms(doc, [0.0, 90.0]),
                               [0.0, 90.0])
    assert not rows[0]["ok"]
    assert rows[0]["violations"][0]["uid"] == "bad-part"
    assert rows[0]["violations"][0]["lateral_mm"] > 0.5


def test_check_supports_gating_at_zero():
    # A support 3 mm off the axis at zero is interface structure, not
    # an axis support: tracked by drift, never a violation.
    doc = make_doc()
    li = link_index(doc)
    supports = {"J1": [{"uid": "far-part", "point_mm": [3.0, 0.0, 5.0],
                        "link": "D1"}],
                "J2": []}
    rows = check_supports_pose(doc, supports, li,
                               link_transforms(doc, [90.0, 0.0]),
                               [90.0, 0.0])
    assert rows[0]["ok"]
    assert rows[0]["n_axis_supports"] == 0
    assert rows[0]["max_drift_mm"] == pytest.approx(0.0)


def test_check_gears_same_link_preserved_cross_link_drifts():
    doc = make_doc()
    li = link_index(doc)
    by_uid = {"g0": {"position_mm": [10, 0, 0]},
              "g1": {"position_mm": [20, 0, 0]},
              "g2": {"position_mm": [10, 0, 50]}}
    pairs = [{"a": "g0", "b": "g1", "kind": "external",
              "nominal_mm": 10.0, "link_a": "D0", "link_b": "D0"},
             {"a": "g0", "b": "g2", "kind": "external",
              "nominal_mm": 50.0, "link_a": "D0", "link_b": "D1"}]
    rows = check_gears_pose(pairs, by_uid, li,
                            link_transforms(doc, [90.0, 0.0]))
    assert rows[0]["ok"] and rows[0]["drift_mm"] == pytest.approx(0.0)
    assert not rows[1]["ok"]
    assert rows[1]["drift_mm"] > 0.5


def test_pair_penetration_sign():
    assert pair_penetration([0, 0, 0], [10, 10, 10],
                            [5, 5, 5], [15, 15, 15]) == pytest.approx(5.0)
    assert pair_penetration([0, 0, 0], [10, 10, 10],
                            [10, 10, 10], [20, 20, 20]) == \
        pytest.approx(0.0)  # touching faces
    assert pair_penetration([0, 0, 0], [10, 10, 10],
                            [50, 50, 50], [60, 60, 60]) < 0.0


def _box(lo, size=10.0):
    lo = np.asarray(lo, dtype=float)
    return (lo, lo + size)


def test_contact_analysis_flags_separated_now_touching():
    link_of = {"a": "D0", "b": "D1", "c": "D1"}
    z = {"a": _box([0, 0, 0]), "b": _box([50, 0, 0]),
         "c": _box([0, 0, 0])}
    p = {"a": _box([0, 0, 0]), "b": _box([5, 0, 0]),
         "c": _box([0, 0, 0])}
    out = contact_analysis(z, p, link_of)
    # (a,b): separated by 40 at zero, overlapping at pose -> contact.
    # (a,c): overlapping at zero -> baseline. (b,c): same link.
    assert [(c["a"], c["b"]) for c in out["contacts"]] == [("a", "b")]
    assert out["n_contacts"] == 1
    assert out["contacts"][0]["zero_gap_mm"] == pytest.approx(40.0)


def test_contact_analysis_adjacent_band_not_failed():
    link_of = {"a": "D0", "b": "D1"}
    z = {"a": _box([0, 0, 0]), "b": _box([11, 0, 0])}  # 1 mm gap
    p = {"a": _box([0, 0, 0]), "b": _box([5, 0, 0])}
    out = contact_analysis(z, p, link_of)
    assert out["n_contacts"] == 0
    assert out["n_adjacent"] == 1


def test_find_cross_link_meshes_same_link_excluded():
    parts = [{"uid": "g1", "part_id": "x.dat",
              "position_mm": [0, 0, 0], "rotation": [[1, 0, 0]] * 3},
             {"uid": "g2", "part_id": "x.dat",
              "position_mm": [5, 0, 0], "rotation": [[1, 0, 0]] * 3}]
    catalog = {"x.dat": {"category": "gear", "teeth": 8}}
    assert find_cross_link_meshes(parts, catalog,
                                  {"g1": "D0", "g2": "D0"}) == []


def test_collision_range_summary_shape():
    report = {"poses": [
        {"q_deg": [10.0, 0.0], "contacts": []},
        {"q_deg": [-170.0, 5.0],
         "contacts": [{"a": "u1", "b": "u2"}]},
        {"q_deg": [170.0, -5.0],
         "contacts": [{"a": "u3", "b": "u4"}]},
    ]}
    # joint ids come from the first pose's supports entries
    report["poses"] = [
        dict(p, supports=[{"joint": "J1"}, {"joint": "J2"}])
        for p in report["poses"]]
    rows = collision_range_summary(report)
    assert rows[0]["joint"] == "J1"
    assert rows[0]["colliding_q_deg"] == [-170.0, 170.0]
    assert rows[0]["clean_min_deg"] == 10.0


def test_collision_range_summary_all_colliding():
    report = {"poses": [
        dict(q_deg=[10.0, 0.0], contacts=[{"a": "u1", "b": "u2"}],
             supports=[{"joint": "J1"}, {"joint": "J2"}]),
    ]}
    rows = collision_range_summary(report)
    assert rows[0]["colliding_q_deg"] == [10.0]
    assert rows[0]["clean_min_deg"] is None
    assert rows[0]["clean_max_deg"] is None


def test_extract_candidates_excludes_known_meshes_and_caps():
    pairs = [{"a": "g1", "b": "g2"}]
    report = {"poses": [
        {"id": "p1",
         "contacts": [{"a": "g1", "b": "g2", "pose_depth_mm": 50.0},
                      {"a": "x", "b": "y", "pose_depth_mm": 5.0},
                      {"a": "m", "b": "n", "pose_depth_mm": 30.0}]}]}
    out = extract_candidates(report, pairs, top_n=1)
    assert out == {"p1": [["m", "n"]]}


def test_collision_range_prefers_mesh_truth():
    report = {"poses": [
        dict(q_deg=[10.0, 0.0],
             contacts=[{"a": "u1", "b": "u2"}],
             mesh_contacts=[],
             supports=[{"joint": "J1"}, {"joint": "J2"}]),
    ]}
    rows = collision_range_summary(report)
    assert rows[0]["colliding_q_deg"] == []
    assert rows[0]["clean_min_deg"] == 10.0
