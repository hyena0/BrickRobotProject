"""Stage 7 linear-joint extension tests (additive; revolute path untouched).

Synthetic fixtures only: no real model, no GT, no Spark calls.
"""

from __future__ import annotations

from src.agent.schema import validate_links_joints
from src.tools import linear as lin

CAT = {
    "pin8.dat": {"category": "gear", "teeth": 8, "gear_kind": "spur",
                 "axis_local": [0, 0, 1]},
    "rack14.dat": {"category": "rack", "teeth": 32,
                   "linear_axis": [1, 0, 0],
                   "tooth_line_offset": [0.0, -22.5, 0.0]},
    "rack_noline.dat": {"category": "rack", "teeth": 32,
                        "linear_axis": [1, 0, 0]},
}

RACK = {"uid": "r1", "part_id": "rack14.dat", "colour": "0",
        "position_mm": [0.0, 0.0, 0.0],
        "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}


def _pinion(uid, pos, axis=(0, 0, 1)):
    return {"uid": uid, "part_id": "pin8.dat", "colour": "0",
            "position_mm": list(pos),
            "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "_axis": axis}


def _cat(axis):
    c = dict(CAT)
    c["pin8.dat"] = {**c["pin8.dat"], "axis_local": list(axis)}
    return c


def test_rack_mesh_at_pitch_radius():
    # 8T pinion: R = 1.25*8 = 10 LDU = 4 mm. Tooth line at local
    # y=-22.5 LDU = -9 mm; pinion centre 4 mm above it meshes.
    parts = [RACK, _pinion("g1", [20.0, -5.0, 0.0])]
    res = lin.check_rack_mesh("g1", "r1", parts, CAT)
    assert res["meshes"] is True
    assert res["kind"] == "rack"
    assert res["expected_ldu"] == 10.0
    assert res["travel_mm_per_rev"] == 8 * 3.2


def test_rack_mesh_rejects_far_and_parallel():
    parts = [RACK, _pinion("g1", [20.0, 30.0, 0.0]),
             _pinion("g2", [20.0, -5.0, 0.0], axis=(1, 0, 0))]
    assert lin.check_rack_mesh("g1", "r1", parts, CAT)["meshes"] is False
    assert lin.check_rack_mesh("g2", "r1", parts,
                               _cat((1, 0, 0)))["meshes"] is False


def test_rack_mesh_needs_tooth_line_data():
    rack = dict(RACK, uid="r2", part_id="rack_noline.dat")
    parts = [rack, _pinion("g1", [20.0, -5.0, 0.0])]
    res = lin.check_rack_mesh("g1", "r2", parts, CAT)
    assert res["meshes"] is False
    assert "tooth line" in res["note"]


def test_order_loop_closes_and_reports_gaps():
    assert lin.order_loop(
        {"a": ["b"], "b": ["a", "c"], "c": ["b", "d"],
         "d": ["c", "a"]})["closed"] is True
    assert "dead end" in lin.order_loop(
        {"a": ["b"], "b": ["a"]})["gap"]
    fork = lin.order_loop(
        {"a": ["b"], "b": ["a", "c", "x"], "c": ["b"], "x": ["b"]})
    assert "branch at b" in fork["gap"]
    leaf = lin.order_loop(
        {"a": ["b", "c"], "b": ["a"], "c": ["a"]})
    assert "dead end at c" in leaf["gap"]
    assert "unexplored spurs at ['b']" in leaf["gap"]
    spur = lin.order_loop(
        {"a": ["b"], "b": ["a", "c", "x"], "c": ["b"], "x": ["b"]},
        {"a": [0.0, 0.0, 0.0], "b": [10.0, 0.0, 0.0],
         "c": [20.0, 0.0, 0.0], "x": [10.0, 8.0, 0.0]})
    assert "dead end at c" in spur["gap"]
    assert "unexplored spurs at ['x']" in spur["gap"]
    straight = lin.order_loop(
        {"a": ["b", "c"], "b": ["a", "c", "x"], "c": ["b", "a"],
         "x": ["b"]},
        {"a": [0.0, 0.0, 0.0], "b": [10.0, 0.0, 0.0],
         "c": [20.0, 0.0, 0.0], "x": [10.0, 10.0, 0.0]})
    assert straight["closed"] is True
    assert straight["order"] == ["b", "c", "a"] or \
        set(straight["order"]) == {"a", "b", "c"}
    assert "dead end" in lin.order_loop(
        {"a": ["b"], "b": ["a", "c"], "c": ["b"]})["gap"]


def test_chain_sprocket_rule_uses_pitch_radius():
    # 16T gear: pitch radius 1.25*16*0.4 = 8 mm. A loop point 8 mm
    # from centre adopts; 20 mm away does not.
    loop = ["l1"]
    gear = {"uid": "g16", "part_id": "gear16.dat", "colour": "0",
            "position_mm": [8.0, 0.0, 0.0],
            "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    link = {"uid": "l1", "part_id": "3711.dat", "colour": "0",
            "position_mm": [0.0, 0.0, 0.0],
            "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    cat = {"gear16.dat": {"category": "gear", "teeth": 16},
           "3711.dat": {"category": "other"}}
    assert lin.chain_sprockets([link, gear], cat, ["l1"])[0]["uid"] \
        == "g16"
    far = dict(gear, position_mm=[20.0, 0.0, 0.0])
    assert lin.chain_sprockets([link, far], cat, ["l1"]) == []


def test_motor_to_sprocket_bfs_product():
    parts = [{"uid": "m", "part_id": "54696.dat"},
             {"uid": "s", "part_id": "g16.dat"}]
    graph = {"nodes": [{"id": "A", "gears": []},
                       {"id": "B", "gears": [{"uid": "s"}]}],
             "edges": [{"a": "A", "b": "B", "ta": 8, "tb": 16,
                        "kind": "external"}],
             "motor_drive": {"m": "A"}}
    res = lin.motor_to_sprocket(
        parts, {"g16.dat": {"category": "gear", "teeth": 16}},
        graph, "m", "s", 6.4)
    assert res["ratio"] == -0.5
    assert res["mm_per_motor_rev"] == round(0.5 * 16 * 6.4, 2)


def _joint(jid="J1", jtype="prismatic"):
    return {"id": jid, "type": jtype, "parent": "L0", "child": "L1",
            "axis": [1, 0, 0], "point_mm": [0.0, 0.0, 0.0],
            "confidence": 0.8, "reason": "t",
            "travel_mm_per_rev": 25.6}


def test_schema_accepts_prismatic_leaves_revolute():
    doc = {"links": [{"id": "L0"}, {"id": "L1"}],
           "joints": [_joint("J1", "prismatic"), _joint("J2", "revolute")],
           "base": {"link": "L0"}, "end_effector": {"link": "L1"}}
    assert validate_links_joints(doc) == []
    bad = {"links": [{"id": "L0"}, {"id": "L1"}],
           "joints": [_joint("J1", "spherical")],
           "base": {"link": "L0"}, "end_effector": {"link": "L1"}}
    assert any("prismatic" in e or "revolute" in e
               for e in validate_links_joints(bad))
