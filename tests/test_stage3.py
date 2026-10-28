"""Stage 3 tests: transmission tools on synthetic fixtures.

Synthetic only. The real model (runs/, assets/private/) and ground
truth (eval/ground_truth/) are never read here.
"""

from __future__ import annotations

import pytest

from src.parse.ldr import parse_file, parse_text
from src.tools.transmission import (axle_carrier, gear_graph,
                                    solve_transmission)
import os

FIX = os.path.join(os.path.dirname(__file__), "fixtures")

CATALOG = {
    "gear8t.dat": {"category": "gear", "teeth": 8, "gear_kind": "spur",
                   "axis_local": [0, 0, 1]},
    "gear16t.dat": {"category": "gear", "teeth": 16, "gear_kind": "spur",
                    "axis_local": [0, 0, 1]},
    "gear24t.dat": {"category": "gear", "teeth": 24, "gear_kind": "spur",
                    "axis_local": [0, 0, 1]},
    "axle4.dat": {"category": "axle", "axis_local": [0, 1, 0],
                  "axle_length": 2},
}


def test_gear_graph_two_gear_train():
    from src.parse.ldr import parse_file
    parts = parse_file(os.path.join(FIX, "gears.ldr"))
    graph = gear_graph(parts, CATALOG)
    assert len(graph["nodes"]) == 2
    assert len(graph["edges"]) == 1
    edge = graph["edges"][0]
    assert {edge["ta"], edge["tb"]} == {8, 24}
    assert edge["kind"] == "external"
    assert graph["intra_meshes"] == []
    assert graph["motor_drive"] == {}


def test_gear_graph_link_split():
    # One rigid axle through two coaxial gears in different links
    # (e.g. a hinge pivot): without link_of they share a node, with
    # link_of observed link motion wins and they split.
    from src.parse.ldr import parse_text
    text = ("1 0 0 0 0 1 0 0 0 1 0 0 0 1 ga.dat\n"
            "1 0 0 0 0 1 0 0 0 1 0 0 0 1 gb.dat\n"
            "1 0 0 0 0 1 0 0 0 1 0 0 0 1 ax.dat\n")
    parts = parse_text(text)
    hole = [{"kind": "axle", "pos": [0, 0, 0]}]
    catalog = {
        "ga.dat": {"category": "gear", "teeth": 8, "gear_kind": "spur",
                   "axis_local": [0, 0, 1], "holes": hole},
        "gb.dat": {"category": "gear", "teeth": 24, "gear_kind": "spur",
                   "axis_local": [0, 0, 1], "holes": hole},
        "ax.dat": {"category": "axle", "axis_local": [0, 0, 1],
                   "axle_length": 8},
    }
    plain = gear_graph(parts, catalog)
    assert len(plain["nodes"]) == 1
    link_of = {parts[0]["uid"]: "L0", parts[1]["uid"]: "L1",
               parts[2]["uid"]: "L0"}
    split = gear_graph(parts, catalog, link_of=link_of)
    assert len(split["nodes"]) == 2
    assert split["edges"] == []
    assert split["intra_meshes"] == []


def test_solver_simple_two_gear_ratio():
    topo = {
        "motors": [{"id": "M1", "axle": "A0"}],
        "joints": [{"id": "J1", "axle": "A1"}],
        "meshes": [{"a": "A0", "ta": 8, "b": "A1", "tb": 24,
                    "kind": "external"}],
    }
    res = solve_transmission(topo)
    assert res["rank_ok"] is True
    assert res["C"][0][0] == pytest.approx(-8 / 24)
    assert res["signs"]["J1/M1"] == -1


def test_solver_idler_keeps_sign():
    # 8T - 16T(idler) - 24T: two external meshes give overall +.
    topo = {
        "motors": [{"id": "M1", "axle": "A0"}],
        "joints": [{"id": "J1", "axle": "A2"}],
        "meshes": [
            {"a": "A0", "ta": 8, "b": "A1", "tb": 16,
             "kind": "external"},
            {"a": "A1", "ta": 16, "b": "A2", "tb": 24,
             "kind": "external"},
        ],
    }
    res = solve_transmission(topo)
    assert res["rank_ok"] is True
    assert res["C"][0][0] == pytest.approx(8 / 24)
    assert res["signs"]["J1/M1"] == 1


def test_solver_internal_mesh_sign():
    topo = {
        "motors": [{"id": "M1", "axle": "A0"}],
        "joints": [{"id": "J1", "axle": "A1"}],
        "meshes": [{"a": "A0", "ta": 8, "b": "A1", "tb": 24,
                    "kind": "internal"}],
    }
    res = solve_transmission(topo)
    assert res["rank_ok"] is True
    assert res["C"][0][0] == pytest.approx(8 / 24)


def test_solver_hollow_carrier_coupling():
    # Shaft through a hollow rotating carrier: mesh carrier is J1,
    # J2 is measured relative to J1. Expect off-diagonal coupling.
    topo = {
        "motors": [{"id": "M1", "axle": "A0"},
                   {"id": "M2", "axle": "A2"}],
        "joints": [{"id": "J1", "axle": "A2"},
                   {"id": "J2", "axle": "A1", "carrier_joint": "J1"}],
        "meshes": [{"a": "A0", "ta": 8, "b": "A1", "tb": 24,
                    "kind": "external", "carrier_joint": "J1"}],
    }
    res = solve_transmission(topo)
    assert res["rank_ok"] is True
    C = res["C"]
    # J1 follows M2 directly.
    assert C[0][0] == pytest.approx(0.0, abs=1e-9)
    assert C[0][1] == pytest.approx(1.0)
    # J2 = -1/3 M1 + 1/3 M2.
    assert C[1][0] == pytest.approx(-8 / 24)
    assert C[1][1] == pytest.approx(8 / 24)
    assert abs(C[1][1]) > 0.02  # coupling term is significant


def test_solver_rank_deficient_without_motor():
    topo = {
        "motors": [],
        "joints": [{"id": "J1", "axle": "A1"}],
        "meshes": [{"a": "A0", "ta": 8, "b": "A1", "tb": 24,
                    "kind": "external"}],
    }
    res = solve_transmission(topo)
    assert res["rank_ok"] is False


def test_axle_carrier_majority_vote():
    links = [{"id": "L0", "members": ["p1", "p2", "p3"]},
             {"id": "L1", "members": ["p4", "p5"]}]
    assert axle_carrier(["p4", "p9"], links) == "L1"
    assert axle_carrier(["pz"], links) is None


def _perp_pair(dist_ldu: float, ta: int, ka: str, tb: int, kb: str):
    """Two gears, world axes Y and X (perpendicular), dist apart on X."""
    from src.parse.ldr import parse_text
    from src.tools.geometry import check_gear_mesh
    text = ("1 0 0 0 0 1 0 0 0 1 0 0 0 1 ga.dat\n"
            "1 0 %s 0 0 1 0 0 0 1 0 0 0 1 gb.dat\n" % dist_ldu)
    parts = parse_text(text)
    catalog = {
        "ga.dat": {"category": "gear", "teeth": ta, "gear_kind": ka,
                   "axis_local": [0, 0, 1]},
        "gb.dat": {"category": "gear", "teeth": tb, "gear_kind": kb,
                   "axis_local": [1, 0, 0]},
    }
    a, b = parts[0]["uid"], parts[1]["uid"]
    res = check_gear_mesh(a, b, parts, catalog)
    assert res["axis_relation"] == "perpendicular"
    return res


def test_bevel_pair_meshes_at_pitch_cone_distance():
    res = _perp_pair(23.2, 12, "bevel", 12, "bevel")
    assert res["kind"] == "bevel"
    assert res["meshes"] is True
    res = _perp_pair(40.0, 12, "bevel", 12, "bevel")
    assert res["meshes"] is False


def test_double_bevel_pair_meshes_at_cone_distance():
    res = _perp_pair(28.0, 12, "double-bevel", 12, "bevel")
    assert res["kind"] == "bevel"
    assert res["meshes"] is True


def test_knob_pair_meshes_by_lobe_interleave():
    res = _perp_pair(28.0, 4, "knob", 4, "knob")
    assert res["kind"] == "knob"
    assert res["meshes"] is True


def test_crown_pair_meshes_at_spur_sum_plus_allowance():
    res = _perp_pair(82.5, 56, "turntable", 8, "spur")
    assert res["kind"] == "crown"
    assert res["meshes"] is True
    res = _perp_pair(60.0, 56, "turntable", 8, "spur")
    assert res["meshes"] is False


def test_turntable_halves_never_mesh():
    res = _perp_pair(5.0, 56, "turntable", 56, "turntable")
    assert res["meshes"] is False


def test_parallel_turntable_ring_meshes_like_spur():
    from src.parse.ldr import parse_text
    from src.tools.geometry import check_gear_mesh
    text = ("1 0 0 0 0 1 0 0 0 1 0 0 0 1 ring.dat\n"
            "1 0 90 0 0 1 0 0 0 1 0 0 0 1 pin.dat\n")
    parts = parse_text(text)
    catalog = {
        "ring.dat": {"category": "turntable", "teeth": 60,
                     "gear_kind": "turntable", "axis_local": [0, 0, 1]},
        "pin.dat": {"category": "gear", "teeth": 12,
                    "gear_kind": "double-bevel", "axis_local": [0, 0, 1]},
    }
    res = check_gear_mesh(parts[0]["uid"], parts[1]["uid"], parts,
                          catalog)
    assert res["axis_relation"] == "parallel"
    assert res["expected_ldu"] == 90.0
    assert res["meshes"] is True


def _stacked_pair(with_axle: bool):
    """Two coaxial spur gears stacked 8 mm apart, optionally sharing
    one through-axle. Bores are cataloged off the shaft line so no
    rigid fit is detected (as in the real 16T+8T stacks)."""
    from src.parse.ldr import parse_text
    lines = ["1 16 0 0 0 1 0 0 0 1 0 0 0 1 ga.dat",
             "1 16 0 0 20 1 0 0 0 1 0 0 0 1 gb.dat"]
    if with_axle:
        lines.append("1 0 0 0 0 1 0 0 0 1 0 0 0 1 ax.dat")
    parts = parse_text("\n".join(lines) + "\n")
    hole = [{"kind": "round", "pos": [50, 0, 0]}]
    catalog = {
        "ga.dat": {"category": "gear", "teeth": 16, "gear_kind": "spur",
                   "axis_local": [0, 0, 1], "holes": hole},
        "gb.dat": {"category": "gear", "teeth": 8, "gear_kind": "spur",
                   "axis_local": [0, 0, 1], "holes": hole},
        "ax.dat": {"category": "axle", "axis_local": [0, 0, 1],
                   "axle_length": 8},
    }
    return parts, catalog


def test_stacked_gears_on_through_axle_are_compound():
    from src.tools.geometry import check_gear_mesh
    parts, catalog = _stacked_pair(True)
    res = check_gear_mesh(parts[0]["uid"], parts[1]["uid"], parts,
                          catalog)
    assert res["axis_relation"] == "parallel"
    assert res["kind"] == "compound"
    assert res["meshes"] is True


def test_stacked_gears_without_axle_do_not_compound():
    from src.tools.geometry import check_gear_mesh
    parts, catalog = _stacked_pair(False)
    res = check_gear_mesh(parts[0]["uid"], parts[1]["uid"], parts,
                          catalog)
    assert res["kind"] != "compound"
    assert res["meshes"] is False


def test_compound_edge_same_link_only():
    from src.tools.transmission import gear_graph
    parts, catalog = _stacked_pair(True)
    uids = [p["uid"] for p in parts]
    same = gear_graph(parts, catalog,
                      link_of={u: "L0" for u in uids})
    assert len(same["edges"]) == 1
    edge = same["edges"][0]
    assert edge["kind"] == "compound"
    assert (edge["ta"], edge["tb"]) == (1, 1)
    assert same["intra_meshes"] == []
    split = gear_graph(parts, catalog,
                       link_of={uids[0]: "L0", uids[1]: "L1",
                                uids[2]: "L0"})
    assert split["edges"] == []


def test_solver_compound_is_unit_ratio():
    topo = {
        "motors": [{"id": "M1", "axle": "A0"}],
        "joints": [{"id": "J1", "axle": "A1"}],
        "meshes": [{"a": "A0", "ta": 1, "b": "A1", "tb": 1,
                    "kind": "compound"}],
    }
    res = solve_transmission(topo)
    assert res["rank_ok"] is True
    assert res["C"][0][0] == pytest.approx(1.0)
    assert res["signs"]["J1/M1"] == 1


def test_solver_grounded_ring_internal_walk():
    # Planetary walk: motor-driven 12T pinion (A0) inside a grounded
    # T140 internal ring, carrier J1 about the shared vertical axis.
    # Internal form: 12*(w - q) = +(140)*(0 - q) -> q = -12/128 m.
    # The joint rides its own free axle (A1); the ring side is
    # ground (b None), so only its carrier term survives.
    topo = {
        "motors": [{"id": "M1", "axle": "A0"}],
        "joints": [{"id": "J1", "axle": "A1"}],
        "meshes": [{"a": "A0", "ta": 12, "b": None, "tb": 140,
                    "kind": "internal",
                    "carrier_a": {"J1": 1.0},
                    "carrier_b": {"J1": 1.0}}],
    }
    res = solve_transmission(topo)
    assert res["rank_ok"] is True
    assert res["C"][0][0] == pytest.approx(-12 / 128)
    assert res["signs"]["J1/M1"] == -1


def test_ring_kind_excluded_from_external_mesh():
    # A 140T ring beside an 8T spur at exactly the spur-sum
    # distance would mesh as external; rings engage internally
    # only, so the rule must refuse with its note.
    from src.parse.ldr import parse_text
    from src.tools.geometry import check_gear_mesh
    text = ("1 16 0 0 0 1 0 0 0 1 0 0 0 1 ga.dat\n"
            "1 16 185 0 0 1 0 0 0 1 0 0 0 1 gb.dat\n")
    parts = parse_text(text)
    catalog = {
        "ga.dat": {"category": "gear", "teeth": 140,
                   "gear_kind": "ring", "axis_local": [0, 0, 1]},
        "gb.dat": {"category": "gear", "teeth": 8,
                   "gear_kind": "spur", "axis_local": [0, 0, 1]},
    }
    res = check_gear_mesh(parts[0]["uid"], parts[1]["uid"], parts,
                          catalog)
    assert res["meshes"] is False
    assert res["note"] == "ring kind excluded from external mesh"


def test_joint_gears_rejects_off_axis_parallel_gear():
    # Quantized rotations leave ~1e-4 rad residual tilt between
    # parallel axes; the skew branch then reads a noise direction
    # (J1's 382 once passed at 1.11 mm while truly 64 mm off-axis),
    # so near-parallel axes must use lateral offset. The tilted
    # gear's axis is built in the LDraw frame (identity rotation
    # passes through unconverted) to control the tilt exactly.
    from src.parse.ldr import parse_text
    from src.agent.transmission import joint_gears
    text = ("1 16 0 0 0 1 0 0 0 1 0 0 0 1 ga.dat\n"
            "1 16 160 0 0 1 0 0 0 1 0 0 0 1 gb.dat\n")
    parts = parse_text(text)
    catalog = {
        "ga.dat": {"category": "gear", "teeth": 12,
                   "gear_kind": "double-bevel",
                   "axis_local": [0, 0, 1]},
        "gb.dat": {"category": "gear", "teeth": 12,
                   "gear_kind": "double-bevel",
                   "axis_local": [0.0001, 0, 1]},
    }
    joint = {"id": "J1", "axis": [0, 1, 0],
             "point_mm": [0.0, 0.0, 0.0], "parent": "D0",
             "child": "D1"}
    group_of = {p["uid"]: "D1" for p in parts}
    gears = joint_gears(joint, parts, catalog, group_of)
    assert [g["uid"] for g in gears] == [parts[0]["uid"]]
    assert gears[0]["line_dist_mm"] == pytest.approx(0.0, abs=0.5)


def test_path_product_tooth_chain():
    # Edges are node-oriented; traversal A0->A1->A2->A3 runs the
    # middle stage b->a, so its teeth invert (-(24/8) there).
    from src.agent.transmission import _path_product
    path = [{"a": "A0", "b": "A1", "ta": 8, "tb": 24,
             "kind": "external"},
            {"a": "A2", "b": "A1", "ta": 24, "tb": 8,
             "kind": "external"},
            {"a": "A2", "b": "A3", "ta": 12, "tb": 60,
             "kind": "external"}]
    res = _path_product(path, "A0")
    assert res["product"] == pytest.approx(-(8 / 24) * (8 / 24)
                                           * (12 / 60))
    assert res["sign"] == -1
    assert res["stages"] == 3
    internal = [{"a": "A0", "b": "A1", "ta": 12, "tb": 140,
                 "kind": "internal"}]
    assert _path_product(internal, "A0")["product"] == pytest.approx(
        12 / 140)
    assert _path_product([], None)["note"].startswith("hollow")


def _mini_div():
    return {"links": [{"id": "DP", "members": []},
                      {"id": "DC", "members": []}],
            "joints": [{"id": "J", "parent": "DP", "child": "DC",
                        "axis": [0, 1, 0],
                        "point_mm": [0.0, 0.0, 0.0]}]}


def test_keep_turntable_edge_drives_output_half():
    from src.agent.transmission import _keep_turntable_edge
    div = _mini_div()
    group_of = {"Hc": "DC", "Hp": "DP", "Pp": "DP", "Pc": "DC"}
    half_joint = {"Hc": "J", "Hp": "J"}

    def edge(a, b):
        return {"gear_a": a, "gear_b": b}

    # Parent-mounted pinion: keep child half, drop parent half
    # (identical to the old cross-link heuristic).
    assert _keep_turntable_edge(edge("Pp", "Hc"), "spur",
                               "turntable", group_of, half_joint,
                               div) is True
    assert _keep_turntable_edge(edge("Pp", "Hp"), "spur",
                               "turntable", group_of, half_joint,
                               div) is False
    # Child-mounted pinion: keep the OUTPUT half too (old rule
    # drove the parent and orphaned the joint output).
    assert _keep_turntable_edge(edge("Pc", "Hc"), "spur",
                               "turntable", group_of, half_joint,
                               div) is True
    assert _keep_turntable_edge(edge("Pc", "Hp"), "spur",
                               "turntable", group_of, half_joint,
                               div) is False
    # Unmapped half keeps the cross-link fallback.
    assert _keep_turntable_edge(edge("Pp", "Hx"), "spur",
                               "turntable",
                               {"Pp": "DP", "Hx": "DC"}, {}, div) \
        is True
    assert _keep_turntable_edge(edge("Pp", "Hx"), "spur",
                               "turntable",
                               {"Pp": "DP", "Hx": "DP"}, {}, div) \
        is False


def test_map_turntable_halves_by_coaxiality():
    from src.parse.ldr import parse_text
    from src.agent.transmission import _map_turntable_halves
    text = ("1 16 0 0 0 1 0 0 0 1 0 0 0 1 h1.dat\n"
            "1 16 160 0 0 1 0 0 0 1 0 0 0 1 h2.dat\n")
    parts = parse_text(text)
    catalog = {
        "h1.dat": {"category": "turntable", "teeth": 60,
                   "gear_kind": "turntable",
                   "axis_local": [0, 0, 1]},
        "h2.dat": {"category": "turntable", "teeth": 60,
                   "gear_kind": "turntable",
                   "axis_local": [0, 0, 1]},
    }
    div = _mini_div()
    mapping = _map_turntable_halves(parts, catalog, div)
    assert mapping[parts[0]["uid"]] == "J"
    assert parts[1]["uid"] not in mapping


def test_solver_internal_lock_coordinated_drive():
    # Grounded differential lock in internal (same-direction) form:
    # 16*w0 = +12*w1 with zero carriers. Independent single-motor
    # commands are infeasible (the lock fights them), but the
    # coordinated pattern (+side at 1.0, -side slaved by the lock)
    # solves exactly. This is the 6573 wrist form: 780 (12T)
    # drives the 16T -side, 086 rides the 16T +side shaft.
    from src.agent.transmission import _coordination_report
    topo = {
        "motors": [{"id": "M1", "axle": "A0"},
                   {"id": "M2", "axle": "A1"}],
        "joints": [{"id": "J1", "axle": "A2"}],
        "meshes": [
            {"a": "A0", "ta": 16, "b": "A1", "tb": 12,
             "kind": "internal", "note": "test diff (virtual)"},
            {"a": "A0", "ta": 20, "b": "A2", "tb": 40,
             "kind": "external"},
        ],
    }
    raw = solve_transmission(topo)
    assert raw["residual"] > 0.1  # independent columns infeasible
    rep = _coordination_report(topo)
    assert rep is not None
    assert rep["lock"] == "16*w_M1 -12*w_M2 = 0 (model pose)"
    assert rep["pattern"] == {"M1": 1.0, "M2": 16 / 12}
    assert rep["residual"] < 1e-9
    assert rep["response"] == {"J1": pytest.approx(-0.5)}
    assert _coordination_report({"motors": [], "joints": [],
                                "meshes": []}) is None


def test_confirmed_mapping_records_user_pairs_without_touching_solver():
    # User-confirmed 148->J3 / 149->J2 must be carried in the
    # report verbatim, while the solver inputs stay derived from
    # graph evidence only (149 has no CAD path, so no ratio).
    from src.agent.transmission import confirmed_mapping
    mapping = confirmed_mapping()
    by_motor = {m["motor"]: m for m in mapping}
    assert by_motor["p000148"]["joint"] == "J3"
    assert by_motor["p000149"]["joint"] == "J2"
    for m in mapping:
        assert set(m) == {"motor", "joint", "basis"}
        assert m["basis"]
    # Report-only: solving never sees it (no ratio/rate fields).
    assert all("ratio" not in m and "rate" not in m for m in mapping)
