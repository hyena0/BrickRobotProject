"""Tier-1 chain joint tests (Stage 2.10, synthetic only)."""

from __future__ import annotations

import math

import numpy as np

from src.tools import chain_joints as chain

IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

CATALOG = {
    "axle.dat": {"category": "axle", "axis_local": [1, 0, 0],
                 "axle_length": 4, "holes": []},
    "beam.dat": {"category": "beam", "axis_local": None,
                 "holes": [{"kind": "round", "pos": [0, 0, 0]}]},
    "bush.dat": {"category": "bush", "axis_local": None,
                 "holes": [{"kind": "axle", "pos": [0, 0, 0]}]},
    "gear.dat": {"category": "gear", "axis_local": [0, 0, 1], "teeth": 8,
                 "holes": [{"kind": "axle", "pos": [0, 0, 0]}]},
    "ringA.dat": {"category": "turntable", "axis_local": [0, 1, 0],
                  "holes": [{"kind": "round", "pos": [10, 0, 0]},
                            {"kind": "round", "pos": [-10, 0, 0]},
                            {"kind": "round", "pos": [0, 10, 0]}]},
    "ringB.dat": {"category": "turntable", "axis_local": [0, 1, 0],
                  "holes": [{"kind": "round", "pos": [10, 0, 0]},
                            {"kind": "round", "pos": [-10, 0, 0]},
                            {"kind": "round", "pos": [0, 10, 0]}]},
}


def _part(uid, pid, pos):
    return {"uid": uid, "part_id": pid, "position_mm": list(pos),
            "rotation": [list(r) for r in IDENT]}


def test_nested_symmetric_rings_anchor():
    # Symmetric ring halves offset from the world origin nest by
    # origin coincidence; direction follows the rotor (dn) side.
    cat = dict(CATALOG)
    cat["ringA.dat"] = {"category": "turntable", "axis_local": [0, 1, 0],
                        "holes": []}
    cat["ringB.dat"] = {"category": "turntable", "axis_local": [0, 1, 0],
                        "holes": []}
    parts = [_part("top", "ringA.dat", [100, 0, 0.5]),
             _part("bot", "ringB.dat", [100, 0, -0.5])]
    anc = chain.nested_anchor({"uid": "top"}, {"uid": "bot"},
                              parts, cat)
    assert anc["ok"]
    assert abs(anc["center"][0] - 100.0) < 0.5
    assert abs(anc["direction"][2]) > 0.99


def test_pin_both_clamped_is_boundary():
    # Gears clamp the axle on both sides -> rigid across -> boundary.
    parts = [_part("axle", "axle.dat", [8, 0, 0]),
             _part("gearA", "gear.dat", [0, 0, 0]),
             _part("gearB", "gear.dat", [16, 0, 0])]
    anchors = chain.pin_anchors({"gearA"}, {"gearB"}, parts, CATALOG)
    assert len(anchors) == 1
    cls, _note = chain.classify_pin(anchors[0])
    assert cls == "boundary"


def test_bush_bore_counts_as_bearing():
    # Bush bores are bearings (Tier-1 rule): the axle spins in the
    # bush, so bush fits never clamp, even on both sides.
    parts = [_part("axle", "axle.dat", [8, 0, 0]),
             _part("bushA", "bush.dat", [0, 0, 0]),
             _part("bushB", "bush.dat", [16, 0, 0])]
    anchors = chain.pin_anchors({"bushA"}, {"bushB"}, parts, CATALOG)
    assert len(anchors) == 1
    cls, _note = chain.classify_pin(anchors[0])
    assert cls == "hinge"


def test_verify_passes_true_hinge():
    parts = [_part("beamA", "beam.dat", [0, 0, 0]),
             _part("beamB", "beam.dat", [16, 0, 0]),
             _part("pin", "axle.dat", [8, 0, 0])]
    anchors = chain.pin_anchors({"beamA"}, {"beamB"}, parts, CATALOG)
    assert len(anchors) == 1
    ver = chain.verify_joint([8, 0, 0], [1, 0, 0], {"beamA"}, {"beamB"},
                             [{**anchors[0], "kind": "pin"}],
                             parts, CATALOG)
    assert ver["verdict"] == "pass", ver["notes"]


def test_verify_fails_wrong_axis():
    parts = [_part("beamA", "beam.dat", [0, 0, 0]),
             _part("beamB", "beam.dat", [16, 0, 0]),
             _part("pin", "axle.dat", [8, 0, 0])]
    anchors = chain.pin_anchors({"beamA"}, {"beamB"}, parts, CATALOG)
    # Rotate about a parallel but 5mm-offset axis: bearing must part.
    ver = chain.verify_joint([8, 5, 0], [1, 0, 0], {"beamA"}, {"beamB"},
                             [{**anchors[0], "kind": "pin"}],
                             parts, CATALOG)
    assert ver["verdict"] == "fail"
    assert any("coaxial" in n for n in ver["notes"])


def test_rack_bundle_fits_stack_axis():
    cat = dict(CATALOG)
    cat["arc.dat"] = {"category": "rack", "axis_local": None, "holes": []}
    parts = [_part(f"a{i}", "arc.dat", [21.0, 79.0, -150.0 - i])
             for i in range(4)] + \
        [_part(f"b{i}", "arc.dat", [21.0, 79.0, -143.0 - i])
         for i in range(4)]
    up = {f"a{i}" for i in range(4)}
    dn = {f"b{i}" for i in range(4)}
    rb = chain.rack_bundle(up, dn, parts, cat)
    assert rb["ok"] and rb["spread"] == "linear"
    assert abs(rb["direction"][2]) > 0.99
    assert abs(rb["center"][0] - 21.0) < 0.1
    ver = chain.verify_joint(rb["center"], rb["direction"], up, dn,
                             [{**rb, "uids": rb["uids"]}],
                             parts, cat)
    assert ver["verdict"] == "pass", ver["notes"]


def test_nested_pair_anchors_despite_axis_mismatch():
    # C3': teeth/mismatch never disqualify a nested pair; the mismatch
    # is a diagnostic (C3'').
    cat = dict(CATALOG)
    cat["ringC.dat"] = {"category": "turntable", "axis_local": [0, 1, 0],
                        "holes": [{"kind": "round", "pos": [10, 0, 0]},
                                  {"kind": "round", "pos": [-10, 0, 0]},
                                  {"kind": "round", "pos": [0, 10, 0]}]}
    cat["ringD.dat"] = {"category": "turntable",
                        "axis_local": [1, 0, 0],  # 90 deg off, like mesh
                        "holes": [{"kind": "round", "pos": [10, 0, 0]},
                                  {"kind": "round", "pos": [-10, 0, 0]},
                                  {"kind": "round", "pos": [0, 10, 0]}]}
    parts = [_part("rc", "ringC.dat", [0, 0, 0]),
             _part("rd", "ringD.dat", [0, 0, 0.2])]
    anc = chain.nested_anchor({"uid": "rc"}, {"uid": "rd"}, parts, cat)
    assert anc["ok"]
    assert anc["diagnostics"]["nest_dist_mm"] < 2.0
    assert "rd" in anc["diagnostics"].get("axis_mismatch_deg", {})


def test_non_nested_pair_rejected():
    cat = dict(CATALOG)
    cat["ringC.dat"] = {"category": "turntable", "axis_local": [0, 1, 0],
                        "holes": [{"kind": "round", "pos": [10, 0, 0]},
                                  {"kind": "round", "pos": [-10, 0, 0]},
                                  {"kind": "round", "pos": [0, 10, 0]}]}
    parts = [_part("rc", "ringC.dat", [0, 0, 0]),
             _part("rc2", "ringC.dat", [50, 0, 0])]
    anc = chain.nested_anchor({"uid": "rc"}, {"uid": "rc2"}, parts, cat)
    assert not anc["ok"]


def test_judge_clamped_drive_becomes_shaft():
    # Geared axle failing C5 as a bearing -> drive shaft.
    parts = [_part("axle", "axle.dat", [8, 0, 0]),
             _part("gearA", "gear.dat", [0, 0, 0]),
             _part("gearB", "gear.dat", [16, 0, 0])]
    anchors = chain.pin_anchors({"gearA"}, {"gearB"}, parts, CATALOG)
    assert len(anchors) == 1
    cls, _ = chain.judge_clamped(anchors[0], [8, 40, 0], [1, 0, 0],
                                 {"gearA"}, {"gearB"}, parts, CATALOG)
    assert cls == "shaft"


def test_chain_needs_seven_groups():
    doc = chain.infer_chain.__name__
    assert doc == "infer_chain"
    assert chain.CHAIN == ["D0", "D1", "D2", "D3", "D4", "D5", "D6"]


def _ring_lib_text(pid="ringT.dat"):
    # Synthetic annulus: outer R=20 / inner r=10 octagons at y=+-1.
    lines = [f"0 FILE {pid}", "0 Name: synthetic test annulus"]
    for y in (-1.0, 1.0):
        for i in range(8):
            a0 = 2 * math.pi * i / 8
            a1 = 2 * math.pi * (i + 1) / 8
            o0 = (20 * math.cos(a0), y, 20 * math.sin(a0))
            o1 = (20 * math.cos(a1), y, 20 * math.sin(a1))
            n0 = (10 * math.cos(a0), y, 10 * math.sin(a0))
            n1 = (10 * math.cos(a1), y, 10 * math.sin(a1))
            nums = " ".join(f"{v:.4f}" for p in (o0, o1, n1, n0)
                            for v in p)
            lines.append(f"4 16 {nums}")
    return "\n".join(lines) + "\n"


def _ring_lib():
    from src.catalog.review import Library

    return Library(_ring_lib_text())


RING_CAT = {
    "ringT.dat": {"category": "turntable", "axis_local": [0, 1, 0],
                  "holes": []},
}


def test_annulus_joint_axis_ignores_third_party():
    # C3''': the axis is the joint annulus-plane fit even when the
    # rotor-side catalog axis points elsewhere (drive-oriented X).
    cat = {"ringT.dat": {"category": "turntable",
                         "axis_local": [0, 1, 0], "holes": []},
           "ringD.dat": {"category": "turntable",
                         "axis_local": [1, 0, 0], "holes": []}}
    from src.catalog.review import Library

    lib = Library(_ring_lib_text() + _ring_lib_text("ringD.dat"))
    parts = [_part("top", "ringT.dat", [0, 0, 0.3]),
             _part("bot", "ringD.dat", [0, 0, -0.3])]
    anc = chain.nested_anchor({"uid": "top"}, {"uid": "bot"},
                              parts, cat, lib)
    assert anc["ok"]
    assert anc["diagnostics"]["axis_source"] == "annulus-joint"
    # Annulus planes are normal to Y(LDraw)/Z(world); the X catalog
    # axis must not be followed.
    assert abs(anc["direction"][2]) > 0.99
    assert abs(anc["ring_radius"] - 8.0) < 0.3
    assert anc["halves_lateral_mm"] < 0.5


def test_c5_sense_pass_and_c5n_rejects():
    # Restored C5(c) passes the true axis; C5-n rejects a 90deg error.
    lib = _ring_lib()
    parts = [_part("top", "ringT.dat", [0, 0, 0.3]),
             _part("bot", "ringT.dat", [0, 0, -0.3])]
    anc = chain.nested_anchor({"uid": "top"}, {"uid": "bot"},
                              parts, RING_CAT, lib)
    assert anc["ok"]
    ver = chain.verify_joint(anc["center"], anc["direction"],
                             {"top"}, {"bot"}, [anc], parts,
                             RING_CAT, lib)
    assert ver["verdict"] == "pass", (ver["notes"], ver["sense"])
    ctrl = chain.negative_control(anc["center"], anc["direction"],
                                  {"top"}, {"bot"}, [anc], parts,
                                  RING_CAT, lib)
    assert ctrl["rejected"], ctrl
    assert ctrl["verify_verdict"] == "fail"


def test_sweep_passes_true_axis():
    lib = _ring_lib()
    parts = [_part("top", "ringT.dat", [0, 0, 0.3]),
             _part("bot", "ringT.dat", [0, 0, -0.3])]
    sw = chain.sweep_penetration([0, 0, 0], [0, 0, 1],
                                 {"top"}, {"bot"}, parts, lib)
    assert sw["verdict"] == "pass", sw["new_penetrations"]


def _box_lib():
    from src.catalog.review import Library

    c = [-10.0, 10.0]
    v = [(x, y, z) for x in c for y in c for z in c]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 2, 6, 4),
             (1, 5, 7, 3), (0, 4, 5, 1), (2, 3, 7, 6)]
    lines = ["0 FILE box.dat", "0 Name: synthetic test box"]
    for q in quads:
        nums = " ".join(f"{v[i][k]:.1f}" for i in q for k in range(3))
        lines.append(f"4 16 {nums}")
    return Library("\n".join(lines) + "\n")


def test_mesh_penetration_verdict():
    # Overlapping boxes intersect; separated boxes report the gap.
    lib = _box_lib()
    ta = chain._tris_local(lib, "box.dat")
    assert ta is not None and len(ta) == 12
    wa = chain._world_tris([0, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                           ta)
    wb = chain._world_tris([6, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                           ta)
    inside, _gap = chain._meshes_intersect(wa, wb)
    assert inside
    wc = chain._world_tris([20, 0, 0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                           ta)
    inside, gap = chain._meshes_intersect(wa, wc)
    assert not inside
    # Boxes span +-4mm; centres 20mm apart -> 12mm gap.
    assert abs(gap - 12.0) < 0.5, gap


def _sweep_parts(offset):
    # Dn box starts on +X at R=20 and swings to (17.32, 10) about the
    # Z axis at +30deg; the up box sits `offset` away from there,
    # turned 30deg so corners bite faces with real depth (axis-aligned
    # box corners land exactly on faces, hiding depth).
    c, s = 0.8660254, 0.5
    rot30 = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    up = {"uid": "up", "part_id": "box.dat",
          "position_mm": [17.32 + offset, 10, 0],
          "rotation": [list(r) for r in rot30]}
    dn = {"uid": "dn", "part_id": "box.dat",
          "position_mm": [20, 0, 0],
          "rotation": [list(r) for r in ident]}
    return [up, dn]


def test_sweep_depth_gates_verdict():
    # Deep sweep overlap (6mm) rejects; shallow graze (1mm) passes
    # as near-miss. Both pairs start clear at angle 0.
    lib = _box_lib()
    sw = chain.sweep_penetration([0, 0, 0], [0, 0, 1], {"up"}, {"dn"},
                                 _sweep_parts(2), lib)
    assert sw["verdict"] == "fail", sw["new_penetrations"]
    first = next(iter(sw["new_penetrations"].values()))[0]
    assert first[4] > chain.PENETRATE_DEPTH_MM, first
    sw = chain.sweep_penetration([0, 0, 0], [0, 0, 1], {"up"}, {"dn"},
                                 _sweep_parts(7), lib)
    assert sw["verdict"] == "pass", sw["new_penetrations"]
    assert sw["near_miss_sub_mm"], "shallow graze must be logged"


def test_confirmed_counts_real_burial():
    # Obliquely overlapped boxes confirm (stencil stays inside) with
    # real depth; axis-aligned corners sit exactly on faces (depth 0).
    lib = _box_lib()
    ta = chain._tris_local(lib, "box.dat")
    c, s = 0.8660254, 0.5
    rot30 = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    wa = chain._world_tris([0, 0, 0], ident, ta)
    wb = chain._world_tris([2, 0, 0], rot30, ta)
    n_in, depth, n_conf = chain._contain_confirmed(wa, wb)
    assert n_in > 0 and n_conf > 0
    assert depth > 0.3, depth


def test_contain_depth_measures_overlap():
    # Point-to-mesh depth on a known interior point (box corners sit
    # exactly on faces, so presence and depth are tested separately).
    lib = _box_lib()
    ta = chain._tris_local(lib, "box.dat")
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    wb = chain._world_tris([5, 0, 0], ident, ta)
    got = chain._point_tri_depth([[3, 0, 0]], wb)
    # Box B spans x in [1, 9]: interior point at x=3 -> depth 2.
    assert abs(float(got[0]) - 2.0) < 0.3, got


def test_tris_cross_finds_stab_not_hover():
    # Oblique deep overlap crosses shells; separated boxes do not.
    lib = _box_lib()
    ta = chain._tris_local(lib, "box.dat")
    c, s = 0.8660254, 0.5
    rot30 = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    wa = chain._world_tris([0, 0, 0], ident, ta)
    wb = chain._world_tris([2, 0, 0], rot30, ta)
    cross, n = chain._tris_cross(wa, wb)
    assert cross and n >= 3, (cross, n)
    wc = chain._world_tris([20, 0, 0], ident, ta)
    cross, _n = chain._tris_cross(wa, wc)
    assert not cross


def test_decimate_keeps_box():
    # 20mm box on a 1mm voxel: corners survive, faces intact.
    lib = _box_lib()
    ta = chain._tris_local(lib, "box.dat")
    dec = chain._decimate(np.asarray(ta, dtype=float), 1.0)
    assert len(dec) == 12, len(dec)
    lo, hi = dec.reshape(-1, 3).min(axis=0), dec.reshape(-1, 3).max(axis=0)
    assert list(np.round(lo, 1)) == [-4.0, -4.0, -4.0]
    assert list(np.round(hi, 1)) == [4.0, 4.0, 4.0]


def test_spin_free_off_axis_only():
    # Bore-mounted gear off the axis spins; centred on it it does not.
    parts = [_part("gearX", "gear.dat", [8, 0, 0]),
             _part("axleP", "axle.dat", [8, 0, 0])]
    by_uid = {p["uid"]: p for p in parts}
    assert chain._spin_free("gearX", parts, CATALOG, by_uid,
                            [40, 0, 0], [0, 0, 1], {})
    assert not chain._spin_free("gearX", parts, CATALOG, by_uid,
                                [8, 0, 0], [0, 0, 1], {})


def test_spinning_gear_detected():
    # A toothed gear riding an axle through an axle-hole seat spins
    # (transmission); a beam is structure even with holes.
    parts = [_part("gearX", "gear.dat", [8, 0, 0]),
             _part("axleP", "axle.dat", [8, 0, 0]),
             _part("beam", "beam.dat", [40, 0, 0])]
    by_uid = {p["uid"]: p for p in parts}
    assert chain._spinning_gear("gearX", parts, CATALOG, by_uid, {})
    assert not chain._spinning_gear("beam", parts, CATALOG, by_uid,
                                    {})
    assert not chain._spinning_gear("axleP", parts, CATALOG, by_uid,
                                    {})


def test_axle_membership_follows_seats():
    # Axle line hits both groups but pierces a hole only downstream:
    # it rides with dn under C5.
    cat = dict(CATALOG)
    cat["beamFar.dat"] = {
        "category": "beam", "axis_local": None,
        "holes": [{"kind": "round", "pos": [0, 50, 0]}]}
    parts = [_part("beamU", "beamFar.dat", [0, 1, 0]),
             _part("beamD", "beam.dat", [16, 0, 0]),
             _part("pin", "axle.dat", [8, 0.5, 0])]
    anchors = chain.pin_anchors({"beamU"}, {"beamD"}, parts, cat)
    assert len(anchors) == 1
    note = chain._axle_membership(anchors[0], [8, 40, 0], [1, 0, 0],
                                  "D0", "D1", parts, cat)
    assert "D1-owned" in note, note


def test_judge_clamped_reports_spanning_axle():
    parts = [_part("axle", "axle.dat", [8, 0, 0]),
             _part("gearA", "gear.dat", [0, 0, 0]),
             _part("gearB", "gear.dat", [16, 0, 0])]
    anchors = chain.pin_anchors({"gearA"}, {"gearB"}, parts, CATALOG)
    cls, note = chain.judge_clamped(
        anchors[0], [8, 40, 0], [1, 0, 0], {"gearA"}, {"gearB"},
        parts, CATALOG, "D0", "D1")
    assert cls == "shaft"
    assert "spans the boundary" in note, note
