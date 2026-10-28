"""Stage 1 tests: parser and geometry tools on synthetic fixtures.

Fixtures are own work (tiny LDraw snippets). The real model is never
read here: `runs/` and `assets/private/` stay out of committed tests.
"""

from __future__ import annotations

import os

import pytest

from src.parse.ldr import parse_file, parse_text
from src.tools.geometry import (axis_world, check_gear_mesh,
                                find_connections, get_parts, hole_world,
                                rigid_groups, trace_axle)

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
    "beamhole.dat": {"category": "beam", "axis_local": [0, 1, 0],
                     "holes": [{"kind": "round"}]},
    "beamaxle.dat": {"category": "beam", "axis_local": [0, 1, 0],
                     "holes": [{"kind": "axle"}]},
    "turnbase.dat": {"category": "turntable", "axis_local": [0, 1, 0]},
    "turntop.dat": {"category": "turntable", "axis_local": [0, 1, 0]},
    "pin.dat": {"category": "pin", "axis_local": [0, 0, 1]},
}


def test_parser_reads_all_fixture_parts():
    parts = parse_file(os.path.join(FIX, "gears.ldr"))
    assert len(parts) == 2
    assert parts[0]["part_id"] == "gear8t.dat"
    # 40 LDU * 0.4 = 16 mm apart on X.
    assert parts[1]["position_mm"][0] == pytest.approx(16.0)
    # Z-up conversion: LDraw -Y becomes +Z.
    assert parts[1]["position_mm"][2] == pytest.approx(0.0)


def test_rotated_part_world_axis_uses_conjugation():
    # Axle along LDraw +X, part rotated 90 deg about LDraw +Z
    # (X -> Y). True world axis: C @ Ry90 @ ex = -Z.
    parts = parse_text(
        "1 0 0 0 0 0 -1 0 1 0 0 0 0 1 rotaxle.dat\n")
    cat = {"rotaxle.dat": {"category": "axle",
                           "axis_local": [1, 0, 0]}}
    axis = axis_world(parts[0], cat)
    assert abs(axis[2]) == pytest.approx(1.0)
    assert axis[2] < 0
    assert abs(axis[0]) == pytest.approx(0.0)
    assert abs(axis[1]) == pytest.approx(0.0)


def test_hole_world_matches_part_frame():
    # Hole 10 LDU along local +Z of an identity-placed beam lands
    # 4 mm along world +Y (point map: (x, y, z) -> (x, z, -y)).
    parts = parse_text("1 0 0 0 0 1 0 0 0 1 0 0 0 1 holebeam.dat\n")
    cat = {"holebeam.dat": {"category": "beam",
                            "axis_local": [0, 0, 1],
                            "holes": [{"pos": [0, 0, 10],
                                       "kind": "round"}]}}
    hole = hole_world(parts[0], cat["holebeam.dat"]["holes"][0])
    assert list(hole) == pytest.approx([0.0, 4.0, 0.0])


def test_parser_flattens_submodels_without_names():
    text = ("0 FILE main\n"
            "0 STEP\n"
            "1 0 0 0 0 1 0 0 0 1 0 -0 1 0 sub.ldr\n"
            "0 NOFILE\n"
            "0 FILE sub.ldr\n"
            "1 0 10 0 0 1 0 0 0 1 0 -0 1 0 gear8t.dat\n"
            "0 NOFILE\n")
    parts = parse_text(text)
    assert len(parts) == 1
    assert parts[0]["part_id"] == "gear8t.dat"
    assert parts[0]["position_mm"][0] == pytest.approx(4.0)
    assert not any("sub" in p["uid"] for p in parts)


def test_gears_mesh_at_expected_distance():
    parts = parse_file(os.path.join(FIX, "gears.ldr"))
    a, b = parts[0]["uid"], parts[1]["uid"]
    res = check_gear_mesh(a, b, parts, CATALOG)
    assert res["axis_relation"] == "parallel"
    assert res["expected_ldu"] == pytest.approx(40.0)
    assert res["meshes"] is True
    assert res["kind"] == "external"


def test_idler_train_meshes_pairwise_only():
    parts = parse_file(os.path.join(FIX, "train.ldr"))
    uids = [p["uid"] for p in parts]
    assert check_gear_mesh(uids[0], uids[1], parts, CATALOG)["meshes"] is True
    assert check_gear_mesh(uids[1], uids[2], parts, CATALOG)["meshes"] is True
    res = check_gear_mesh(uids[0], uids[2], parts, CATALOG)
    assert res["meshes"] is False


def test_axle_in_round_hole_is_free():
    parts = parse_file(os.path.join(FIX, "axle_hole.ldr"))
    axle, beam = parts[0]["uid"], parts[1]["uid"]
    conns = find_connections(axle, parts, CATALOG)
    assert conns[0]["uid"] == beam
    assert conns[0]["connection"] == "axle_in_round_hole"
    traced = trace_axle(axle, parts, CATALOG)
    # Axle local +Y maps to world -Z, so the beam at +Z sorts first.
    assert [t["uid"] for t in traced] == [beam, axle]
    groups = rigid_groups(parts, CATALOG)
    assert len(groups) == 2  # free rotation never joins groups


def test_axle_in_axle_hole_is_rigid():
    parts = parse_text(
        "1 0 0 0 0 1 0 0 0 1 0 0 0 1 axle4.dat\n"
        "1 0 0 -20 0 1 0 0 0 1 0 0 0 1 beamaxle.dat\n")
    axle, beam = parts[0]["uid"], parts[1]["uid"]
    conns = find_connections(axle, parts, CATALOG)
    assert conns[0]["connection"] == "axle_in_axle_hole"
    groups = rigid_groups(parts, CATALOG)
    assert len(groups) == 1
    assert groups[0]["part_count"] == 2


def test_turntable_reports_free_rotation():
    parts = parse_file(os.path.join(FIX, "turntable.ldr"))
    base, top = parts[0]["uid"], parts[1]["uid"]
    conns = find_connections(base, parts, CATALOG)
    assert conns[0]["uid"] == top
    assert conns[0]["connection"] == "axle_in_round_hole"


def _line_parts():
    # Friction pin along local +X at origin; beam holes pierced.
    # World: +X stays +X under the Z-up conversion.
    text = ("1 0 0 0 0 1 0 0 0 1 0 0 0 1 pin3.dat\n"
            "1 0 -25 0 0 1 0 0 0 1 0 0 0 1 beamA.dat\n"
            "1 0 25 0 0 1 0 0 0 1 0 0 0 1 beamB.dat\n")
    parts = parse_text(text)
    catalog = {
        "pin3.dat": {"category": "pin", "axis_local": [1, 0, 0],
                     "friction": True, "pin_length_studs": 3},
        "beama.dat": {"category": "beam", "axis_local": [0, 0, 1],
                      "holes": [{"pos": [0, 0, 0], "kind": "round"}]},
        "beamb.dat": {"category": "beam", "axis_local": [0, 0, 1],
                      "holes": [{"pos": [0, 0, 0], "kind": "round"}]},
    }
    return parts, catalog


def test_friction_pin_joins_beams_rigidly():
    parts, catalog = _line_parts()
    pin = parts[0]["uid"]
    conns = find_connections(pin, parts, catalog)
    assert {c["connection"] for c in conns} == {"pin_friction"}
    groups = rigid_groups(parts, catalog)
    assert len(groups) == 1
    assert groups[0]["part_count"] == 3


def test_frictionless_pin_stays_free():
    parts, catalog = _line_parts()
    catalog["pin3.dat"] = {**catalog["pin3.dat"], "friction": False}
    conns = find_connections(parts[0]["uid"], parts, catalog)
    assert {c["connection"] for c in conns} == {"pin_frictionless"}
    assert len(rigid_groups(parts, catalog)) == 3


def test_position_aware_hole_classification():
    # Axle along Y; the round-hole beam is pierced through its round
    # bore (free), the axle-hole beam through its axle bore (rigid).
    text = ("1 0 0 0 0 1 0 0 0 1 0 0 0 1 axleY.dat\n"
            "1 0 0 -20 0 1 0 0 0 1 0 0 0 1 beamR.dat\n"
            "1 0 0 20 0 1 0 0 0 1 0 0 0 1 beamX.dat\n")
    parts = parse_text(text)
    catalog = {
        "axley.dat": {"category": "axle", "axis_local": [0, 1, 0]},
        "beamr.dat": {"category": "beam", "axis_local": [0, 0, 1],
                      "holes": [{"pos": [0, 0, 0], "kind": "round"}]},
        "beamx.dat": {"category": "beam", "axis_local": [0, 0, 1],
                      "holes": [{"pos": [0, 0, 0], "kind": "axle"}]},
    }
    by_id = {c["uid"]: c["connection"]
             for c in find_connections(parts[0]["uid"], parts, catalog)}
    assert by_id[parts[1]["uid"]] == "axle_in_round_hole"
    assert by_id[parts[2]["uid"]] == "axle_in_axle_hole"


def test_motor_shaft_is_transmission_not_rigid():
    text = ("1 0 0 0 0 1 0 0 0 1 0 0 0 1 motorM.dat\n"
            "1 0 0 0 60 1 0 0 0 1 0 0 0 1 gearG.dat\n")
    parts = parse_text(text)
    catalog = {
        "motorm.dat": {"category": "motor", "axis_local": [0, 0, 1]},
        "gearg.dat": {"category": "gear", "axis_local": [0, 0, 1]},
    }
    conns = find_connections(parts[0]["uid"], parts, catalog)
    assert conns[0]["connection"] == "motor_shaft"
    assert len(rigid_groups(parts, catalog)) == 2


def test_get_parts_filters_category_and_radius():
    parts = parse_file(os.path.join(FIX, "train.ldr"))
    gears = get_parts(parts, CATALOG, category="gear")
    assert len(gears) == 3
    near = get_parts(parts, CATALOG, near=[0, 0, 0], radius_mm=1.0)
    assert len(near) == 1


def _write(tmp_path, name: str, content: str) -> str:
    path = tmp_path / name
    path.write_text(content, encoding="utf-8")
    return str(path)


def test_wiring_check_passes_on_consistent_files(tmp_path):
    from src.tools.wiring import check

    parts = _write(tmp_path, "parts.json",
                   '[{"uid": "p000001", "part_id": "m1.dat"}]')
    catalog = _write(tmp_path, "catalog.yaml",
                     '"m1.dat":\n  category: motor\n')
    ports = _write(tmp_path, "ports.yaml",
                   "motors:\n  - {uid: p000001, brick: left, port: A}\n")
    assert check(parts, catalog, ports) == []


def test_wiring_check_rejects_semantic_keys(tmp_path):
    from src.tools.wiring import check

    parts = _write(tmp_path, "parts.json",
                   '[{"uid": "p000001", "part_id": "m1.dat"}]')
    catalog = _write(tmp_path, "catalog.yaml",
                     '"m1.dat":\n  category: motor\n')
    ports = _write(tmp_path, "ports.yaml",
                   "motors:\n  - {uid: p000001, brick: left, port: A, "
                   "joint: J1}\n")
    errors = check(parts, catalog, ports)
    assert any("joint" in e for e in errors)


def test_parser_asymmetric_rotation_and_submodel_compose():
    # Rz(90) @ Rx(90) is asymmetric: a transposed read gives a
    # different matrix, so this fixture traps row/column swaps.
    # Submodel child (Rx90 at (10,0,0) LDU) inside a holder
    # (Rz90 at (0,0,40) LDU) checks compose order and that the
    # -Y-up to Z-up conversion applies exactly once.
    import numpy as np

    parts = parse_file(os.path.join(FIX, "rotated.ldr"))
    by_part = {p["part_id"]: p for p in parts}
    # Converted: pos (x, z, -y) * 0.4; R rows remap(L r0),
    # remap(L r2), -remap(L r1) with remap(r) = (r0, r2, -r1).
    assert by_part["gear16t.dat"]["position_mm"] == [4.0, 12.0, -8.0]
    assert by_part["gear8t.dat"]["position_mm"] == [0.0, 16.0, -4.0]
    want = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]
    for pid in ("gear16t.dat", "gear8t.dat"):
        got = by_part[pid]["rotation"]
        assert np.allclose(np.asarray(got), np.asarray(want)), pid
        assert not np.allclose(np.asarray(got),
                               np.asarray(got).T), pid
