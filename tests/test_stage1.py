"""Stage 1 tests: parser and geometry tools on synthetic fixtures.

Fixtures are own work (tiny LDraw snippets). The real model is never
read here: `runs/` and `assets/private/` stay out of committed tests.
"""

from __future__ import annotations

import os

import pytest

from src.parse.ldr import parse_file, parse_text
from src.tools.geometry import (check_gear_mesh, find_connections, get_parts,
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
    assert [t["uid"] for t in traced] == [axle, beam]
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
