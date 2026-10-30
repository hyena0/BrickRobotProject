"""Regression tests for the 5D.4 assembly fixes (synthetic only)."""

import json

from src.gen import assemble as asm
from src.gen import ldraw as ldraw_io
from src.gen import mounts as _mounts


def _beam_entry(n, step=20.0):
    return {"holes": [{"pos": [0, 0, i * step], "kind": "round"}
                      for i in range(n)]}


def test_solve_beam_origin_subset_cover():
    entry = _beam_entry(7)
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    pts = [[12.0, 0.0, 0.0], [12.0, 8.0, 0.0],
           [12.0, 32.0, 0.0], [12.0, 40.0, 0.0]]
    solved = _mounts.solve_beam_origin(entry, ident, pts)
    assert solved["origin_ldu"] == [30, 0, 0]
    assert len(solved["kinds"]) == 4


def test_solve_beam_origin_consecutive_still_works():
    entry = _beam_entry(5)
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    pts = [[0.0, 0.0, 0.0], [0.0, 8.0, 0.0], [0.0, 16.0, 0.0]]
    solved = _mounts.solve_beam_origin(entry, ident, pts)
    assert solved["origin_ldu"] == [0, 0, 0]
    assert len(solved["kinds"]) == 3


def test_solve_beam_origin_rejects_non_collinear():
    import pytest

    entry = _beam_entry(7)
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    pts = [[12.0, 0.0, 0.0], [12.0, 8.0, 0.0], [20.0, 32.0, 0.0]]
    with pytest.raises(ValueError, match="collinear"):
        _mounts.solve_beam_origin(entry, ident, pts)


def _pitch_module():
    return {"joint": {"axis": [0, 1, 0], "point_mm": [0, 0, 20]}}


def test_pre3_duplicate_module_refs():
    spec = {"joints": ["pitch", "pitch"], "link_lengths_mm": [20, 60]}
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    lines = []
    for pos in ([0, 0, 0], [0, 0, 60]):
        line, _ = ldraw_io.format_type1(
            {"position_mm": [float(v) for v in pos],
             "rotation": ident, "part_id": "M_dup.ldr", "colour": "16"})
        lines.append(line)
    text = "\n".join(lines) + "\n"
    modules = [_pitch_module(), _pitch_module()]
    joints = [{"axis": [0, 1, 0], "point": [0, 0, 20]},
              {"axis": [0, 1, 0], "point": [0, 0, 80]}]
    got = asm.check_pre3_frames(text, modules, ["M_dup", "M_dup"],
                                spec, joints)
    assert got["ok"] is True
    assert [r["pass"] for r in got["joints"]] == [True, True]


def _conn(uid):
    return {"uid": uid, "connection": "pin_friction"}


def test_audit_element_matings_gap_beam_rule(monkeypatch):
    import src.tools.geometry as geom

    parts = [
        {"uid": "e1", "part_id": "2780.dat"},
        {"uid": "e2", "part_id": "2780.dat"},
        {"uid": "b1", "part_id": "32524.dat", "_segment": 1},
        {"uid": "m1", "part_id": "99455.dat", "_segment": 1},
        {"uid": "m0", "part_id": "32316.dat", "_segment": 0},
    ]
    # e1: in-side pin, everything in one rigid group (old rule fails).
    # e2: out-side pin across the articulation boundary.
    neighbours = {"e1": [_conn("b1"), _conn("m1")],
                  "e2": [_conn("b1"), _conn("m0")]}
    monkeypatch.setattr(geom, "find_connections",
                        lambda uid, ps, cat: list(neighbours[uid]))
    got = asm.audit_element_matings(
        parts, {}, {"e1", "e2"}, {"e1": "b1", "e2": "b1"}, {"b1"})
    assert got["ok"] is True


def test_audit_element_matings_rejects_dangling(monkeypatch):
    import src.tools.geometry as geom

    parts = [
        {"uid": "e3", "part_id": "2780.dat"},
        {"uid": "m1", "part_id": "99455.dat", "_segment": 1},
    ]
    monkeypatch.setattr(geom, "find_connections",
                        lambda uid, ps, cat: [_conn("m1")])
    got = asm.audit_element_matings(
        parts, {}, {"e3"}, {"e3": "b9"}, {"b9"})
    assert got["ok"] is False


def test_write_outputs_keeps_module_refs(tmp_path):
    mod_dir = tmp_path / "modules" / "M_x"
    mod_dir.mkdir(parents=True)
    (mod_dir / "module.json").write_text(
        json.dumps({"geometry": "module.ldr"}), encoding="utf-8")
    (mod_dir / "module.ldr").write_text(
        "1 7 0 0 0 1 0 0 0 1 0 0 0 1 11478.dat\n", encoding="utf-8")
    local = ldraw_io.read_module_geometry(mod_dir, {"geometry": "module.ldr"})
    assert len(local) == 1
    report = {"parts": [{**local[0], "uid": "p000001", "_segment": 0}],
              "_names": ["M_x"], "_gaps": [],
              "expected_kinematics": {}}
    out = tmp_path / "out"
    asm.write_outputs(report, out, tmp_path / "modules", "t")
    text = (out / "robot.ldr").read_text(encoding="utf-8")
    assert "M_x.ldr" in text
