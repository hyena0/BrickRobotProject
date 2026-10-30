"""Tests for src.gen.ldraw (synthetic fixtures only)."""

import math

import pytest

from src.gen import ldraw as ld
from src.parse import parse_text


def _part(pos, rot, part_id="11478.dat", colour="7"):
    return {"uid": "p000001", "part_id": part_id, "colour": colour,
            "position_mm": list(pos), "rotation": [list(r) for r in rot]}


def _rot_z(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


@pytest.mark.parametrize("rot", [
    [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    _rot_z(90.0),
    _rot_z(180.0),
    [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
])
def test_write_parse_roundtrip(rot):
    part = _part([8.0, -16.4, 40.0], rot)
    line, snap_mm = ld.format_type1(part)
    assert snap_mm == pytest.approx(0.0)
    back = parse_text(line + "\n")[0]
    assert back["part_id"] == part["part_id"]
    assert back["colour"] == part["colour"]
    assert back["position_mm"] == pytest.approx(part["position_mm"])
    for got, want in zip(back["rotation"], part["rotation"]):
        assert got == pytest.approx(want)


def test_snap_reports_off_grid_distance():
    part = _part([8.1, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    _, snap_mm = ld.format_type1(part)
    assert snap_mm == pytest.approx(0.1)


def test_write_file_header_and_counts(tmp_path):
    parts = [_part([0.0, 0.0, 0.0],
                   [[1, 0, 0], [0, 1, 0], [0, 0, 1]])]
    out = tmp_path / "robot.ldr"
    report = ld.write_robot_ldraw(parts, out)
    assert report == {"n_parts": 1, "max_snap_mm": 0.0}
    text = out.read_text(encoding="utf-8")
    assert text.splitlines()[0].startswith("0 ")
    assert len(parse_text(text)) == 1
