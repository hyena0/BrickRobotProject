"""Tests for the 5D joint-template generator (Stage 5D-B).

Templates must emit module.json files that pass the
src/gen/modules.py validator (mount-interface checks included), with
geometry the real inference tools read back exactly (meshes, rigid
fits, drive nodes, declared ratios). Uses a synthetic uniform-stock
inventory only; never the private config/inventory.local.yaml.
"""

import json

import pytest

from src.gen import ldraw as ldraw_io
from src.gen import templates as tpl
from src.gen.inventory import load_catalog_merged
from src.gen.modules import validate_module
from src.tools.geometry import find_connections
from src.tools.transmission import gear_graph

CAT = load_catalog_merged()

IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _mount(x, y, z):
    return {"position_mm": [x, y, z], "rotation": [row[:] for row in IDENT]}


def _inventory_for(bom, total=999):
    return {"parts": {pid: {"total": total} for pid in bom}}


MOUNT_BEAM = "32316.dat"


def _build(name, inventory=None):
    if name == "pitch":
        return tpl.pitch([0, 1, 0], _mount(0, 0, -4), _mount(0, 0, 60),
                         0.6, "Large", catalog=CAT, inventory=inventory,
                         mount_beam=MOUNT_BEAM)
    if name == "base_yaw":
        return tpl.base_yaw([0, 0, 1], _mount(0, 0, 0), _mount(0, 0, 52),
                            1.0, "Medium", catalog=CAT, inventory=inventory,
                            mount_beam=MOUNT_BEAM)
    return tpl.wrist_roll([0, 0, 1], _mount(0, 0, 0), _mount(0, 0, 52),
                          1.0, "Medium", catalog=CAT, inventory=inventory,
                          mount_beam=MOUNT_BEAM)


@pytest.mark.parametrize("name", ["base_yaw", "pitch", "wrist_roll"])
def test_template_validates(tmp_path, name):
    module, parts = _build(name)
    inv = _inventory_for(module["bom"])
    module, parts = _build(name, inventory=inv)
    out = tpl.write_module_files(tmp_path, name, module, parts,
                                 catalog=CAT, inventory=inv)
    assert (out / "module.ldr").is_file()
    assert validate_module(module, out, CAT, inv) == []
    reread = json.loads((out / "module.json").read_text())
    assert reread["bom"] == module["bom"]
    back = ldraw_io.read_module_geometry(out, module)
    assert len(back) == len(parts)


@pytest.mark.parametrize("name", ["base_yaw", "pitch", "wrist_roll"])
def test_template_physics(name):
    module, _ = _build(name)
    inv = _inventory_for(module["bom"])
    module, parts = _build(name, inventory=inv)
    by_uid = {p["uid"]: p for p in parts}
    motor = next(p for p in parts
                 if (CAT[p["part_id"]].get("motor")))
    conns = find_connections(motor["uid"], parts, CAT)
    assert any(c["connection"] == "motor_shaft" and
               (CAT[by_uid[c["uid"]]["part_id"]].get("teeth"))
               for c in conns), "motor must drive its pinion"
    assert any(c["connection"] == "axle_in_round_hole" for p in parts
               for c in find_connections(p["uid"], parts, CAT)), \
        "shafts must spin free in bearing holes"
    graph = gear_graph(parts, CAT)
    assert graph["intra_meshes"] == []
    assert len(graph["edges"]) == 2
    assert all(e["kind"] == "external" for e in graph["edges"])
    assert graph["motor_drive"][motor["uid"]] is not None
    ratio = module["joint"]["joint_deg_per_motor_deg"]
    teeth = sorted((CAT[p["part_id"]]["teeth"] for p in parts
                    if CAT[p["part_id"]].get("teeth")))
    if name == "pitch":
        assert teeth == [12, 16, 16, 20]
        assert ratio == pytest.approx(0.6)
        compound = [n for n in graph["nodes"] if len(n["gears"]) == 2]
        assert len(compound) == 1
    elif name == "base_yaw":
        assert teeth == [12, 12, 20]
        assert ratio == pytest.approx(1.0)
    else:
        assert teeth == [16, 16, 16]
        assert ratio == pytest.approx(1.0)


def test_pitch_output_bearing_and_drive():
    module, _ = _build("pitch")
    inv = _inventory_for(module["bom"])
    _, parts = _build("pitch", inventory=inv)
    out_gear = next(p for p in parts
                    if p["position_mm"] == [0.0, 36.0, 48.0])
    kinds = {c["connection"] for c in
             find_connections(out_gear["uid"], parts, CAT)}
    assert "axle_in_axle_hole" in kinds  # output gear fixed to its shaft


def test_ratio_must_match_train():
    with pytest.raises(ValueError, match="supports ratio"):
        tpl.pitch([0, 1, 0], _mount(0, 0, -4), _mount(0, 0, 60),
                  0.2, "Large", catalog=CAT)


def test_motor_kind_validated():
    with pytest.raises(ValueError, match="Large or Medium"):
        tpl.pitch([0, 1, 0], _mount(0, 0, -4), _mount(0, 0, 60),
                  0.6, "huge", catalog=CAT)
    module, _ = tpl.pitch([0, 1, 0], _mount(0, 0, -4), _mount(0, 0, 60),
                          0.6, "LARGE", catalog=CAT)
    assert module["motor"]["kind"] == "large"


def test_axis_must_be_cardinal():
    with pytest.raises(ValueError, match="cardinal"):
        tpl.pitch([0, 1, 1], _mount(0, 0, -4), _mount(0, 0, 60),
                  0.6, "Large", catalog=CAT)


def test_mounts_must_be_identity_vertical_grid():
    bad_rot = {"position_mm": [0, 0, -4],
               "rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}
    with pytest.raises(ValueError, match="identity"):
        tpl.pitch([0, 1, 0], bad_rot, _mount(0, 0, 60),
                  0.6, "Large", catalog=CAT)
    with pytest.raises(ValueError, match="height"):
        tpl.pitch([0, 1, 0], _mount(0, 0, -4), _mount(0, 0, 68),
                  0.6, "Large", catalog=CAT)
    with pytest.raises(ValueError, match="LDU grid"):
        tpl.pitch([0, 1, 0], _mount(0, 0, -4.2), _mount(0, 0, 59.8),
                  0.6, "Large", catalog=CAT)


def test_stock_shortfall_fails_loud(tmp_path):
    module, _ = _build("pitch")
    inv = _inventory_for(module["bom"])
    victim = next(iter(module["bom"]))
    inv["parts"][victim]["total"] = 0
    with pytest.raises(ValueError, match="need|no axle|no friction"):
        module2, parts2 = _build("pitch", inventory=inv)
        tpl.write_module_files(tmp_path, "pitch", module2, parts2,
                               catalog=CAT, inventory=inv)


@pytest.mark.parametrize("name", ["base_yaw", "pitch", "wrist_roll"])
def test_template_mount_blocks(name):
    module, _ = _build(name)
    for key, legacy in (("mount_in", "input_mount"),
                        ("mount_out", "output_mount")):
        block = module[key]
        assert len(block["frame"]) == 3
        assert all(len(row) == 4 for row in block["frame"])
        assert [row[3] for row in block["frame"]] == \
            module[legacy]["position_mm"]
        assert len(block["holes"]) == 2
        for hole in block["holes"]:
            assert hole["type"] in ("pin", "axle")
            assert all(v % 10 == 0 for v in hole["pos_ldu"])


def test_mount_beam_rejected_when_wide_or_holeless():
    with pytest.raises(ValueError, match="across the chain"):
        tpl.base_yaw([0, 0, 1], _mount(0, 0, 0), _mount(0, 0, 52),
                     1.0, "Medium", catalog=CAT, mount_beam="14720.dat")
    with pytest.raises(ValueError, match="hole centres"):
        tpl.base_yaw([0, 0, 1], _mount(0, 0, 0), _mount(0, 0, 52),
                     1.0, "Medium", catalog=CAT, mount_beam="32270.dat")


def test_mounts_compatible():
    ok, _ = tpl.mounts_compatible(_mount(0, 0, 60), _mount(0, 0, 0))
    assert ok
    tilted = {"position_mm": [0, 0, 0],
              "rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}
    ok, reason = tpl.mounts_compatible(_mount(0, 0, 60), tilted)
    assert not ok and "mismatch" in reason
