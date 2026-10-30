"""Tests for src.gen.modules (synthetic fixtures only)."""

import copy
import json

import pytest

from src.gen import modules as mod


def _valid_module():
    return {
        "name": "M_test",
        "frame": "module local, Z up, mm",
        "geometry": "module.ldr",
        "joint": {"axis": [0, 0, 1], "point_mm": [0, 0, 5],
                  "range_deg": [-90, 90],
                  "joint_deg_per_motor_deg": 0.1},
        "motor": {"kind": "medium"},
        "sensor_mount": {"kind": "touch", "position_mm": [10, 0, 5]},
        "input_mount": {"position_mm": [0, 0, 0],
                        "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        "output_mount": {"position_mm": [0, 0, 40],
                         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        "mount_in": {"frame": [[1, 0, 0, 0], [0, 1, 0, 0],
                               [0, 0, 1, 0]],
                     "holes": [{"pos_ldu": [0, 0, 0], "type": "pin"},
                               {"pos_ldu": [0, 20, 0], "type": "pin"}]},
        "mount_out": {"frame": [[1, 0, 0, 0], [0, 1, 0, 0],
                                [0, 0, 1, 40]],
                      "holes": [{"pos_ldu": [0, 0, 0], "type": "pin"},
                                {"pos_ldu": [0, 20, 0], "type": "pin"}]},
        "size_mm": [60, 60, 50],
        "bom": {"11478.dat": 4, "99455.dat": 1},
    }


def _write_module(tmp_path, module):
    (tmp_path / "module.ldr").write_text(
        "1 7 0 0 0 1 0 0 0 1 0 0 0 1 11478.dat\n", encoding="utf-8")
    path = tmp_path / "module.json"
    path.write_text(json.dumps(module), encoding="utf-8")
    return path


def _catalog():
    return {"11478.dat": {
        "category": "beam",
        "holes": [{"kind": "round", "pos": [0, 0, 0]},
                  {"kind": "round", "pos": [0, 0, 20]}]},
        "99455.dat": {"category": "motor", "motor": "medium"}}


def _inventory():
    return {"parts": {"11478.dat": {"total": 10},
                      "99455.dat": {"total": 1}}}


def test_valid_module(tmp_path):
    module = _valid_module()
    path = _write_module(tmp_path, module)
    assert mod.validate_module(mod.load_module(path), tmp_path,
                               catalog=_catalog(),
                               inventory=_inventory()) == []


def test_missing_key_and_geometry(tmp_path):
    module = _valid_module()
    del module["joint"]
    path = tmp_path / "module.json"
    path.write_text(json.dumps(module), encoding="utf-8")
    errors = mod.validate_module(mod.load_module(path), tmp_path)
    assert any("joint" in e for e in errors)
    assert any("geometry" in e for e in errors)


@pytest.mark.parametrize("key, value", [
    ("axis", [0, 0, 2]),
    ("axis", [0, 0, 0]),
    ("range_deg", [90, -90]),
    ("range_deg", [0]),
    ("joint_deg_per_motor_deg", 0),
    ("joint_deg_per_motor_deg", -1),
])
def test_bad_joint_fields(tmp_path, key, value):
    module = _valid_module()
    module["joint"][key] = value
    path = _write_module(tmp_path, module)
    assert mod.validate_module(mod.load_module(path), tmp_path) != []


def test_bad_rotation_not_orthonormal(tmp_path):
    module = _valid_module()
    module["input_mount"]["rotation"] = [[2, 0, 0], [0, 1, 0], [0, 0, 1]]
    path = _write_module(tmp_path, module)
    errors = mod.validate_module(mod.load_module(path), tmp_path)
    assert any("orthonormal" in e for e in errors)


def test_bad_motor_and_sensor(tmp_path):
    module = _valid_module()
    module["motor"] = {"kind": "huge"}
    module["sensor_mount"] = {"kind": "laser", "position_mm": [0, 0, 0]}
    path = _write_module(tmp_path, module)
    errors = mod.validate_module(mod.load_module(path), tmp_path)
    assert any("motor.kind" in e for e in errors)
    assert any("sensor_mount" in e for e in errors)


def test_null_sensor_mount_ok(tmp_path):
    module = _valid_module()
    module["sensor_mount"] = None
    path = _write_module(tmp_path, module)
    assert mod.validate_module(mod.load_module(path), tmp_path,
                               catalog=_catalog()) == []


def test_mount_keys_required(tmp_path):
    module = _valid_module()
    del module["mount_in"]
    del module["mount_out"]
    path = _write_module(tmp_path, module)
    errors = mod.validate_module(mod.load_module(path), tmp_path,
                                 catalog=_catalog())
    assert any("mount_in" in e for e in errors)
    assert any("mount_out" in e for e in errors)


def test_mount_hole_grid_and_count(tmp_path):
    module = _valid_module()
    module["mount_in"]["holes"][0]["pos_ldu"] = [0, 5, 0]
    module["mount_out"]["holes"] = module["mount_out"]["holes"][:1]
    path = _write_module(tmp_path, module)
    errors = mod.validate_module(mod.load_module(path), tmp_path,
                                 catalog=_catalog())
    assert any("grid" in e for e in errors)
    assert any(">= 2 holes" in e for e in errors)


def test_mount_holes_must_hit_subfile_beams(tmp_path):
    module = _valid_module()
    module["mount_in"]["holes"][1]["pos_ldu"] = [0, 40, 0]
    path = _write_module(tmp_path, module)
    errors = mod.validate_module(mod.load_module(path), tmp_path,
                                 catalog=_catalog())
    assert any("matches no subfile beam hole" in e for e in errors)


def test_mount_holes_coaxial_and_frame_mismatch(tmp_path):
    module = _valid_module()
    module["mount_in"]["holes"] = [{"pos_ldu": [0, 0, 0], "type": "pin"},
                                   {"pos_ldu": [20, 0, 0], "type": "pin"}]
    module["mount_out"]["frame"][2][3] = 41.0
    path = _write_module(tmp_path, module)
    errors = mod.validate_module(mod.load_module(path), tmp_path,
                                 catalog=_catalog())
    assert any("coaxial" in e for e in errors)
    assert any("must match the legacy mount" in e for e in errors)


def test_mount_axle_needs_axle_bore(tmp_path):
    module = _valid_module()
    module["mount_in"]["holes"][0]["type"] = "axle"
    path = _write_module(tmp_path, module)
    errors = mod.validate_module(mod.load_module(path), tmp_path,
                                 catalog=_catalog())
    assert any("axle bore" in e for e in errors)


def test_bom_checks(tmp_path):
    module = _valid_module()
    path = _write_module(tmp_path, module)
    assert mod.validate_module(mod.load_module(path), tmp_path,
                               catalog=_catalog()) == []
    assert any("catalog" in e for e in mod.validate_module(
        mod.load_module(path), tmp_path, catalog={}))

    over = copy.deepcopy(module)
    over["bom"] = {"11478.dat": 99}
    over_path = _write_module(tmp_path, over)
    errors = mod.validate_module(mod.load_module(over_path), tmp_path,
                                 catalog=_catalog(),
                                 inventory=_inventory())
    assert any("need 99, own 10" in e for e in errors)
