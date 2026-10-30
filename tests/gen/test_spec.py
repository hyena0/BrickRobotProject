"""Tests for src.gen.spec."""

import copy
import json

from src.gen import spec as spec_mod


def _valid_spec():
    return {"joints": ["yaw", "pitch"],
            "link_lengths_mm": [0, 120],
            "ranges_deg": [[-150, 150], [-30, 100]],
            "gripper": False}


def test_valid_spec_and_reference_file(tmp_path):
    assert spec_mod.validate_spec(_valid_spec()) == []
    ref = spec_mod.load_spec("specs/arm_4dof.json")
    assert spec_mod.validate_spec(ref) == []
    assert len(ref["joints"]) == 4


def test_spec_json_roundtrip(tmp_path):
    path = tmp_path / "s.json"
    path.write_text(json.dumps(_valid_spec()), encoding="utf-8")
    assert spec_mod.validate_spec(spec_mod.load_spec(path)) == []


def test_bad_specs():
    bad = copy.deepcopy(_valid_spec())
    del bad["joints"]
    assert spec_mod.validate_spec(bad) != []

    bad = copy.deepcopy(_valid_spec())
    bad["joints"] = ["yaw", "twist"]
    assert spec_mod.validate_spec(bad) != []

    bad = copy.deepcopy(_valid_spec())
    bad["link_lengths_mm"] = [0]
    assert spec_mod.validate_spec(bad) != []

    bad = copy.deepcopy(_valid_spec())
    bad["ranges_deg"] = [[-150, 150], [100, -30]]
    assert spec_mod.validate_spec(bad) != []

    bad = copy.deepcopy(_valid_spec())
    bad["gripper"] = "no"
    assert spec_mod.validate_spec(bad) != []
