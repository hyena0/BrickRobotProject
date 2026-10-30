"""Stage 5A tests: limits, coupled checks, package validator.

Synthetic fixtures only. The real model (runs/, assets/private/)
and ground truth (eval/ground_truth/, gt/) are never read here.
"""

from __future__ import annotations

import json
import os

import pytest

from src.codegen.generate import build_plan
from src.limits.enforce import (attach_coupled, build_limits, check_pose,
                                is_path_safe)
from src.package.validate import validate_doc


def make_limits():
    return {
        "limits_deg": {"J1": [-60.0, 60.0], "J2": [-45.0, 45.0]},
        "limit_source": {"J1": {"lo": "sensor", "hi": "collision"},
                         "J2": {"lo": "collision", "hi": "collision"}},
        "coupled_limits": [{"joints": ["J1", "J2"], "a": [1.0, 1.0],
                            "b": 60.0, "op": ">="}],
    }


def test_build_limits_takes_tighter_side():
    sens = {"bounds": [
        {"joint": "J1", "status": "ok", "homing_direction": "+",
         "boundary_deg": 70.0}]}
    coll = {"bounds": [
        {"joint": "J1",
         "pos": {"boundary_deg": 50.0},
         "neg": {"boundary_deg": -80.0}}]}
    doc = build_limits(sens, coll, narrow={})
    assert doc["limits_deg"]["J1"] == [-80.0, 50.0]
    assert doc["limit_source"]["J1"] == {"lo": "collision",
                                         "hi": "collision"}
    assert doc["limits_deg"]["J1"][1] == 50.0  # collision wins


def test_build_limits_applies_manual_narrowing():
    from src.limits.enforce import MANUAL_NARROW_DEG
    sens = {"bounds": []}
    coll = {"bounds": [
        {"joint": "J1",
         "pos": {"boundary_deg": 100.0},
         "neg": {"boundary_deg": -100.0}}]}
    doc = build_limits(sens, coll)
    assert doc["limits_deg"]["J1"][0] == MANUAL_NARROW_DEG["J1"]["lo"]
    assert doc["range_narrowing_applied"]["J1"]["lo"] == \
        MANUAL_NARROW_DEG["J1"]["lo"]
    plain = build_limits(sens, coll, narrow={})
    assert plain["limits_deg"]["J1"] == [-100.0, 100.0]
    assert plain["range_narrowing_applied"] == {}


def test_build_limits_sensor_side_with_margin():
    sens = {"bounds": [
        {"joint": "J1", "status": "ok", "homing_direction": "-",
         "boundary_deg": -70.0}]}
    coll = {"bounds": [
        {"joint": "J1",
         "pos": {"boundary_deg": None},
         "neg": {"boundary_deg": None}}]}
    doc = build_limits(sens, coll, narrow={})
    assert doc["limits_deg"]["J1"][0] == pytest.approx(-65.0)
    assert doc["limit_source"]["J1"]["lo"] == "sensor"


def test_build_limits_empty_range_raises():
    sens = {"bounds": [
        {"joint": "J1", "status": "ok", "homing_direction": "+",
         "boundary_deg": -200.0}]}
    coll = {"bounds": [
        {"joint": "J1",
         "pos": {"boundary_deg": 10.0},
         "neg": {"boundary_deg": -10.0}}]}
    with pytest.raises(ValueError):
        build_limits(sens, coll)


def test_check_pose_rejects_out_of_range_and_fold():
    lim = make_limits()
    assert check_pose([0.0, 0.0], ["J1", "J2"], lim) == []
    assert check_pose([61.0, 0.0], ["J1", "J2"], lim) != []
    # q1+q2 = -80 < -60 violates the >= inequality.
    bad = check_pose([-40.0, -40.0], ["J1", "J2"], lim)
    assert any("coupled" in b for b in bad)
    assert check_pose([40.0, 30.0], ["J1", "J2"], lim) == []


def test_is_path_safe_flags_midpoint_fold():
    lim = make_limits()
    ok = is_path_safe([40.0, 30.0], [45.0, 25.0], ["J1", "J2"], lim)
    assert ok["safe"] is True
    ok = is_path_safe([-60.0, -40.0], [60.0, 45.0], ["J1", "J2"], lim)
    assert ok["safe"] is False
    assert ok["violations"]


def test_is_path_safe_step_bound():
    lim = make_limits()
    ok = is_path_safe([0.0, 0.0], [60.0, 0.0], ["J1", "J2"], lim,
                       step_deg=5.0)
    assert ok["safe"] is True
    assert ok["n"] == 12


def test_attach_coupled_explicit_form():
    doc = {"limits_deg": {}, "limit_source": {}}
    out = attach_coupled(doc, {"inequalities": [
        {"joints": ["J2", "J3"], "form": "sum", "op": ">=", "c": -120.0},
        {"joints": ["J2", "J3"], "form": "diff", "op": "<=", "c": 90.0}]})
    assert out["coupled_limits"][0] == {"joints": ["J2", "J3"],
                                        "a": [1.0, 1.0], "b": 120.0,
                                        "op": ">="}
    assert out["coupled_limits"][1]["a"] == [1.0, -1.0]


def test_build_plan_limits_overlay(tmp_path):
    from src.kin.model import load_model  # noqa: F401
    model = {"frame": "base link, Z up, mm",
             "links": [{"id": "D0"}, {"id": "D1"}],
             "joints": [{"id": "J1", "type": "revolute",
                         "parent": "D0", "child": "D1",
                         "axis": [0, 0, 1], "point_mm": [0, 0, 0],
                         "limits_deg": [-180, 180],
                         "zero": "pose in the source file"}],
             "actuators": [{"id": "m1", "brick": "left", "port": "A",
                            "motor": "medium"}],
             "transmission": {"coupling": "q = C m",
                              "C": [[1.0]]},
             "end_effector": {"link": "D1", "point_mm": [0, 0, 0]}}
    ports = {"bricks": {}, "motors": [{"uid": "m1", "brick": "left",
                                       "port": "A"}],
             "sensors": []}
    plan = build_plan(model, ports, [], {},
                      {"limits_deg": {"J1": [-30.0, 40.0]},
                       "limit_source": {"J1": {"lo": "sensor",
                                               "hi": "collision"}},
                       "coupled_limits": []})
    assert plan["joints"][0]["limits_deg"] == [-30.0, 40.0]
    assert plan["joints"][0]["limit_source"] == {"lo": "sensor",
                                                 "hi": "collision"}
    assert plan["joints"][0]["soft_limits_deg"] == [-25.0, 35.0]


def test_plan_move_rejects_coupled_fold(tmp_path):
    import importlib
    import sys
    from src.codegen.generate import generate
    model_p = tmp_path / "model.json"
    model_p.write_text(json.dumps({
        "frame": "base link, Z up, mm",
        "links": [{"id": "D0"}, {"id": "D1"}, {"id": "D2"}],
        "joints": [
            {"id": "J1", "type": "revolute", "parent": "D0",
             "child": "D1", "axis": [0, 0, 1], "point_mm": [0, 0, 0],
             "limits_deg": [-90, 90], "zero": "pose in the source file"},
            {"id": "J2", "type": "revolute", "parent": "D1",
             "child": "D2", "axis": [0, 1, 0], "point_mm": [0, 0, 100],
             "limits_deg": [-90, 90], "zero": "pose in the source file"}],
        "actuators": [
            {"id": "m1", "brick": "left", "port": "A",
             "motor": "medium"},
            {"id": "m2", "brick": "left", "port": "B",
             "motor": "medium"}],
        "transmission": {"coupling": "q = C m",
                         "C": [[1.0, 0.0], [0.0, 1.0]]},
        "end_effector": {"link": "D2", "point_mm": [0, 0, 100]}}),
        encoding="utf-8")
    ports_p = tmp_path / "ports.yaml"
    ports_p.write_text("bricks: {}\nmotors:\n"
                       "  - {uid: m1, brick: left, port: A}\n"
                       "  - {uid: m2, brick: left, port: B}\n"
                       "sensors: []\n", encoding="utf-8")
    lim_p = tmp_path / "limits.json"
    lim_p.write_text(json.dumps({
        "limits_deg": {"J1": [-90, 90], "J2": [-90, 90]},
        "limit_source": {"J1": {"lo": "model", "hi": "model"},
                         "J2": {"lo": "model", "hi": "model"}},
        "coupled_limits": [{"joints": ["J1", "J2"], "a": [1.0, 1.0],
                            "b": -60.0, "op": ">="}]}),
        encoding="utf-8")
    generate(str(model_p), str(ports_p), str(tmp_path / "pkg"),
             None, None, robot="pkg", limits_path=str(lim_p))
    sys.path.insert(0, str(tmp_path))
    try:
        for mod in ("pkg.robot_config", "pkg.motion"):
            sys.modules.pop(mod, None)
        sys.modules.pop("pkg", None)
        motion = importlib.import_module("pkg.motion")
        motion.plan_move([40.0, 30.0])
        with pytest.raises(ValueError):
            motion.plan_move([20.0, 20.0])
        ok, _info = motion.check_path([40.0, 30.0], [45.0, 25.0])
        assert ok is True
        ok, info = motion.check_path([-60.0, 45.0], [60.0, 45.0])
        assert ok is False
        assert info["violations"]
    finally:
        sys.path.remove(str(tmp_path))


def base_doc():
    return {
        "schema_version": "0.1.0", "package_version": 1,
        "robot": "synthetic", "frame": "base link, Z up, mm",
        "joints": [{"id": "J1", "type": "revolute", "parent": "D0",
                    "child": "D1", "axis": [0, 0, 1],
                    "point_mm": [0, 0, 0], "limits_deg": [-60, 60],
                    "limit_source": {"lo": "sensor", "hi": "collision"}}],
        "coupling": {"coupling": "q = C m", "C": [[1.0]],
                     "motor_ids": ["M1"]},
        "coupled_limits": [], "coupled_lookup": {},
        "operating_margin_deg": 5.0,
        "end_effector": {"link": "D1", "point_mm": [0, 0, 0]},
        "safe_poses": [{"id": "safe01", "q_deg": [0.0]}],
        "speed_caps": {"first_run_deg_s": 30.0, "max_deg_s": 100.0,
                       "homing_deg_s": 20.0},
        "link_envelopes": [{"link": "D0", "center_mm": [0.0, 0.0, 0.0],
                            "size_mm": [10.0, 10.0, 10.0],
                            "orientation": [0.0, 0.0, 0.0]}],
    }


def test_validator_accepts_clean_package():
    assert validate_doc(base_doc()) == []


def test_validator_rejects_mesh():
    doc = base_doc()
    doc["link_mesh"] = {"vertices": [0.0]}
    assert validate_doc(doc)


def test_validator_rejects_part_id():
    doc = base_doc()
    doc["safe_poses"] = [{"id": "safe01", "q_deg": [0.0],
                          "motor": "p000147"}]
    assert validate_doc(doc)


def test_validator_rejects_second_box():
    doc = base_doc()
    doc["link_envelopes"].append(
        {"link": "D0", "center_mm": [1.0, 0.0, 0.0],
         "size_mm": [5.0, 5.0, 5.0], "orientation": [0.0, 0.0, 0.0]})
    assert validate_doc(doc)


def test_validator_rejects_oversize(tmp_path):
    doc = base_doc()
    doc["note"] = "x" * (100 * 1024)
    path = os.path.join(str(tmp_path), "pkg.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(doc, fh)
    from src.package.validate import validate_file
    assert validate_file(path)


def test_validator_rejects_private_reference():
    doc = base_doc()
    doc["preview"] = "assets/private/renders/x.png"
    assert validate_doc(doc)


def test_link_rotation_angles_reads_link_frames():
    import numpy as np
    from src.limits.common import link_rotation_angles
    ident = np.eye(4)
    rot = np.eye(4)
    rot[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0]])
    frames = {"D0": ident, "D1": rot}
    phi = link_rotation_angles(frames, ["D0", "D1"])
    assert phi[0] == 0.0
    assert abs(phi[1] - np.pi / 2.0) < 1e-9


def test_aabb_tier_passes_thin_deep_stab():
    import numpy as np
    from src.limits.common import aabb_may_collide
    # Needle (thin in x, long in y) stabbed deep into a plate:
    # min-axis overlap is 0.3 mm while the stab is 8 mm deep.
    needle = np.array([[[-0.15, -10.0, -0.15]] * 8]) \
        .reshape(8, 3)
    needle[:, 1] = [-10.0, -10.0, 10.0, 10.0, -10.0, -10.0, 10.0,
                    10.0]
    needle[:, 0] = [-0.15, 0.15, -0.15, 0.15, -0.15, 0.15, -0.15,
                    0.15]
    needle[:, 2] = [-0.15, -0.15, -0.15, -0.15, 0.15, 0.15, 0.15,
                    0.15]
    plate = np.array([[x, y, z] for x in (-5.0, 5.0)
                      for y in (-1.0, 1.0) for z in (-5.0, 5.0)])
    assert aabb_may_collide(needle, plate) is True
    far = plate + np.array([100.0, 0.0, 0.0])
    assert aabb_may_collide(needle, far) is False


def test_wrist_poses_hashable():
    from src.limits.coupled import WRIST_POSES
    for w in WRIST_POSES:
        hash((w, (0.0, 0.0), (1.0, 1.0)))


def test_tighten_skips_timeout_only():
    from src.limits.coupled import tighten
    ineq = [{"joints": ["J2", "J3"], "form": "sum", "op": ">=",
             "c": -120.0}]
    bad = [{"q_deg": [0.0, 0.0],
            "hits": [{"a": "TIMEOUT", "b": "TIMEOUT"}]}]
    assert tighten(ineq, ["J2", "J3"], bad) == 0
    assert ineq[0]["c"] == -120.0


def test_tighten_pushes_past_real_hit():
    from src.limits.coupled import tighten
    ineq = [{"joints": ["J2", "J3"], "form": "sum", "op": ">=",
             "c": -120.0}]
    bad = [{"q_deg": [10.0, -100.0],
            "hits": [{"a": "p1", "b": "p2", "depth_mm": 3.0}]}]
    assert tighten(ineq, ["J2", "J3"], bad) == 1
    assert ineq[0]["c"] == pytest.approx(-87.5)


def test_check_inequalities():
    from src.limits.coupled import check_inequalities
    ineq = [{"joints": ["J2", "J3"], "form": "sum", "op": ">=",
             "c": -120.0}]
    assert check_inequalities([0.0, 0.0], ["J2", "J3"], ineq) is True
    assert check_inequalities([0.0, -130.0], ["J2", "J3"],
                              ineq) is False


def test_fit_excludes_every_colliding_point():
    from src.limits.coupled import fit_inequalities, prune_forms
    coll = {(10.0, 0.0), (0.0, 10.0), (-5.0, -5.0)}
    forms = fit_inequalities(coll)
    for f in forms:
        for (a, b) in coll:
            v = a + b if f["form"] == "sum" else a - b
            if f["op"] == ">=":
                assert v < f["c"]
            else:
                assert v > f["c"]
    kept = prune_forms(forms, "J2", "J3", coll, {(100.0, 100.0)})
    assert kept, "a cover must exist when free space is available"


def test_attribute_spans_maps_links_to_joint_pairs():
    from src.limits.coupled import attribute_spans
    ctx = {"link_ids": ["D0", "D1", "D2", "D3"],
           "ordered": [{"id": "J1"}, {"id": "J2"}, {"id": "J3"}]}
    bad = [{"q_deg": [0.0, 0.0, 0.0],
            "hits": [{"a": "u1", "b": "u2", "link_a": "D1",
                      "link_b": "D3"}]}]
    assert ("J2", "J3") in attribute_spans(ctx, bad)


def test_check_lookup_rejects_cell_square():
    from src.limits.enforce import check_lookup
    tables = {"J2_J3": {"cells": [[50.0, -60.0]], "radius_deg": 2.5},
              "_residual_lookup": [[0.0] * 6]}
    names = ["J1", "J2", "J3", "J4", "J5", "J6"]
    assert check_lookup([0.0] * 6, names, {}) is True
    assert check_lookup([0.0, 50.0, -60.0, 0.0, 0.0, 0.0],
                        names, tables) is False
    assert check_lookup([0.0, 52.4, -61.0, 0.0, 0.0, 0.0],
                        names, tables) is False
    assert check_lookup([0.0, 60.0, -60.0, 0.0, 0.0, 0.0],
                        names, tables) is True
    assert check_lookup([0.0, 50.0, -40.0, 0.0, 0.0, 0.0],
                        names, tables) is True


def test_check_pose_lookup_switched_off():
    # Conservative finish: check_pose ignores lookup tables while
    # USE_COUPLED_LOOKUP is off (check_lookup itself stays tested).
    from src.limits.enforce import check_pose
    doc = {"limits_deg": {"J2": [-180.0, 180.0],
                          "J3": [-180.0, 180.0]},
           "coupled_limits": [],
           "coupled_lookup": {"J2_J3": {"cells": [[50.0, -60.0]],
                                       "radius_deg": 2.5}}}
    assert check_pose([50.0, -60.0], ["J2", "J3"], doc) == []
    assert check_pose([0.0, 0.0], ["J2", "J3"], doc) == []


def test_record_table_stores_cells():
    from src.limits.coupled import record_table
    tables: dict = {}
    record_table(tables, {"joints": ["J2", "J3"], "mode": "lookup",
                          "cells": [[50.0, -60.0]],
                          "n_free_cells": 73})
    assert tables["J2_J3"]["cells"] == [[50.0, -60.0]]
    assert tables["J2_J3"]["radius_deg"] == 2.5
    record_table(tables, {"joints": ["J4", "J5"], "mode": "linear"})
    assert "J4_J5" not in tables


def test_grow_lookup_cells_tiles_observed_folds():
    from src.limits.coupled import grow_lookup_cells
    ctx = {"link_ids": ["D0", "D1", "D2", "D3"]}
    names = ["J1", "J2", "J3"]
    tables = {"J2_J3": {"cells": [], "radius_deg": 2.5}}
    bad = [{"q_deg": [0.0, 50.0, -60.0],
            "hits": [{"a": "u1", "b": "u2", "link_a": "D1",
                      "link_b": "D3"}]}]
    grown, handled = grow_lookup_cells(ctx, tables, names, bad)
    assert grown == 1 and handled == {0}
    assert tables["J2_J3"]["cells"] == [[50.0, -60.0]]
    # Same fold again: already covered, no duplicate cell.
    grown, handled = grow_lookup_cells(ctx, tables, names, bad)
    assert grown == 0 and handled == {0}
    assert len(tables["J2_J3"]["cells"]) == 1
    # Single-joint span: no adjacent pair, left for tightening.
    bad2 = [{"q_deg": [10.0, 0.0, 0.0],
             "hits": [{"a": "u1", "b": "u2", "link_a": "D0",
                       "link_b": "D1"}]}]
    grown, handled = grow_lookup_cells(ctx, tables, names, bad2)
    assert grown == 0 and handled == set()
    # Linear-fitted pair without a table: table created on demand.
    tables2: dict = {}
    bad3 = [{"q_deg": [0.0, 30.0, 40.0],
             "hits": [{"a": "u1", "b": "u2", "link_a": "D1",
                       "link_b": "D3"}]}]
    grown, handled = grow_lookup_cells(ctx, tables2, names, bad3)
    assert grown == 1 and handled == {0}
    assert tables2["J2_J3"]["cells"] == [[30.0, 40.0]]


def test_record_table_merges_grown_cells():
    from src.limits.coupled import record_table
    tables = {"J2_J3": {"cells": [[1.0, 2.0]], "radius_deg": 2.5,
                       "n_free_cells": 0}}
    record_table(tables, {"joints": ["J2", "J3"], "mode": "lookup",
                          "cells": [[1.0, 2.0], [3.0, 4.0]],
                          "n_free_cells": 73})
    assert tables["J2_J3"]["cells"] == [[1.0, 2.0], [3.0, 4.0]]
    record_table(tables, {"joints": ["J4", "J5"], "mode": "linear"})
    assert "J4_J5" not in tables


def test_select_conservative_covers_j2j3_cells():
    # The 10 deterministic J2-J3 march cells (identical across
    # repeated oracle runs) must be excludable by at most two
    # linear bounds that keep the home pose.
    from src.limits.coupled import check_inequalities, select_conservative
    cells = {(-116.25, -48.15), (-113.53, -47.02),
             (-101.27, -41.95), (-58.11, -58.11),
             (-56.78, -56.78), (-55.45, -55.45),
             (-50.7, 21.0), (-30.62, -73.91),
             (-30.23, -72.98), (-0.0, -37.19)}
    free = {(0.0, 0.0), (10.0, 10.0), (-50.0, 0.0)}
    from src.limits.coupled import fit_inequalities
    forms = fit_inequalities(cells, free)
    sel, covers, loss = select_conservative(forms, "J2", "J3",
                                            cells, free)
    assert covers is True
    assert 1 <= len(sel) <= 2
    assert loss < 1.0
    out = [{"joints": ["J2", "J3"], "form": f["form"], "op": f["op"],
            "c": f["c"]} for f in sel]
    assert check_inequalities([0.0, 0.0, 0.0],
                              ["J1", "J2", "J3"], out) is True
    for qa, qb in cells:
        assert check_inequalities([0.0, qa, qb],
                                  ["J1", "J2", "J3"], out) is False


def test_plan_move_lookup_switched_off(tmp_path):
    # Conservative finish: generated packages carry an empty lookup
    # table, so a pose on a known fold cell is NOT rejected by the
    # lookup path (only ranges + inequalities gate).
    import importlib
    import sys
    from src.codegen.generate import generate
    model_p = tmp_path / "model.json"
    model_p.write_text(json.dumps({
        "frame": "base link, Z up, mm",
        "links": [{"id": "D0"}, {"id": "D1"}, {"id": "D2"}],
        "joints": [
            {"id": "J1", "type": "revolute", "parent": "D0",
             "child": "D1", "axis": [0, 0, 1], "point_mm": [0, 0, 0],
             "limits_deg": [-90, 90], "zero": "pose in the source file"},
            {"id": "J2", "type": "revolute", "parent": "D1",
             "child": "D2", "axis": [0, 1, 0], "point_mm": [0, 0, 100],
             "limits_deg": [-90, 90], "zero": "pose in the source file"}],
        "actuators": [
            {"id": "m1", "brick": "left", "port": "A",
             "motor": "medium"},
            {"id": "m2", "brick": "left", "port": "B",
             "motor": "medium"}],
        "transmission": {"coupling": "q = C m",
                         "C": [[1.0, 0.0], [0.0, 1.0]]},
        "end_effector": {"link": "D2", "point_mm": [0, 0, 100]}}),
        encoding="utf-8")
    ports_p = tmp_path / "ports.yaml"
    ports_p.write_text("bricks: {}\nmotors:\n"
                       "  - {uid: m1, brick: left, port: A}\n"
                       "  - {uid: m2, brick: left, port: B}\n"
                       "sensors: []\n", encoding="utf-8")
    lim_p = tmp_path / "limits.json"
    lim_p.write_text(json.dumps({
        "limits_deg": {"J1": [-90, 90], "J2": [-90, 90]},
        "limit_source": {"J1": {"lo": "model", "hi": "model"},
                         "J2": {"lo": "model", "hi": "model"}},
        "coupled_limits": [],
        "coupled_lookup": {"J1_J2": {"cells": [[40.0, 30.0]],
                                     "radius_deg": 2.5}}}),
        encoding="utf-8")
    generate(str(model_p), str(ports_p), str(tmp_path / "pkg2"),
             None, None, robot="pkg2", limits_path=str(lim_p))
    sys.path.insert(0, str(tmp_path))
    try:
        for mod in ("pkg2.robot_config", "pkg2.motion"):
            sys.modules.pop(mod, None)
        sys.modules.pop("pkg2", None)
        pkg2_config = importlib.import_module("pkg2.robot_config")
        assert pkg2_config.COUPLED_LOOKUP == {}
        motion = importlib.import_module("pkg2.motion")
        motion.plan_move([0.0, 0.0])
        motion.plan_move([40.0, 30.0])
        ok, _info = motion.check_path([0.0, 0.0], [40.0, 30.0])
        assert ok is True
    finally:
        sys.path.remove(str(tmp_path))


def test_sweep_side_bisect_negative_direction_regression():
    # Stubbed-oracle regression test: first collision at -78 deg must
    # bisect to ~= -78 (boundary -73), not to the coarse grid point
    # -75. A mirrored lo/hi reports the coarse point instead.
    from types import SimpleNamespace
    from src.limits.collision_bounds import sweep_side

    class Oracle:
        def __init__(self, thresh, neg):
            self.thresh = thresh
            self.neg = neg

        def map(self, qs, **kw):
            out = []
            for q in qs:
                a = q[2]
                hit = (a <= self.thresh) if self.neg else (a >= self.thresh)
                out.append([{"pair": ("x", "y"), "depth_mm": 3.0}]
                           if hit else [])
            return out

    mesh = SimpleNamespace(ctx={"ordered": [{"id": "J%d" % i}
                                            for i in range(1, 7)]})
    neg = sweep_side(Oracle(-78.0, True), mesh, "J3", set(), set(),
                     direction=-1.0)
    assert neg["status"] == "ok"
    assert neg["collision_deg"] == pytest.approx(-78.0, abs=1.0)
    assert neg["boundary_deg"] == pytest.approx(
        neg["collision_deg"] + 5.0, abs=1e-9)
    pos = sweep_side(Oracle(62.0, False), mesh, "J3", set(), set(),
                     direction=+1.0)
    assert pos["status"] == "ok"
    assert pos["collision_deg"] == pytest.approx(62.0, abs=1.0)
    assert pos["boundary_deg"] == pytest.approx(
        pos["collision_deg"] - 5.0, abs=1e-9)
