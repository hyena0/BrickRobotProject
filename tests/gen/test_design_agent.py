"""Tests for src.gen.design_agent (synthetic fixtures, fake model)."""

import json

import pytest

from src.gen import design_agent as da


def _module(axis, span, kind="medium"):
    return {
        "joint": {"axis": axis, "point_mm": [0, 0, 20],
                  "range_deg": list(span),
                  "joint_deg_per_motor_deg": 0.1},
        "motor": {"kind": kind},
        "sensor_mount": None,
        "input_mount": {"position_mm": [0, 0, 0],
                        "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        "output_mount": {"position_mm": [0, 0, 40],
                         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        "mount_in": {"frame": [[1, 0, 0, 0], [0, 1, 0, 0],
                               [0, 0, 1, 0]],
                     "holes": [{"pos_ldu": [30, 0, -40], "type": "pin"},
                               {"pos_ldu": [30, 0, -20], "type": "pin"}]},
        "mount_out": {"frame": [[1, 0, 0, 0], [0, 1, 0, 0],
                                [0, 0, 1, 40]],
                      "holes": [{"pos_ldu": [30, 0, 100],
                                 "type": "pin"},
                                {"pos_ldu": [30, 0, 120],
                                 "type": "pin"}]},
        "bom": {"11478.dat": 1},
    }


def _spec():
    return {"joints": ["yaw", "pitch"],
            "link_lengths_mm": [20, 60],
            "ranges_deg": [[-150, 150], [-30, 100]],
            "gripper": False}


def _library():
    return {"M_yaw": _module([0, 0, 1], [-150, 150]),
            "M_pitch": _module([0, 1, 0], [-30, 100], "large")}


def _beams():
    return [{"part_id": "b3.dat", "holes": 3, "span_mm": 16.0},
            {"part_id": "b5.dat", "holes": 5, "span_mm": 32.0}]


def _report(ok=True):
    return {"ok": ok, "score": [3, 8, 1],
            "link_errors_mm": [0.5, 1.0],
            "reach_spec_mm": 80.0, "reach_actual_mm": 81.0,
            "overlap": {"ok": True, "pairs": []},
            "bom_check": {"ok": True, "shortfalls": []}}


def _fake_model(replies):
    calls = {"n": 0}

    def call(history):
        reply = replies[min(calls["n"], len(replies) - 1)]
        calls["n"] += 1
        return reply

    return call, calls


def _design(module_a="M_yaw", beam_a="b3.dat",
            module_b="M_pitch", beam_b="b5.dat"):
    return {"joints": [{"module": module_a, "beam": beam_a, "reason": "r1"},
                       {"module": module_b, "beam": beam_b, "reason": "r2"}]}


def test_extract_json():
    assert da.extract_json('```json\n{"a": 1}\n```') == {"a": 1}
    assert da.extract_json('note {"a": 1} tail') == {"a": 1}
    with pytest.raises(ValueError, match="no JSON"):
        da.extract_json("no braces here")


def test_validate_design():
    spec, library = _spec(), _library()
    assert da.validate_design(_design(), spec, library,
                              {"b3.dat", "b5.dat"}) == []
    bad = _design(module_a="M_nope", beam_b="xx.dat")
    del bad["joints"][0]["reason"]
    errors = da.validate_design(bad, spec, library, {"b3.dat", "b5.dat"})
    assert len(errors) == 3
    short = {"joints": _design()["joints"][:1]}
    assert da.validate_design(short, spec, library,
                              {"b3.dat", "b5.dat"}) != []


def test_loop_succeeds_first_try():
    call, calls = _fake_model([json.dumps(_design())])
    outcome = da.run_design_loop(call, lambda d: _report(True), _spec(),
                                 _library(), _beams(), None)
    assert outcome["rounds"] == 1
    assert outcome["report"]["ok"] is True
    assert calls["n"] == 1


def test_loop_revises_on_shape_error_then_succeeds():
    replies = [json.dumps(_design(module_a="M_nope")),
               json.dumps(_design())]
    seen = {}

    def evaluate(design):
        seen["design"] = design
        return _report(True)

    call, _ = _fake_model(replies)
    outcome = da.run_design_loop(call, evaluate, _spec(), _library(),
                                 _beams(), None)
    assert outcome["rounds"] == 2
    assert "unknown module" in outcome["history"][0]["detail"]
    assert seen["design"]["joints"][0]["module"] == "M_yaw"


def test_loop_revises_on_assembly_failure():
    replies = [json.dumps(_design()), json.dumps(_design())]

    def evaluate(design):
        if not hasattr(evaluate, "once"):
            evaluate.once = True
            raise ValueError("J1: beam b3.dat is not straight")
        return _report(True)

    call, _ = _fake_model(replies)
    outcome = da.run_design_loop(call, evaluate, _spec(), _library(),
                                 _beams(), None)
    assert outcome["rounds"] == 2
    assert "assembly failed" in outcome["history"][0]["detail"]


def test_loop_exhaustion_raises():
    call, calls = _fake_model([json.dumps(_design())])
    with pytest.raises(RuntimeError, match="no feasible design"):
        da.run_design_loop(call, lambda d: _report(False), _spec(),
                           _library(), _beams(), None, max_attempts=2)
    assert calls["n"] == 2


def _beam_catalog():
    def holes(n):
        return [{"kind": "round", "pos": [0, 0, 20 * i]}
                for i in range(n)]

    return {"b3.dat": {"category": "beam", "holes": holes(3)},
            "b5.dat": {"category": "beam", "holes": holes(5)},
            "2780.dat": {"category": "pin", "friction": True,
                         "axis_local": [1, 0, 0]},
            "32062.dat": {"category": "axle", "axle_length": 2,
                          "axis_local": [1, 0, 0]}}


def test_enumerate_ranks_and_deterministic():
    import time

    spec = {"joints": ["yaw", "pitch"],
            "link_lengths_mm": [20, 64],
            "ranges_deg": [[-150, 150], [-30, 100]],
            "gripper": False}
    library = _library()
    catalog = _beam_catalog()
    t0 = time.perf_counter()
    ranked = da.enumerate_designs(spec, library, catalog, None)
    elapsed = time.perf_counter() - t0
    assert len(ranked) >= 1
    assert elapsed < 1.0
    again = da.enumerate_designs(spec, library, catalog, None)
    assert ranked == again
    top = ranked[0]
    assert [j["module"] for j in top["design"]["joints"]] == [
        "M_yaw", "M_pitch"]
    assert top["score"][0] == pytest.approx(0.0)
    scores = [c["score"] for c in ranked]
    assert scores == sorted(scores)


def test_enumerate_stock_eliminates():
    spec = {"joints": ["yaw", "pitch"],
            "link_lengths_mm": [20, 64],
            "ranges_deg": [[-150, 150], [-30, 100]],
            "gripper": False}
    catalog = _beam_catalog()
    inv = {"parts": {"b3.dat": {"total": 10}, "b5.dat": {"total": 10},
                     "11478.dat": {"total": 10},
                     "2780.dat": {"total": 0},
                     "32062.dat": {"total": 10}}}
    with pytest.raises(ValueError, match="no candidate survives stock"):
        da.enumerate_designs(spec, _library(), catalog, inv)


def test_enumerate_errors():
    with pytest.raises(ValueError, match="no compatible module"):
        da.enumerate_designs(dict(_spec(), ranges_deg=[[170, 179],
                                                       [-30, 100]]),
                             _library(), _beam_catalog(), None)
    with pytest.raises(ValueError, match="gripper"):
        da.enumerate_designs(dict(_spec(), gripper=True),
                             _library(), _beam_catalog(), None)
    with pytest.raises(ValueError, match="no beams"):
        da.enumerate_designs(_spec(), _library(), {}, None)
