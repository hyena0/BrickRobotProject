"""Stage 2 tests: schema, heuristic baseline, verifiers, overlay.

Synthetic only. The real model (`runs/`, `assets/private/`) and ground
truth (`eval/ground_truth/`) are never read here.
"""

from __future__ import annotations

import json

from src.agent import heuristic
from src.agent.schema import to_public_projection, validate_links_joints
from src.agent.sensors import associate
from src.verify import joints as joints_verify
from src.verify import segmentation as seg_verify
from src.viz import overlay


def _parts():
    return [
        {"uid": "p000001", "part_id": "beamaxle.dat",
         "position_mm": [0.0, 0.0, 0.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        {"uid": "p000002", "part_id": "beamaxle.dat",
         "position_mm": [0.0, 0.0, 8.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        {"uid": "p000003", "part_id": "turntop.dat",
         "position_mm": [0.0, 0.0, 40.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
    ]


CATALOG = {
    "beamaxle.dat": {"category": "beam", "axis_local": [0, 0, 1],
                     "holes": [{"kind": "axle"}]},
    "turntop.dat": {"category": "turntable", "axis_local": [0, 0, 1]},
}


def test_schema_accepts_minimal_joint_doc():
    doc = {"links": [{"id": "L0"}, {"id": "L1"}],
           "joints": [{"id": "J1", "type": "revolute",
                       "parent": "L0", "child": "L1",
                       "axis": [0, 0, 1], "point_mm": [0, 0, 1],
                       "confidence": 0.9, "reason": "test"}],
           "base": {"link": "L0"},
           "end_effector": {"link": "L1"}}
    assert validate_links_joints(doc) == []
    public = to_public_projection(doc)
    assert public["joints"][0]["zero"] == "pose in the source file"
    assert "members" not in str(public)


def test_schema_rejects_unknown_link_and_bad_axis():
    doc = {"links": [{"id": "L0"}],
           "joints": [{"id": "J1", "type": "revolute",
                       "parent": "L0", "child": "LX",
                       "axis": [0, 0], "point_mm": [0, 0, 0],
                       "confidence": 2.0, "reason": "bad"}],
           "base": {"link": "L0"}, "end_effector": {"link": "LX"}}
    errors = validate_links_joints(doc)
    assert any("child" in e for e in errors)
    assert any("axis" in e for e in errors)
    assert any("confidence" in e for e in errors)


def test_heuristic_reports_interfaces_without_claiming_joints():
    doc = heuristic.infer(_parts(), CATALOG)
    assert validate_links_joints(doc) == []
    assert doc["method"] == "heuristic-baseline"
    assert doc["base"]["link"]
    assert doc["end_effector"]["link"]


def test_segmentation_agreement_scores_perfect_mapping():
    pred = {"L0": ["p1", "p2"], "L1": ["p3"]}
    gt = {"p1": "A", "p2": "A", "p3": "B"}
    result = seg_verify.agreement(pred, gt)
    assert result["accuracy"] == 1.0


def test_joint_metrics_detect_parallel_offset():
    ang = joints_verify.axis_angle_deg([0, 0, 1], [0, 0, -1])
    assert ang == 0.0
    dist = joints_verify.axis_line_distance([0, 0, 1], [5, 0, 0],
                                            [0, 0, 1], [0, 0, 0])
    assert dist == 5.0


def test_sensor_association_mounts_and_defers_direction():
    parts = [
        {"uid": "s1", "position_mm": [0.0, 0.0, 0.0]},
        {"uid": "m1", "position_mm": [0.0, 0.0, 30.0]},
    ]
    links = [{"id": "L0", "members": ["s1"]},
             {"id": "L1", "members": ["m1"]}]
    joints = [{"id": "J1", "parent": "L0", "child": "L1",
               "axis": [0, 0, 1], "point_mm": [0, 0, 15.0]}]
    result = associate([{"uid": "s1", "brick": "left", "port": 1}],
                       parts, links, joints)
    assert result[0]["mount_link"] == "L0"
    assert result[0]["joint"] == "J1"
    assert result[0]["homing_direction"] == "unknown"


class _FakeResponses:
    def __init__(self, script):
        self.script = list(script)
        self.calls = []

    def create(self, **kwargs):
        self.calls.append(kwargs)
        return self.script.pop(0)


class _FakeClient:
    def __init__(self, script):
        self.responses = _FakeResponses(script)


def _msg(text):
    return {"type": "message",
            "content": [{"type": "output_text", "text": text}]}


def _call(name, args):
    return {"type": "function_call", "call_id": "c1", "name": name,
            "arguments": json.dumps(args)}


class _Resp:
    def __init__(self, output):
        self.output = output
        self.usage = None


def test_run_loop_executes_tools_and_validates(tmp_path, monkeypatch):
    from src.agent import run as runmod

    monkeypatch.chdir(tmp_path)
    parts = [
        {"uid": "p1", "part_id": "a.dat",
         "position_mm": [0.0, 0.0, 0.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        {"uid": "p2", "part_id": "b.dat",
         "position_mm": [0.0, 0.0, 8.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
    ]
    catalog = {
        "a.dat": {"category": "axle", "axis_local": [0, 1, 0]},
        "b.dat": {"category": "beam", "axis_local": [0, 1, 0],
                  "holes": [{"kind": "axle"}]},
    }
    final = {"links": [{"id": "L0"}, {"id": "L1"}],
             "joints": [{"id": "J1", "type": "revolute",
                         "parent": "L0", "child": "L1",
                         "axis": [0, 0, 1], "point_mm": [0, 0, 4],
                         "confidence": 0.8, "reason": "fake"}],
             "base": {"link": "L0"}, "end_effector": {"link": "L1"}}
    client = _FakeClient([
        _Resp([_call("rigid_groups", {})]),
        _Resp([_msg("here: " + json.dumps(final))]),
    ])
    doc = runmod.run_loop(client, parts, catalog, "t-loop", max_calls=5)
    assert doc["joints"][0]["id"] == "J1"
    assert len(client.responses.calls) == 2
    assert (tmp_path / "runs" / "usage.jsonl").exists()
    assert (tmp_path / "runs" / "t-loop" / "trace.jsonl").exists()


def test_run_loop_repairs_invalid_json(tmp_path, monkeypatch):
    from src.agent import run as runmod

    monkeypatch.chdir(tmp_path)
    parts = [{"uid": "p1", "part_id": "a.dat",
              "position_mm": [0.0, 0.0, 0.0],
              "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}]
    catalog = {"a.dat": {"category": "beam"}}
    good = {"links": [{"id": "L0"}], "joints": [],
            "base": {"link": "L0"}, "end_effector": {"link": "L0"}}
    client = _FakeClient([
        _Resp([_msg('{"links": []}')]),
        _Resp([_msg(json.dumps(good))]),
    ])
    doc = runmod.run_loop(client, parts, catalog, "t-repair", max_calls=5)
    assert doc["links"] == [{"id": "L0"}]
    assert len(client.responses.calls) == 2


def test_overlay_groups_parts_and_arrows(tmp_path):
    pred_path = tmp_path / "pred.json"
    pred_path.write_text(
        '{"links": [{"id": "L0", "members": ["p1"]}], '
        '"joints": [{"id": "J1", "axis": [0,0,1], "point_mm": [0,0,0], '
        '"parent": "L0", "child": "L0", "confidence": 0.2}], '
        '"base": {"link": "L0"}, "end_effector": {"link": "L0"}}',
        encoding="utf-8")
    overlay_data = overlay.build_overlay(str(pred_path))
    assert overlay_data["colours"]["p1"]["link"] == "L0"
    assert len(overlay_data["arrows"]) == 1
