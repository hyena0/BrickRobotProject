"""Tests for src.render.modal_render (pure parts only, no Blender)."""

import math
import os
import threading

import pytest

from src.parse import parse_text
from src.render import modal_render as mr

FIX = os.path.join(os.path.dirname(__file__), "..", "fixtures")


def test_dir_from_az_el():
    assert mr.dir_from_az_el(0.0, 0.0) == pytest.approx([1, 0, 0])
    assert mr.dir_from_az_el(90.0, 0.0) == pytest.approx([0, 1, 0])
    assert mr.dir_from_az_el(0.0, 90.0) == pytest.approx([0, 0, 1])
    vec = mr.dir_from_az_el(35.0, 20.0)
    assert math.sqrt(sum(v * v for v in vec)) == pytest.approx(1.0)


def test_quat_axis_angle_and_apply():
    quat = mr.quat_from_axis_angle([0, 0, 1], math.pi / 2)
    assert mr.quat_apply(quat, [1, 0, 0]) == pytest.approx([0, 1, 0])
    assert mr.quat_apply(quat, [0, 0, 1]) == pytest.approx([0, 0, 1])
    ident = mr.quat_from_axis_angle([1, 0, 0], 0.0)
    assert mr.quat_apply(ident, [1, 2, 3]) == pytest.approx([1, 2, 3])


@pytest.mark.parametrize("forward", [
    [1, 0, 0], [0, 1, 0], [0, 0, -1], [-1, -1, -1], [0.3, -0.5, -0.8],
])
def test_look_quat_faces_forward(forward):
    norm = math.sqrt(sum(v * v for v in forward))
    unit = [v / norm for v in forward]
    got = mr.quat_apply(mr.look_quat(unit), [0, 0, -1])
    assert got == pytest.approx(unit)


def test_job_build_validate_tasks():
    poses = [{"id": "zero", "q_deg": [0.0, 0.0]},
             {"id": "J1p", "q_deg": [10.0, 0.0]}]
    job = mr.build_render_job("gen1", poses)
    assert mr.validate_job(job) == []
    tasks = mr.iter_tasks(job)
    assert len(tasks) == 2 * 3
    assert tasks[0]["id"] == "zero_front"
    assert tasks[-1]["id"] == "J1p_top"

    bad = dict(job, poses=[{"id": "x", "q_deg": [0.0, 0.0]},
                              {"id": "y", "q_deg": [0.0]}])
    assert mr.validate_job(bad) != []
    bad = dict(job, resolution=[1280, 0])
    assert mr.validate_job(bad) != []
    bad = dict(job, samples=0)
    assert mr.validate_job(bad) != []
    bad = dict(job)
    del bad["views"]
    assert mr.validate_job(bad) != []


def test_default_poses():
    exp = {"joints": [
        {"id": "J1", "parent": "L0", "child": "L1",
         "limits_deg": [-100, 100]},
        {"id": "J2", "parent": "L1", "child": "L2",
         "limits_deg": [0, 80]},
    ]}
    poses = mr.default_poses(exp)
    assert [p["id"] for p in poses] == ["zero", "J1p", "J1m", "J2p",
                                        "J2m"]
    assert poses[0]["q_deg"] == [0.0, 0.0]
    assert poses[1]["q_deg"] == [50.0, 0.0]
    assert poses[4]["q_deg"] == [0.0, -20.0]


def test_check_denylist():
    mr.check_denylist("generated_robots/gen1")
    mr.check_denylist("/tmp/smoke5d/gen1")
    mr.check_denylist("config/hardware.local.yaml.example")
    with pytest.raises(ValueError, match="private"):
        mr.check_denylist("assets/private/model.io")
    with pytest.raises(ValueError, match="private"):
        mr.check_denylist("runs/stage1/parts.json")
    with pytest.raises(ValueError, match="private"):
        mr.check_denylist("runs/usage.jsonl")
    with pytest.raises(ValueError, match="private"):
        mr.check_denylist("config/inventory.local.yaml")
    with pytest.raises(ValueError, match="private"):
        mr.check_denylist("/repo/config/surplus.local.yaml")


def _axle_catalog():
    return {
        "axle4.dat": {"category": "axle", "axis_local": [0, 1, 0],
                      "axle_length": 2},
        "beamhole.dat": {"category": "beam", "axis_local": [0, 1, 0],
                         "holes": [{"kind": "round"}]},
    }


def test_split_links_free_joint():
    from src.parse.ldr import parse_file

    parts = parse_file(os.path.join(FIX, "axle_hole.ldr"))
    by_part = {p["part_id"]: p["uid"] for p in parts}
    segments = {p["uid"]: 0 for p in parts}
    beams = [{"uid": by_part["beamhole.dat"], "part_id": "beamhole.dat",
              "segment": 0}]
    out = mr.split_links(parts, _axle_catalog(), segments, beams)
    assert out["warnings"] == []
    assert out["links"] == {
        "L0": [by_part["axle4.dat"]],
        "L1": [by_part["beamhole.dat"]],
    }


def test_split_links_seeds_output_train():
    catalog = {
        "gear8.dat": {"category": "gear", "teeth": 8,
                      "axis_local": [0, 0, 1], "holes": []},
        "beam.dat": {"category": "beam", "holes": []},
        "block.dat": {"category": "brick", "holes": []},
    }
    parts = [
        {"uid": "g1", "part_id": "gear8.dat",
         "position_mm": [0.0, 0.0, 0.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "colour": "7"},
        {"uid": "b1", "part_id": "beam.dat",
         "position_mm": [100.0, 0.0, 0.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "colour": "7"},
        {"uid": "s1", "part_id": "block.dat",
         "position_mm": [0.0, 100.0, 0.0],
         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "colour": "7"},
    ]
    segments = {"g1": 0, "b1": 0, "s1": 0}
    beams = [{"uid": "b1", "part_id": "beam.dat", "segment": 0}]
    plain = mr.split_links(parts, catalog, segments, beams)
    assert plain["links"]["L1"] == ["b1"]
    joints = [{"axis": [0, 1, 0], "point_mm": [0.0, 0.0, 0.0]}]
    seeded = mr.split_links(parts, catalog, segments, beams, joints=joints)
    assert seeded["links"]["L1"] == ["b1", "g1"]
    assert seeded["links"]["L0"] == ["s1"]


def test_split_links_all_rigid_raises():
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
    segments = {p["uid"]: 0 for p in parts}
    beams = [{"uid": parts[1]["uid"], "part_id": "beamA.dat",
              "segment": 0}]
    with pytest.raises(ValueError, match="no free joint"):
        mr.split_links(parts, catalog, segments, beams)


def test_inner_script_compiles():
    script = mr.build_inner_script()
    assert "@@MATH@@" not in script
    for name in ("dir_from_az_el", "quat_from_axis_angle", "look_quat",
                 "import_scene.gltf", "compute_device_type"):
        assert name in script
    compile(script, "inner.py", "exec")


def test_modal_definitions_import():
    assert mr._MODAL_OK is True
    assert mr.app is not None
    assert hasattr(mr.render_task, "map")
    assert mr.BLENDER_VERSION == "5.2.2"


def test_heavy_lock_serializes(tmp_path):
    order = []
    release = threading.Event()
    entered = threading.Event()

    def hold():
        with mr.heavy_lock(str(tmp_path / "heavy.lock")):
            entered.set()
            order.append("a-in")
            assert release.wait(timeout=10)
            order.append("a-out")

    def wait():
        assert entered.wait(timeout=10)
        with mr.heavy_lock(str(tmp_path / "heavy.lock")):
            order.append("b-in")

    first = threading.Thread(target=hold)
    first.start()
    second = threading.Thread(target=wait)
    second.start()
    assert entered.wait(timeout=10)
    release.set()
    first.join(timeout=10)
    second.join(timeout=10)
    assert order == ["a-in", "a-out", "b-in"]


def test_blender_command_shape():
    cmd = mr.blender_command("s.py", "/bin/Blender", ["--x", "1"])
    assert cmd[:4] == ["nice", "-n", "15", "/bin/Blender"]
    assert "--threads" in cmd and "2" in cmd
    assert cmd[-3:] == ["--", "--x", "1"]
