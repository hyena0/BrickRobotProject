"""Tests for src.gen.skeleton (spec-pure, no assembler imports)."""

import json

from src.gen import skeleton as sk
from src.gen.spec import load_spec


def _spec():
    return load_spec("specs/arm_3dof.json")


def test_reference_spec_green(tmp_path):
    report = sk.run_skeleton("specs/arm_3dof.json", tmp_path)
    assert report["ok"] is True
    assert report["overall_pass"] is True
    assert report["checks"]["a_axes"]["pass"] is True
    assert report["checks"]["b_links"]["pass"] is True
    assert report["checks"]["c_fk"]["pass"] is True
    assert (tmp_path / "frames.json").is_file()
    assert (tmp_path / "skeleton.ldr").is_file()
    assert (tmp_path / "skeleton_report.json").is_file()


def test_frames_stack_and_axes():
    frames = sk.joint_frames(_spec())
    assert [f["id"] for f in frames] == ["J1", "J2", "J3"]
    assert [f["point_mm"] for f in frames] == [
        [0.0, 0.0, 16.0], [0.0, 0.0, 152.0], [0.0, 0.0, 240.0]]
    assert [f["axis"] for f in frames] == [
        [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0]]
    for frame in frames:
        mat = frame["matrix"]
        assert len(mat) == 4 and all(len(row) == 4 for row in mat)
        assert mat[3] == [0.0, 0.0, 0.0, 1.0]


def test_pair_rules():
    ok, _ = sk._pair_ok("yaw", [0, 0, 1], "pitch", [0, 1, 0])
    assert ok
    ok, _ = sk._pair_ok("pitch", [0, 1, 0], "pitch", [0, 1, 0])
    assert ok
    ok, note = sk._pair_ok("yaw", [0, 0, 1], "pitch", [0, 1, 0.05])
    assert not ok and "FAIL" in note
    ok, _ = sk._pair_ok("pitch", [0, 1, 0], "pitch", [0, 1, 0.1])
    assert not ok


def test_bad_spec_fails_loud(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"joints": ["yaw"], "link_lengths_mm": [1],
                               "ranges_deg": [[0, 0]], "gripper": False}))
    report = sk.run_skeleton(bad, tmp_path / "out")
    assert report["ok"] is False
    assert report["overall_pass"] is False


def test_skeleton_ldr_marks_joints(tmp_path):
    out = tmp_path / "skeleton.ldr"
    sk.write_skeleton_ldr(sk.joint_frames(_spec()), out)
    text = out.read_text()
    assert text.count("0 // joint J") == 3
    # One cube (12 tris) + one stub (12 tris) per joint.
    assert sum(1 for ln in text.splitlines()
               if ln.startswith("4 ")) == 72
