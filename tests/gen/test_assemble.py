"""Tests for src.gen.assemble (synthetic fixtures only)."""

import json

import pytest

from src.gen import assemble as asm
from src.gen.inventory import load_catalog_merged
from src.kin import fk as fk_mod
from src.kin import model as kin_model


def _module(name, axis, bom):
    return {
        "name": name,
        "frame": "module local, Z up, mm",
        "geometry": "module.ldr",
        "joint": {"axis": axis, "point_mm": [0, 0, 20],
                  "range_deg": [-150, 150],
                  "joint_deg_per_motor_deg": 0.1},
        "motor": {"kind": "medium"},
        "sensor_mount": None,
        "input_mount": {"position_mm": [0, 0, 0],
                        "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        "output_mount": {"position_mm": [0, 0, 40],
                         "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]},
        "size_mm": [60, 60, 50],
        "bom": bom,
    }


def _write_module(root, module, ldr_lines):
    folder = root / module["name"]
    folder.mkdir(parents=True)
    (folder / "module.json").write_text(json.dumps(module),
                                        encoding="utf-8")
    (folder / "module.ldr").write_text("\n".join(ldr_lines) + "\n",
                                       encoding="utf-8")


def _two_modules(tmp_path):
    line = "1 7 0 0 0 1 0 0 0 1 0 0 0 1 11478.dat"
    _write_module(tmp_path, _module("M_a", [0, 0, 1], {"11478.dat": 1}),
                  [line])
    _write_module(tmp_path, _module("M_b", [0, 1, 0], {"11478.dat": 1}),
                  [line])
    return tmp_path


def test_list_beams_real_catalog():
    beams = asm.list_beams(load_catalog_merged(), None)
    ids = {b["part_id"] for b in beams}
    straight = {"14720.dat", "32523.dat", "6632.dat", "32449.dat",
                "11478.dat", "32017.dat", "32316.dat", "32524.dat",
                "40490.dat", "32525.dat", "41239.dat", "32278.dat"}
    assert straight <= ids
    bent = {"32140.dat", "32526.dat", "60484.dat", "64178.dat",
            "64179.dat", "99773.dat", "41677.dat", "60483.dat"}
    assert ids.isdisjoint(bent)
    spans = [b["span_mm"] for b in beams]
    assert spans == sorted(spans)
    assert all(3 <= b["holes"] <= 15 for b in beams)


def test_list_beams_filters_non_straight():
    catalog = {
        "curve.dat": {"category": "beam", "holes": [
            {"pos": [0, 0, 0]}, {"pos": [20, 0, 0]}, {"pos": [20, 20, 0]}]},
        "short.dat": {"category": "beam", "holes": [
            {"pos": [0, 0, 0]}, {"pos": [20, 0, 0]}]},
        "ok.dat": {"category": "beam", "holes": [
            {"pos": [0, 0, 0]}, {"pos": [20, 0, 0]}, {"pos": [40, 0, 0]}]},
        "pin.dat": {"category": "pin", "holes": [
            {"pos": [0, 0, 0]}, {"pos": [20, 0, 0]}, {"pos": [40, 0, 0]}]},
    }
    beams = asm.list_beams(catalog, None)
    assert [b["part_id"] for b in beams] == ["ok.dat"]
    assert beams[0]["holes"] == 3
    assert beams[0]["span_mm"] == pytest.approx(16.0)


def test_place_chain_geometry(tmp_path):
    root = _two_modules(tmp_path)
    catalog = load_catalog_merged()
    beams = asm.list_beams(catalog, None)[:2]
    modules = [json.loads((root / n / "module.json").read_text())
               for n in ("M_a", "M_b")]
    geometries = [[{"uid": "p1", "part_id": "11478.dat", "colour": "7",
                    "position_mm": [0.0, 0.0, 0.0],
                    "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}],
                  [{"uid": "p2", "part_id": "11478.dat", "colour": "7",
                    "position_mm": [0.0, 0.0, 0.0],
                    "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}]]
    out = asm.place_chain(modules, geometries, beams, catalog)
    s1, s2 = beams[0]["span_mm"], beams[1]["span_mm"]
    assert out["joints"][0]["point"] == pytest.approx([0, 0, 20])
    assert out["joints"][1]["point"] == pytest.approx([0, 0, 60 + s1])
    assert out["joints"][1]["axis"] == pytest.approx([0, 1, 0])
    assert out["link_lengths"] == pytest.approx([20, 40 + s1])
    assert out["ee"] == pytest.approx([0, 0, 80 + s1 + s2])
    assert len(out["parts"]) == 4  # 2 module parts + 2 beams


def test_check_overlap_flags_only_non_adjacent():
    def part(uid, seg, pos):
        return {"uid": uid, "part_id": "unknown.dat", "colour": "7",
                "position_mm": list(pos),
                "rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "_segment": seg}

    adjacent = [part("p1", 0, [0, 0, 0]), part("p2", 1, [0, 0, 0])]
    assert asm.check_overlap(adjacent, {})["ok"] is True
    distant = [part("p1", 0, [0, 0, 0]), part("p2", 2, [0, 0, 0])]
    report = asm.check_overlap(distant, {})
    assert report["ok"] is False
    assert len(report["pairs"]) == 1


def test_assemble_design_end_to_end(tmp_path):
    root = _two_modules(tmp_path)
    catalog = load_catalog_merged()
    beams = asm.list_beams(catalog, None)[:2]
    s1 = beams[0]["span_mm"]
    spec = {"joints": ["yaw", "pitch"],
            "link_lengths_mm": [20, 40 + s1],
            "ranges_deg": [[-150, 150], [-30, 100]],
            "gripper": False}
    design = {"joints": [{"module": "M_a", "beam": beams[0]["part_id"]},
                         {"module": "M_b", "beam": beams[1]["part_id"]}]}
    inventory = {"parts": {
        "11478.dat": {"total": 50},
        beams[0]["part_id"]: {"total": 5},
        beams[1]["part_id"]: {"total": 5}},
        "precious": []}
    report = asm.assemble_design(design, spec, root, catalog, inventory)
    assert report["ok"] is True
    assert report["link_errors_mm"] == pytest.approx([0, 0])
    assert report["design_ranges_deg"] == [[-150, 150], [-30, 100]]
    assert report["overlap"]["ok"] is True
    assert report["bom_check"]["ok"] is True

    out_dir = tmp_path / "gen1"
    asm.write_outputs(report, out_dir, "gen1")
    assert (out_dir / "robot.ldr").is_file()
    assert (out_dir / "expected_kinematics.json").is_file()
    assert (out_dir / "report.json").is_file()
    saved = json.loads((out_dir / "report.json").read_text())
    assert len(saved["segments"]) == 4
    assert len(saved["beams"]) == 2
    assert {b["segment"] for b in saved["beams"]} == {0, 1}

    doc = kin_model.load_model(str(out_dir / "expected_kinematics.json"))
    assert [j["id"] for j in kin_model.chain_order(doc)] == ["J1", "J2"]
    zero = fk_mod.fk_model([0, 0], doc)
    assert zero["pos_mm"] == pytest.approx(report["expected_kinematics"]
                                           ["end_effector"]["point_mm"])
    flat_c = [v for row in doc["transmission"]["C"] for v in row]
    assert flat_c == pytest.approx([0.1, 0.0, 0.0, 0.1])


def test_assemble_design_rejects_bad_inputs(tmp_path):
    root = _two_modules(tmp_path)
    catalog = load_catalog_merged()
    beams = asm.list_beams(catalog, None)[:2]
    spec = {"joints": ["yaw", "pitch"],
            "link_lengths_mm": [20, 60],
            "ranges_deg": [[-150, 150], [-30, 100]],
            "gripper": False}
    good = {"joints": [{"module": "M_a", "beam": beams[0]["part_id"]},
                       {"module": "M_b", "beam": beams[1]["part_id"]}]}

    bad_beam = {"joints": [{"module": "M_a", "beam": "nope.dat"},
                           {"module": "M_b", "beam": beams[1]["part_id"]}]}
    with pytest.raises(ValueError, match="not a straight"):
        asm.assemble_design(bad_beam, spec, root, catalog, None)

    bad_range = dict(spec, ranges_deg=[[200, 300], [-30, 100]])
    with pytest.raises(ValueError, match="misses module"):
        asm.assemble_design(good, bad_range, root, catalog, None)

    with pytest.raises(ValueError, match="gripper"):
        asm.assemble_design(good, dict(spec, gripper=True), root,
                            catalog, None)

    short = {"joints": good["joints"][:1]}
    with pytest.raises(ValueError, match="must match spec"):
        asm.assemble_design(short, spec, root, catalog, None)


def test_assemble_design_rejects_mount_mismatch(tmp_path):
    root = _two_modules(tmp_path)
    other = _module("M_b", [0, 1, 0], {"11478.dat": 1})
    other["input_mount"]["rotation"] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    (root / "M_b" / "module.json").write_text(json.dumps(other),
                                              encoding="utf-8")
    catalog = load_catalog_merged()
    beams = asm.list_beams(catalog, None)[:2]
    spec = {"joints": ["yaw", "pitch"],
            "link_lengths_mm": [20, 60],
            "ranges_deg": [[-150, 150], [-30, 100]],
            "gripper": False}
    design = {"joints": [{"module": "M_a", "beam": beams[0]["part_id"]},
                         {"module": "M_b", "beam": beams[1]["part_id"]}]}
    with pytest.raises(ValueError, match="orientation mismatch"):
        asm.assemble_design(design, spec, root, catalog, None)


def test_build_expected_kinematics_motor_limit():
    modules = [{"joint": {"joint_deg_per_motor_deg": 1.0},
                "motor": {"kind": "medium"}}] * 5
    assembly = {"joints": [{"axis": [0, 0, 1], "point": [0, 0, 0]}] * 5,
                "ee": [0, 0, 0]}
    with pytest.raises(ValueError, match="at most 4 motors"):
        asm.build_expected_kinematics(modules, assembly,
                                      [[-90, 90]] * 5)
