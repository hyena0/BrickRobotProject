"""Tests for src.gen.assembly_docs (synthetic fixtures only)."""

import json

from src.gen import assembly_docs as ad
from src.parse import parse_text


def _exp_doc():
    return {
        "frame": "base link, Z up, mm",
        "links": [{"id": "L0"}, {"id": "L1"}, {"id": "L2"}],
        "joints": [
            {"id": "J1", "type": "revolute", "parent": "L0",
             "child": "L1", "axis": [0, 0, 1], "point_mm": [0, 0, 20],
             "limits_deg": [-150, 150], "zero": "pose in the source file"},
            {"id": "J2", "type": "revolute", "parent": "L1",
             "child": "L2", "axis": [0, 1, 0], "point_mm": [0, 0, 60],
             "limits_deg": [-30, 100], "zero": "pose in the source file"},
        ],
        "actuators": [
            {"id": "M1", "brick": "left", "port": "A",
             "motor": "medium"},
            {"id": "M2", "brick": "left", "port": "B", "motor": "large"},
        ],
        "transmission": {"coupling": "q = C m",
                         "C": [[0.1, 0.0], [0.0, 0.05]]},
        "end_effector": {"link": "L2", "point_mm": [0, 0, 100]},
    }


def _robot(tmp_path):
    robot = tmp_path / "gen1"
    robot.mkdir()
    ldr = "\n".join([
        "0 test robot",
        "1 7 0 0 0 1 0 0 0 1 0 0 0 1 99455.dat",
        "1 7 0 0 100 1 0 0 0 1 0 0 0 1 95648.dat",
        "1 7 0 0 200 1 0 0 0 1 0 0 0 1 95658.dat",
        "1 7 0 0 300 1 0 0 0 1 0 0 0 1 11478.dat",
    ]) + "\n"
    (robot / "robot.ldr").write_text(ldr, encoding="utf-8")
    (robot / "expected_kinematics.json").write_text(
        json.dumps(_exp_doc()), encoding="utf-8")
    (robot / "design.json").write_text(json.dumps({"design": {"joints": [
        {"module": "M_a", "beam": "11478.dat"},
        {"module": "M_b", "beam": "11478.dat"}]}, "rounds": 1}),
        encoding="utf-8")
    (robot / "report.json").write_text(json.dumps({
        "bom": {"99455.dat": 1, "95648.dat": 1, "95658.dat": 1,
                "11478.dat": 1},
        "segments": {"p000001": 0, "p000002": 0, "p000003": 1,
                     "p000004": 1},
        "beams": [{"uid": "p000004", "part_id": "11478.dat",
                   "segment": 1}],
    }), encoding="utf-8")
    mods = tmp_path / "modules"
    for name, dock in (("M_a", {"kind": "touch",
                                "position_mm": [0, 0, 0]}),
                       ("M_b", None)):
        folder = mods / name
        folder.mkdir(parents=True)
        (folder / "module.json").write_text(json.dumps({
            "bom": {"99455.dat": 1} if name == "M_a" else {},
            "sensor_mount": dock}), encoding="utf-8")
    return robot, mods


def _inventory():
    return {"parts": {"99455.dat": {"total": 4},
                      "95648.dat": {"total": 4},
                      "95658.dat": {"total": 2},
                      "11478.dat": {"total": 50}},
            "precious": ["99455.dat"]}


def test_build_materials_end_to_end(tmp_path):
    robot, mods = _robot(tmp_path)
    summary = ad.build_materials(robot, mods, _inventory())
    assert summary == {"bom_ok": True, "motors": 2, "sensors": 1,
                       "steps": str(robot / "robot_steps.ldr")}

    bom = json.loads((robot / "bom.json").read_text())
    assert bom["bom"]["within_stock"] is True
    assert bom["bom"]["total_parts"] == 4
    assert bom["wiring_map"]["A"]["joint"] == "J1"
    assert bom["wiring_map"]["B"]["joint"] == "J2"
    assert bom["wiring_map"]["1"]["kind"] == "touch"
    assert bom["homing"] == {
        "J1": {"method": "sensor", "sensor_port": None,
               "dock": "touch"},
        "J2": {"method": "stall", "sensor_port": None, "dock": None}}

    steps = (robot / "robot_steps.ldr").read_text()
    assert steps.count("0 STEP") == 2
    assert "M_a" in steps and "M_b" in steps
    assert len(parse_text(steps)) == 4

    # The hardware layer must satisfy the committed wiring check.
    import yaml

    from src.tools.wiring import check as wiring_check

    parts_json = tmp_path / "parts.json"
    parts_json.write_text(json.dumps(
        [{"uid": f"p{i:06d}", "part_id": pid} for i, pid in
         enumerate(["99455.dat", "95648.dat", "95658.dat",
                    "11478.dat"], 1)]), encoding="utf-8")
    catalog_yaml = tmp_path / "catalog.yaml"
    catalog_yaml.write_text(yaml.safe_dump({
        "99455.dat": {"category": "motor"},
        "95648.dat": {"category": "sensor"},
        "95658.dat": {"category": "motor"},
        "11478.dat": {"category": "beam"}}), encoding="utf-8")
    assert wiring_check(str(parts_json), str(catalog_yaml),
                        str(robot / "wiring.yaml")) == []


def test_build_bom_shortfall_and_unknown_stock():
    bom = {"99455.dat": 5}
    out = ad.build_bom(bom, _inventory(), {})
    assert out["within_stock"] is False
    assert out["shortfalls"] == [{"part_id": "99455.dat", "need": 5,
                                  "own": 4}]
    out = ad.build_bom(bom, None, {})
    assert out["within_stock"] is None
    assert out["shortfalls"] == []


def test_build_wiring_rejects_mismatch():
    exp = _exp_doc()
    motors = [{"uid": "p1", "part_id": "99455.dat"},
              {"uid": "p2", "part_id": "99455.dat"}]
    catalog = {"99455.dat": {"category": "motor", "motor": "medium"}}
    try:
        ad.build_wiring(motors, [], catalog, exp)
    except ValueError as exc:
        assert "misses model" in str(exc)
    else:
        raise AssertionError("expected kind mismatch")
    many = [{"uid": f"p{i}", "part_id": "99455.dat"} for i in range(5)]
    try:
        ad.build_wiring(many, [], catalog, exp)
    except ValueError as exc:
        assert "second brick" in str(exc)
    else:
        raise AssertionError("expected brick overflow")


def test_build_wiring_two_brick_branch():
    actuators = [{"id": f"M{i + 1}", "brick": "left" if i < 4 else "right",
                  "port": "ABCD"[i % 4], "motor": "medium"}
                 for i in range(5)]
    exp = {"actuators": actuators}
    motors = [{"uid": f"p{i}", "part_id": "99455.dat"} for i in range(5)]
    catalog = {"99455.dat": {"category": "motor", "motor": "medium"}}
    wiring, assembly = ad.build_wiring(
        motors, [], catalog, exp,
        bricks={"count": 2, "names": ["left", "right"]})
    assert [m["brick"] for m in wiring["motors"]] == \
        ["left"] * 4 + ["right"]
    assert [m["port"] for m in wiring["motors"]] == list("ABCDA")
    assert assembly["right/A"]["joint"] == "J5"
    many = motors + [{"uid": "p5", "part_id": "99455.dat"}] * 4
    try:
        ad.build_wiring(many, [], catalog, exp,
                        bricks={"count": 2, "names": ["left", "right"]})
    except ValueError as exc:
        assert "more than 2 bricks" in str(exc)
    else:
        raise AssertionError("expected brick overflow")


def test_load_bricks_default_and_bad():
    count, names = ad.load_bricks({"count": 1, "names": ["left"]})
    assert (count, names) == (1, ["left"])
    count, names = ad.load_bricks("/nonexistent/bricks.yaml")
    assert (count, names) == (1, ["left"])
    try:
        ad.load_bricks({"count": 2, "names": ["left"]})
    except ValueError as exc:
        assert "bad brick topology" in str(exc)
    else:
        raise AssertionError("expected bad topology")


def test_homing_falls_back_without_sensor_parts(tmp_path):
    robot, mods = _robot(tmp_path)
    plan = ad.homing_plan({"joints": [{"module": "M_a"}]}, mods, {})
    assert plan == {"J1": {"method": "stall", "sensor_port": None,
                           "dock": "touch"}}


def test_main_exit_codes(tmp_path):
    robot, mods = _robot(tmp_path)
    inv = tmp_path / "inv.yaml"
    import yaml

    inv.write_text(yaml.safe_dump(_inventory()), encoding="utf-8")
    assert ad.main(["--robot-dir", str(robot), "--modules", str(mods),
                    "--inventory", str(inv)]) == 0
    short = dict(_inventory(), parts={
        "99455.dat": {"total": 0}, "95648.dat": {"total": 0},
        "95658.dat": {"total": 0}, "11478.dat": {"total": 0}})
    inv.write_text(yaml.safe_dump(short), encoding="utf-8")
    assert ad.main(["--robot-dir", str(robot), "--modules", str(mods),
                    "--inventory", str(inv)]) == 2
