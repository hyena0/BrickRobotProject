"""Stage 5 tests: code generator and generated motion (synthetic only).

The real model (runs/, assets/private/) and ground truth
(eval/ground_truth/, gt/) are never read here; fixtures are built in
tmp_path.
"""

from __future__ import annotations

import importlib
import json
import sys

import pytest
import yaml

from src.codegen.generate import (build_plan, drive_columns, generate,
                                  motor_port_full, sensor_port_full)


def make_model():
    axes = [[0, 0, 1], [0, 1, 0], [1, 0, 0]]
    points = [[0, 0, 0], [0, 0, 100], [100, 0, 100]]
    joints = [{"id": f"J{i + 1}", "type": "revolute",
               "parent": f"D{i}", "child": f"D{i + 1}",
               "axis": axes[i], "point_mm": points[i],
               "limits_deg": [-90, 90],
               "zero": "pose in the source file"} for i in range(3)]
    return {"frame": "base link, Z up, mm",
            "links": [{"id": f"D{i}"} for i in range(4)],
            "joints": joints,
            "actuators": [
                {"id": "m1", "brick": "left", "port": "A",
                 "motor": "large"},
                {"id": "m2", "brick": "left", "port": "B",
                 "motor": "medium"},
                {"id": "m3", "brick": "right", "port": "A",
                 "motor": "medium"}],
            "transmission": {"coupling": "q = C m",
                             "C": [[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.5],
                                   [0.0, 0.0, 1.0]]},
            "end_effector": {"link": "D3", "point_mm": list(points[-1])}}


def make_ports():
    return {"bricks": {"left": {"role": "master"},
                       "right": {"role": "slave"}},
            "motors": [{"uid": "m1", "brick": "left", "port": "A"},
                       {"uid": "m2", "brick": "left", "port": "B"},
                       {"uid": "m3", "brick": "right", "port": "A"}],
            "sensors": [{"uid": "s1", "brick": "left", "port": 1},
                        {"uid": "s2", "brick": "right", "port": 2}]}


def make_sensors():
    return [{"uid": "s1", "brick": "left", "port": 1, "mount_link": "D0",
             "joint": "J1", "homing_direction": "unknown"},
            {"uid": "s2", "brick": "right", "port": 2, "mount_link": "D2",
             "joint": "J2", "homing_direction": "unknown"}]


def write_inputs(tmp_path, homing=None):
    model_p = tmp_path / "model.json"
    model_p.write_text(json.dumps(make_model()), encoding="utf-8")
    ports_p = tmp_path / "ports.yaml"
    ports_p.write_text(yaml.safe_dump(make_ports()), encoding="utf-8")
    sens_p = tmp_path / "sensors.json"
    sens_p.write_text(json.dumps(make_sensors()), encoding="utf-8")
    hom_p = None
    if homing is not None:
        hom_p = tmp_path / "homing.yaml"
        hom_p.write_text(yaml.safe_dump(homing), encoding="utf-8")
    return model_p, ports_p, sens_p, hom_p


def load_pkg(tmp_path, name="tstbot"):
    sys.path.insert(0, str(tmp_path))
    try:
        for mod in (f"{name}.robot_config", f"{name}.motion",
                    f"{name}.home", f"{name}.guards"):
            sys.modules.pop(mod, None)
        sys.modules.pop(name, None)
        cfg = importlib.import_module(f"{name}.robot_config")
        motion = importlib.import_module(f"{name}.motion")
        home = importlib.import_module(f"{name}.home")
        guards = importlib.import_module(f"{name}.guards")
        return cfg, motion, home, guards
    finally:
        sys.path.remove(str(tmp_path))


def test_port_normalization():
    assert motor_port_full("B") == "outB"
    assert motor_port_full("outC") == "outC"
    assert sensor_port_full(1) == "in1"
    assert sensor_port_full("in2") == "in2"


def test_drive_columns_argmax_and_zero_row_raises():
    assert drive_columns([[1.0, 0.0], [0.0, 0.5]]) == [0, 1]
    assert drive_columns([[0.2, 0.8]]) == [1]
    with pytest.raises(ValueError):
        drive_columns([[0.0, 0.0]])


def test_generate_writes_importable_package(tmp_path):
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    res = generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
                   str(sens_p), None, robot="tstbot")
    assert len(res["written"]) == 5
    cfg, motion, home, guards = load_pkg(tmp_path)
    assert cfg.JOINT_IDS == ["J1", "J2", "J3"]
    assert cfg.MOTOR_IDS == ["m1", "m2", "m3"]
    assert hasattr(motion, "plan_move") and hasattr(motion, "move_joints")
    assert hasattr(home, "home_all")
    assert hasattr(guards, "apply_guards") and hasattr(guards, "clear_all")


def test_soft_limits_narrowed_by_five_deg(tmp_path):
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    res = generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
                   str(sens_p), None, robot="tstbot")
    for j in res["plan"]["joints"]:
        lo, hi = j["limits_deg"], j["soft_limits_deg"]
        assert hi[0] == lo[0] + 5.0 and hi[1] == lo[1] - 5.0


def test_guards_cover_every_sensor(tmp_path):
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    res = generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
                   str(sens_p), None, robot="tstbot")
    rules = res["plan"]["guard_rules"]
    assert rules["left"] == [{"sensor": "in1", "invert": False}]
    assert rules["right"] == [{"sensor": "in2", "invert": False}]


def test_home_plan_methods(tmp_path):
    # J1 sensor and motor share left -> local; J2 sensor is on right
    # while its motor is on left -> remote; J3 has no sensor -> stall.
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    res = generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
                   str(sens_p), None, robot="tstbot")
    by_joint = {h["joint"]: h for h in res["plan"]["home_plan"]}
    assert by_joint["J1"]["method"] == "local"
    assert by_joint["J2"]["method"] == "remote"
    assert by_joint["J3"]["method"] == "stall"
    assert all(h["direction"] == -1 and not h["direction_confirmed"]
               for h in res["plan"]["home_plan"])


def test_homing_file_overrides(tmp_path):
    homing = {"zeros": {"m1": 2.5},
              "homing_direction": {"J1": 1},
              "homing_confirmed": {"J1": True}}
    model_p, ports_p, sens_p, hom_p = write_inputs(tmp_path, homing)
    res = generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
                   str(sens_p), str(hom_p), robot="tstbot")
    assert res["plan"]["zero_offsets"]["m1"] == 2.5
    by_joint = {h["joint"]: h for h in res["plan"]["home_plan"]}
    assert by_joint["J1"]["direction"] == 1
    assert by_joint["J1"]["direction_confirmed"] is True
    assert by_joint["J2"]["direction_confirmed"] is False


def test_generated_files_carry_no_assembly_data(tmp_path):
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    res = generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
                   str(sens_p), None, robot="tstbot")
    for path in res["written"]:
        if path.endswith(".py"):
            text = open(path, encoding="utf-8").read()
            assert "position_mm" not in text
            assert '"members"' not in text


def test_plan_move_synchronized_speeds(tmp_path):
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
             str(sens_p), None, robot="tstbot")
    _cfg, motion, _home, _guards = load_pkg(tmp_path)
    cmds, plan = motion.plan_move([30.0, 0.0, 0.0], duration_s=4.0)
    assert cmds["m1"] == pytest.approx(30.0)
    assert plan["speeds_deg_s"]["m1"] == pytest.approx(7.5)
    assert plan["duration_s"] == pytest.approx(4.0)


def test_plan_move_soft_limit_raises(tmp_path):
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
             str(sens_p), None, robot="tstbot")
    _cfg, motion, _home, _guards = load_pkg(tmp_path)
    with pytest.raises(ValueError):
        motion.plan_move([88.0, 0.0, 0.0])  # soft cap is 85


def test_plan_move_cap_stretches_duration(tmp_path):
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
             str(sens_p), None, robot="tstbot")
    _cfg, motion, _home, _guards = load_pkg(tmp_path)
    _cmds, plan = motion.plan_move([80.0, 0.0, 0.0], duration_s=1.0)
    assert plan["duration_s"] > 1.0
    assert max(plan["speeds_deg_s"].values()) <= 30.0 + 1e-9


def test_home_all_shared_motor_homed_once(tmp_path):
    from src.codegen.bringup import build_full_fake
    model_p, ports_p, sens_p, _ = write_inputs(tmp_path)
    generate(str(model_p), str(ports_p), str(tmp_path / "tstbot"),
             str(sens_p), None, robot="tstbot")
    cfg, _motion, home, guards = load_pkg(tmp_path)
    driver = build_full_fake(
        [{"brick": "left", "port": "outA"},
         {"brick": "left", "port": "outB"},
         {"brick": "right", "port": "outA"}],
        {"left": [{"sensor": "in1", "invert": False}],
         "right": [{"sensor": "in2", "invert": False}]})
    try:
        guards.apply_guards(driver)
        report = home.home_all(driver)
        methods = [r["method"] for r in report["joints"]]
        assert methods[0] == "local"
        assert set(report["encoders"]) == {"left:outA", "left:outB",
                                           "right:outA"}
    finally:
        driver.close()


def test_build_plan_needs_coupling(tmp_path):
    doc = make_model()
    doc["transmission"] = {}
    with pytest.raises(ValueError):
        build_plan(doc, make_ports(), make_sensors(), {})
