"""Tests for src.gen.ev3_gate (synthetic fixtures + arm integration).

Uses a synthetic uniform-stock inventory only; never the private
config/inventory.local.yaml. The arm_4dof pass case rebuilds the pinned
design from templates with granted stock.
"""

import copy
import json

from src.gen import assemble as asm
from src.gen import ev3_gate as gate
from src.gen import templates as tpl
from src.gen.inventory import load_catalog_merged
from src.gen.spec import load_spec

CAT = load_catalog_merged()
IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _part(uid, pid, pos, seg=0, beam=False):
    return {"uid": uid, "part_id": pid, "position_mm": list(pos),
            "rotation": [row[:] for row in IDENT], "colour": "7",
            "_segment": seg, "_beam": beam}


def _joint(jid, axis, point, child="L1", parent="L0"):
    return {"id": jid, "type": "revolute", "parent": parent, "child": child,
            "axis": list(axis), "point_mm": list(point),
            "limits_deg": [-180, 180], "confidence": 1.0, "reason": "fake"}


def _report(parts, joints, actuators, lengths, honey_c):
    n = len(joints)
    return {
        "parts": parts,
        "link_lengths": list(lengths),
        "expected_kinematics": {
            "frame": "base link, Z up, mm",
            "links": [{"id": f"L{i}"} for i in range(n + 1)],
            "joints": joints,
            "actuators": actuators,
            "transmission": {"coupling": "q = C m", "C": honey_c},
            "end_effector": {"link": f"L{n}",
                             "point_mm": [0.0, 0.0, 100.0]},
        },
    }


def _one_joint_report():
    parts = [_part("m1", "99455.dat", [0, 0, 0]),
             _part("b1", "32316.dat", [500, 0, 0], beam=True)]
    joints = [_joint("J1", [0, 1, 0], [0, 0, 0])]
    actuators = [{"id": "M1", "brick": "left", "port": "A",
                  "motor": "medium"}]
    return _report(parts, joints, actuators, [100.0], [[1.0]])


def _config():
    return copy.deepcopy(gate.load_config(None))


def test_ports_fail_five_motors():
    parts = [_part(f"m{i}", "99455.dat", [i * 500.0, 0, 0])
             for i in range(5)]
    parts.append(_part("b1", "32316.dat", [3000, 0, 0], beam=True))
    joints = [_joint(f"J{i + 1}", [0, 0, 1], [0, 0, i * 10.0],
                     child=f"L{i + 1}", parent=f"L{i}")
              for i in range(5)]
    actuators = [{"id": f"M{i + 1}", "brick": "left",
                  "port": "ABCD"[i] if i < 4 else "A",
                  "motor": "medium"} for i in range(5)]
    eye = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    report = _report(parts, joints, actuators, [10.0] * 5, eye)
    verdict = gate.check_ev3(report, CAT, None, _config())
    assert verdict["pass"] is False
    assert verdict["ports"]["pass"] is False
    assert any("second brick" in p
               for p in verdict["ports"]["problems"])


def test_torque_fail_excess_mass():
    cfg = _config()
    cfg["masses_g"]["by_category"]["beam"] = 4000.0
    cfg["masses_g"]["by_part"]["32316.dat"] = 4000.0
    verdict = gate.check_ev3(_one_joint_report(), CAT, None, cfg)
    assert verdict["torque"]["pass"] is False
    assert verdict["pass"] is False


def test_cable_fail_short_cable():
    cfg = _config()
    cfg["cable"]["length_mm"] = 100.0
    verdict = gate.check_ev3(_one_joint_report(), CAT, None, cfg)
    assert verdict["cable"]["pass"] is False
    assert verdict["pass"] is False


def test_stock_fail_shortfall():
    inv = {"parts": {"99455.dat": {"total": 0}}}
    verdict = gate.check_ev3(_one_joint_report(), CAT, inv, _config())
    assert verdict["stock"]["pass"] is False
    assert verdict["stock"]["shortfalls"] == [
        {"kind": "medium", "need": 1, "own": 0}]


def test_check_coupling():
    assert gate.check_coupling([[1.0, 0.0], [0.0, 0.5]], 2, 2) == []
    assert gate.check_coupling([[0.0, 1.0], [0.5, 0.0]], 2, 2) == []
    bad = gate.check_coupling([[1.0, 0.0], [0.5, 0.0]], 2, 2)
    assert any("coupling" in p for p in bad)
    bad = gate.check_coupling([[1.0, 0.0, 0.0]], 1, 3)
    assert any("3 motors for 1 joints" in p for p in bad)


def _mount(z):
    return {"position_mm": [0.0, 0.0, z],
            "rotation": [row[:] for row in IDENT]}


def test_arm_4dof_passes(tmp_path):
    picks = [("base_yaw", "Medium", [0, 0, 1], 0.0, 1.0, "32316.dat"),
             ("pitch", "Large", [0, 1, 0], -4.0, 0.6, "32316.dat"),
             ("pitch", "Large", [0, 1, 0], -4.0, 0.6, "6632.dat"),
             ("wrist_roll", "Medium", [0, 0, 1], 0.0, 1.0, "6632.dat")]
    height = {"base_yaw": 52.0, "pitch": 64.0, "wrist_roll": 52.0}
    built = {}
    for name, motor, axis, in_z, ratio, _ in picks:
        if name in built:
            continue
        built[name] = tpl.TEMPLATES[name](
            axis, _mount(in_z), _mount(in_z + height[name]), ratio, motor,
            catalog=CAT, inventory=None)
    bom = {}
    for module, _ in built.values():
        for pid, count in module["bom"].items():
            bom[pid] = bom.get(pid, 0) + count
    for _, _, _, _, _, beam in picks:
        bom[beam] = bom.get(beam, 0) + 1
    inv = {"parts": {pid: {"total": 999} for pid in bom}, "precious": []}
    modules_root = tmp_path / "modules"
    for name, (module, parts) in built.items():
        tpl.write_module_files(modules_root, name, module, parts,
                               catalog=CAT, inventory=inv)
    design = {"joints": [{"module": name, "beam": beam, "reason": "test"}
                         for name, _, _, _, _, beam in picks]}
    spec = load_spec("specs/arm_4dof.json")
    report = asm.assemble_design(design, spec, modules_root, CAT, inv)
    assert report["ok"] is True
    verdict = report["ev3_gate"]
    assert verdict["pass"] is True, json.dumps(verdict, indent=1)[:2000]
    for key in ("ports", "stock", "torque", "cable", "driver"):
        assert verdict[key]["pass"] is True, key
