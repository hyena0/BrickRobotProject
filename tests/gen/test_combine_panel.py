"""Tests for src.render.combine_panel (pure flow, no Blender)."""

import json

from src.gen import templates as tpl
from src.gen.inventory import load_catalog_merged
from src.gen.spec import load_spec
from src.render import combine_panel as panel

CAT = load_catalog_merged()
IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
PICKS = [("base_yaw", "Medium", [0, 0, 1], 0.0, 1.0, "32316.dat"),
         ("pitch", "Large", [0, 1, 0], -4.0, 0.6, "32316.dat"),
         ("pitch", "Large", [0, 1, 0], -4.0, 0.6, "6632.dat"),
         ("wrist_roll", "Medium", [0, 0, 1], 0.0, 1.0, "6632.dat")]
HEIGHT = {"base_yaw": 52.0, "pitch": 64.0, "wrist_roll": 52.0}


def _mount(z):
    return {"position_mm": [0.0, 0.0, z],
            "rotation": [row[:] for row in IDENT]}


def _library(tmp_path):
    built = {}
    for name, motor, axis, in_z, ratio, _ in PICKS:
        if name in built:
            continue
        built[name] = tpl.TEMPLATES[name](
            axis, _mount(in_z), _mount(in_z + HEIGHT[name]), ratio, motor,
            catalog=CAT, inventory=None)
    bom = {}
    for module, _ in built.values():
        for pid, count in module["bom"].items():
            bom[pid] = bom.get(pid, 0) + count
    for _, _, _, _, _, beam in PICKS:
        bom[beam] = bom.get(beam, 0) + 1
    inv = {"parts": {pid: {"total": 999} for pid in bom}, "precious": []}
    root = tmp_path / "modules"
    for name, (module, parts) in built.items():
        tpl.write_module_files(root, name, module, parts,
                               catalog=CAT, inventory=inv)
    picks = [{"module": name, "beam": beam} for name, _, _, _, _, beam
             in PICKS]
    return root, picks, inv


def test_list_templates_excludes_extracted(tmp_path):
    root, _, _ = _library(tmp_path)
    (root / "M_old").mkdir()
    (root / "M_old" / "module.json").write_text(json.dumps({
        "name": "M_old", "frame": "module local, Z up, mm",
        "geometry": "module.ldr"}), encoding="utf-8")
    (root / "M_old" / "module.ldr").write_text("0 x\n", encoding="utf-8")
    (root / "broken").mkdir()
    out = panel.list_templates(root)
    assert sorted(e["name"] for e in out["templates"]) == [
        "base_yaw", "pitch", "wrist_roll"]
    assert sorted(out["skipped"]) == ["M_old", "broken"]
    assert panel.list_templates(root / "nope")["templates"] == []


def test_combine_status_shows_gates(tmp_path):
    root, picks, inv = _library(tmp_path)
    spec = load_spec("specs/arm_4dof.json")
    report = panel.combine(root, picks, spec, tmp_path / "bot", "bot",
                           CAT, inv)
    status = panel.panel_status(report)
    assert status["assemble_ok"] is True
    assert status["reach_actual_mm"] == 276.0
    assert status["stock_ok"] is True
    assert status["ev3_pass"] is True
    assert status["ports"] == ["A", "B", "C", "D"]
    assert [row[0] for row in status["torque"]] == ["J1", "J2", "J3",
                                                   "J4"]
    assert len(status["cable"]) == 4
    assert status["driver_pass"] is True
    text = panel.format_status(status)
    assert "ev3 pass=True" in text


def test_run_flow_no_render(tmp_path):
    root, picks, inv = _library(tmp_path)
    spec = load_spec("specs/arm_4dof.json")
    out = panel.run_flow(root, picks, spec, tmp_path / "bot", "bot",
                         CAT, inv, render=False)
    assert out["roundtrip"]["overall_pass"] is True
    assert out["blend"] is None
    for key in ("robot.ldr", "roundtrip.json", "bom.json",
                "wiring.yaml", "robot_steps.ldr"):
        assert (tmp_path / "bot" / key).is_file(), key


def test_no_bpy_fallback():
    assert panel.bpy is None
    assert panel.CLASSES == ()
    panel.register()
    panel.unregister()
