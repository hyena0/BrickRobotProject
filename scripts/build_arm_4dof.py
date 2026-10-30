"""Build the arm_4dof generated robot from joint templates.

Pinned design (the greedy designer cannot tell coaxial yaw/roll modules
apart, so module picks are explicit)::

    J1 base_yaw   (Medium) + 32316 beam (32 mm)
    J2 pitch      (Large)  + 32316 beam (32 mm)
    J3 pitch      (Large)  + 6632 beam (16 mm)
    J4 wrist_roll (Medium) + 6632 beam (16 mm, end effector)

Writes modules + design + assembly + docs into
generated_robots/arm_4dof/::

    modules/{base_yaw,pitch,wrist_roll}/module.{json,ldr}
    design.json robot.ldr expected_kinematics.json report.json
    bom.json wiring.yaml

Usage: PYTHONPATH=. .venv/bin/python scripts/build_arm_4dof.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.gen import assemble as asm  # noqa: E402
from src.gen import assembly_docs as docs  # noqa: E402
from src.gen import templates as tpl  # noqa: E402
from src.gen.inventory import load_catalog_merged  # noqa: E402
from src.gen.spec import load_spec  # noqa: E402

OUT = ROOT / "generated_robots" / "arm_4dof"
IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

PICKS = [
    ("base_yaw", "Medium", [0, 0, 1], 0.0, 1.0, "32316.dat", 32.0),
    ("pitch", "Large", [0, 1, 0], -4.0, 0.6, "32316.dat", 32.0),
    ("pitch", "Large", [0, 1, 0], -4.0, 0.6, "6632.dat", 16.0),
    ("wrist_roll", "Medium", [0, 0, 1], 0.0, 1.0, "6632.dat", 16.0),
]
HEIGHT = {"base_yaw": 52.0, "pitch": 64.0, "wrist_roll": 52.0}


def _mount(z):
    return {"position_mm": [0.0, 0.0, z],
            "rotation": [row[:] for row in IDENT]}


def main() -> int:
    import yaml

    catalog = load_catalog_merged()
    inv_path = ROOT / "config" / "inventory.local.yaml"
    inventory = yaml.safe_load(inv_path.read_text()) if inv_path.exists() \
        else None
    if inventory is None:
        print("note: config/inventory.local.yaml missing, "
              "skipping stock check")
    modules_root = OUT / "modules"
    made = set()
    for name, motor, axis, in_z, ratio, _, _ in PICKS:
        if name in made:
            continue
        made.add(name)
        builder = tpl.TEMPLATES[name]
        module, parts = builder(axis, _mount(in_z),
                                _mount(in_z + HEIGHT[name]), ratio, motor,
                                catalog=catalog, inventory=inventory)
        tpl.write_module_files(modules_root, name, module, parts,
                               catalog=catalog, inventory=inventory)
        print(f"module {name}: {len(parts)} parts, "
              f"ratio {module['joint']['joint_deg_per_motor_deg']}, "
              f"motor {module['motor']['kind']}")
    beam_infos = {b["part_id"]: b for b in asm.list_beams(catalog, None)}
    design = {"joints": []}
    for name, _, _, _, _, beam_id, span in PICKS:
        info = beam_infos.get(beam_id)
        if info is None or abs(info["span_mm"] - span) > 1e-9:
            raise ValueError(f"beam {beam_id} span mismatch: "
                             f"want {span}, catalog has {info}")
        design["joints"].append({"module": name, "beam": beam_id,
                                 "reason": "pinned arm_4dof design"})
    (OUT / "design.json").write_text(
        json.dumps({"design": design, "rounds": 0,
                    "notes": "pinned: greedy cannot tell coaxial yaw/roll "
                             "modules apart"}, indent=2) + "\n",
        encoding="utf-8")
    spec = load_spec(ROOT / "specs" / "arm_4dof.json")
    report = asm.assemble_design(design, spec, modules_root, catalog,
                                 inventory)
    print(f"assemble: ok={report['ok']} "
          f"link_errors={report['link_errors_mm']} "
          f"overlap={report['overlap']['ok']} "
          f"bom={report['bom_check']['ok']} score={report['score']}")
    gate = report.get("ev3_gate") or {}
    subs = " ".join(f"{k}={gate[k]['pass']}"
                    for k in ("ports", "stock", "torque", "cable",
                              "driver") if k in gate)
    print(f"ev3_gate: pass={gate.get('pass')} {subs}")
    if not report["ok"]:
        raise SystemExit("assembly infeasible, see errors above")
    asm.write_outputs(report, OUT, "arm_4dof")
    docs.main(["--robot-dir", str(OUT), "--modules", str(modules_root)])
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
