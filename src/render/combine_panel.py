"""Blender combine panel for template robots (Stage 5D).

Pure flow (no Blender needed) plus a thin Blender addon UI over it:

* :func:`list_templates` shows template-built modules only (``module.json``
  carries a ``"template"`` marker); extracted/unmarked modules are excluded.
* :func:`combine` assembles one combination and :func:`panel_status`
  projects reach, stock and the EV3 gate (ports/torque/cable/driver)
  for display.
* :func:`run_flow` runs the existing output flow unchanged: oracle
  roundtrip -> local convert (.blend) -> assembly docs.

Two entries, one flow. Interactive Blender registers the panel below
(full flow works there: the heavy guard only blocks headless ``-b``
processes, so an interactive session converts fine). Headless runs go
through ``scripts/panel_combine.py`` (plain Python; Blender is reached
only as the convert subprocess, because a headless outer Blender would
trip the guard for the inner convert).
"""

from __future__ import annotations

import json
from pathlib import Path

try:
    import bpy  # type: ignore[import]
except ImportError:  # plain Python (tests, headless driver): UI disabled
    bpy = None


# ------------------------------------------------------- pure flow


def list_templates(modules_dir: str | Path) -> dict:
    """Template modules in a library dir; extracted ones excluded."""
    root = Path(modules_dir)
    found, skipped = [], []
    if not root.is_dir():
        return {"templates": found, "skipped": skipped,
                "reason": f"{root} is not a directory"}
    for folder in sorted(root.iterdir()):
        if not folder.is_dir():
            continue
        doc_path = folder / "module.json"
        try:
            doc = json.loads(doc_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            skipped.append(folder.name)
            continue
        if not doc.get("template"):
            skipped.append(folder.name)
            continue
        if not (folder / str(doc.get("geometry", ""))).is_file():
            skipped.append(folder.name)
            continue
        found.append({"name": doc.get("name", folder.name),
                      "template": doc["template"],
                      "motor": (doc.get("motor") or {}).get("kind"),
                      "ratio": (doc.get("joint") or {}).get(
                          "joint_deg_per_motor_deg"),
                      "range_deg": (doc.get("joint") or {}).get(
                          "range_deg")})
    return {"templates": found, "skipped": skipped}


def combine(modules_dir: str | Path, picks: list[dict], spec: dict,
            out_dir: str | Path, name: str, catalog: dict,
            inventory: dict | None = None) -> dict:
    """Assemble one combination; returns the assembly report."""
    from src.gen import assemble as asm

    modules_dir, out_dir = Path(modules_dir), Path(out_dir)
    design = {"joints": [{"module": p["module"], "beam": p["beam"],
                          "reason": p.get("reason", "panel")}
                         for p in picks]}
    report = asm.assemble_design(design, spec, modules_dir, catalog,
                                 inventory)
    asm.write_outputs(report, out_dir, name)
    (out_dir / "design.json").write_text(
        json.dumps({"design": design, "rounds": 0,
                    "notes": "panel combination"}, indent=2) + "\n",
        encoding="utf-8")
    return report


def panel_status(report: dict) -> dict:
    """Reach, stock and EV3 gate projection for panel display."""
    gate = report.get("ev3_gate") or {}
    bom = report.get("bom_check") or {}
    torque = gate.get("torque") or {}
    cable = gate.get("cable") or {}
    return {
        "assemble_ok": bool(report.get("ok")),
        "reach_spec_mm": report.get("reach_spec_mm"),
        "reach_actual_mm": report.get("reach_actual_mm"),
        "link_errors_mm": report.get("link_errors_mm"),
        "stock_ok": bom.get("ok"),
        "shortfalls": bom.get("shortfalls", []),
        "ev3_pass": gate.get("pass"),
        "ports": (gate.get("ports") or {}).get("ports"),
        "torque": [(j.get("joint"), j.get("required_Ncm"),
                    j.get("available_Ncm"), j.get("margin"),
                    j.get("pass")) for j in torque.get("joints", [])],
        "cable": [(m.get("uid"), m.get("required_mm"),
                   m.get("cable_mm"), m.get("pass"))
                  for m in cable.get("motors", [])],
        "driver_pass": (gate.get("driver") or {}).get("pass"),
    }


def format_status(status: dict) -> str:
    """Multiline panel/console rendering of :func:`panel_status`."""
    lines = [
        f"assemble ok={status['assemble_ok']}",
        f"reach actual={status['reach_actual_mm']} "
        f"spec={status['reach_spec_mm']} "
        f"errors={status['link_errors_mm']}",
        f"stock ok={status['stock_ok']} "
        f"shortfalls={len(status['shortfalls'])}",
        f"ev3 pass={status['ev3_pass']} ports={status['ports']} "
        f"driver={status['driver_pass']}",
    ]
    for joint, req, avail, margin, ok in status["torque"]:
        lines.append(f"  torque {joint}: need={req} have={avail} "
                     f"margin={margin} pass={ok}")
    for uid, req, total, ok in status["cable"]:
        lines.append(f"  cable {uid}: need={req}/{total} pass={ok}")
    return "\n".join(lines)


def run_flow(modules_dir: str | Path, picks: list[dict], spec: dict,
             out_dir: str | Path, name: str, catalog: dict,
             inventory: dict | None = None, render: bool = True,
             blender_bin: str | None = None,
             studio_ldraw: str | None = None) -> dict:
    """Full panel flow; returns paths + verdicts (raises loudly)."""
    from src.gen import assembly_docs as docs
    from src.gen import roundtrip as rt
    from src.render import modal_render as mr

    out_dir = Path(out_dir)
    report = combine(modules_dir, picks, spec, out_dir, name, catalog,
                     inventory)
    status = panel_status(report)
    print(format_status(status))
    if not report.get("ok"):
        raise RuntimeError(f"assembly infeasible: "
                           f"errors={report.get('link_errors_mm')} "
                           f"overlap={report.get('overlap')} "
                           f"bom={report.get('bom_check')}")
    gate = report.get("ev3_gate") or {}
    if gate.get("pass") is False:
        print("WARNING: ev3 gate failed (advisory, continuing); "
              "see report.json")
    trip = rt.run_roundtrip(
        out_dir, infer_fn=lambda _p, _c: rt.oracle_doc(out_dir),
        mode="oracle")
    rt.write_roundtrip(trip, out_dir)
    print(f"roundtrip oracle: overall={trip['overall_pass']} " +
          " ".join(f"{k}={v['pass']}" for k, v in trip["gates"].items()))
    if not trip["overall_pass"]:
        raise RuntimeError("roundtrip gates failed; see roundtrip.json")
    blend = None
    if render:
        mr.local_convert(out_dir, blender_bin or mr.BLENDER_MAC_BIN,
                         studio_ldraw or mr.STUDIO_LDRAW)
        blend = out_dir / "model.blend"
    summary = docs.build_materials(out_dir, Path(modules_dir), inventory)
    return {"report": report, "status": status, "roundtrip": trip,
            "blend": str(blend) if blend else None,
            "docs": summary,
            "paths": {"robot.ldr": str(out_dir / "robot.ldr"),
                      "roundtrip.json": str(out_dir / "roundtrip.json"),
                      "model.blend": str(out_dir / "model.blend"),
                      "bom.json": str(out_dir / "bom.json"),
                      "wiring.yaml": str(out_dir / "wiring.yaml"),
                      "robot_steps.ldr": str(
                          out_dir / "robot_steps.ldr")}}


# ------------------------------------------------------- Blender UI


if bpy is not None:
    bl_info = {
        "name": "MetaHack Combine",
        "blender": (4, 0, 0),
        "category": "3D View",
        "description": "Combine template joint modules (5D)",
    }

    class METAHACK_OT_combine(bpy.types.Operator):
        """Combine the panel design and run the output flow."""

        bl_idname = "metahack.combine"
        bl_label = "Combine (templates)"

        def execute(self, context):
            from src.gen.inventory import load_catalog_merged
            from src.gen.spec import load_spec

            scene = context.scene
            try:
                import yaml

                picks = []
                for entry in scene.metahack_design.split(","):
                    module, _, beam = entry.partition(":")
                    module, beam = module.strip(), beam.strip()
                    if not module or not beam:
                        raise ValueError(
                            f"bad design entry {entry!r} "
                            "(want module:beam,...)")
                    picks.append({"module": module, "beam": beam})
                catalog = load_catalog_merged()
                inv_path = Path("config/inventory.local.yaml")
                inventory = yaml.safe_load(inv_path.read_text(
                    encoding="utf-8")) if inv_path.exists() else None
                spec = load_spec(scene.metahack_spec or
                                 "specs/arm_4dof.json")
                out = run_flow(scene.metahack_modules, picks, spec,
                               scene.metahack_out or "generated_robots/"
                               "panel_bot", scene.metahack_name or
                               "panel_bot", catalog, inventory,
                               render=bool(scene.metahack_render))
            except Exception as exc:  # noqa: BLE001 - panel must not die
                scene.metahack_status = f"FAILED: {exc}"[:2000]
                self.report({"ERROR"}, str(exc)[:300])
                return {"CANCELLED"}
            scene.metahack_status = format_status(out["status"])
            self.report({"INFO"}, "combine done")
            return {"FINISHED"}

    class METAHACK_PT_combine(bpy.types.Panel):
        """Template list + combine + gate status."""

        bl_label = "MetaHack Combine"
        bl_idname = "METAHACK_PT_combine"
        bl_space_type = "VIEW_3D"
        bl_region_type = "UI"
        bl_category = "MetaHack"

        def draw(self, context):
            scene = context.scene
            layout = self.layout
            layout.prop(scene, "metahack_modules")
            layout.prop(scene, "metahack_spec")
            layout.prop(scene, "metahack_design")
            layout.prop(scene, "metahack_out")
            layout.prop(scene, "metahack_render")
            box = layout.box()
            box.label(text="Templates (extracted excluded):")
            try:
                found = list_templates(
                    scene.metahack_modules or ".")["templates"]
            except Exception as exc:  # noqa: BLE001 - listing is display
                box.label(text=f"listing failed: {exc}"[:80])
                found = []
            for entry in found:
                box.label(
                    text=f"{entry['name']} [{entry['motor']} "
                    f"x{entry['ratio']}]"[:60])
            if not found:
                box.label(text="(none)")
            layout.operator("metahack.combine")
            for line in (scene.metahack_status or "(not run)").split(
                    "\n"):
                layout.label(text=line[:120])

    CLASSES = (METAHACK_OT_combine, METAHACK_PT_combine)

    def register():
        for cls in CLASSES:
            bpy.utils.register_class(cls)
        scene = bpy.types.Scene
        scene.metahack_modules = bpy.props.StringProperty(
            name="Modules", default="generated_robots/arm_4dof/modules",
            subtype="DIR_PATH")
        scene.metahack_spec = bpy.props.StringProperty(
            name="Spec", default="specs/arm_4dof.json", subtype="FILE_PATH")
        scene.metahack_design = bpy.props.StringProperty(
            name="Design (module:beam,...)",
            default="base_yaw:32316.dat,pitch:32316.dat,"
                    "pitch:6632.dat,wrist_roll:6632.dat")
        scene.metahack_out = bpy.props.StringProperty(
            name="Out", default="generated_robots/panel_bot",
            subtype="DIR_PATH")
        scene.metahack_name = bpy.props.StringProperty(
            name="Name", default="panel_bot")
        scene.metahack_render = bpy.props.BoolProperty(
            name="Render .blend", default=True)
        scene.metahack_status = bpy.props.StringProperty(
            name="Status", default="(not run)")

    def unregister():
        scene = bpy.types.Scene
        for prop in ("metahack_modules", "metahack_spec",
                     "metahack_design", "metahack_out", "metahack_name",
                     "metahack_render", "metahack_status"):
            try:
                delattr(scene, prop)
            except AttributeError:
                pass
        for cls in reversed(CLASSES):
            bpy.utils.unregister_class(cls)

else:
    CLASSES = ()

    def register():
        """No-op without Blender (plain Python imports stay usable)."""

    def unregister():
        """No-op without Blender."""
