"""Stage 5D.7: assembly materials for generated robots.

Builds, in ``generated_robots/<name>/``:

- ``bom.json``: part needs vs owned stock, per-module breakdown, the
  port-to-module assembly map and per-joint homing methods,
- ``robot_steps.ldr``: the model with LDraw STEP breaks per segment
  (parses to identical parts),
- ``wiring.yaml``: motor/sensor uid to brick/port assignment in the
  ``config/ports.local.yaml`` hardware-layer shape (no semantic keys,
  so ``src.tools.wiring.check`` passes; the assembly map lives in
  ``bom.json`` instead).

Single EV3 brick: at most 4 motors and 4 sensors; ports follow the
chain order (motors A..D, sensors 1..4) and must match the model's
provisional actuator assignment.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from src.gen.inventory import load_catalog_merged, sensor_kind, short_label
from src.gen.modules import load_module
from src.parse import parse_file

MOTOR_PORTS = ["A", "B", "C", "D"]
SENSOR_PORTS = ["1", "2", "3", "4"]
BRICK = "left"
DEFAULT_BRICKS = Path("config/bricks.yaml")


def load_bricks(source=None) -> tuple[int, list[str]]:
    """Brick topology (count, names); missing config = one "left" brick."""
    if source is None:
        source = DEFAULT_BRICKS
    if isinstance(source, dict):
        data = source
    else:
        path = Path(source)
        if not path.is_file():
            return 1, [BRICK]
        with open(path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    count = int(data.get("count", 1))
    names = [str(n) for n in data.get("names", [BRICK])]
    if count < 1 or len(names) < count:
        raise ValueError(f"bad brick topology: {data}")
    return count, names[:count]


def _load_json(path: Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def collect_drives(parts: list[dict], catalog: dict,
                   segments: dict) -> tuple[list[dict], list[dict]]:
    """Motor/sensor parts in chain (segment, uid) order."""
    seg_of = {uid: int(seg) for uid, seg in segments.items()}
    motors, sensors = [], []
    for part in parts:
        entry = catalog.get(part["part_id"]) or {}
        category = entry.get("category")
        if category == "motor":
            motors.append(part)
        elif category == "sensor":
            sensors.append(part)
    order = lambda p: (seg_of.get(p["uid"], 999), p["uid"])
    return sorted(motors, key=order), sorted(sensors, key=order)


def build_bom(bom: dict, inventory: dict | None, catalog: dict) -> dict:
    rows, shortfalls = {}, []
    for part_id in sorted(bom):
        need = bom[part_id]
        own = None
        if inventory is not None:
            own = (inventory.get("parts") or {}).get(part_id, {}).get(
                "total", 0)
        ok = True if own is None else need <= own
        if not ok:
            shortfalls.append({"part_id": part_id, "need": need,
                               "own": own})
        rows[part_id] = {
            "label": short_label(part_id, catalog.get(part_id)),
            "need": need, "own": own, "ok": ok,
        }
    within = None if inventory is None else not shortfalls
    return {"parts": rows, "shortfalls": shortfalls,
            "within_stock": within,
            "total_kinds": len(rows),
            "total_parts": sum(bom.values())}


def build_steps_ldraw(robot_dir: Path, segments: dict,
                      design: dict) -> Path:
    """Write robot_steps.ldr grouped by segment (STEP breaks)."""
    lines = (robot_dir / "robot.ldr").read_text(
        encoding="utf-8").splitlines()
    header = [ln for ln in lines if not ln.strip().startswith("1 ")]
    body = [ln for ln in lines if ln.strip().startswith("1 ")]
    parts = parse_file(str(robot_dir / "robot.ldr"))
    if len(body) != len(parts):
        raise ValueError("robot.ldr changed shape; cannot map uids")
    seg_of = {uid: int(seg) for uid, seg in segments.items()}
    names = [pick["module"] for pick in design["joints"]]
    by_seg: dict[int, list[str]] = {}
    for line, part in zip(body, parts):
        by_seg.setdefault(seg_of.get(part["uid"], -1), []).append(line)
    out = list(header)
    for seg in sorted(by_seg):
        label = names[seg] if 0 <= seg < len(names) else f"extra-{seg}"
        out.append(f"0 // STEP {seg + 1}: {label} + beam")
        out.append("0 STEP")
        out.extend(by_seg[seg])
    path = robot_dir / "robot_steps.ldr"
    path.write_text("\n".join(out) + "\n", encoding="utf-8")
    return path


def build_wiring(motors: list[dict], sensors: list[dict],
                 catalog: dict, exp_doc: dict,
                 bricks=None) -> tuple[dict, dict]:
    """Hardware-layer wiring + assembly map (chain-order ports).

    Brick topology comes from ``config/bricks.yaml`` (or the ``bricks``
    override): one brick fills A-D / 1-4 on a single brick; more bricks
    spill over in fill order with ``brick/port`` assembly keys.
    """
    count, names = load_bricks(bricks)
    capacity = len(MOTOR_PORTS) * count
    if len(motors) > capacity:
        if count == 1:
            raise ValueError(
                f"{len(motors)} motors need a second brick")
        raise ValueError(f"{len(motors)} motors need more than "
                         f"{count} bricks")
    if len(sensors) > len(SENSOR_PORTS) * count:
        if count == 1:
            raise ValueError(
                f"{len(sensors)} sensors need a second brick")
        raise ValueError(f"{len(sensors)} sensors need more than "
                         f"{count} bricks")
    expected = [(a.get("motor"), a.get("port"))
                for a in exp_doc.get("actuators", [])]
    if len(expected) != len(motors):
        raise ValueError(f"model has {len(expected)} actuators for "
                         f"{len(motors)} motors")
    wiring: dict = {"motors": [], "sensors": []}
    assembly: dict = {}
    for i, part in enumerate(motors):
        brick, port = names[i // len(MOTOR_PORTS)], \
            MOTOR_PORTS[i % len(MOTOR_PORTS)]
        kind = (catalog.get(part["part_id"]) or {}).get("motor")
        if (kind, port) != expected[i]:
            raise ValueError(
                f"motor {part['uid']}: chain order {(kind, port)} "
                f"misses model {expected[i]}")
        wiring["motors"].append({"uid": part["uid"], "brick": brick,
                                 "port": port})
        key = port if count == 1 else f"{brick}/{port}"
        assembly[key] = {"uid": part["uid"], "kind": kind,
                         "joint": f"J{i + 1}"}
    for i, part in enumerate(sensors):
        brick, port = names[i // len(SENSOR_PORTS)], \
            SENSOR_PORTS[i % len(SENSOR_PORTS)]
        kind = sensor_kind(short_label(
            part["part_id"], catalog.get(part["part_id"])))
        wiring["sensors"].append({"uid": part["uid"], "brick": brick,
                                  "port": port})
        key = port if count == 1 else f"{brick}/{port}"
        assembly[key] = {"uid": part["uid"], "kind": kind}
    return wiring, assembly


def homing_plan(design: dict, modules_dir: Path, assembly: dict) -> dict:
    """Per-joint homing method (sensor where docked, else stall)."""
    sensor_ports = {info["uid"]: port for port, info in assembly.items()
                    if port in SENSOR_PORTS}
    plan = {}
    for i, pick in enumerate(design["joints"], 1):
        module = load_module(modules_dir / pick["module"] / "module.json")
        dock = module.get("sensor_mount") or {}
        plan[f"J{i}"] = {"method": "stall", "sensor_port": None,
                         "dock": dock.get("kind")}
        if dock.get("kind"):
            plan[f"J{i}"]["method"] = "sensor"
    void = [jid for jid, row in plan.items()
            if row["method"] == "sensor"]
    if void and not sensor_ports:
        print("warning: sensor docks declared but no sensor parts found; "
              "those joints fall back to stall")
        for jid in void:
            plan[jid]["method"] = "stall"
    return plan


def build_materials(robot_dir: str | Path, modules_dir: str | Path,
                    inventory: dict | None = None,
                    bricks=None) -> dict:
    """Write bom.json, robot_steps.ldr and wiring.yaml; returns summary."""
    from src.kin.model import chain_order

    robot_dir, modules_dir = Path(robot_dir), Path(modules_dir)
    design = _load_json(robot_dir / "design.json")["design"]
    report = _load_json(robot_dir / "report.json")
    exp_doc = _load_json(robot_dir / "expected_kinematics.json")
    _ = chain_order(exp_doc)
    catalog = load_catalog_merged()
    parts = parse_file(str(robot_dir / "robot.ldr"))

    motors, sensors = collect_drives(parts, catalog, report["segments"])
    wiring, assembly = build_wiring(motors, sensors, catalog, exp_doc,
                                    bricks=bricks)
    bom = build_bom(report["bom"], inventory, catalog)
    by_module = {}
    for pick in design["joints"]:
        module = load_module(modules_dir / pick["module"] / "module.json")
        by_module[pick["module"]] = dict(module["bom"])
    bom_doc = {"robot": robot_dir.name, "bom": bom,
               "modules": by_module,
               "wiring_map": assembly,
               "homing": homing_plan(design, modules_dir, assembly)}
    (robot_dir / "bom.json").write_text(json.dumps(bom_doc, indent=1),
                                        encoding="utf-8")
    steps = build_steps_ldraw(robot_dir, report["segments"], design)
    (robot_dir / "wiring.yaml").write_text(
        yaml.safe_dump(wiring, sort_keys=False), encoding="utf-8")
    parsed_back = parse_file(str(steps))
    if len(parsed_back) != len(parts):
        raise ValueError("steps file lost parts")
    return {"bom_ok": bom["within_stock"], "motors": len(motors),
            "sensors": len(sensors), "steps": str(steps)}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="5D assembly materials.")
    parser.add_argument("--robot-dir", required=True)
    parser.add_argument("--modules", default="assets/modules")
    parser.add_argument("--inventory", default="config/inventory.local.yaml")
    parser.add_argument("--bricks", default="config/bricks.yaml")
    args = parser.parse_args(argv)

    inventory = None
    inventory_path = Path(args.inventory)
    if inventory_path.exists():
        with open(args.inventory, encoding="utf-8") as fh:
            inventory = yaml.safe_load(fh) or {}
    else:
        print(f"note: {args.inventory} missing, stock unknown")
    try:
        summary = build_materials(args.robot_dir, args.modules, inventory,
                                  bricks=args.bricks)
    except (ValueError, OSError) as exc:
        print(f"assembly materials failed: {exc}")
        return 1
    print(f"wrote bom.json, robot_steps.ldr, wiring.yaml: "
          f"bom_ok={summary['bom_ok']} motors={summary['motors']} "
          f"sensors={summary['sensors']}")
    return 0 if summary["bom_ok"] is not False else 2


if __name__ == "__main__":
    raise SystemExit(main())
