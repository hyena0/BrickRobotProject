"""Stage 5D EV3 gate: single-brick feasibility after assembly.

Runs after :func:`src.gen.assemble.assemble_design` and leaves a
pass/fail verdict in the report (advisory: it never flips the
assembly ``ok``). Five sub-gates:

* ports: at most 4 motors on ports A-D, no duplicates.
* stock: motors needed per kind (large/medium) within owned stock.
* torque: worst-case static joint torque (mid-link mass model, fully
  extended horizontal pose) against motor rating / ratio x efficiency.
* cable: brick-to-motor path within cable length over the full joint
  range (triangle-inequality bound, so valid for every pose).
* driver: one motor per joint, no coupling (the 5D driver envelope);
  outside the envelope fails loudly instead of guessing.

Assumed values (motor ratings, efficiency, part masses, cable, brick
pose) come from ``config/ev3_gate.yaml`` and are echoed with sources
into the report. Missing inputs (no config, no inventory, no motor
parts) leave the affected sub-gate passing with a note, mirroring
``check_bom``; broken designs fail.
"""

from __future__ import annotations

import math
from pathlib import Path

DEFAULT_CONFIG = Path("config/ev3_gate.yaml")
PORTS = ("A", "B", "C", "D")
G = 9.81


def load_config(source=None) -> dict:
    """Load gate assumptions (dict passes through, None loads default)."""
    if source is None:
        source = DEFAULT_CONFIG
    if isinstance(source, dict):
        return source
    path = Path(source)
    if not path.is_file():
        return {}
    import yaml

    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _links_of(report: dict, parts: list[dict], catalog: dict) -> dict:
    from src.render.modal_render import split_links

    segments = {}
    beams = []
    for part in parts:
        if "_segment" not in part:
            raise ValueError("parts lack _segment markers (call before "
                             "write_outputs)")
        segments[part["uid"]] = int(part["_segment"])
        if part.get("_beam"):
            beams.append({"uid": part["uid"],
                          "segment": int(part["_segment"])})
    joints = report["expected_kinematics"]["joints"]
    split = split_links(parts, catalog, segments, beams, joints=joints)
    return split["links"]


def check_ports(report: dict) -> dict:
    """Motor count <= 4 on distinct ports A-D."""
    actuators = report["expected_kinematics"]["actuators"]
    ports = [a["port"] for a in actuators]
    problems = []
    if len(actuators) > 4:
        problems.append(f"{len(actuators)} motors need a second brick")
    if len(set(ports)) != len(ports):
        problems.append(f"duplicate ports: {sorted(ports)}")
    if any(p not in PORTS for p in ports):
        problems.append(f"ports outside A-D: {sorted(set(ports))}")
    return {"pass": not problems, "n_motors": len(actuators),
            "ports": ports, "problems": problems}


def check_stock(report: dict, parts: list[dict], catalog: dict,
                inventory: dict | None) -> dict:
    """Motors needed per kind within owned stock."""
    if inventory is None:
        return {"pass": True, "note": "no inventory given (unchecked)"}
    need: dict[str, int] = {}
    for part in parts:
        kind = (catalog.get(part["part_id"]) or {}).get("motor")
        if kind:
            need[kind] = need.get(kind, 0) + 1
    stocked: dict[str, int] = {}
    for pid, row in (inventory.get("parts") or {}).items():
        kind = (catalog.get(pid) or {}).get("motor")
        if kind:
            stocked[kind] = stocked.get(kind, 0) + int(row.get("total", 0))
    shortfalls = [{"kind": kind, "need": count,
                   "own": stocked.get(kind, 0)}
                  for kind, count in sorted(need.items())
                  if count > stocked.get(kind, 0)]
    return {"pass": not shortfalls, "need": need, "own": stocked,
            "shortfalls": shortfalls}


def _part_mass_g(part_id: str, catalog: dict, masses: dict) -> float:
    by_part = masses.get("by_part") or {}
    if part_id in by_part:
        return float(by_part[part_id])
    category = (catalog.get(part_id) or {}).get("category")
    return float((masses.get("by_category") or {}).get(
        category, masses.get("default", 0.0)))


def _tilts(joints: list[dict]) -> list[int]:
    """0 when a joint axis is fixed-vertical, else 1 (conservative)."""
    vertical = []
    for joint in joints:
        axis = [float(v) for v in joint["axis"]]
        norm = math.sqrt(sum(v * v for v in axis)) or 1.0
        vertical.append(abs(axis[2] / norm) >= 0.99)
    tilts = []
    for i in range(len(joints)):
        tilts.append(0 if all(vertical[:i + 1]) else 1)
    return tilts


def check_torque(report: dict, parts: list[dict], catalog: dict,
                 links: dict[str, list[str]], config: dict) -> dict:
    """Worst-case static torque per joint against drive capacity.

    Mid-link mass model: each downstream link weighs at its link
    midpoint with the arm fully extended horizontal. Joints whose axis
    is fixed-vertical carry no gravity torque. Proximal-link radial
    wobble (off-axis COM orbiting the joint) is neglected as
    second-order for centred designs; margins must cover it.
    """
    exp = report["expected_kinematics"]
    joints = exp["joints"]
    actuators = exp["actuators"]
    lengths = [float(v) for v in report["link_lengths"]]
    ee = [float(v) for v in exp["end_effector"]["point_mm"]]
    last = [float(v) for v in joints[-1]["point_mm"]]
    spans = lengths + [math.dist(ee, last)]
    masses = config.get("masses_g") or {}
    gravity = float(config.get("gravity_m_s2", G))
    efficiency = float((config.get("efficiency") or {}).get("value", 1.0))
    rated = {kind: float(row["rated_Ncm"])
             for kind, row in (config.get("motors") or {}).items()}
    members = {lid: set(uids) for lid, uids in links.items()}
    link_mass = {}
    for i in range(len(joints) + 1):
        uids = members.get(f"L{i}", set())
        link_mass[i] = sum(_part_mass_g(
            next(p["part_id"] for p in parts if p["uid"] == uid),
            catalog, masses) for uid in uids) / 1000.0
    rows = []
    tilts = _tilts(joints)
    for i, joint in enumerate(joints):
        dist_mass = 0.0
        for k in range(i + 1, len(joints) + 1):
            dist = sum(spans[i + 1:k]) + spans[k] / 2.0
            dist_mass += link_mass[k] * dist / 1000.0
        tilt = tilts[i]
        required = tilt * gravity * dist_mass * 100.0
        kind = actuators[i]["motor"]
        ratio = exp["transmission"]["C"][i][i]
        if ratio is None or ratio <= 0:
            rows.append({"joint": joint["id"], "pass": False,
                         "reason": f"non-positive ratio {ratio}"})
            continue
        available = rated.get(kind, 0.0) / ratio * efficiency
        margin = available / required if required > 0 else None
        rows.append({"joint": joint["id"], "motor": kind,
                     "required_Ncm": round(required, 3),
                     "available_Ncm": round(available, 3),
                     "margin": round(margin, 3) if margin else None,
                     "unloaded": required == 0,
                     "pass": bool(available >= required)})
    return {"pass": all(r["pass"] for r in rows), "joints": rows,
            "link_masses_kg": {f"L{i}": round(m, 4)
                               for i, m in link_mass.items()},
            "gravity_m_s2": gravity, "efficiency": efficiency,
            "rated_Ncm": rated}


def check_cable(report: dict, motors: list[dict],
                links: dict[str, list[str]], config: dict) -> dict:
    """Brick-to-motor reach within cable length for every pose.

    Triangle-inequality bound: |brick -> motor| <= |brick -> base| +
    chain reach to the motor link + rigid offset inside the link.
    Pose-independent, hence valid over the full joint range.
    """
    exp = report["expected_kinematics"]
    joints = exp["joints"]
    lengths = [float(v) for v in report["link_lengths"]]
    brick = [float(v) for v in config.get("brick_pos_mm", [0, 0, 0])]
    cable = config.get("cable") or {}
    total = float(cable.get("length_mm", 0.0))
    slack = float(cable.get("slack_mm", 0.0))
    link_of = {}
    for lid, uids in links.items():
        for uid in uids:
            link_of[uid] = int(lid[1:])
    rows = []
    base_dist = math.dist(brick, [0.0, 0.0, 0.0])
    for part in motors:
        uid = part["uid"]
        link = link_of.get(uid)
        if link is None:
            continue
        pos = [float(v) for v in part["position_mm"]]
        if link == 0:
            required = math.dist(brick, pos) + slack
        else:
            reach = sum(lengths[:link])
            anchor = [float(v) for v in joints[link - 1]["point_mm"]]
            required = base_dist + reach + math.dist(pos, anchor) + slack
        rows.append({"uid": uid, "link": f"L{link}",
                     "required_mm": round(required, 1),
                     "cable_mm": total,
                     "pass": bool(required <= total)})
    rows.sort(key=lambda r: r["uid"])
    return {"pass": all(r["pass"] for r in rows), "motors": rows,
            "brick_pos_mm": brick, "slack_mm": slack}


def check_driver(report: dict, parts: list[dict], catalog: dict,
                 links: dict[str, list[str]]) -> dict:
    """One motor per joint, no coupling (5D driver envelope)."""
    from src.gen.roundtrip import infer_transmission

    exp = report["expected_kinematics"]
    joints = exp["joints"]
    n = len(joints)
    motors = sorted(p["uid"] for p in parts
                    if (catalog.get(p["part_id"]) or {}).get("motor"))
    if not motors:
        return {"pass": True,
                "note": "no motor parts modeled (unchecked)"}
    doc = {
        "links": [{"id": lid, "members": sorted(uids)}
                  for lid, uids in sorted(links.items())],
        "joints": [{**j, "confidence": 1.0,
                    "reason": "ev3 gate assembly truth"} for j in joints],
        "base": {"link": "L0"}, "end_effector": {"link": f"L{n}"},
    }
    try:
        trains = infer_transmission(parts, catalog, doc)
    except ValueError as exc:
        return {"pass": False, "reason": f"driver: {exc}"[:300]}
    matrix = trains["C"]
    problems = check_coupling(matrix, n, len(motors))
    return {"pass": not problems, "problems": problems,
            "C": [[round(v, 5) for v in row] for row in matrix],
            "motors": trains["motor_ids"]}


def check_coupling(matrix: list[list[float]], n_joints: int,
                   n_motors: int) -> list[str]:
    """Coupling problems: C must be a bijective (permuted) diagonal."""
    problems = []
    ncol = len(matrix[0]) if matrix else 0
    if len(matrix) != n_joints or ncol != n_motors:
        problems.append(f"C shape {len(matrix)}x{ncol}, "
                        f"want {n_joints}x{n_motors}")
        return problems
    if n_motors != n_joints:
        problems.append(f"{n_motors} motors for {n_joints} joints")
        return problems
    rows_ok = [sum(1 for v in row if abs(v) > 1e-9) == 1
               for row in matrix]
    cols_ok = [sum(1 for row in matrix if abs(row[j]) > 1e-9) == 1
               for j in range(n_joints)]
    if not all(rows_ok):
        problems.append("a joint has zero or several motors")
    if not all(cols_ok):
        problems.append("a motor drives zero or several joints "
                        "(coupling)")
    return problems


def check_ev3(report: dict, catalog: dict,
              inventory: dict | None = None,
              config=None) -> dict:
    """Run the five EV3 sub-gates; returns a JSON-serializable verdict."""
    cfg = load_config(config)
    parts = report["parts"]
    links: dict[str, list[str]] | None = None
    link_error: str | None = None
    try:
        links = _links_of(report, parts, catalog)
    except ValueError as exc:
        link_error = str(exc)
    ports = check_ports(report)
    stock = check_stock(report, parts, catalog, inventory)
    if links is None:
        torque = {"pass": False, "reason": f"links: {link_error}"}
        cable = {"pass": False, "reason": f"links: {link_error}"}
        driver = {"pass": False, "reason": f"links: {link_error}"}
    else:
        if cfg.get("masses_g") and cfg.get("motors"):
            torque = check_torque(report, parts, catalog, links, cfg)
        else:
            torque = {"pass": True,
                      "note": "no gate config (unchecked)"}
        motors_here = [p for p in parts
                       if (catalog.get(p["part_id"]) or {}).get("motor")]
        if cfg.get("cable"):
            cable = check_cable(report, motors_here, links, cfg)
        else:
            cable = {"pass": True, "note": "no gate config (unchecked)"}
        driver = check_driver(report, parts, catalog, links)
    verdict = {"pass": all(g["pass"]
                           for g in (ports, stock, torque, cable, driver)),
               "ports": ports, "stock": stock, "torque": torque,
               "cable": cable, "driver": driver,
               "config": {key: cfg.get(key) for key in
                          ("motors", "efficiency", "gravity_m_s2",
                           "gravity_source", "masses_g", "cable",
                           "brick_pos_mm", "brick_note")}}
    return verdict
