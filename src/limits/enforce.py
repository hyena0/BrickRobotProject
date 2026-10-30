"""Limit model, pose checking and path safety (Stage 5A.4).

The merged ``runs/stage5a/limits.json`` carries per-joint operating
ranges plus coupled constraints. IK, the generated code and the
robot package all read this one file; the mesh oracle stays in the
PC authoring tools.
"""

from __future__ import annotations

import argparse
import json
import math

OPERATING_MARGIN_DEG = 5.0
# Stage 5A.3 conservative finish: manual range narrowing from the
# 2000-pose validation folds (oracle-confirmed, non-bearing):
# J5 lo -130.62 -> -110 (D4-D5/D3-D6 folds at J5 <= -117.7),
# J2 hi 46 -> 40 (D1-D2 fold at J2 = 44.97, 6.0 mm deep).
# Cut points are exact single-joint-sweep grid points verified clean.
MANUAL_NARROW_DEG = {"J5": {"lo": -90.0,
                            "why": ("D5-D4 folds at J5 -96..-99 with "
                                    "J1/J4 off-home; home+same-J5 is "
                                    "clean (modeling question for 5C)")},
                     "J2": {"hi": 40.0, "why": "D1-D2 fold"},
                     "J1": {"lo": -50.0,
                            "why": "5 residual folds all at J1 <= -58"},
                     "J4": {"hi": 125.0, "lo": -115.0,
                            "why": ("D5-D6 folds at J4 = 134.24 and "
                                    "-124.16 (Blender-confirmed)")},
                     "J6": {"hi": 135.0,
                            "why": ("D4-D3 fold at J6 = 161.6, D5-D4 "
                                    "fold at J6 = 140.6")}}
# Stage 5A.3 conservative finish: the grid lookup fallback stays in
# the code but is switched OFF (lookup tables are neither fitted for
# enforcement nor consulted). Runtime safety uses only limits_deg
# and coupled_limits; the authoring record in coupled.json keeps the
# observed fold cells for Stage 5C review.
USE_COUPLED_LOOKUP = False
# Lookup rejection square half-side: colliding ray cells are exact
# first-hit points (~1 deg); the true boundary sits within ~1 deg of
# a cell along its ray, so a 2.5 deg square is conservative without
# eating measurable free space (a handful of cells per pair).
LOOKUP_RADIUS_DEG = 2.5


def lookup_rejects(qa: float, qb: float, cells: list,
                   radius: float = LOOKUP_RADIUS_DEG) -> bool:
    """True when (qa, qb) lands on a known-colliding grid cell."""
    return any(abs(qa - ca) <= radius + 1e-9
               and abs(qb - cb) <= radius + 1e-9 for ca, cb in cells)


def check_lookup(q: list[float], names: list[str], tables: dict) -> bool:
    """True when no lookup-table pair rejects the pose.

    Tables map ``"JA_JB"`` to ``{"cells": [[qa, qb], ...],
    "radius_deg": R}``; a pose is rejected when it lands within the
    square of a colliding cell. The check ignores the other joints
    (superset: a cell collided in some wrist context), so passing is
    conservative. Metadata keys starting with ``"_"`` are skipped.
    """
    pos = {jid: v for jid, v in zip(names, q)}
    for key, tab in (tables or {}).items():
        if key.startswith("_") or not isinstance(tab, dict):
            continue
        try:
            ja, jb = key.split("_")
            cells = tab.get("cells", [])
            radius = float(tab.get("radius_deg", LOOKUP_RADIUS_DEG))
        except (ValueError, AttributeError, TypeError):
            continue
        if ja in pos and jb in pos and cells and \
                lookup_rejects(pos[ja], pos[jb], cells, radius):
            return False
    return True


def build_limits(sensor_bounds: dict, collision_bounds: dict,
                 narrow: dict | None = None) -> dict:
    """Merge sensor and collision sides into per-joint ranges.

    Each side takes the tighter of the sensor operating bound and
    the collision boundary; the source records which one won. Joints
    without a usable sensor side fall back to collision bounds on
    both sides. Manual narrowing (default MANUAL_NARROW_DEG, pass {}
    to disable) then clamps validation-driven cuts; it is a separate
    injectable step so merge logic stays unit-testable.
    """
    if narrow is None:
        narrow = MANUAL_NARROW_DEG
    sens_by_joint: dict[str, dict] = {}
    for row in sensor_bounds.get("bounds", []):
        if row.get("status") == "ok":
            sens_by_joint[row["joint"]] = row
    coll_by_joint = {r["joint"]: r
                     for r in collision_bounds.get("bounds", [])}
    limits: dict[str, list[float]] = {}
    sources: dict[str, dict[str, str]] = {}
    joints = list(coll_by_joint)
    for jid in joints:
        lo_candidates, hi_candidates = [], []
        sens = sens_by_joint.get(jid)
        coll = coll_by_joint[jid]
        # Collision side bounds (None means clean to the span end).
        cneg = coll.get("neg", {}).get("boundary_deg")
        cpos = coll.get("pos", {}).get("boundary_deg")
        lo_c = cneg if cneg is not None else -180.0
        hi_c = cpos if cpos is not None else 180.0
        lo_candidates.append((lo_c, "collision"))
        hi_candidates.append((hi_c, "collision"))
        if sens is not None:
            b = sens["boundary_deg"]
            side = sens["homing_direction"]
            if side == "+":
                hi_s = b - OPERATING_MARGIN_DEG
                hi_candidates.append((hi_s, "sensor"))
            elif side == "-":
                lo_s = b + OPERATING_MARGIN_DEG
                lo_candidates.append((lo_s, "sensor"))
        lo, lo_src = max(lo_candidates, key=lambda kv: kv[0])
        hi, hi_src = min(hi_candidates, key=lambda kv: kv[0])
        nar = narrow.get(jid, {})
        if "lo" in nar:
            lo = max(lo, nar["lo"])
        if "hi" in nar:
            hi = min(hi, nar["hi"])
        if not hi > lo:
            raise ValueError(f"joint {jid}: empty range [{lo}, {hi}]")
        limits[jid] = [round(lo, 2), round(hi, 2)]
        sources[jid] = {"lo": lo_src, "hi": hi_src}
    narrowing = {j: {"lo": narrow[j].get("lo"),
                     "hi": narrow[j].get("hi"),
                     "why": narrow[j].get("why")}
                 for j in narrow}
    return {"limits_deg": limits, "limit_source": sources,
            "range_narrowing_applied": narrowing}


def attach_coupled(limits_doc: dict, coupled: dict) -> dict:
    """Store inequalities with explicit a/b/c for runtime checkers."""
    out = []
    for d in coupled.get("inequalities", []):
        form = d.get("form", "sum")
        a = [1.0, 1.0] if form == "sum" else [1.0, -1.0]
        c, op = float(d["c"]), d["op"]
        out.append({"joints": list(d["joints"]), "a": a,
                    "b": -c if op == ">=" else c, "op": op})
    limits_doc["coupled_limits"] = out
    tables = coupled.get("lookup_tables", {})
    if tables:
        limits_doc["coupled_lookup"] = tables
    return limits_doc


def check_pose(q_deg: list[float], joint_ids: list[str],
               limits_doc: dict) -> list[str]:
    """Human-readable violations of box limits + coupled constraints."""
    bad = []
    lims = limits_doc.get("limits_deg", {})
    for jid, v in zip(joint_ids, q_deg):
        lo, hi = lims.get(jid, [-180.0, 180.0])
        if not lo - 1e-9 <= v <= hi + 1e-9:
            bad.append(f"{jid}: {v:.2f}deg outside [{lo:.1f}, {hi:.1f}]")
    pos = {jid: v for jid, v in zip(joint_ids, q_deg)}
    for d in limits_doc.get("coupled_limits", []):
        ja, jb = d["joints"]
        val = d["a"][0] * pos[ja] + d["a"][1] * pos[jb] + d["b"]
        if d["op"] == ">=" and val < -1e-9:
            bad.append(f"coupled {'+'.join(d['joints'])}: "
                       f"{val:.2f} < 0")
        if d["op"] == "<=" and val > 1e-9:
            bad.append(f"coupled {'+'.join(d['joints'])}: "
                       f"{val:.2f} > 0")
    if USE_COUPLED_LOOKUP and not check_lookup(
            q_deg, joint_ids, limits_doc.get("coupled_lookup", {})):
        bad.append("coupled lookup: pose lands on a colliding grid cell")
    return bad


def is_path_safe(q_from: list[float], q_to: list[float],
                 joint_ids: list[str], limits_doc: dict,
                 step_deg: float = 5.0) -> dict:
    """Interpolated waypoints every <= step_deg, all inside limits."""
    dist = max(abs(b - a) for a, b in zip(q_from, q_to))
    n = max(1, int(math.ceil(dist / step_deg)))
    for k in range(n + 1):
        q = [a + (b - a) * k / n for a, b in zip(q_from, q_to)]
        bad = check_pose(q, joint_ids, limits_doc)
        if bad:
            return {"safe": False, "waypoint": k, "n": n,
                    "q_deg": [round(v, 2) for v in q],
                    "violations": bad}
    return {"safe": True, "n": n}


def main() -> None:
    ap = argparse.ArgumentParser(description="Combine 5A limits")
    ap.add_argument("--sensors", default="runs/stage5a/sensor_bounds.json")
    ap.add_argument("--collisions",
                    default="runs/stage5a/collision_bounds.json")
    ap.add_argument("--coupled", default="runs/stage5a/coupled.json")
    ap.add_argument("--out", default="runs/stage5a/limits.json")
    args = ap.parse_args()
    sens = json.load(open(args.sensors, encoding="utf-8"))
    coll = json.load(open(args.collisions, encoding="utf-8"))
    doc = build_limits(sens, coll)
    doc["sensor_bounds"] = sens.get("bounds", [])
    try:
        coupled = json.load(open(args.coupled, encoding="utf-8"))
        doc = attach_coupled(doc, coupled)
    except OSError:
        doc["coupled_limits"] = []
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, indent=2)
    print(f"joints={list(doc['limits_deg'])} coupled="
          f"{len(doc.get('coupled_limits', []))} -> {args.out}")


if __name__ == "__main__":
    main()
