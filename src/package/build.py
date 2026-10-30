"""Robot package builder v0 (Stage 5A.6, PC authoring side).

The only runtime input of the controller package: abstract
kinematics, limits, coupled constraints, safe poses, speed caps.
No meshes, part ids/poses, colours, images or file references.
"""

from __future__ import annotations

import argparse
import json
import math

import numpy as np

from src.kin.model import chain_order, coupling, load_model
from src.limits.common import ensure_dir, load_inputs
from src.limits.enforce import USE_COUPLED_LOOKUP

SCHEMA_VERSION = "0.1.0"
SIZE_LIMIT_BYTES = 100 * 1024


def link_envelopes(ctx: dict) -> list[dict]:
    """At most one oriented box per link from member part positions.

    Numbers only (centre/size in mm, orientation in degrees), rounded
    for the 1 mm / 1 deg package rule.
    """
    out = []
    for lid in ctx["link_ids"]:
        pts = np.array([ctx["by_uid"][u]["position_mm"]
                        for u in ctx["members"].get(lid, set())
                        if u in ctx["by_uid"]], dtype=float)
        if not len(pts):
            continue
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        out.append({"link": lid,
                    "center_mm": [round(float(v), 0) for v in
                                  ((lo + hi) / 2.0)],
                    "size_mm": [round(float(v), 0) for v in (hi - lo)],
                    "orientation": [0.0, 0.0, 0.0]})
    return out


def build(robot: str, model_path: str, limits_path: str,
          poses_path: str, out_path: str) -> dict:
    """Assemble and write robot_package.json; returns the document."""
    from src.limits.common import load_inputs as _load

    doc = load_model(model_path)
    ordered = chain_order(doc)
    coup = coupling(doc)
    limits = json.load(open(limits_path, encoding="utf-8"))
    safe = json.load(open(poses_path, encoding="utf-8"))
    ctx = _load(model_path=model_path)
    pkg = {
        "schema_version": SCHEMA_VERSION,
        "package_version": 1,
        "robot": robot,
        "frame": doc.get("frame", "base link, Z up, mm"),
        "joints": [{"id": j["id"], "type": j.get("type", "revolute"),
                    "parent": j["parent"], "child": j["child"],
                    "axis": [round(float(v), 4) for v in j["axis"]],
                    "point_mm": [round(float(v), 2) for v in
                                 j["point_mm"]],
                    "limits_deg": limits["limits_deg"][j["id"]],
                    "limit_source": limits["limit_source"][j["id"]]}
                   for j in ordered],
        # v0 carries no hardware identity: columns are generic motor
        # slots M1..Mn in actuator order. Stage 5C binds each slot
        # to a brick port (port wiring), never part uids.
        "coupling": {"coupling": "q = C m",
                     "C": coup["C"] if coup else [],
                     "motor_ids": [f"M{n + 1}" for n in
                                   range(len(coup["C"][0]) if coup else
                                         [])]},
        "coupled_limits": limits.get("coupled_limits", []),
        # Conservative finish: lookup stays implemented but off;
        # the package carries ranges + inequalities only.
        "coupled_lookup": (limits.get("coupled_lookup", {})
                           if USE_COUPLED_LOOKUP else {}),
        "operating_margin_deg": 5.0,
        "end_effector": doc.get("end_effector", {}),
        "safe_poses": safe.get("poses", safe if isinstance(safe, list)
                               else []),
        "speed_caps": {"first_run_deg_s": 30.0, "max_deg_s": 100.0,
                       "homing_deg_s": 20.0},
        "link_envelopes": link_envelopes(ctx),
    }
    ensure_dir(out_path)
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(pkg, fh, indent=2)
    size = len(json.dumps(pkg).encode("utf-8"))
    if size > SIZE_LIMIT_BYTES:
        raise ValueError(f"package {size} bytes over the 100 KB limit")
    return pkg


def main() -> None:
    ap = argparse.ArgumentParser(description="Build robot package v0")
    ap.add_argument("--robot", default="robot_inferred")
    ap.add_argument("--model", default="models/robot_inferred.json")
    ap.add_argument("--limits", default="runs/stage5a/limits.json")
    ap.add_argument("--poses", default="runs/stage5a/poses10_safe.json")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = args.out or f"packages/{args.robot}/robot_package.json"
    pkg = build(args.robot, args.model, args.limits, args.poses, out)
    print(f"package v{pkg['package_version']} joints={len(pkg['joints'])}"
          f" envelopes={len(pkg['link_envelopes'])} -> {out}")


if __name__ == "__main__":
    main()
