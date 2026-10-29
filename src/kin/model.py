"""Kinematic model helpers (Stage 4).

The model JSON follows docs/schema.md: joints with unit axis,
point_mm, limits_deg; actuators; transmission C (q = C m, normalized
so each drive column has self +1); end_effector link + point_mm.
"""

from __future__ import annotations

import json
import math


def load_model(path: str) -> dict:
    """Load and validate a kinematic model JSON file."""
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    joints = doc.get("joints", [])
    if not joints:
        raise ValueError(f"no joints in {path}")
    for j in joints:
        if j.get("type", "revolute") != "revolute":
            raise ValueError(f"joint {j.get('id')}: only revolute "
                             f"supported by this stage")
        if len(j["axis"]) != 3 or len(j["point_mm"]) != 3:
            raise ValueError(f"joint {j.get('id')}: axis/point_mm "
                             f"need 3 numbers")
        if len(j.get("limits_deg", [])) != 2:
            raise ValueError(f"joint {j.get('id')}: limits_deg "
                             f"needs [lo, hi]")
    return doc


def norm(v) -> list[float]:
    n = math.sqrt(sum(x * x for x in v))
    if n == 0.0:
        raise ValueError("zero axis vector")
    return [x / n for x in v]


def chain_order(doc: dict) -> list[dict]:
    """Order joints root-to-tip by following parent/child links."""
    joints = doc["joints"]
    children = {j["parent"] for j in joints}
    roots = [j for j in joints if j["parent"] not in
             {k["child"] for k in joints}]
    if len(roots) != 1:
        raise ValueError("model needs exactly one root joint")
    by_parent = {j["parent"]: j for j in joints}
    if len(by_parent) != len(joints):
        raise ValueError("joint parents are not unique links")
    ordered, cur = [], roots[0]
    seen = set()
    while cur is not None:
        if cur["id"] in seen:
            raise ValueError("joint chain has a cycle")
        seen.add(cur["id"])
        ordered.append(cur)
        cur = by_parent.get(cur["child"])
    if len(ordered) != len(joints):
        raise ValueError("joint chain is not a single serial chain")
    _ = children  # link ids are informational only
    return ordered


def joint_arrays(doc: dict) -> dict:
    """Axes (unit), points (mm), limits (rad) in chain order."""
    ordered = chain_order(doc)
    return {
        "ids": [j["id"] for j in ordered],
        "axes": [norm(j["axis"]) for j in ordered],
        "points": [[float(x) for x in j["point_mm"]] for j in ordered],
        "limits": [[math.radians(j["limits_deg"][0]),
                    math.radians(j["limits_deg"][1])] for j in ordered],
    }


def ee_point(doc: dict) -> list[float]:
    """End-effector point in mm (base frame at zero pose).

    Uses end_effector.point_mm when present, else the last joint
    point (wrist center). The choice is recorded in the model file
    that is evaluated, so both sides stay consistent.
    """
    arr = joint_arrays(doc)
    ee = doc.get("end_effector", {})
    pt = ee.get("point_mm")
    if pt is not None:
        if len(pt) != 3:
            raise ValueError("end_effector.point_mm needs 3 numbers")
        return [float(x) for x in pt]
    return list(arr["points"][-1])


def coupling(doc: dict) -> dict | None:
    """Transmission block: C rows=joints, cols=motors, or None."""
    trans = doc.get("transmission", {})
    c = trans.get("C")
    if not c:
        return None
    acts = doc.get("actuators", [])
    return {"C": [[float(x) for x in row] for row in c],
            "joint_ids": [j["id"] for j in chain_order(doc)],
            "motor_ids": [a["id"] for a in acts]}
