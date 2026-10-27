"""Deterministic Stage 2 baseline (no model calls).

The heuristic never concludes robot joints from catalog-free geometry
alone. It reports what the tools support today:
- rigid groups from the reviewed catalog entries,
- candidate free-rotation interfaces between groups,
- a base-link guess (largest group touching the ground plane).

The Spark agent (see `run.py`) uses the same facts to decide which
interfaces are robot joints. This baseline output is schema-valid with
zero joints when no reviewed rotary interface exists, so the verifier
fails loudly instead of scoring a guess.
"""

from __future__ import annotations

import numpy as np

from src.tools import geometry as geom

FREE_TYPES = {"axle_in_round_hole", "pin_frictionless"}
# Transmission, never a robot joint: the motor shaft spins relative to
# its own body. Recorded in review_note, excluded from candidacy.
TRANSMISSION_TYPES = {"motor_shaft"}


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def infer(parts: list[dict], catalog: dict) -> dict:
    groups = geom.rigid_groups(parts, catalog)
    by_uid = {p["uid"]: p for p in parts}
    group_of = {}
    for group in groups:
        for uid in group["uids"]:
            group_of[uid] = group["id"]

    interfaces: dict[tuple[str, str], dict] = {}
    transmission = 0
    for part in parts:
        for conn in geom.find_connections(part["uid"], parts, catalog):
            if conn["connection"] in TRANSMISSION_TYPES:
                transmission += 1
                continue
            if conn["connection"] not in FREE_TYPES:
                continue
            a, b = group_of[part["uid"]], group_of[conn["uid"]]
            if a == b:
                continue
            key = tuple(sorted((a, b)))
            entry = interfaces.setdefault(key, {"pairs": 0,
                                                "positions": []})
            entry["pairs"] += 1
            pa = np.asarray(by_uid[part["uid"]]["position_mm"],
                            dtype=float)
            pb = np.asarray(by_uid[conn["uid"]]["position_mm"],
                            dtype=float)
            entry["positions"].append(((pa + pb) / 2.0).tolist())

    # Base guess: largest group touching the lowest Z layer.
    zmin = min(p["position_mm"][2] for p in parts)
    base_id = None
    base_score = -1
    for group in groups:
        uids = group["uids"]
        touches = sum(1 for uid in uids
                      if by_uid[uid]["position_mm"][2] - zmin < 5.0)
        score = (touches > 0, len(uids))
        if score > (base_score > 0, 0) and touches > 0:
            base_id, base_score = group["id"], len(uids)
    if base_id is None:
        base_id = max(groups, key=lambda g: g["part_count"])["id"]

    largest = sorted(groups, key=lambda g: g["part_count"],
                     reverse=True)
    far = max(parts, key=lambda p: p["position_mm"][2])
    ee_id = group_of[far["uid"]]

    joints = []
    for i, ((ga, gb), info) in enumerate(
            sorted(interfaces.items(),
                   key=lambda kv: -kv[1]["pairs"])):
        pts = np.asarray(info["positions"], dtype=float)
        center = pts.mean(axis=0)
        # Axis estimate: smallest-variance direction of the contact
        # centroid cloud; low confidence by construction.
        if len(pts) >= 3:
            _, _, vh = np.linalg.svd(pts - center)
            axis = _unit(vh[-1])
        else:
            axis = np.array([0.0, 0.0, 1.0])
        joints.append({
            "id": f"J{i + 1}",
            "type": "revolute",
            "parent": ga,
            "child": gb,
            "axis": [round(float(v), 4) for v in axis],
            "point_mm": [round(float(v), 2) for v in center],
            "limits_deg": [-180, 180],
            "confidence": 0.2,
            "reason": (f"candidate free-rotation interface "
                       f"({info['pairs']} contacts); needs agent review"),
        })

    links = [{"id": g["id"], "part_count": g["part_count"],
              "members": sorted(g["uids"]),
              "has_motor": g.get("has_motor", False),
              "has_sensor": g.get("has_sensor", False),
              "bbox_min_mm": g.get("bbox_min_mm"),
              "bbox_max_mm": g.get("bbox_max_mm")} for g in groups]
    return {
        "method": "heuristic-baseline",
        "links": links,
        "joints": joints,
        "base": {"link": base_id},
        "end_effector": {"link": ee_id},
        "sensors": [],
        "actuators": [],
        "transmission": {
            "coupling": "q = C · m, q in joint degrees, m in motor degrees",
            "C": [],
        },
        "review_note": ("candidate interfaces only; robot-joint decision "
                        f"belongs to the agent; {transmission} motor-shaft "
                        "interfaces excluded as transmission"),
    }
