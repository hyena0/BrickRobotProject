"""Deterministic geometry tools for the agent (Stage 1.3).

Tools return facts only: poses, distances, mesh booleans, groups.
They never conclude joints, links or transmissions.

All thresholds are in millimetres. LDU values from the stage plan
are converted with 0.4 mm/LDU.
"""

from __future__ import annotations

import math

import numpy as np
import yaml

LDU_TO_MM = 0.4
# Mesh tolerance from the stage plan: +-2 LDU.
MESH_TOL_MM = 2 * LDU_TO_MM
# Two parts belong to one axle line when their centres are this close
# to the line (Stage 1 fixtures use exact geometry).
AXLE_LINE_TOL_MM = 2.0
# Fallback contact distance when the catalog lacks hole geometry.
CONTACT_MM = 12.0


def load_catalog(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _entry(catalog: dict, part_id: str) -> dict:
    return catalog.get(part_id, {}) or {}


def _pos(part: dict) -> np.ndarray:
    return np.asarray(part["position_mm"], dtype=float)


def _rot(part: dict) -> np.ndarray:
    return np.asarray(part["rotation"], dtype=float)


def axis_world(part: dict, catalog: dict) -> np.ndarray | None:
    """Rotation/insertion axis of a part in world frame, or None."""
    axis = _entry(catalog, part["part_id"]).get("axis_local")
    if axis is None:
        return None
    return _rot(part) @ np.asarray(axis, dtype=float)


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cosv = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosv))))


def get_parts(parts: list[dict], catalog: dict, category: str | None = None,
              near: list[float] | None = None,
              radius_mm: float | None = None) -> list[dict]:
    """Filter parts by catalog category and/or distance to a point."""
    out = []
    center = np.asarray(near, dtype=float) if near is not None else None
    for part in parts:
        if category is not None:
            if _entry(catalog, part["part_id"]).get("category") != category:
                continue
        if center is not None and radius_mm is not None:
            if float(np.linalg.norm(_pos(part) - center)) > radius_mm:
                continue
        info = _entry(catalog, part["part_id"])
        item = {"uid": part["uid"], "part_id": part["part_id"],
                "category": info.get("category"),
                "position_mm": list(part["position_mm"])}
        axis = axis_world(part, catalog)
        item["axis"] = [float(v) for v in axis] if axis is not None else None
        out.append(item)
    return out


def _classify_axle_fit(other: dict, catalog: dict) -> str:
    holes = _entry(catalog, other["part_id"]).get("holes") or []
    kinds = {h.get("kind") for h in holes if isinstance(h, dict)}
    if "axle" in kinds:
        return "axle_in_axle_hole"  # locks rotation
    return "axle_in_round_hole"  # free rotation


def find_connections(uid: str, parts: list[dict],
                     catalog: dict) -> list[dict]:
    """Neighbour parts of `uid` with a connection type.

    Tier 1 (catalog-driven): an `axle` part defines an axle line; parts
    whose centre lies on that line are axle-hole connections, classified
    by the catalog hole kind. Tier 2 (fallback): centre distance below
    CONTACT_MM reports `contact`.
    """
    by_uid = {p["uid"]: p for p in parts}
    me = by_uid[uid]
    my_axis = axis_world(me, catalog)
    my_cat = _entry(catalog, me["part_id"]).get("category")
    out = []
    for other in parts:
        if other["uid"] == uid:
            continue
        rel = _pos(other) - _pos(me)
        dist = float(np.linalg.norm(rel))
        conn = None
        if my_cat == "axle" and my_axis is not None:
            lateral = rel - float(np.dot(rel, my_axis)) * my_axis
            if float(np.linalg.norm(lateral)) <= AXLE_LINE_TOL_MM:
                conn = _classify_axle_fit(other, catalog)
        if conn is None:
            other_axis = axis_world(other, catalog)
            other_cat = _entry(catalog, other["part_id"]).get("category")
            if other_cat == "axle" and other_axis is not None:
                rel2 = _pos(me) - _pos(other)
                lateral = rel2 - float(np.dot(rel2, other_axis)) * other_axis
                if float(np.linalg.norm(lateral)) <= AXLE_LINE_TOL_MM:
                    conn = _classify_axle_fit(me, catalog)
        if conn is None and my_axis is not None:
            other_axis = axis_world(other, catalog)
            other_cat = _entry(catalog, other["part_id"]).get("category")
            if other_axis is not None and {"turntable"} & {my_cat, other_cat}:
                axis = my_axis / np.linalg.norm(my_axis)
                lateral = rel - float(np.dot(rel, axis)) * axis
                cosv = abs(float(np.dot(my_axis, other_axis)))
                cosv /= float(np.linalg.norm(my_axis) * np.linalg.norm(other_axis))
                if float(np.linalg.norm(lateral)) <= AXLE_LINE_TOL_MM and cosv > 0.99:
                    conn = "axle_in_round_hole"  # free rotation
        if conn is None and dist <= CONTACT_MM:
            conn = "contact"
        if conn is not None:
            out.append({"uid": other["uid"], "part_id": other["part_id"],
                        "connection": conn, "distance_mm": round(dist, 3)})
    out.sort(key=lambda e: e["distance_mm"])
    return out


def check_gear_mesh(uid_a: str, uid_b: str, parts: list[dict],
                    catalog: dict) -> dict:
    """Test whether two gears mesh. Returns distances and `meshes`."""
    by_uid = {p["uid"]: p for p in parts}
    a, b = by_uid[uid_a], by_uid[uid_b]
    ea, eb = _entry(catalog, a["part_id"]), _entry(catalog, b["part_id"])
    ax_a, ax_b = axis_world(a, catalog), axis_world(b, catalog)
    result: dict = {"a": uid_a, "b": uid_b, "meshes": False,
                    "kind": None, "note": ""}
    if ax_a is None or ax_b is None:
        result["note"] = "missing axis in catalog"
        return result
    ang = _angle_deg(ax_a, ax_b)
    ang = min(ang, 180.0 - ang)
    if ang <= 5.0:
        relation = "parallel"
    elif 85.0 <= ang <= 95.0:
        relation = "perpendicular"
    else:
        relation = "skew"
    result["axis_relation"] = relation
    dist_mm = float(np.linalg.norm(_pos(a) - _pos(b)))
    result["centre_mm"] = round(dist_mm, 3)
    dist_ldu = dist_mm / LDU_TO_MM
    result["centre_ldu"] = round(dist_ldu, 3)
    ta, tb = ea.get("teeth"), eb.get("teeth")
    if ta is None or tb is None:
        result["note"] = "missing teeth in catalog"
        return result
    override = ea.get("mesh_distance_ldu") or eb.get("mesh_distance_ldu")
    if override is not None:
        expected = float(override)
        kind = eb.get("gear_kind") or ea.get("gear_kind")
    elif relation == "parallel" and {ea.get("gear_kind"),
                                     eb.get("gear_kind")} <= {"spur",
                                                             "double-bevel",
                                                             None}:
        # Stage plan rule: centre = 1.25 * (T1 + T2) LDU.
        expected = 1.25 * (ta + tb)
        kind = "external"
    elif relation == "parallel":
        expected = 1.25 * (ta + tb)
        kind = "external"
    elif relation == "perpendicular":
        result["note"] = "bevel/worm mesh needs mesh_distance_ldu entry"
        result["expected_ldu"] = None
        return result
    else:
        result["note"] = "skew axes never mesh"
        result["expected_ldu"] = None
        return result
    result["expected_ldu"] = round(expected, 3)
    result["kind"] = kind
    result["meshes"] = abs(dist_ldu - expected) <= 2.0
    return result


def trace_axle(uid: str, parts: list[dict], catalog: dict) -> list[dict]:
    """All parts on the axle line through `uid`, ordered along the axis."""
    by_uid = {p["uid"]: p for p in parts}
    me = by_uid[uid]
    axis = axis_world(me, catalog)
    if axis is None:
        return [{"uid": uid, "connection": "self",
                 "along_mm": 0.0}]
    axis = axis / np.linalg.norm(axis)
    origin = _pos(me)
    traced = []
    for other in parts:
        rel = _pos(other) - origin
        lateral = rel - float(np.dot(rel, axis)) * axis
        if float(np.linalg.norm(lateral)) > AXLE_LINE_TOL_MM:
            continue
        along = round(float(np.dot(rel, axis)), 3)
        if other["uid"] == uid:
            conn = "self"
        else:
            hits = [c["connection"] for c in find_connections(other["uid"],
                                                              parts, catalog)
                    if c["uid"] == uid]
            conn = hits[0] if hits else "contact"
        traced.append({"uid": other["uid"], "part_id": other["part_id"],
                       "connection": conn, "along_mm": along})
    traced.sort(key=lambda e: e["along_mm"])
    return traced


def rigid_groups(parts: list[dict], catalog: dict,
                 rigid_types: set[str] | None = None) -> list[dict]:
    """Union-find groups over connections treated as rigid.

    Default rigid types lock rotation or are press-fit: axle-in-axle-hole,
    friction pins and studs. Free-rotation connections never join groups.
    """
    if rigid_types is None:
        rigid_types = {"axle_in_axle_hole", "pin_friction", "stud"}
    parent = {p["uid"]: p["uid"] for p in parts}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: str, b: str) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for part in parts:
        for conn in find_connections(part["uid"], parts, catalog):
            if conn["connection"] in rigid_types:
                union(part["uid"], conn["uid"])
    groups: dict[str, list[dict]] = {}
    for part in parts:
        groups.setdefault(find(part["uid"]), []).append(part)
    out = []
    for i, (root, members) in enumerate(sorted(groups.items())):
        pts = np.array([_pos(m) for m in members])
        cats = {_entry(catalog, m["part_id"]).get("category")
                for m in members}
        out.append({
            "id": f"G{i}",
            "part_count": len(members),
            "bbox_min_mm": [round(float(v), 2) for v in pts.min(axis=0)],
            "bbox_max_mm": [round(float(v), 2) for v in pts.max(axis=0)],
            "has_motor": "motor" in cats,
            "has_sensor": "sensor" in cats,
            "uids": sorted(m["uid"] for m in members),
        })
    return out
