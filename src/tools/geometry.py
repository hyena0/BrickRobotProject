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


def _conv_dir(local) -> np.ndarray:
    """LDraw-frame direction vector into the converted local frame."""
    v = np.asarray(local, dtype=float)
    return np.array([v[0], v[2], -v[1]])


def axis_world(part: dict, catalog: dict) -> np.ndarray | None:
    """Rotation/insertion axis of a part in world frame, or None.

    Stored rotations are true world rotations, so local LDraw-frame
    axes convert with the same point remap as positions.
    """
    axis = _entry(catalog, part["part_id"]).get("axis_local")
    if axis is None:
        return None
    return _rot(part) @ _conv_dir(axis)


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


def hole_world(part: dict, hole: dict) -> np.ndarray:
    """Centre of a catalog hole in world frame (mm).

    Catalog positions are part-local LDraw LDU; they get the same
    axis remap as parser points, then the part rotation/translation.
    """
    local = np.asarray(hole.get("pos") or [0, 0, 0], dtype=float)
    conv = _conv_dir(local) * LDU_TO_MM
    return _pos(part) + _rot(part) @ conv


def _classify_axle_fit(other: dict, catalog: dict,
                       line_point: np.ndarray | None = None,
                       line_dir: np.ndarray | None = None) -> str:
    holes = _entry(catalog, other["part_id"]).get("holes") or []
    holes = [h for h in holes if isinstance(h, dict)]
    if _entry(catalog, other["part_id"]).get("category") == "bush":
        return "axle_in_axle_hole"  # bush clamps the axle
    if line_point is not None and line_dir is not None and holes:
        # Position-aware: the bore the line actually passes through
        # decides. Bores are tunnels, not points: match the segment
        # from each hole through the part centre to its mirror, so a
        # shaft through a gear's mid-plane counts even when only the
        # face entries are catalogued. Nearest segment wins.
        centre = _pos(other)
        direction = line_dir / float(np.linalg.norm(line_dir))
        best = None
        for hole in holes:
            face = hole_world(other, hole)
            mirror = 2.0 * centre - face
            closest = float("inf")
            for _t in (0.0, 0.5, 1.0):
                probe = mirror + _t * (face - mirror)
                rel = probe - line_point
                lateral = rel - float(np.dot(rel, direction)) * direction
                closest = min(closest, float(np.linalg.norm(lateral)))
            if best is None or closest < best[0]:
                best = (closest, hole.get("kind"))
        if best is not None and best[0] <= AXLE_LINE_TOL_MM:
            return ("axle_in_axle_hole" if best[1] == "axle"
                    else "axle_in_round_hole")
    kinds = {h.get("kind") for h in holes}
    if "axle" in kinds:
        return "axle_in_axle_hole"  # locks rotation
    return "axle_in_round_hole"  # free rotation


def _pin_span_mm(part: dict, catalog: dict) -> float:
    """Half-length of a pin's joining segment in mm."""
    entry = _entry(catalog, part["part_id"])
    studs = entry.get("pin_length_studs") or 3
    return float(studs) * 8.0 / 2.0 + 2.0


def _line_points(other: dict, catalog: dict) -> list[np.ndarray]:
    """Test points of a part: its centre plus its catalog hole centres."""
    pts = [_pos(other)]
    for hole in _entry(catalog, other["part_id"]).get("holes") or []:
        if isinstance(hole, dict):
            pts.append(hole_world(other, hole))
    return pts


def _line_hits(me: dict, my_axis: np.ndarray, other: dict,
               catalog: dict, span_mm: float | None) -> bool:
    """Whether `other` meets the line (centre or hole, within span)."""
    axis = my_axis / np.linalg.norm(my_axis)
    origin = _pos(me)
    for pt in _line_points(other, catalog):
        rel = pt - origin
        along = float(np.dot(rel, axis))
        if span_mm is not None and abs(along) > span_mm:
            continue
        lateral = rel - along * axis
        if float(np.linalg.norm(lateral)) <= AXLE_LINE_TOL_MM:
            return True
    return False


def _axle_span_mm(part: dict, catalog: dict) -> float:
    """Half-length of an axle in mm (infinite when unknown)."""
    studs = _entry(catalog, part["part_id"]).get("axle_length")
    if not studs:
        return float("inf")
    return float(studs) * 8.0 / 2.0 + 3.0


def find_connections(uid: str, parts: list[dict],
                     catalog: dict) -> list[dict]:
    """Neighbour parts of `uid` with a connection type.

    Line tiers (catalog-driven): `axle` parts define infinite axle
    lines, classified position-aware by the hole the line pierces
    (axle bore locks rotation, round bore is free). `pin` parts
    define segments of pin length: a friction pin locks everything
    it pierces (`pin_friction`, rigid), a frictionless pin stays free.
    `motor` parts define shaft lines (transmission, never rigid to
    the motor body). Tier 2 (fallback): centre distance below
    CONTACT_MM reports `contact`.
    """
    by_uid = {p["uid"]: p for p in parts}
    me = by_uid[uid]
    my_axis = axis_world(me, catalog)
    my_entry = _entry(catalog, me["part_id"])
    my_cat = my_entry.get("category")
    out = []
    for other in parts:
        if other["uid"] == uid:
            continue
        rel = _pos(other) - _pos(me)
        dist = float(np.linalg.norm(rel))
        conn = None
        if my_cat == "axle" and my_axis is not None:
            if _line_hits(me, my_axis, other, catalog,
                           _axle_span_mm(me, catalog)):
                conn = _classify_axle_fit(other, catalog,
                                          _pos(me), my_axis)
        if conn is None and my_cat == "pin" and my_axis is not None:
            if _line_hits(me, my_axis, other, catalog,
                           _pin_span_mm(me, catalog)):
                conn = ("pin_friction" if my_entry.get("friction", True)
                        else "pin_frictionless")
        if conn is None and my_cat == "motor" and my_axis is not None:
            if _line_hits(me, my_axis, other, catalog, 60.0):
                conn = "motor_shaft"  # transmission, free vs body
        if conn is None:
            other_axis = axis_world(other, catalog)
            other_entry = _entry(catalog, other["part_id"])
            other_cat = other_entry.get("category")
            if other_cat == "axle" and other_axis is not None:
                if _line_hits(other, other_axis, me, catalog,
                               _axle_span_mm(other, catalog)):
                    conn = _classify_axle_fit(me, catalog,
                                              _pos(other), other_axis)
            elif other_cat == "pin" and other_axis is not None:
                if _line_hits(other, other_axis, me, catalog,
                               _pin_span_mm(other, catalog)):
                    conn = ("pin_friction"
                            if other_entry.get("friction", True)
                            else "pin_frictionless")
            elif other_cat == "motor" and other_axis is not None:
                if _line_hits(other, other_axis, me, catalog, 60.0):
                    conn = "motor_shaft"
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
    if relation == "parallel" and \
            "turntable" not in {ea.get("gear_kind"), eb.get("gear_kind")}:
        # Compound (same-shaft) test first: centres on one line within
        # bore tolerance, coupled by one axle part whose span covers
        # both gear planes. Blender close-ups (stage3-mesh/stack*_iso)
        # show stacked 16T+8T pairs riding one through-axle although
        # the bore-fit detector reports only contact, so a spanning
        # through-axle counts as the rigid coupling here. Turntable
        # halves are excluded: they bolt to links (no axle bore), so
        # a shaft through their clearance never locks them.
        ua = np.asarray(ax_a, dtype=float)
        ua = ua / np.linalg.norm(ua)
        rel = _pos(b) - _pos(a)
        along = float(np.dot(rel, ua))
        lat = float(np.linalg.norm(rel - along * ua))
        if lat <= AXLE_LINE_TOL_MM:
            lo, hi = min(0.0, along), max(0.0, along)
            for p in parts:
                if p["uid"] in (uid_a, uid_b):
                    continue
                if _entry(catalog, p["part_id"]).get("category") \
                        != "axle":
                    continue
                pax = axis_world(p, catalog)
                if pax is None:
                    continue
                if min(_angle_deg(pax, ua),
                       180.0 - _angle_deg(pax, ua)) > 2.0:
                    continue
                prel = _pos(p) - _pos(a)
                s = float(np.dot(prel, ua))
                if float(np.linalg.norm(prel - s * ua)) > \
                        AXLE_LINE_TOL_MM:
                    continue
                half = _axle_span_mm(p, catalog)
                tol = 4.0
                if s - half - tol <= lo and s + half + tol >= hi:
                    result["kind"] = "compound"
                    result["expected_ldu"] = result["centre_ldu"]
                    result["meshes"] = True
                    result["note"] = ("same shaft via %s" % p["uid"])
                    return result
    override = ea.get("mesh_distance_ldu") or eb.get("mesh_distance_ldu")
    if override is not None:
        expected = float(override)
        kind = eb.get("gear_kind") or ea.get("gear_kind")
        tol_ldu = 2.0
    elif relation == "parallel" and {ea.get("gear_kind"),
                                     eb.get("gear_kind")} <= {"spur",
                                                             "double-bevel",
                                                             None}:
        # Stage plan rule: centre = 1.25 * (T1 + T2) LDU.
        expected = 1.25 * (ta + tb)
        kind = "external"
        tol_ldu = 2.0
    elif relation == "parallel":
        if "ring" in {ea.get("gear_kind"), eb.get("gear_kind")}:
            # Internal ring gears engage pinions inside the ring
            # (solver-virtualized); never as external spur meshes.
            result["note"] = "ring kind excluded from external mesh"
            result["expected_ldu"] = None
            return result
        expected = 1.25 * (ta + tb)
        kind = "external"
        tol_ldu = 2.0
    elif relation == "perpendicular":
        # Bevel geometry measured from the embedded LDraw library
        # (tip radii, tooth placements) and validated against assembly
        # pair distances. Bevel mounts allow more axial variance than
        # spur mounts, hence the wider tolerance.
        ka, kb = ea.get("gear_kind"), eb.get("gear_kind")
        kinds = {ka, kb}
        tol_ldu = 2.5
        if kinds <= {"bevel"}:
            # Equal-mitre bevel pairs share one pitch-cone apex:
            # centre follows the pitch radii plus a 2 LDU apex
            # allowance (12T pairs observed at 21.5-25.2 LDU).
            expected = 1.25 * math.sqrt(ta * ta + tb * tb) + 2.0
            kind = "bevel"
        elif "double-bevel" in kinds and kinds <= {"bevel",
                                                  "double-bevel",
                                                  "turntable"}:
            # Double-bevel teeth mesh at 90 deg with a thicker cone;
            # constant validated on 12T-class pairs (26.8-29.3 LDU).
            expected = 28.0
            kind = "bevel"
        elif kinds == {"knob"}:
            # 4-lobe knob wheels interleave lobes; tip radius ~30 LDU
            # measured from part geometry (single-site validation).
            expected = 28.0
            kind = "knob"
        elif "turntable" in kinds and kinds <= {"turntable", "spur"}:
            # Crown mesh: spur pinion against turntable face teeth;
            # spur sum plus one module allowance for the rim cut
            # (56T site observed at 81.9-83.0 LDU vs 80.0).
            expected = 1.25 * (ta + tb) + 2.5
            kind = "crown"
        else:
            result["note"] = ("unsupported perpendicular kinds %s" %
                              sorted(k or "?" for k in kinds))
            result["expected_ldu"] = None
            return result
    else:
        result["note"] = "skew axes never mesh"
        result["expected_ldu"] = None
        return result
    result["expected_ldu"] = round(expected, 3)
    result["kind"] = kind
    result["meshes"] = abs(dist_ldu - expected) <= (
        tol_ldu if relation == "perpendicular" else 2.0)
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
        # A part is on the line when its centre or one of its holes is;
        # ordering uses the centre (stable for symmetric parts).
        on_line = any(
            float(np.linalg.norm(
                (pt - origin) - float(np.dot(pt - origin, axis)) * axis))
            <= AXLE_LINE_TOL_MM
            for pt in _line_points(other, catalog))
        if not on_line:
            continue
        along = round(float(np.dot(_pos(other) - origin, axis)), 3)
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
