"""Joint-axis tools for Stage 2 re-identification (Stage 2.9).

Axes come from part geometry (catalog hole centres, axle lines,
turntable rotation axes) -- never from part origins or bounding boxes.

Pipeline: `joint_candidates` -> `sweep_test` -> Spark selection ->
`check_kinematic_consistency` (retry x3) -> links_joints doc.
"""

from __future__ import annotations

import math

import numpy as np

from src.tools import geometry as geom

FREE_TYPES = {"axle_in_round_hole", "pin_frictionless"}
RIGID_TYPES = {"axle_in_axle_hole", "pin_friction", "stud"}
# A group is powertrain when every member is a transmission part.
POWERTRAIN_CATS = {"axle", "gear", "bush"}
JOINER_PARTS = {"59443.dat"}
COAX_LATERAL_MM = 2.0
COAX_ANGLE_DEG = 3.0
SWEEP_START_DEG = 10.0
SWEEP_STEP_DEG = 5.0
SWEEP_MAX_DEG = 180.0
# Drive-path terminal to joint-point assignment radius. Turntable mesh
# radii put path-end gears up to ~75 mm from the joint centre.
MOTOR_ASSIGN_MM = 80.0


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def fit_axis_line(points: list) -> dict:
    """Least-squares axis through 3D support points.

    Returns point (centroid), direction (principal axis), fit error
    (max lateral residual, mm) and support count.
    """
    pts = np.asarray(points, dtype=float)
    center = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - center)
    direction = _unit(vh[0])
    lateral = np.linalg.norm(
        (pts - center) - np.outer((pts - center) @ direction, direction),
        axis=1)
    return {"point_mm": [round(float(v), 2) for v in center],
            "axis": [round(float(v), 4) for v in direction],
            "fit_error_mm": round(float(lateral.max()), 3),
            "n_supports": len(points)}


def _hole_centres_on_line(part: dict, catalog: dict,
                          line_point: np.ndarray,
                          line_dir: np.ndarray) -> list[np.ndarray]:
    """Catalog hole centres of `part` lying on the axle line."""
    out = []
    holes = geom._entry(catalog, part["part_id"]).get("holes") or []
    for hole in holes:
        if not isinstance(hole, dict):
            continue
        try:
            c = geom.hole_world(part, hole)
        except Exception:
            continue
        rel = c - line_point
        lateral = np.linalg.norm(
            rel - float(np.dot(rel, line_dir)) * line_dir)
        if lateral <= geom.AXLE_LINE_TOL_MM:
            out.append(c)
    return out


def _crossing_shafts(parts: list[dict], catalog: dict, by_uid: dict,
                     uids_a: set, uids_b: set) -> list[dict]:
    """Axle/pin parts whose line pierces members of both groups."""
    shafts = []
    for part in parts:
        entry = geom._entry(catalog, part["part_id"])
        if entry.get("category") not in ("axle", "pin", "motor"):
            continue
        axis = geom.axis_world(part, catalog)
        if axis is None:
            continue
        direction = _unit(axis)
        span = (geom._axle_span_mm(part, catalog)
                if entry.get("category") == "axle" else 60.0)
        hits_a = [u for u in uids_a
                  if geom._line_hits(part, axis, by_uid[u], catalog, span)]
        hits_b = [u for u in uids_b
                  if geom._line_hits(part, axis, by_uid[u], catalog, span)]
        if hits_a and hits_b:
            shafts.append({"uid": part["uid"], "direction": direction,
                           "hits_a": hits_a, "hits_b": hits_b})
    return shafts


def interface_supports(parts: list[dict], catalog: dict, by_uid: dict,
                       uids_a: set, uids_b: set) -> dict:
    """Support points + known direction for one group-pair interface."""
    supports: list[np.ndarray] = []
    known_dir = None
    turntable_hits = 0
    for shaft in _crossing_shafts(parts, catalog, by_uid, uids_a, uids_b):
        me = by_uid[shaft["uid"]]
        supports.append(geom._pos(me))
        for u in shaft["hits_a"] + shaft["hits_b"]:
            supports.extend(
                _hole_centres_on_line(by_uid[u], catalog,
                                      geom._pos(me), shaft["direction"]))
        if known_dir is None:
            known_dir = shaft["direction"]
    # Turntable rotation axis: centre + catalog axis, zero fit error.
    for u in uids_a | uids_b:
        part = by_uid[u]
        if geom._entry(catalog, part["part_id"]).get("category") \
                == "turntable":
            axis = geom.axis_world(part, catalog)
            if axis is not None:
                turntable_hits += 1
                if known_dir is None:
                    known_dir = _unit(axis)
                    supports.append(geom._pos(part))
    # Dedupe to 0.1 mm so repeated piercings do not inflate the count.
    unique: list = []
    seen: set = set()
    for p in supports:
        key = tuple(round(float(v), 1) for v in np.asarray(p).ravel())
        if key not in seen:
            seen.add(key)
            unique.append(p)
    return {"points": unique, "direction": known_dir,
            "turntable_hits": turntable_hits}


def _collinear(fit_a: dict, fit_b: dict) -> bool:
    pa = np.asarray(fit_a["point_mm"], dtype=float)
    pb = np.asarray(fit_b["point_mm"], dtype=float)
    da = _unit(np.asarray(fit_a["axis"], dtype=float))
    db = _unit(np.asarray(fit_b["axis"], dtype=float))
    ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(da @ db))))))
    if ang > COAX_ANGLE_DEG:
        return False
    lateral = float(np.linalg.norm((pb - pa) - float((pb - pa) @ da) * da))
    return lateral <= COAX_LATERAL_MM


def joint_candidates(parts: list[dict], catalog: dict) -> dict:
    """Stage-2 joint candidates with per-step counts.

    (a) groups made only of axle/gear/bush/joiner parts are powertrain;
    (b) only free-rotation connections between structural groups qualify;
    (c) coaxial connections merge into one candidate;
    (d) pairs also rigid-coupled through another path are excluded.
    """
    counts = {}
    groups = geom.rigid_groups(parts, catalog)
    by_uid = {p["uid"]: p for p in parts}
    group_of = {}
    for group in groups:
        for uid in group["uids"]:
            group_of[uid] = group["id"]
    counts["n_groups"] = len(groups)

    powertrain: set[str] = set()
    for group in groups:
        cats = set()
        joiner_only = True
        for uid in group["uids"]:
            pid = by_uid[uid]["part_id"]
            cats.add(geom._entry(catalog, pid).get("category"))
            if pid not in JOINER_PARTS and \
                    geom._entry(catalog, pid).get("category") \
                    not in POWERTRAIN_CATS:
                joiner_only = False
        if cats <= POWERTRAIN_CATS or joiner_only:
            powertrain.add(group["id"])
    counts["n_powertrain_groups"] = len(powertrain)

    raw: dict[tuple[str, str], dict] = {}
    for part in parts:
        for conn in geom.find_connections(part["uid"], parts, catalog):
            if conn["connection"] not in FREE_TYPES:
                continue
            a, b = group_of[part["uid"]], group_of[conn["uid"]]
            if a == b:
                continue
            key = tuple(sorted((a, b)))
            entry = raw.setdefault(key, {"pairs": 0, "rigid_also": False})
            entry["pairs"] += 1
    counts["n_raw_interfaces"] = len(raw)

    structural = {k: v for k, v in raw.items()
                  if k[0] not in powertrain and k[1] not in powertrain}
    counts["n_structural"] = len(structural)
    members = {g["id"]: set(g["uids"]) for g in groups}

    for (a, b), entry in structural.items():
        for uid in members[a] | members[b]:
            part = by_uid[uid]
            for conn in geom.find_connections(part["uid"], parts, catalog):
                if conn["connection"] in RIGID_TYPES and \
                        group_of.get(conn["uid"]) in (a, b) and \
                        group_of.get(conn["uid"]) != group_of[part["uid"]]:
                    entry["rigid_also"] = True
    free = {k: v for k, v in structural.items() if not v["rigid_also"]}
    counts["n_excluded_rigid"] = len(structural) - len(free)

    fitted: list[dict] = []
    for (a, b), entry in free.items():
        sup = interface_supports(parts, catalog, by_uid,
                                 members[a], members[b])
        if len(sup["points"]) >= 2:
            fit = fit_axis_line([np.asarray(p, dtype=float)
                                 for p in sup["points"]])
        elif len(sup["points"]) == 1 and sup["direction"] is not None:
            c = np.asarray(sup["points"][0], dtype=float)
            fit = {"point_mm": [round(float(v), 2) for v in c],
                   "axis": [round(float(v), 4) for v in sup["direction"]],
                   "fit_error_mm": 0.0, "n_supports": 1}
        else:
            continue
        fitted.append({"groups": (a, b), "pairs": entry["pairs"],
                       "fit": fit,
                       "turntable_hits": sup["turntable_hits"]})

    merged: list[dict] = []
    for cand in fitted:
        for done in merged:
            if set(cand["groups"]) == set(done["groups"]) and \
                    _collinear(cand["fit"], done["fit"]):
                done["pairs"] += cand["pairs"]
                done["turntable_hits"] += cand["turntable_hits"]
                break
        else:
            merged.append(cand)
    counts["n_merged"] = len(fitted) - len(merged)
    counts["n_candidates"] = len(merged)
    print("candidate counts: " + ", ".join(
        f"{k}={v}" for k, v in counts.items()))
    return {"counts": counts, "candidates": merged, "groups": groups,
            "powertrain": sorted(powertrain)}


def _rotate(points: np.ndarray, axis_pt: np.ndarray,
            axis_dir: np.ndarray, angle_deg: float) -> np.ndarray:
    """Rotate points about an axis (Rodrigues)."""
    theta = math.radians(angle_deg)
    k = axis_dir / float(np.linalg.norm(axis_dir))
    v = points - axis_pt
    return (axis_pt + v * math.cos(theta)
            + np.cross(k, v) * math.sin(theta)
            + np.outer(np.dot(v, k), k) * (1.0 - math.cos(theta)))


def _aabb_overlap(a_min, a_max, b_min, b_max, margin=0.5) -> bool:
    return bool(np.all(a_max + margin > b_min)
                and np.all(b_max + margin > a_min))


def sweep_test(candidate: dict, parts: list[dict], catalog: dict,
               groups: list[dict]) -> dict:
    """Rotate the child-side subtree about the candidate axis.

    Starts at +-SWEEP_START_DEG and expands outward; the first new
    bounding-box overlap (beyond assembly-time contacts) estimates the
    motion range in each direction.
    """
    by_uid = {p["uid"]: p for p in parts}
    members = {g["id"]: set(g["uids"]) for g in groups}
    ga, gb = candidate["groups"]
    adj: dict[str, set[str]] = {}
    for g in groups:
        adj[g["id"]] = set()
    for g in groups:
        for uid in g["uids"]:
            for conn in geom.find_connections(uid, parts, catalog):
                other = next((h for h in groups
                              if conn["uid"] in h["uids"]), None)
                if other is not None and other["id"] != g["id"]:
                    adj[g["id"]].add(other["id"])
    # Child side: BFS from gb without passing through ga.
    child_groups = {gb}
    frontier = [gb]
    while frontier:
        cur = frontier.pop()
        for nxt in adj[cur]:
            if nxt != ga and nxt not in child_groups:
                child_groups.add(nxt)
                frontier.append(nxt)
    child_uids = set()
    for gid in child_groups:
        child_uids |= members[gid]
    axis_pt = np.asarray(candidate["fit"]["point_mm"], dtype=float)
    axis_dir = np.asarray(candidate["fit"]["axis"], dtype=float)

    corners = {}
    for uid in child_uids:
        p = np.asarray(by_uid[uid]["position_mm"], dtype=float)
        half = np.array([4.0, 4.0, 4.0])  # coarse part box; refined later
        grid = np.array(np.meshgrid(*[[-h, h] for h in half])).T.reshape(-1, 3)
        corners[uid] = p + grid
    static_boxes = []
    for part in parts:
        if part["uid"] in child_uids:
            continue
        p = np.asarray(part["position_mm"], dtype=float)
        static_boxes.append((part["uid"], p - 4.0, p + 4.0))

    base_hits = set()
    for uid, pts in corners.items():
        for sid, smin, smax in static_boxes:
            if _aabb_overlap(pts.min(axis=0), pts.max(axis=0), smin, smax):
                base_hits.add((uid, sid))

    result = {}
    for sign in (1, -1):
        angle = SWEEP_START_DEG
        first_hit = None
        while angle <= SWEEP_MAX_DEG:
            hit = None
            for uid, pts in corners.items():
                moved = _rotate(pts, axis_pt, axis_dir, sign * angle)
                for sid, smin, smax in static_boxes:
                    if (uid, sid) in base_hits:
                        continue
                    if _aabb_overlap(moved.min(axis=0), moved.max(axis=0),
                                     smin, smax):
                        hit = sid
                        break
                if hit is not None:
                    break
            if hit is not None:
                first_hit = {"angle_deg": angle, "hit_uid": hit}
                break
            angle += SWEEP_STEP_DEG
        result["pos" if sign > 0 else "neg"] = first_hit
    result["range_note"] = (
        f"+{result['pos']['angle_deg']}deg" if result["pos"] else "+180deg+",
        f"-{result['neg']['angle_deg']}deg" if result["neg"] else "-180deg+")
    return result


def check_kinematic_consistency(doc: dict, motor_claims: dict) -> dict:
    """Validate joints vs motor paths and base-to-tip chain.

    `motor_claims`: motor uid -> joint id it drives. Returns
    {"errors": [...], "gaps": [...]} sentence lists. Errors are
    selection-fixable (retried): a motor path must land on a selected
    joint. Gaps record CAD physics the model does not capture (kept,
    not retried): joints without drive paths, shared drives, and chain
    breaks (the base lattice and the arm share no candidate interface,
    so no selection can chain base to tip).
    """
    errors: list[str] = []
    gaps: list[str] = []
    joints = doc.get("joints", [])
    claimed = [j["id"] for j in joints
               if j.get("status", "selected") == "selected"]
    by_joint: dict[str, list[str]] = {}
    for motor, jid in motor_claims.items():
        by_joint.setdefault(jid, []).append(motor)
    for jid in claimed:
        motors = by_joint.get(jid, [])
        if not motors:
            gaps.append(
                f"Joint {jid} is reached by no motor path.")
        elif len(motors) > 1:
            gaps.append(
                f"Joint {jid} is claimed by {len(motors)} motors "
                f"({', '.join(sorted(motors))}); expected exactly one.")
    for motor, jid in motor_claims.items():
        if jid not in claimed:
            errors.append(
                f"Motor {motor} drives {jid}, which is not a selected "
                f"joint.")
    seen: dict[str, list[str]] = {}
    for motor, jid in motor_claims.items():
        seen.setdefault(motor, []).append(jid)
    for motor, jids in seen.items():
        if len(set(jids)) > 1:
            gaps.append(
                f"Motor {motor} drives {len(set(jids))} joints "
                f"({', '.join(sorted(set(jids)))}); expected exactly one.")
    # Base-to-tip chain through parent/child links.
    links = {link["id"] for link in doc.get("links", [])}
    base = (doc.get("base") or {}).get("link")
    ee = (doc.get("end_effector") or {}).get("link")
    adj: dict[str, set[str]] = {}
    for j in joints:
        if j.get("status", "selected") != "selected":
            continue
        adj.setdefault(j["parent"], set()).add(j["child"])
        adj.setdefault(j["child"], set()).add(j["parent"])
    reached = {base} if base else set()
    frontier = [base] if base else []
    while frontier:
        cur = frontier.pop()
        for nxt in adj.get(cur, set()):
            if nxt not in reached:
                reached.add(nxt)
                frontier.append(nxt)
    joint_links = set(adj)
    missing = sorted(joint_links - reached - {None})
    if missing:
        gaps.append(
            f"Chain from base {base} does not reach link(s) "
            f"{', '.join(missing)}.")
    if ee is not None and ee not in reached:
        gaps.append(
            f"End effector on {ee} is not connected to base {base}.")
    return {"errors": errors, "gaps": gaps}
