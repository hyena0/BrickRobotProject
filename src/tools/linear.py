"""Linear (prismatic) joint recognition (Stage 7 T.3, additive).

Rack+pinion and chain+sprocket live here. Nothing in the revolute
path is modified: new functions, new catalog fields, new joint type.

Conventions (repo's own):
- spur pitch diameter 2.5 LDU/tooth (mesh rule: centre 1.25*(T1+T2)).
- rack pitch 8 LDU = 3.2 mm (measured T.1b: 18942, 32 teeth at 8.0 LDU).
- mesh tolerance +-2 LDU; parallel axes within 5 deg.
"""

from __future__ import annotations

import math

import numpy as np

LDU_TO_MM = 0.4
RACK_PITCH_LDU = 8.0
RACK_PITCH_MM = RACK_PITCH_LDU * LDU_TO_MM
MESH_TOL_LDU = 2.0
PARALLEL_DEG = 5.0
CHAIN_CATEGORIES = ("chain_link",)
# Chain-set part types: articulating links/treads/pins. `other` is the
# catalog-neutral category assigned in T.1; chain membership here is by
# explicit part list so unrelated `other` parts never join a loop.
CHAIN_PARTS = frozenset({"3711.dat", "24375.dat", "3873.dat"})


def _rot(part: dict) -> np.ndarray:
    return np.asarray(part["rotation"], dtype=float)


def travel_world(part: dict, entry: dict) -> np.ndarray | None:
    """Rack travel direction in world frame, or None.

    Uses the catalog `linear_axis` (part-local LDraw direction), converted
    with the same rotation the parser guarantees for poses.
    """
    axis = entry.get("linear_axis")
    if axis is None:
        return None
    return _rot(part) @ np.asarray(axis, dtype=float)


def check_rack_mesh(pinion_uid: str, rack_uid: str, parts: list[dict],
                    catalog: dict,
                    tol_ldu: float = MESH_TOL_LDU) -> dict:
    """Rack+pinion mesh verdict. Returns dict with `meshes: bool`.

    Conditions: pinion is a spur gear; rack carries `linear_axis` and a
    `tooth_line_offset` (part-local LDU vector from origin to the tooth
    pitch line); pinion axis perpendicular to travel; pinion centre at
    pitch radius (1.25*T LDU) from the tooth line within tolerance.
    Missing tooth-line data -> meshes False with a note (never guessed).
    """
    by_uid = {p["uid"]: p for p in parts}
    pin, rack = by_uid[pinion_uid], by_uid[rack_uid]
    pent = catalog.get(pin["part_id"]) or {}
    rent = catalog.get(rack["part_id"]) or {}
    no = {"a": pinion_uid, "b": rack_uid, "meshes": False}
    if pent.get("category") != "gear" or pent.get("gear_kind") != "spur":
        return {**no, "kind": None, "note": "pinion not a spur gear"}
    if rent.get("category") != "rack" or rent.get("linear_axis") is None:
        return {**no, "kind": None, "note": "rack has no travel axis"}
    off = rent.get("tooth_line_offset")
    if off is None:
        return {**no, "kind": None, "note": "no tooth line data"}
    teeth = pent.get("teeth") or 0
    travel = travel_world(rack, rent)
    if travel is None:
        return {**no, "kind": None, "note": "no travel axis"}
    from src.tools import geometry as geom
    axis = geom.axis_world(pin, catalog)
    if axis is None:
        return {**no, "kind": None, "note": "pinion axis unknown"}
    dot = float(np.dot(axis, travel))
    ang = min(math.degrees(math.acos(max(-1.0, min(1.0, dot)))),
              180.0 - math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
    if abs(ang - 90.0) > PARALLEL_DEG:
        return {**no, "kind": None,
                "note": f"pinion axis {ang:.1f}deg off perpendicular"}
    r_pitch = 1.25 * teeth
    rp = np.asarray(rack["position_mm"]) / LDU_TO_MM
    tooth_pt = rp + _rot(rack) @ np.asarray(off, dtype=float)
    rel = np.asarray(pin["position_mm"]) / LDU_TO_MM - tooth_pt
    t = travel / np.linalg.norm(travel)
    dist = float(np.linalg.norm(rel - float(np.dot(rel, t)) * t))
    ok = abs(dist - r_pitch) <= tol_ldu
    travel_mm = teeth * RACK_PITCH_MM if ok else 0.0
    return {"a": pinion_uid, "b": rack_uid, "meshes": ok, "kind": "rack",
            "axis_relation": "perpendicular",
            "centre_ldu": round(dist, 2), "expected_ldu": round(r_pitch, 2),
            "travel_mm_per_rev": round(travel_mm, 3),
            "note": "" if ok else f"{dist:.1f} vs {r_pitch:.1f} LDU"}


def find_rack_meshes(parts: list[dict], catalog: dict) -> list[dict]:
    """All adopted rack+pinion meshes over the model."""
    pinions = [p["uid"] for p in parts
               if (catalog.get(p["part_id"]) or {}).get("category")
               == "gear"]
    racks = [p["uid"] for p in parts
             if (catalog.get(p["part_id"]) or {}).get("category")
             == "rack"]
    out = []
    for r in racks:
        for g in pinions:
            res = check_rack_mesh(g, r, parts, catalog)
            if res["meshes"]:
                out.append(res)
    return out


def _straightest(cur: str, prev: str, candidates: list[str],
                 positions: dict[str, list[float]]) -> list[str]:
    """Branch resolution: continue the straightest tangent (general
    chain-loop following; contact graphs branch where strands touch)."""
    vin = np.asarray(positions[cur]) - np.asarray(positions[prev])
    scored = []
    for cand in candidates:
        vout = np.asarray(positions[cand]) - np.asarray(positions[cur])
        ni, no = np.linalg.norm(vin), np.linalg.norm(vout)
        if ni == 0 or no == 0:
            continue
        cosv = float(np.dot(vin, vout) / (ni * no))
        scored.append((math.degrees(math.acos(max(-1.0, min(1.0,
                                                            cosv)))),
                       cand))
    if not scored:
        return candidates
    best = min(s for s, _ in scored)
    return [c for s, c in scored if abs(s - best) < 1e-6]


def order_loop(adjacency: dict[str, list[str]],
               positions: dict[str, list[float]] | None = None) -> dict:
    """Order a degree-2 contact graph into a loop (pure helper).

    Returns {"order": [...], "closed": bool, "gap": note}. Branches
    (degree != 2) stop the trace with an explicit gap note unless
    `positions` resolves the straightest continuation.
    """
    if not adjacency:
        return {"order": [], "closed": False, "gap": "empty"}
    deg2 = sorted(u for u, ns in adjacency.items() if len(ns) == 2)
    degn = sorted(u for u, ns in adjacency.items() if ns)
    start = (deg2 or degn or sorted(adjacency))[0]
    # Loop starts orient the walk (both neighbours visible otherwise).
    prev = None
    if len(adjacency.get(start, [])) == 2:
        prev = sorted(adjacency[start])[0]
    order, seen, cur = [], {start}, start
    positions = positions or {}
    while True:
        order.append(cur)
        nxt = [n for n in adjacency.get(cur, []) if n != prev]
        if not nxt:
            spurs = sorted({node for node in order
                            for nb in adjacency.get(node, [])
                            if nb not in seen
                            and len(adjacency.get(nb, [])) >= 2})
            if spurs:
                return {"order": order, "closed": False,
                        "gap": f"branch at {spurs[0]}"}
            forks = sorted({nb for node in order
                            for nb in adjacency.get(node, [])
                            if nb not in seen})
            extra = f"; unexplored spurs at {forks}" if forks else ""
            return {"order": order, "closed": False,
                    "gap": f"dead end at {cur}{extra}"}
        if len(nxt) > 1:
            if cur in positions and prev in positions:
                nxt = _straightest(cur, prev, nxt, positions)
                if len(nxt) > 1:
                    return {"order": order, "closed": False,
                            "gap": f"branch at {cur}"}
            else:
                return {"order": order, "closed": False,
                        "gap": f"branch at {cur}"}
        prev, cur = cur, nxt[0]
        if cur == start:
            return {"order": order, "closed": True, "gap": ""}
        if cur in seen:
            return {"order": order, "closed": False,
                    "gap": f"re-entry at {cur}"}
        seen.add(cur)


def trace_chain(parts: list[dict], catalog: dict) -> dict:
    """Trace the drive-chain loop through contact-connected chain parts.

    Returns ordered uids, closed flag, link pitch (median consecutive
    origin distance, mm) and loop length. Chain membership is the
    explicit CHAIN_PARTS list, never the neutral `other` category.
    """
    from src.tools import geometry as geom
    by_uid = {p["uid"]: p for p in parts}
    chain = [p["uid"] for p in parts if p["part_id"] in CHAIN_PARTS]
    adj: dict[str, list[str]] = {u: [] for u in chain}
    inset = set(chain)
    for u in chain:
        try:
            conns = geom.find_connections(u, parts, catalog)
        except Exception:
            continue
        for c in conns:
            v = c.get("uid")
            if v in inset and v != u and v not in adj[u]:
                adj[u].append(v)
    pos = {p["uid"]: p["position_mm"] for p in parts}
    loop = order_loop(adj, {u: pos[u] for u in adj if u in pos})
    pitch, length = 0.0, 0.0
    if len(loop["order"]) >= 2:
        ds = []
        for a, b in zip(loop["order"], loop["order"][1:]):
            ds.append(float(np.linalg.norm(
                np.asarray(by_uid[a]["position_mm"])
                - np.asarray(by_uid[b]["position_mm"]))))
        if loop["closed"]:
            ds.append(float(np.linalg.norm(
                np.asarray(by_uid[loop["order"][-1]]["position_mm"])
                - np.asarray(by_uid[loop["order"][0]]["position_mm"]))))
        ds.sort()
        pitch = float(ds[len(ds) // 2])
        length = float(sum(ds))
    return {"order": loop["order"], "closed": loop["closed"],
            "gap": loop["gap"], "n": len(loop["order"]),
            "link_pitch_mm": round(pitch, 3),
            "loop_length_mm": round(length, 1)}


def chain_sprockets(parts: list[dict], catalog: dict,
                    loop: list[str]) -> list[dict]:
    """Sprocket candidates: gears whose pitch circle meets the loop.

    General rule mirroring the spur-mesh convention: a sprocket's pitch
    radius (1.25*T LDU) should reach the chain path within the repo's
    +-2 LDU mesh tolerance. Gears without teeth data are skipped.
    """
    by_uid = {p["uid"]: p for p in parts}
    path = np.array([by_uid[u]["position_mm"] for u in loop])
    out = []
    for p in parts:
        entry = catalog.get(p["part_id"]) or {}
        if entry.get("category") != "gear":
            continue
        teeth = entry.get("teeth")
        if not teeth:
            continue
        r_pitch_mm = 1.25 * teeth * LDU_TO_MM
        d = float(np.min(np.linalg.norm(
            path - np.asarray(p["position_mm"]), axis=1)))
        gap_ldu = (d - r_pitch_mm) / LDU_TO_MM
        if abs(gap_ldu) <= MESH_TOL_LDU:
            out.append({"uid": p["uid"], "part_id": p["part_id"],
                        "teeth": teeth, "min_dist_mm": round(d, 2),
                        "pitch_radius_mm": round(r_pitch_mm, 2)})
    return sorted(out, key=lambda e: e["min_dist_mm"])


def motor_to_sprocket(parts: list[dict], catalog: dict, graph: dict,
                      motor_uid: str, sprocket_uid: str,
                      link_pitch_mm: float) -> dict:
    """BFS motor node -> sprocket node over gear-graph edges.

    Ratio product reuses the repo's tooth-product rule (external-like
    negative, internal positive, compound 1). Travel uses the measured
    link pitch passed in (trace_chain), never a constant.
    """
    from collections import deque
    from src.agent.transmission import _path_product
    node_of: dict[str, str] = {}
    for n in graph.get("nodes", []):
        for g in n.get("gears", []):
            uid = g.get("uid") if isinstance(g, dict) else g
            node_of[uid] = n["id"]
    adj: dict[str, list[dict]] = {}
    for e in graph.get("edges", []):
        adj.setdefault(e["a"], []).append(e)
        adj.setdefault(e["b"], []).append(e)
    start = (graph.get("motor_drive") or {}).get(motor_uid)
    target = node_of.get(sprocket_uid)
    if start is None:
        return {"ratio": 0.0, "stages": 0, "mm_per_motor_rev": 0.0,
                "note": "motor has no drive node"}
    if target is None:
        return {"ratio": 0.0, "stages": 0, "mm_per_motor_rev": 0.0,
                "note": "sprocket outside gear graph"}
    prev: dict[str, dict | None] = {start: None}
    queue = deque([start])
    while queue:
        cur = queue.popleft()
        if cur == target:
            break
        for e in adj.get(cur, []):
            nxt = e["b"] if e["a"] == cur else e["a"]
            if nxt not in prev:
                prev[nxt] = {"from": cur, "edge": e}
                queue.append(nxt)
    if target not in prev:
        return {"ratio": 0.0, "stages": 0, "mm_per_motor_rev": 0.0,
                "note": "no graph path motor -> sprocket"}
    path, cur = [], target
    while prev[cur] is not None:
        path.append(prev[cur]["edge"])
        cur = prev[cur]["from"]
    path.reverse()
    prod = _path_product(path, cur)
    teeth = None
    for p in parts:
        if p["uid"] == sprocket_uid:
            teeth = (catalog.get(p["part_id"]) or {}).get("teeth")
    mm = abs(float(prod["product"])) * (teeth or 0) * link_pitch_mm
    return {"ratio": float(prod["product"]), "stages": prod["stages"],
            "mm_per_motor_rev": round(mm, 2),
            "note": prod.get("note", "")}
