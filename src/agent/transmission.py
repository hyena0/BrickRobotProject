"""Stage 3.2 topology builder (agent interpretation layer).

Fallback split: links/joints are human-provided (div file); this
module decides motor paths, active meshes, carriers and explanations
from tool facts only. Ground truth is never read.

Method per Stage 3.2 steps:
1. gear_graph nodes/edges; motor output nodes from motor_drive.
2. Joint gears: toothed gears in each joint's child link coaxial
   with the joint axis (turntable halves, coaxial pinions).
3. BFS from each motor drive node to every joint gear over mesh
   edges; active meshes are edges on motor-to-joint paths, the rest
   are inactive/decorative.
4. Special mechanisms reported, not hidden: differentials (no teeth
   in catalog), hollow through-shafts, one pinion driving two gears.
5. Carriers from mounting links with axis-dot coefficients.
"""

from __future__ import annotations

import json
from collections import deque

import numpy as np

from src.tools import geometry as geom
from src.tools.transmission import (_build_matrices, _is_gear, _mesh_sign,
                                    axle_carrier, gear_graph,
                                    solve_transmission)


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def joint_gears(joint: dict, parts: list[dict], catalog: dict,
                group_of: dict) -> list[dict]:
    """Toothed gears in the child link coaxial with the joint axis."""
    by_uid = {p["uid"]: p for p in parts}
    axis = _unit(np.asarray(joint["axis"], dtype=float))
    point = np.asarray(joint["point_mm"], dtype=float)
    out = []
    for p in parts:
        if group_of.get(p["uid"]) != joint["child"]:
            continue
        teeth = (catalog.get(p["part_id"]) or {}).get("teeth")
        if teeth is None:
            continue
        a = geom.axis_world(p, catalog)
        if a is None:
            continue
        a = _unit(np.asarray(a, dtype=float))
        if abs(float(np.dot(a, axis))) < 0.99:
            continue
        w = np.asarray(p["position_mm"], dtype=float) - point
        cross = np.cross(a, axis)
        n = float(np.linalg.norm(cross))
        # Near-parallel axes (quantized rotations leave ~1e-4 rad
        # residual tilt) use lateral offset; the skew formula's
        # direction is numerical noise there (e.g. J1's 382: true
        # 64 mm off-axis once read as 1.11 mm).
        dist = (float(np.linalg.norm(w - float(np.dot(w, a)) * a))
                if n < 1e-3 else abs(float(np.dot(w, cross / n))))
        if dist > 15.0:
            continue
        out.append({"uid": p["uid"], "part_id": p["part_id"],
                    "teeth": teeth, "line_dist_mm": round(dist, 2)})
    out.sort(key=lambda e: e["line_dist_mm"])
    return out


def _map_turntable_halves(parts: list[dict], catalog: dict,
                            div: dict) -> dict[str, str]:
    """Map turntable-half uid -> joint id by coaxiality.

    Same geometry as joint_gears but both links (halves bolt to
    either side). Nearest joint wins; ties keep the first.
    """
    import numpy as _np
    from src.tools import geometry as _geom
    by_uid = {p["uid"]: p for p in parts}
    mapping: dict[str, str] = {}
    best: dict[str, float] = {}
    for joint in div["joints"]:
        axis = joint["axis"]
        ax = _np.asarray(axis, dtype=float)
        ax = ax / _np.linalg.norm(ax)
        point = _np.asarray(joint["point_mm"], dtype=float)
        for p in parts:
            entry = catalog.get(p["part_id"]) or {}
            if entry.get("gear_kind") != "turntable":
                continue
            if entry.get("teeth") is None:
                continue
            a = _geom.axis_world(p, catalog)
            if a is None:
                continue
            a = _np.asarray(a, dtype=float)
            a = a / _np.linalg.norm(a)
            if abs(float(_np.dot(a, ax))) < 0.99:
                continue
            w = _np.asarray(p["position_mm"], dtype=float) - point
            cross = _np.cross(a, ax)
            n = float(_np.linalg.norm(cross))
            dist = (float(_np.linalg.norm(w - float(_np.dot(w, a))
                                          * a))
                    if n < 1e-3
                    else abs(float(_np.dot(w, cross / n))))
            if dist > 15.0:
                continue
            if p["uid"] not in best or dist < best[p["uid"]]:
                best[p["uid"]] = dist
                mapping[p["uid"]] = joint["id"]
    return mapping


def _keep_turntable_edge(edge: dict, ka: str | None, kb: str | None,
                         group_of: dict, half_joint: dict,
                         div: dict) -> bool:
    """Keep pinion-half edges that drive the OUTPUT half.

    The half must sit on its joint's child link; the pinion side
    is unrestricted (parent- and child-mounted pinions alike).
    Halves mapped to no joint keep the old cross-link fallback.
    """
    children = {j["id"]: j["child"] for j in div["joints"]}
    for hid, kid in ((edge["gear_a"], edge["gear_b"]),
                     (edge["gear_b"], edge["gear_a"])):
        hkind = ka if hid == edge["gear_a"] else kb
        if hkind == "turntable" and hid in half_joint:
            return group_of.get(hid) == children[half_joint[hid]]
    for hid, kid in ((edge["gear_a"], edge["gear_b"]),
                     (edge["gear_b"], edge["gear_a"])):
        hkind = ka if hid == edge["gear_a"] else kb
        if hkind != "turntable" and \
                group_of.get(hid) != group_of.get(kid):
            return True
    return False


def _path_product(path: list[dict], root: str | None) -> dict:
    """Exact static-carrier tooth product along a BFS path.

    Edges are node-oriented (a/ta, b/tb), but traversal may run
    b->a, so each stage uses the teeth in traversal order: -ta/tb
    or -tb/ta (external/bevel/crown/knob, all external-like here),
    +ta/tb or +tb/ta (internal, same direction), 1.0 (compound).
    Empty paths (motor seed on a joint-gear node: hollow
    through-shaft) are shared axles, not drives. Valid with
    carriers held fixed; absolute coupling differs when carriers
    move.
    """
    if not path:
        return {"product": 1.0, "sign": 1, "stages": 0,
                "note": "hollow/shared-node, not a drive"}
    # Root = the BFS seed end (prev None there).
    cur = root if root is not None else path[0]["a"]
    ratio = 1.0
    for e in path:
        if e["a"] == cur:
            ta, tb = float(e["ta"]), float(e["tb"])
            cur = e["b"]
        elif e["b"] == cur:
            ta, tb = float(e["tb"]), float(e["ta"])
            cur = e["a"]
        else:
            return {"product": ratio, "sign": 1 if ratio >= 0 else -1,
                    "stages": len(path), "note": "broken chain"}
        kind = e.get("kind", "external")
        if kind == "compound":
            continue
        if kind == "internal":
            ratio *= ta / tb
        else:
            ratio *= -ta / tb
    return {"product": ratio, "sign": 1 if ratio >= 0 else -1,
            "stages": len(path)}


def build_topology(parts: list[dict], catalog: dict,
                   div: dict) -> tuple[dict, dict]:
    """Derive the transmission topology. Returns (topology, report)."""
    # Virtual J1 slew ring (D1 output, rigid on the child link).
    # Evidence: 8 curved-rack segments (24121) form two stacked rings
    # (D0 stator z=-152, D1 rotor z=-143) about J1's axis; Blender
    # slew_rings render shows inward teeth; tooth-band measurement
    # gives 35 teeth/segment = T140, pitch R 70 mm, and the 522
    # pinion at radial 64.05 coplanar with the stator (internal mesh
    # (140-12)/2 = 64 teeth-units exact). Builder-local catalog copy
    # so the reviewed parts.yaml stays untouched.
    parts = list(parts)
    catalog = dict(catalog)
    catalog["RING140"] = {"category": "gear", "teeth": 140,
                          "gear_kind": "ring", "axis_local": [0, 0, 1],
                          "holes": []}
    j1 = next(j for j in div["joints"] if j["id"] == "J1")
    ring_pos = [float(j1["point_mm"][0]), float(j1["point_mm"][1]),
                -143.0]
    by_uid522 = {p["uid"]: p for p in parts}["p000522"]
    parts.append({"uid": "RING_D1", "part_id": "RING140",
                  "position_mm": ring_pos,
                  "rotation": [list(r) for r in
                               by_uid522["rotation"]]})
    by_uid = {p["uid"]: p for p in parts}
    group_of = {}
    for link in div["links"]:
        for uid in link["members"]:
            group_of[uid] = link["id"]
    group_of["RING_D1"] = "D1"
    graph = gear_graph(parts, catalog, link_of=group_of)
    joints = div["joints"]
    jaxis = {j["id"]: _unit(np.asarray(j["axis"], dtype=float))
             for j in joints}
    order = [j["id"] for j in joints]  # serial D0->.. chain order
    upstream = {jid: order[:i] for i, jid in enumerate(order)}

    node_of_gear: dict[str, str] = {}
    for node in graph["nodes"]:
        for g in node["gears"]:
            node_of_gear[g["uid"]] = node["id"]

    def gear_kind_of(uid: str) -> str | None:
        return (catalog.get(by_uid[uid]["part_id"]) or {}).get("gear_kind")

    # Map each turntable half to its joint (coaxial, either link).
    half_joint = _map_turntable_halves(parts, catalog, div)
    # Dual-half cleanup FIRST (before BFS): a pinion beside a mated
    # turntable pair is within mesh distance of BOTH halves, but it
    # engages only one. Kept is the OUTPUT half (child link): it is
    # the informative constraint (determines the joint); the
    # parent-half edge is redundant (parent already moves) or false.
    # This subsumes the old cross-link heuristic (identical when the
    # pinion sits on the parent) and fixes child-mounted pinions
    # (J5's 096/095: old rule drove the parent half and orphaned
    # the output). Unmapped halves keep the cross-link fallback.
    kept = []
    dropped = []
    for e in graph["edges"]:
        ka, kb = gear_kind_of(e["gear_a"]), gear_kind_of(e["gear_b"])
        if "turntable" not in {ka, kb}:
            kept.append(e)
            continue
        ok = _keep_turntable_edge(e, ka, kb, group_of, half_joint,
                                  div)
        (kept if ok else dropped).append(e)

    # Documented sub-tolerance manual edges (never silent): each has
    # visual or metrological proof recorded in its note and is
    # reported under manual_edges. Currently one: 812x802 (both
    # 94925 16T spur) at 42.42 vs 40 LDU. Blender mesh812_802 render
    # shows teeth interleaved; both parts nominal module (94925 tip
    # 17.74 ~= 18.0 measured); no alternative mate for either gear;
    # widening the global tol is unsafe (audit: 211x235 at 2.06 would
    # falsely bridge J2 to the 149 train). Physical re-seat check
    # still wanted.
    manual_edges = []
    if "p000812" in node_of_gear and "p000802" in node_of_gear:
        e812 = {"a": node_of_gear["p000812"],
                "b": node_of_gear["p000802"],
                "gear_a": "p000812", "gear_b": "p000802",
                "ta": 16, "tb": 16, "kind": "external",
                "axis_relation": "parallel", "centre_mm": 16.966,
                "note": "manual sub-tolerance visual"}
        kept.append(e812)
        manual_edges.append({"edge": ["p000812", "p000802"],
                             "kind": "external",
                             "note": e812["note"]})

    adj: dict[str, list[dict]] = {n["id"]: [] for n in graph["nodes"]}
    for e in kept:
        adj[e["a"]].append({"to": e["b"], "edge": e})
        adj[e["b"]].append({"to": e["a"], "edge": e})

    # Joint gears + their nodes (redundant same-link gears verified).
    joint_nodes: dict[str, list[str]] = {}
    jgear_report: dict[str, list[dict]] = {}
    for joint in joints:
        gears = joint_gears(joint, parts, catalog, group_of)
        jgear_report[joint["id"]] = gears
        nodes = []
        for g in gears:
            node = node_of_gear.get(g["uid"])
            if node is not None and node not in nodes:
                nodes.append(node)
        joint_nodes[joint["id"]] = nodes

    # BFS seeds per motor: the drive node plus pinion nodes. A motor
    # drives through the axle parts on its shaft line, so any node
    # holding an axle member collinear with that line (angle and
    # lateral offset within bore tolerance) belongs to the output
    # shaft, wherever along it. Unbounded span: real shafts are long.
    motor_seeds: dict[str, list[str]] = {}
    seed_report: dict[str, list] = {}
    seed_axles: dict[str, dict[str, str]] = {}
    for motor, start in (graph.get("motor_drive") or {}).items():
        # Collinear-axle evidence first (solver drives seeds[0]);
        # the peer vote is only a fallback when no shaft is found.
        seeds: list[str] = []
        maxis = _unit(np.asarray(geom.axis_world(by_uid[motor],
                                                 catalog), dtype=float))
        mpos = np.asarray(by_uid[motor]["position_mm"], dtype=float)
        cands = []
        for node in graph["nodes"]:
            for uid in node["uids"]:
                if (catalog.get(by_uid[uid]["part_id"]) or {}
                        ).get("category") != "axle":
                    continue
                ax = geom.axis_world(by_uid[uid], catalog)
                if ax is None:
                    continue
                ax = _unit(np.asarray(ax, dtype=float))
                dot = float(np.clip(np.dot(maxis, ax), -1.0, 1.0))
                ang = min(float(np.degrees(np.arccos(dot))),
                          180.0 - float(np.degrees(np.arccos(dot))))
                rel = np.asarray(by_uid[uid]["position_mm"]) - mpos
                lat = float(np.linalg.norm(rel - float(np.dot(rel, maxis))
                                           * maxis))
                along = float(np.dot(rel, maxis))
                if ang <= 2.0 and lat <= 2.0:
                    gears = [g["uid"] for g in node["gears"]]
                    cands.append((abs(along), node["id"], uid, gears))
        cands.sort()
        seed_axle: dict[str, str] = {}
        for dist, node_id, uid, gears in cands[:4]:
            if node_id not in seeds:
                seeds.append(node_id)
            seed_axle.setdefault(node_id, uid)
        if start not in seeds:
            seeds.append(start)
        seed_report[motor] = [(round(d, 1), n, u, g)
                              for d, n, u, g in cands[:4]]
        motor_seeds[motor] = seeds
        seed_axles[motor] = seed_axle

    # BFS motor seeds -> joint-gear nodes over kept edges.
    motor_paths: dict[str, dict[str, list[dict]]] = {}
    motor_roots: dict[str, dict[str, str | None]] = {}
    for motor, seeds in motor_seeds.items():
        prev: dict[str, dict | None] = {s: None for s in seeds}
        queue = deque(seeds)
        while queue:
            cur = queue.popleft()
            for leg in adj.get(cur, []):
                if leg["to"] not in prev:
                    prev[leg["to"]] = {"from": cur, "edge": leg["edge"]}
                    queue.append(leg["to"])
        reached: dict[str, list[dict]] = {}
        reached_roots: dict[str, str | None] = {}
        for jid, nodes in joint_nodes.items():
            for target in nodes:
                if target not in prev:
                    continue
                path, cur = [], target
                while prev[cur] is not None:
                    path.append(prev[cur]["edge"])
                    cur = prev[cur]["from"]
                path.reverse()
                if jid not in reached or len(path) < len(reached[jid]):
                    reached[jid] = path
                    reached_roots[jid] = cur
        motor_paths[motor] = reached
        motor_roots[motor] = reached_roots

    # Active meshes: union of kept edges on motor-to-joint paths.
    active_keys = set()
    for reached in motor_paths.values():
        for path in reached.values():
            for e in path:
                active_keys.add((e["a"], e["b"], e["gear_a"],
                                 e["gear_b"]))
    active = [e for e in kept
              if (e["a"], e["b"], e["gear_a"], e["gear_b"]) in active_keys]
    active += [e for e in kept
               if (e["b"], e["a"], e["gear_b"], e["gear_a"]) in active_keys
               and e not in active]

    used_gears = {u for e in active for u in (e["gear_a"], e["gear_b"])}
    inactive = sorted(
        {g["uid"] for n in graph["nodes"] for g in n["gears"]}
        - used_gears)

    # Carriers: mounting link of the mesh by majority vote, with
    # axis-dot coefficients per side for non-parallel chains.
    link_index = {link["id"]: i for i, link in enumerate(div["links"])}
    node_uids = {n["id"]: n["uids"] for n in graph["nodes"]}
    links = div["links"]

    def mounting(uids: list[str]) -> str | None:
        return axle_carrier(uids, links)

    meshes = []
    mesh_report = []
    for e in active:
        auids = node_uids.get(e["a"], [])
        buids = node_uids.get(e["b"], [])
        ca_link = mounting(auids + buids)
        aa = _unit(np.asarray(geom.axis_world(by_uid[e["gear_a"]],
                                              catalog), dtype=float))
        ab = _unit(np.asarray(geom.axis_world(by_uid[e["gear_b"]],
                                              catalog), dtype=float))
        up = upstream_of_link(ca_link, joints, order) if ca_link else []
        carrier_a = {jid: round(float(np.dot(jaxis[jid], aa)), 3)
                     for jid in up
                     if abs(float(np.dot(jaxis[jid], aa))) > 1e-9}
        carrier_b = {jid: round(float(np.dot(jaxis[jid], ab)), 3)
                     for jid in up
                     if abs(float(np.dot(jaxis[jid], ab))) > 1e-9}
        meshes.append({"a": e["a"], "ta": e["ta"], "b": e["b"],
                       "tb": e["tb"], "kind": e["kind"],
                       "carrier_a": carrier_a, "carrier_b": carrier_b})
        mesh_report.append({"edge": [e["gear_a"], e["gear_b"]],
                            "kind": e["kind"],
                            "carrier_link": ca_link,
                            "carrier_a": carrier_a,
                            "carrier_b": carrier_b,
                            "note": e.get("note", "")})

    # Joint attachments: primary joint-gear node + parent projection.
    topology_joints = []
    for joint in joints:
        nodes = joint_nodes[joint["id"]]
        if not nodes:
            raise ValueError("no joint-gear node for %s" % joint["id"])
        primary = nodes[0]
        parent_link = joint["parent"]
        up = upstream_of_link(parent_link, joints, order)
        ja = jaxis[joint["id"]]
        coeffs = {jid: round(float(np.dot(jaxis[jid], ja)), 3)
                  for jid in up
                  if abs(float(np.dot(jaxis[jid], ja))) > 1e-9}
        topology_joints.append({"id": joint["id"], "axle": primary,
                                "carrier_joint": coeffs})

    # Solver motors attach at the seed nodes (collinear-axle shaft
    # evidence), not the motor_drive peer vote: BFS and solve must
    # drive the same axles. One motor drives its first seed; extra
    # seeds are the same physical shaft split by grouping, so unit
    # compound links constrain them equal (solver-only, not meshes).
    motors = [{"id": m, "axle": seeds[0]}
              for m, seeds in motor_seeds.items()]
    unity = []
    for m, seeds in motor_seeds.items():
        axles = seed_axles.get(m, {})
        for node in seeds[1:]:
            if node in axles and seeds[0] in axles:
                unity.append({"a": seeds[0], "ta": 1, "b": node,
                              "tb": 1, "kind": "compound",
                              "gear_a": axles[seeds[0]],
                              "gear_b": axles[node]})
    # Solver-only virtual constraints (never mesh_report/active):
    # the 6573 wrist differential. Evidence: the +side shaft line
    # (807/088/083 colinear through the housing center along J4's
    # axis, 086 press-fit outboard) and the D3-mounted housing
    # (781 = 6573.dat in D3, crown grounded with it). Blender
    # wrist_diff renders show the 743 pinion 780 (32270.dat, 12T
    # double-bevel, measured) engaging the tan 16T side gear, not
    # the crown: 780 drives the -side, 086 rides the +side shaft.
    # Catalog title of 6573.dat is "24-16 Tooth" (24T crown,
    # 16T side gears); grounded equal-side Willis gives a 1:1
    # counter-rotation, so with the 12T input pinion the lock is
    # 16 w_plus = +12 w_780 (same direction: internal form).
    # Carriers mirror the mesh loop over D3 upstream (~0: the side
    # line is perpendicular to J1/J2/J3 at the model pose, so the
    # housing is effectively grounded and independent 736/743
    # commands are infeasible - coordination is required).
    virtual = []
    # Node ids renumber whenever the graph changes; resolve the diff
    # sides by gear uid (086 = +side shaft knob, 780 = -side pinion).
    plus = node_of_gear.get("p000086")
    minus = node_of_gear.get("p000780")
    if plus is not None and minus is not None and \
            motor_seeds.get("p000736", [None])[0] == plus and \
            motor_seeds.get("p000743", [None])[0] == minus:
        side = _unit(np.asarray(geom.axis_world(
            by_uid["p000088"], catalog), dtype=float))
        up = upstream_of_link("D3", joints, order)
        vh = {jid: round(float(np.dot(jaxis[jid], side)), 3)
              for jid in up
              if abs(float(np.dot(jaxis[jid], side))) > 1e-9}
        virtual.append({"a": plus, "ta": 16, "b": minus, "tb": 12,
                        "kind": "internal", "carrier_a": vh,
                        "carrier_b": vh,
                        "note": "6573 diff (virtual)"})
    # Solver-only J1 stator-ring walk (never mesh_report/active):
    # the 522 pinion (vertical axis, coplanar with the D0 ring at
    # z=-152, radial 64.05 = internal pitch (140-12)/2 exact) walks
    # inside the grounded T140 ring. Ground form (b None): the ring
    # is D0/base-fixed, so only its carrier term survives; both
    # carriers are J1's revolution projected on the vertical axis.
    # Hypothesis-tests the live-shaft reading (522 spins on the
    # 521 shaft driven via 382): consistent residual keeps it, a
    # spike reverts it. Ring T140 measured (35 teeth/segment x4).
    if "p000522" in node_of_gear:
        # Both axes vertical (522 pinion, ring): J1 projects 1.0.
        ring_edge = {"a": node_of_gear["p000522"], "ta": 12,
                     "b": None, "tb": 140, "kind": "internal",
                     "carrier_a": {"J1": 1.0},
                     "carrier_b": {"J1": 1.0},
                     "note": "J1 ring walk (virtual)"}
        virtual.append(ring_edge)
    # J2 is held by locked motor 149 through the user-confirmed
    # 149->J2 mapping (2026-10-28): the holding torque reaches J2
    # through the physical path plus the engaged 169/171/205/207
    # half-pinions, but that path is void in CAD, so the solver
    # would let J2 back-drive in every column without this hold.
    # J2's own column stays void-zero (no CAD drive); its ratio and
    # sign come from human fallback (§4), verified blind against the
    # model-derived train inference (disposition B).
    topology = {"motors": motors, "joints": topology_joints,
                "meshes": meshes + unity + virtual,
                "held_joints": ["J2"]}
    result = solve_transmission(topology)
    report = {
        "joint_gears": jgear_report,
        "motor_seeds": motor_seeds,
        "seed_report": seed_report,
        "motor_paths": {
            m: {jid: [(e["gear_a"], e["gear_b"], e["kind"])
                      for e in path]
                for jid, path in reached.items()}
            for m, reached in motor_paths.items()},
        "motor_joints": {m: sorted(reached)
                         for m, reached in motor_paths.items()},
        "path_ratios": {
            m: {jid: _path_product(path, motor_roots[m].get(jid))
                for jid, path in reached.items()}
            for m, reached in motor_paths.items()},
        "n_nodes": len(graph["nodes"]),
        "n_edges": len(graph["edges"]),
        "intra_meshes": graph.get("intra_meshes", []),
        "n_active_meshes": len(active),
        "n_dropped_half_edges": len(dropped),
        "n_inactive_gears": len(inactive),
        "inactive_gears": inactive[:20],
        "mesh_report": mesh_report,
        "virtual_constraints": virtual,
        "unity_constraints": unity,
        "manual_edges": manual_edges,
        "rank_ok": result["rank_ok"],
        "rank": result["rank"],
        "n_unknowns": result["n_unknowns"],
        "residual": result["residual"],
        "C": result["C"],
        "joint_ids": result["joint_ids"],
        "motor_ids": result["motor_ids"],
        "ratios": result["ratios"],
        "signs": result["signs"],
        "coordination": _coordination_report(topology),
        "confirmed_mapping": confirmed_mapping(),
        "special_notes": [
            "6573 wrist differential (24-16T per catalog title, "
            "780 = 12T input pinion measured) locks 736/743 "
            "with a grounded carrier at the model pose; "
            "independent commands are infeasible, see "
            "coordination for the lock law and exact response.",
            "32187 ring extension (no teeth) excluded from the graph.",
            "J2/J3/J5 use dual 60T turntable pairs (load sharing); "
            "one half each chosen as the joint axle.",
        ],
    }
    return topology, report


def confirmed_mapping() -> list[dict]:
    """User-confirmed motor->joint assignments (human-provided).

    Follows the manual_edges precedent: documented human input that
    the graph evidence cannot fully derive. 148->J3 is consistent
    with the traced 3-stage train; 149->J2 is confirmed but has no
    CAD path (J2 input void), so no ratio is modeled for it. Never
    touches the solver inputs: C, ratios and rank are unchanged.
    """
    return [
        {"motor": "p000148", "joint": "J3",
         "basis": "graph path + user-confirmed 2026-10-28"},
        {"motor": "p000149", "joint": "J2",
         "basis": "user-confirmed 2026-10-28; no CAD path (J2 void)"},
    ]


def _coordination_report(topology: dict) -> dict | None:
    """Feasible coordinated drive for a locked motor pair.

    A grounded-carrier differential locks two motor shafts, so the
    independent single-motor columns of C are infeasible command
    patterns and their least-squares entries are a compromise, not
    drive ratios. This reports the lock law plus the exact
    free-joint response to the coordinated pattern (+side at 1.0,
    -side slaved by the lock), which must solve to ~machine
    precision when the lock is the only infeasibility. Returns
    None when no lock virtual is present.
    """
    meshes = topology.get("meshes") or []
    lock = next((m for m in meshes
                 if "diff" in str(m.get("note", ""))), None)
    if lock is None or lock.get("b") is None:
        return None
    ta, tb = float(lock["ta"]), float(lock["tb"])
    s = _mesh_sign(lock.get("kind", "external"), lock.get("s"))
    ratio = ta / (s * tb)  # w_minus per unit w_plus
    motors = topology.get("motors") or []
    m_plus = next((m["id"] for m in motors
                   if m["axle"] == lock["a"]), None)
    m_minus = next((m["id"] for m in motors
                    if m["axle"] == lock["b"]), None)
    if m_plus is None or m_minus is None:
        return None
    mat = _build_matrices(topology)
    E = mat["E"]
    mrow = {m["id"]: i for i, m in enumerate(motors)}
    f = np.zeros(E.shape[0])
    f[mrow[m_plus]] = 1.0
    f[mrow[m_minus]] = ratio
    X, *_ = np.linalg.lstsq(E, f, rcond=None)
    n_ax = mat["n_ax"]
    return {
        "lock": f"{ta:g}*w_{m_plus} {-s * tb:+g}*w_{m_minus} = 0"
                " (model pose)",
        "pattern": {m_plus: 1.0, m_minus: ratio},
        "residual": float(np.abs(E @ X - f).max()),
        "response": {
            jid: float(X[n_ax + i])
            for i, jid in enumerate(mat["joint_ids"])
        },
    }


def upstream_of_link(link_id: str | None, joints: list[dict],
                     order: list[str]) -> list[str]:
    """Joint ids from base up to (not including) the link's own joint."""
    if link_id is None:
        return []
    # Serial chain: link Dk is positioned by joints J1..Jk.
    try:
        k = int(link_id[1:])
    except (ValueError, IndexError):
        return []
    return [jid for i, jid in enumerate(order) if i < k]
