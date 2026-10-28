"""Transmission tools for Stage 3: gears and coupling.

Tools compute facts. The agent interprets them.

- gear_graph: nodes are axle groups (merged trace_axle lines holding a
  gear or motor shaft), edges are gear meshes (check_gear_mesh) with
  teeth and mesh kind.
- axle_carrier: the link each axle group is mounted in, by majority
  vote of Stage 2 link membership.
- solve_transmission: deterministic Willis-constraint solver. The agent
  supplies the topology (motor -> axle, joint child -> axle, active
  meshes with carriers); the solver builds one linear constraint per
  mesh plus motor/joint attachments and returns C with q = C . m.
"""

from __future__ import annotations

from .geometry import check_gear_mesh, trace_axle


def _is_gear(part_id: str, catalog: dict) -> bool:
    entry = catalog.get(part_id) or {}
    if entry.get("teeth") is None:
        return False
    # Turntable rings carry the joint teeth (60T/56T); without them the
    # graph misses every ring-pinion mesh. Halves of one joint share an
    # axle group only when their lines coincide.
    return entry.get("category") in ("gear", "turntable")


def _is_motor(part_id: str, catalog: dict) -> bool:
    return (catalog.get(part_id) or {}).get("category") == "motor"


def gear_graph(parts: list[dict], catalog: dict,
               link_of: dict | None = None) -> dict:
    """Build axle-group nodes and mesh edges.

    Nodes are rigid groups holding at least one gear or motor, with
    two refinements. Under the Gate 1 fallback, `link_of` (human-
    provided link membership) is authoritative: rigid groups never
    span links, because static analysis cannot tell a locked joint
    from a hinge pivot (observed link motion wins). Gears with no
    rigid fit anywhere spin freely and form singleton nodes (idlers).
    Rigid grouping is then exactly right for meshing analysis: two
    meshing gears can never share a node, since a rigid path between
    them would lock the mesh. Edges are gear meshes across nodes.
    Returns {"nodes": [...], "edges": [...],
    "motor_drive": {motor_uid: node_id}, "intra_meshes": [...]} where
    intra_meshes must stay empty (a rigidly connected meshing pair
    would be a locked gearbox or a mesh-rule false positive).
    """
    from .geometry import find_connections, rigid_groups

    by_uid = {p["uid"]: p for p in parts}
    RIGID_TYPES = {"axle_in_axle_hole", "pin_friction", "stud"}
    raw = rigid_groups(parts, catalog)
    groups = []
    for group in raw:
        if link_of is None:
            buckets = {None: group["uids"]}
        else:
            buckets = {}
            for uid in group["uids"]:
                buckets.setdefault(link_of.get(uid), []).append(uid)
        for key, uids in buckets.items():
            gears_here = [u for u in uids
                          if u in by_uid
                          and _is_gear(by_uid[u]["part_id"], catalog)]
            free = []
            fixed = []
            for u in gears_here:
                try:
                    conns = find_connections(u, parts, catalog)
                except KeyError:
                    conns = []
                if any(c["connection"] in RIGID_TYPES for c in conns):
                    fixed.append(u)
                else:
                    free.append(u)
            # Free-spinning idlers leave the group as singletons; the
            # rigid remainder (possibly empty) stays together.
            rest = [u for u in uids if u not in free]
            if rest:
                groups.append({"id": "%s/%s" % (group["id"], key),
                               "uids": rest})
            for u in free:
                groups.append({"id": "%s/%s/%s" % (group["id"], key, u),
                               "uids": [u]})
    group_of = {}
    for group in groups:
        for uid in group["uids"]:
            group_of[uid] = group["id"]

    nodes = []
    node_of_group = {}
    for group in groups:
        gears = [{"uid": u,
                  "part_id": by_uid[u]["part_id"],
                  "teeth": (catalog.get(by_uid[u]["part_id"]) or {}).get("teeth"),
                  "gear_kind": (catalog.get(by_uid[u]["part_id"]) or {}).get("gear_kind")}
                 for u in sorted(group["uids"])
                 if u in by_uid and _is_gear(by_uid[u]["part_id"], catalog)]
        motors = [u for u in sorted(group["uids"])
                  if u in by_uid and _is_motor(by_uid[u]["part_id"], catalog)]
        if not gears and not motors:
            continue
        node_id = f"A{len(nodes)}"
        node_of_group[group["id"]] = node_id
        nodes.append({"id": node_id, "uids": sorted(group["uids"]),
                      "gears": gears, "motors": sorted(motors)})

    # Motor output groups: the node holding most of each motor's
    # shaft-line peers (the pinion/shaft the motor drives).
    motor_drive: dict[str, str | None] = {}
    for node in nodes:
        for motor in node["motors"]:
            peer_groups: dict[str, int] = {}
            for conn in find_connections(motor, parts, catalog):
                if conn["connection"] != "motor_shaft":
                    continue
                gid = group_of.get(conn["uid"])
                if gid in node_of_group and gid != group_of[motor]:
                    peer_groups[node_of_group[gid]] = \
                        peer_groups.get(node_of_group[gid], 0) + 1
            motor_drive[motor] = (max(peer_groups, key=peer_groups.get)
                                  if peer_groups else node["id"])

    # Intra-node mesh audit: must stay empty (see docstring).
    intra_meshes = []
    for node in nodes:
        glist = [g["uid"] for g in node["gears"]]
        for i in range(len(glist)):
            for j in range(i + 1, len(glist)):
                try:
                    res = check_gear_mesh(glist[i], glist[j], parts,
                                          catalog)
                except KeyError:
                    continue
                if res.get("meshes") and res.get("kind") != "compound":
                    # Same-node compound pairs are correctly grouped
                    # (one shaft); only real meshes are audit failures.
                    intra_meshes.append({
                        "node": node["id"], "a": glist[i], "b": glist[j],
                        "kind": res.get("kind")})

    # Edges: any meshing gear pair across two nodes.
    edges = []
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            for ga in nodes[i]["gears"]:
                for gb in nodes[j]["gears"]:
                    res = check_gear_mesh(ga["uid"], gb["uid"],
                                          parts, catalog)
                    if res.get("meshes"):
                        if res.get("kind") == "compound":
                            # Same shaft: unit ratio, not a tooth mesh.
                            # But observed link motion wins: a compound
                            # pair across two links is a hinge pivot,
                            # not a locked shaft (see link-split test).
                            if link_of is not None and link_of.get(
                                    ga["uid"]) != link_of.get(gb["uid"]):
                                continue
                            ta, tb = 1, 1
                        else:
                            ta, tb = ga["teeth"], gb["teeth"]
                        edges.append({
                            "a": nodes[i]["id"], "b": nodes[j]["id"],
                            "gear_a": ga["uid"], "gear_b": gb["uid"],
                            "ta": ta, "tb": tb,
                            "kind": res.get("kind"),
                            "axis_relation": res.get("axis_relation"),
                            "centre_mm": res.get("centre_mm"),
                        })
    return {"nodes": nodes, "edges": edges,
            "motor_drive": motor_drive, "intra_meshes": intra_meshes}


def axle_carrier(axle_uids: list[str], links: list[dict]) -> str | None:
    """Vote which link an axle group is mounted in.

    links use the private Stage 2 format {"id", "members": [uids]}.
    Returns the link id with the largest member overlap, else None.
    """
    members: dict[str, set[str]] = {}
    for link in links:
        members[link["id"]] = set(link.get("members") or [])
    if not members:
        return None
    wanted = set(axle_uids)
    best_id, best_count = None, 0
    for link_id, uids in members.items():
        count = len(wanted & uids)
        if count > best_count:
            best_id, best_count = link_id, count
    return best_id


def _carrier_list(value) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, dict):
        return [k for k in value if value[k] != 0.0]
    return list(value)


def _carrier_weight(value, name: str) -> float:
    if isinstance(value, dict):
        return float(value.get(name, 0.0))
    return 1.0


def _mesh_sign(kind: str, s) -> float:
    if s is not None:
        return float(s)
    if kind in ("external", "bevel", "crown", "knob"):
        # Bevel/crown/knob meshes in this build are all external-like
        # (pinion outside the mating teeth); the agent revisits the
        # topology when a ratio looks implausible (Stage 3.2 step 5).
        return -1.0
    if kind in ("internal", "compound"):
        return 1.0
    raise ValueError("mesh kind %s needs explicit s (+1/-1)" % kind)


def _build_matrices(topology: dict) -> dict:
    """Assemble the Willis constraint matrices E, F with E X = F.

    Shared by solve_transmission and diagnostics probes so both see
    the identical system. Returns E, F plus index metadata.
    """
    import numpy as np

    motors = topology.get("motors") or []
    joints = topology.get("joints") or []
    meshes = topology.get("meshes") or []
    motor_ids = [m["id"] for m in motors]
    joint_ids = [j["id"] for j in joints]

    # Axle index covers every axle named anywhere.
    axle_ids: list[str] = []
    for m in motors:
        if m["axle"] not in axle_ids:
            axle_ids.append(m["axle"])
    for j in joints:
        if j["axle"] not in axle_ids:
            axle_ids.append(j["axle"])
    for mesh in meshes:
        for key in ("a", "b"):
            if mesh[key] is None:
                continue  # grounded side (no unknown)
            if mesh[key] not in axle_ids:
                axle_ids.append(mesh[key])

    n_ax = len(axle_ids)
    n_j = len(joint_ids)
    n_m = len(motor_ids)
    n_x = n_ax + n_j
    ax_idx = {a: i for i, a in enumerate(axle_ids)}
    j_idx = {j: n_ax + i for i, j in enumerate(joint_ids)}
    m_idx = {m: i for i, m in enumerate(motor_ids)}

    rows_E, rows_F = [], []

    def carrier_vec(value) -> np.ndarray:
        vec = np.zeros(n_x)
        for name in _carrier_list(value):
            if name not in j_idx:
                raise ValueError(f"unknown carrier joint {name}")
            vec[j_idx[name]] += _carrier_weight(value, name)
        return vec

    # Axis-frame signs: flipping joint k's frame (q'_k = -q_k,
    # a_k -> -a_k) negates q_k's coefficient everywhere it appears
    # (its attachment row, and its carrier weight in downstream and
    # mesh rows); axle/motor terms keep their coefficients. Absent
    # key = +1 for all joints (legacy behavior unchanged).
    attach = topology.get("attach_signs") or {}

    def scarrier_vec(value) -> np.ndarray:
        vec = np.zeros(n_x)
        for name in _carrier_list(value):
            if name not in j_idx:
                raise ValueError(f"unknown carrier joint {name}")
            vec[j_idx[name]] += (float(attach.get(name, 1.0))
                                 * _carrier_weight(value, name))
        return vec

    # Motor attachments: w_axle - m = 0.
    for m in motors:
        e = np.zeros(n_x)
        e[ax_idx[m["axle"]]] = 1.0
        f = np.zeros(n_m)
        f[m_idx[m["id"]]] = 1.0
        rows_E.append(e)
        rows_F.append(f)

    # Mesh constraints: Ta*(wa - vca) = s*Tb*(wb - vcb), with one
    # carrier velocity per side (they differ for bevel/crown meshes).
    for mesh in meshes:
        ta, tb = float(mesh["ta"]), float(mesh["tb"])
        s = _mesh_sign(mesh.get("kind", "external"), mesh.get("s"))
        e = np.zeros(n_x)
        e[ax_idx[mesh["a"]]] += ta
        if mesh["b"] is not None:
            e[ax_idx[mesh["b"]]] += -s * tb
        # Grounded side (b None, e.g. stator ring): wb = 0, so only
        # its carrier term below survives for that side.
        if "carrier_a" in mesh or "carrier_b" in mesh:
            e -= ta * scarrier_vec(mesh.get("carrier_a"))
            e += s * tb * scarrier_vec(mesh.get("carrier_b"))
        else:
            e -= ((ta - s * tb)
                  * scarrier_vec(mesh.get("carrier_joint")))
        rows_E.append(e)
        rows_F.append(np.zeros(n_m))

    # Joint attachments: q - w_axle + v_parent = 0.
    for j in joints:
        e = np.zeros(n_x)
        e[j_idx[j["id"]]] += float(attach.get(j["id"], 1.0))
        e[ax_idx[j["axle"]]] += -1.0
        e += scarrier_vec(j.get("carrier_joint"))
        rows_E.append(e)
        rows_F.append(np.zeros(n_m))

    # Held joints (optional): q_j = 0. Used when a locked motor's
    # holding torque reaches its joint through a confirmed mapping
    # whose CAD path is void (the solver cannot see the lock
    # otherwise, and the joint would back-drive in every column).
    # Absent key = no held joints (legacy behavior unchanged).
    for jid in topology.get("held_joints") or []:
        if jid not in j_idx:
            raise ValueError(f"unknown held joint {jid}")
        e = np.zeros(n_x)
        e[j_idx[jid]] += 1.0
        rows_E.append(e)
        rows_F.append(np.zeros(n_m))

    if not rows_E:
        raise ValueError("empty topology: no constraints")
    E = np.array(rows_E, dtype=float)
    F = np.array(rows_F, dtype=float)
    return {"E": E, "F": F, "axle_ids": axle_ids,
            "joint_ids": joint_ids, "motor_ids": motor_ids,
            "n_ax": n_ax, "n_j": n_j, "n_m": n_m, "n_x": n_x}


def solve_transmission(topology: dict) -> dict:
    """Solve Willis constraints for C with q = C . m.

    topology = {
      "motors": [{"id": "M1", "axle": "A0"}],
      "joints": [{"id": "J1", "axle": "A1", "carrier_joint": None|str|list|dict}],
      "meshes": [{"a": "A0", "ta": 8, "b": "A1", "tb": 24,
                  "kind": "external"|"internal"|"bevel"|"compound",
                  "s": optional for bevel,
                  "carrier_joint": None|str|list|dict,
                  "carrier_a": dict (optional, this side),
                  "carrier_b": dict (optional, other side)}],
    }
    All speeds are scalars about the local mesh/joint axis. Carrier
    entries weight joint velocities: a list/str counts 1.0 per named
    joint (serial coaxial model); a dict maps joint id to its
    projection coefficient (axis dot products) for non-parallel
    chains, e.g. a vertical slew contributes 0.0 to a horizontal
    mesh. Ground is 0. Per-side carriers (carrier_a/carrier_b) split
    bevel and crown meshes whose sides face different axes.
    """
    import numpy as np

    mat = _build_matrices(topology)
    E, F = mat["E"], mat["F"]
    motor_ids, joint_ids = mat["motor_ids"], mat["joint_ids"]
    axle_ids = mat["axle_ids"]
    n_ax, n_j, n_m, n_x = mat["n_ax"], mat["n_j"], mat["n_m"], mat["n_x"]

    rank = int(np.linalg.matrix_rank(E))
    rank_ok = rank == n_x
    # Solve E X = F for X (n_x x n_m); least-squares when overdetermined.
    X, residuals, _, _ = np.linalg.lstsq(E, F, rcond=None)
    # Max-abs (signed max hides the largest violation: the 736/743
    # motor-fight rows read -0.69 while signed max showed +0.03).
    residual = float(np.abs(E @ X - F).max()) if X.size else 0.0
    C = X[n_ax:, :] if n_j and n_m else np.zeros((n_j, n_m))

    ratios: dict[str, float] = {}
    signs: dict[str, int] = {}
    for i, jid in enumerate(joint_ids):
        row = C[i, :] if n_m else np.zeros(0)
        if len(row) == 0:
            ratios[jid], signs[jid] = 0.0, 1
            continue
        k = int(np.argmax(np.abs(row)))
        primary = motor_ids[k]
        ratios[f"{jid}/{primary}"] = float(row[k])
        signs[f"{jid}/{primary}"] = 1 if row[k] >= 0 else -1

    return {
        "C": [[float(v) for v in row] for row in C.tolist()],
        "joint_ids": joint_ids,
        "motor_ids": motor_ids,
        "axle_ids": axle_ids,
        "ratios": ratios,
        "signs": signs,
        "rank_ok": bool(rank_ok),
        "rank": rank,
        "n_unknowns": n_x,
        "residual": residual,
    }
