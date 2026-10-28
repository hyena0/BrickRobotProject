"""Tier-1 chain joint inference from div.io submodels (Stage 2.10).

C0: rigid groups are NOT inferred -- the div.io link membership
(D0=Base, D1..D6) is adopted as given. The flat .io segmentation track
(Tier 2: src/tools/joints.py, src/agent/heuristic.py) is frozen and
never imported here; only src/tools/geometry.py facts are reused.

C1: Base->L1->...->L6 serial chain; joint k = interface of group k-1
and k, exactly one per adjacent pair, labelled in chain order.
C2: a joint axis is the RELATIVE rotation axis of the two groups.
Drive shafts through the upstream group and internal gear axles are
not joints.
C3: bearing anchors are turntable rings or pins/axles piercing both
groups coaxially. The axis point is the common circle-centre fit of
the ring/pin bundle -- never a part origin (curved rack origins are
not arc centres).
C4: J6 is the only rotation interface of L5/L6; nothing downstream.
C5: self-verification rotates the whole downstream side rigidly about
the candidate axis: (a) all downstream members move together,
(b) upstream poses are byte-identical, (c) bearing elements stay
coaxial. Failures classify as shaft / internal-axis / boundary.
C6: axis = direction (proximal->distal) + an arbitrary point on the
line, unified to the C3 centre (axis position is unobservable).
"""

from __future__ import annotations

import json
import math

import numpy as np

from src.tools import geometry as geom

CHAIN = ["D0", "D1", "D2", "D3", "D4", "D5", "D6"]
COAX_LATERAL_MM = 2.0
COAX_ANGLE_DEG = 3.0
VERIFY_DEG = (30.0, -30.0)
VERIFY_LATERAL_MM = 2.0
# C3': nested-ring coincidence radius (3D centre distance, mm).
NEST_EPS_MM = 2.0
# C5-d sweep schedule (deg). The AABB broadphase is complete
# (unshrunk); the verdict is mesh containment, never box overlap.
SWEEP_DEG = (10.0, -10.0, 20.0, -20.0, 30.0, -30.0)
LDU_MM = 0.4
# Local mesh caches per library object: part_id -> arrays.
_RIM_LDU: dict = {}
_BOX_LOCAL: dict = {}


def _conv_pts(verts: np.ndarray) -> np.ndarray:
    """LDraw type-local LDU vertices into converted local mm."""
    v = np.asarray(verts, dtype=float)
    return np.stack([v[:, 0], v[:, 2], -v[:, 1]], axis=1) * LDU_MM


def _rim_ldu(lib, part_id: str) -> tuple | None:
    """Max-radius rim vertices of a part mesh (type-local LDU).

    Cached per (library, part_id). None when the mesh is missing or
    has no usable rim.
    """
    from src.catalog.mesh import type_vertices

    key = (id(lib), part_id)
    if key not in _RIM_LDU:
        try:
            verts = type_vertices(lib, part_id)
        except Exception:
            verts = np.zeros((0, 3))
        out = None
        if len(verts) >= 12:
            radii = np.linalg.norm(verts, axis=1)
            r_max = float(radii.max())
            if r_max > 0:
                rim = np.asarray(verts)[radii >= 0.97 * r_max]
                if len(rim) >= 6:
                    out = (rim, r_max)
        _RIM_LDU[key] = out
    return _RIM_LDU[key]


def _box_local(lib, part_id: str) -> tuple | None:
    """Converted-local mm AABB corners (8x3) of a part mesh, or None."""
    key = (id(lib), part_id)
    if key not in _BOX_LOCAL:
        from src.catalog.mesh import type_vertices

        try:
            verts = type_vertices(lib, part_id)
        except Exception:
            verts = np.zeros((0, 3))
        out = None
        if len(verts) >= 1:
            q = _conv_pts(np.asarray(verts))
            lo, hi = q.min(axis=0), q.max(axis=0)
            grid = np.stack(np.meshgrid(*zip(lo, hi)), axis=-1)
            out = grid.reshape(-1, 3)
        _BOX_LOCAL[key] = out
    return _BOX_LOCAL[key]


def _world_pts(pos, rot, local: np.ndarray) -> np.ndarray:
    """Converted-local mm points into world mm for a part pose."""
    return (np.asarray(pos, dtype=float)
            + np.asarray(local, dtype=float)
            @ np.asarray(rot, dtype=float).T)


def _plane_fit(pts: np.ndarray) -> tuple:
    """Plane fit: (unit normal, centroid, rms residual mm)."""
    pts = np.asarray(pts, dtype=float)
    centroid = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = _unit(vh[-1])
    resid = (pts - centroid) @ normal
    rms = float(np.sqrt(np.mean(resid ** 2)))
    return normal, centroid, rms


def _aabbs_overlap(a: np.ndarray, b: np.ndarray,
                   shrink: float = 0.0) -> bool:
    """AABB overlap of two corner sets, shrunk against touching faces."""
    alo, ahi = a.min(axis=0) + shrink, a.max(axis=0) - shrink
    blo, bhi = b.min(axis=0) + shrink, b.max(axis=0) - shrink
    return bool(np.all(alo <= bhi) and np.all(blo <= ahi))


_TRIS_LOCAL: dict = {}
# Ray-casting budget: vertices sampled per mesh per test. Deep
# sweep penetrations (the only verdict-grade finding) are caught
# far below this sampling density; grazing is gap-reported anyway.
MESH_SAMPLE = 128
# Sweep mesh decimation voxel (mm): vertex-clustering tessellation
# simplifier for the sweep only (rim fits keep full resolution).
# 1mm << 2mm verdict depth, deterministic, no topology surgery.
SWEEP_VOXEL_MM = 1.0


def _decimate(tris: np.ndarray, voxel: float) -> np.ndarray:
    """Vertex-clustering decimation of world/local triangles."""
    verts = tris.reshape(-1, 3)
    key = np.floor(verts / voxel + 0.5).astype(np.int64)
    _uniq, inverse = np.unique(key, axis=0, return_inverse=True)
    rep = np.zeros((inverse.max() + 1, 3))
    cnt = np.zeros(inverse.max() + 1)
    np.add.at(rep, inverse, verts)
    np.add.at(cnt, inverse, 1)
    rep = rep / cnt[:, None]
    faces = rep[inverse].reshape(-1, 3, 3)
    keep = np.ones(len(faces), dtype=bool)
    for a, b in ((0, 1), (1, 2), (2, 0)):
        same = np.abs(faces[:, a] - faces[:, b]).max(axis=1) < 1e-9
        keep &= ~same
    return faces[keep]
# Chord margin: a dn point at pivot radius R moves at most
# 2*R*sin(15deg) ~= 0.518*R under the +-30deg sweep, plus LDraw
# fidelity slack. Pairs starting farther apart can never touch.
SWEEP_REACH = 0.518
SWEEP_SLACK_MM = 0.5


def _tris_local(lib, part_id: str) -> np.ndarray | None:
    """Converted-local mm triangles (Tx3x3) of a part mesh, or None."""
    key = (id(lib), part_id)
    if key not in _TRIS_LOCAL:
        from src.catalog.mesh import type_tris

        try:
            verts, tris = type_tris(lib, part_id)
        except Exception:
            verts, tris = np.zeros((0, 3)), []
        out = None
        if len(verts) and tris:
            out = _conv_pts(
                np.asarray(verts, dtype=float))[np.asarray(tris)]
        _TRIS_LOCAL[key] = out
    return _TRIS_LOCAL[key]


def _world_tris(pos, rot, tris: np.ndarray) -> np.ndarray:
    """Converted-local mm triangles into world mm (Tx3x3)."""
    return (np.asarray(pos, dtype=float)
            + np.asarray(tris, dtype=float)
            @ np.asarray(rot, dtype=float).T)


def _ray_hits(qry: np.ndarray, tris: np.ndarray,
              direction: np.ndarray) -> np.ndarray:
    """Count +ray triangle crossings per query point (Moller-Trumbore).

    Vectorised over triangles in chunks to bound memory. Degenerate
    edge/vertex grazes may miscount one ray; callers vote over rays.
    """
    direction = _unit(np.asarray(direction, dtype=float))
    n = len(qry)
    counts = np.zeros(n, dtype=np.int64)
    eps = 1e-9
    for s in range(0, len(tris), 2000):
        t = tris[s:s + 2000]
        e1 = t[:, 1] - t[:, 0]
        e2 = t[:, 2] - t[:, 0]
        p = np.cross(direction, e2)
        det = np.einsum("ij,ij->i", e1, p)
        ok = np.abs(det) > eps
        inv = np.zeros_like(det)
        inv[ok] = 1.0 / det[ok]
        tv = qry[:, None, :] - t[None, :, 0]
        u = np.einsum("nij,ij->ni", tv, p) * inv[None, :]
        q = np.cross(tv, e1)
        v = (q @ direction) * inv[None, :]
        tt = np.einsum("ij,nij->ni", e2, q) * inv[None, :]
        hit = (ok[None, :] & (u >= -1e-6) & (v >= -1e-6)
               & (u + v <= 1.0 + 1e-6) & (tt > 1e-6))
        counts += hit.sum(axis=1)
    return counts


def _points_in_mesh(qry: np.ndarray, tris: np.ndarray) -> np.ndarray:
    """Inside test per query point: majority vote over 3 axis rays."""
    if len(qry) == 0 or tris is None or len(tris) == 0:
        return np.zeros(len(qry), dtype=bool)
    votes = np.zeros(len(qry), dtype=np.int64)
    for direction in (np.array([1.0, 0.0, 0.0]),
                      np.array([0.0, 1.0, 0.0]),
                      np.array([0.0, 0.0, 1.0])):
        votes += _ray_hits(qry, tris, direction) % 2
    return votes >= 2


def _sample_verts(world_tris: np.ndarray) -> np.ndarray:
    """Evenly strided vertex sample (<=MESH_SAMPLE) of a world mesh."""
    verts = np.unique(world_tris.reshape(-1, 3), axis=0)
    if len(verts) <= MESH_SAMPLE:
        return verts
    step = len(verts) / MESH_SAMPLE
    idx = (np.arange(MESH_SAMPLE) * step).astype(int)
    return verts[idx]


def _mesh_gap(world_a: np.ndarray, world_b: np.ndarray) -> float:
    """Min vertex-to-vertex distance between two world meshes."""
    va = np.unique(world_a.reshape(-1, 3), axis=0)
    vb = np.unique(world_b.reshape(-1, 3), axis=0)
    best = float("inf")
    for s in range(0, len(va), 512):
        d = np.linalg.norm(va[s:s + 512, None, :] - vb[None, :, :],
                           axis=2)
        best = min(best, float(d.min()))
    return best


def _meshes_contain(world_a: np.ndarray,
                      world_b: np.ndarray) -> bool:
    """Containment only: a sampled vertex of either mesh inside other."""
    sa, sb = _sample_verts(world_a), _sample_verts(world_b)
    return bool(_points_in_mesh(sa, world_b).any()
                or _points_in_mesh(sb, world_a).any())


# Verdict-grade penetration needs real depth: LDraw teeth/axle
# approximations plus open-soup ray-cast noise graze at ~1mm, while
# a genuinely wrong (e.g. orthogonal) axis drives parts through by
# centimetres. Engaged-seat detection stays maximally sensitive.
PENETRATE_DEPTH_MM = 2.0


def _point_tri_depth(pts: np.ndarray, tris: np.ndarray) -> np.ndarray:
    """Min distance from points to triangles (vectorised, chunked)."""
    pts = np.asarray(pts, dtype=float)
    best = np.full(len(pts), np.inf)
    for s in range(0, len(tris), 2000):
        A, B, C = tris[s:s + 2000, 0], tris[s:s + 2000, 1], \
            tris[s:s + 2000, 2]
        AB, AC = B - A, C - A
        n = np.cross(AB, AC)
        nl = np.linalg.norm(n, axis=1, keepdims=True)
        ok = nl[:, 0] > 1e-12
        nn = np.zeros_like(n)
        nn[ok] = n[ok] / nl[ok]
        rel = pts[:, None, :] - A[None, :, :]
        signed = np.einsum("nij,ij->ni", rel, nn)
        proj = rel - signed[:, :, None] * nn[None, :, :]
        v0, v1, v2 = C - A, B - A, proj - A[None, :, :]
        d00 = np.einsum("ij,ij->i", AB, AB)
        d01 = np.einsum("ij,ij->i", AB, AC)
        d11 = np.einsum("ij,ij->i", AC, AC)
        d20 = np.einsum("nij,ij->ni", v2, v0)
        d21 = np.einsum("nij,ij->ni", v2, v1)
        den = np.where(np.abs(d00 * d11 - d01 * d01) > 1e-12,
                       d00 * d11 - d01 * d01, 1)[None, :]
        vv = (d11[None, :] * d20 - d01[None, :] * d21) / den
        ww = (d00[None, :] * d21 - d01[None, :] * d20) / den
        uu = 1 - vv - ww
        inside = (uu >= -1e-6) & (vv >= -1e-6) & (ww >= -1e-6)
        dface = np.where(inside, np.abs(signed), np.inf)
        edge = np.full_like(dface, np.inf)
        for P, Q in ((A, B), (B, C), (C, A)):
            pq = Q - P
            L2 = np.einsum("ij,ij->i", pq, pq)
            t = (np.einsum("nij,ij->ni", pts[:, None, :] - P[None, :],
                           pq) / np.where(L2 > 1e-12, L2, 1)[None, :])
            t = np.clip(t, 0, 1)
            dd = np.linalg.norm(
                pts[:, None, :] - (P[None, :, :]
                                   + t[:, :, None] * pq[None, :, :]),
                axis=2)
            edge = np.minimum(edge, dd)
        best = np.minimum(best, np.minimum(dface, edge).min(axis=1))
    return best


def _spin_free(uid: str, parts: list[dict], catalog: dict,
               by_uid: dict, axis_pt, axis_dir, _cache: dict) -> bool:
    """A bore-mounted part off the joint axis (transmission DOF).

    Gears, bushes, pins and axles riding a bore seat (axle-hole grip
    or round-hole bearing) off the candidate axis have a spin DOF
    relative to their group; freezing them rigidly fakes collisions
    (tooth-through-tooth, bush drag). Friction-seated bolts (no bore
    freedom) and on-axis parts (sweep-invariant anyway) stay
    structural, as do bolted turntable halves. Cached per sweep.
    Documented bias: an axle-gripped gear on a structural (non-spin)
    axle is over-excluded; its axle paradox is still judged by
    judge_clamped, and C5-n guards the ensemble.
    """
    if uid in _cache:
        return _cache[uid]
    out = False
    entry = geom._entry(catalog, by_uid[uid]["part_id"])
    if entry.get("category") in ("gear", "bush", "pin", "axle"):
        rel = np.asarray(by_uid[uid]["position_mm"], dtype=float) \
            - np.asarray(axis_pt, dtype=float)
        d = _unit(np.asarray(axis_dir, dtype=float))
        lateral = float(np.linalg.norm(
            rel - float(rel @ d) * d))
        if lateral > COAX_LATERAL_MM:
            for conn in geom.find_connections(uid, parts, catalog):
                if conn.get("connection") in ("axle_in_axle_hole",
                                              "axle_in_round_hole"):
                    out = True
                    break
    _cache[uid] = out
    return out


def _spinning_gear(uid: str, parts: list[dict], catalog: dict,
                   by_uid: dict, _cache: dict) -> bool:
    """Legacy gear-only spin test (kept for unit tests)."""
    if uid in _cache:
        return _cache[uid]
    out = False
    entry = geom._entry(catalog, by_uid[uid]["part_id"])
    if entry.get("category") == "gear" \
            and entry.get("teeth") is not None:
        for conn in geom.find_connections(uid, parts, catalog):
            if conn.get("connection") != "axle_in_axle_hole":
                continue
            other = by_uid.get(conn["uid"])
            if other is not None and geom._entry(
                    catalog, other["part_id"]).get("category") == "axle":
                out = True
                break
    _cache[uid] = out
    return out


def _tri_boxes(tris: np.ndarray) -> np.ndarray:
    """Per-triangle AABBs (T,2,3)."""
    return np.stack([tris.min(axis=1), tris.max(axis=1)], axis=1)


def _seg_tri_count(s0: np.ndarray, s1: np.ndarray,
                   tris: np.ndarray) -> int:
    """Segment-triangle crossings (Moller-Trumbore, t in [0,1])."""
    total = 0
    eps = 1e-9
    d = s1 - s0
    for qs in range(0, len(tris), 512):
        t = tris[qs:qs + 512]
        e1 = t[:, 1] - t[:, 0]
        e2 = t[:, 2] - t[:, 0]
        for ps in range(0, len(s0), 512):
            p0, pd = s0[ps:ps + 512], d[ps:ps + 512]
            pvec = np.cross(pd[:, None, :], e2[None, :, :])
            det = (e1[None, :, :] * pvec).sum(axis=2)
            ok = np.abs(det) > eps
            inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
            tvec = p0[:, None, :] - t[None, :, 0]
            u = (tvec * pvec).sum(axis=2) * inv
            qvec = np.cross(tvec, e1[None, :, :])
            v = (pd[:, None, :] * qvec).sum(axis=2) * inv
            tt = (e2[None, :, :] * qvec).sum(axis=2) * inv
            hit = (ok & (u >= -1e-6) & (v >= -1e-6)
                   & (u + v <= 1.0 + 1e-6)
                   & (tt >= -1e-6) & (tt <= 1.0 + 1e-6))
            total += int(hit.sum())
    return total


def _tris_cross(world_a: np.ndarray,
                world_b: np.ndarray) -> tuple[bool, int]:
    """Edge-face crossings either way, tri-box prefiltered.

    Answers the hollow-shell question containment cannot: a corner
    hovering in a battery bay crosses no shell triangle (clear),
    while a real stab crosses many. Called only for
    containment/graze pairs, never blindly.
    """
    ba, bb = _tri_boxes(world_a), _tri_boxes(world_b)
    pairs = []
    for s in range(0, len(ba), 512):
        alo = ba[s:s + 512, 0][:, None, :]
        ahi = ba[s:s + 512, 1][:, None, :]
        blo, bhi = bb[:, 0][None, :, :], bb[:, 1][None, :, :]
        over = np.all(alo <= bhi, axis=2) & np.all(blo <= ahi, axis=2)
        ii, jj = np.nonzero(over)
        pairs.extend((i + s, j) for i, j in zip(ii.tolist(),
                                                jj.tolist()))
        if len(pairs) > 20000:
            break
    if not pairs:
        return False, 0
    # Existence test on a deterministic stride subsample: deep stabs
    # cross hundreds of shells (found in any sample), hollow hovering
    # crosses none; this bounds the quadratic segment-triangle cost.
    if len(pairs) > 2000:
        stride = (len(pairs) + 1999) // 2000
        pairs = pairs[::stride]
    ia = sorted({i for i, _ in pairs})
    jb = sorted({j for _, j in pairs})
    ta, tb = world_a[ia], world_b[jb]
    ea = ta[:, [0, 1, 1, 2, 2, 0], :].reshape(-1, 2, 3)
    eb = tb[:, [0, 1, 1, 2, 2, 0], :].reshape(-1, 2, 3)
    n = _seg_tri_count(ea[:, 0], ea[:, 1], tb)
    n += _seg_tri_count(eb[:, 0], eb[:, 1], ta)
    return n > 0, n


def _cross_mounted(u: str, v: str, parts: list[dict],
                   catalog: dict, by_uid: dict) -> bool:
    """Cross-boundary mount between two parts (rigid hardware).

    Friction/axle-hole clamps or a bore seat piercing the other
    part's holes: the pair is mounted across the joint, so a rigid
    sweep breaks the mount by design. Such clashes are C0 boundary
    evidence (spanning hardware or misassignment), never axis
    verdicts.
    """
    RIGID = ("pin_friction", "axle_in_axle_hole")
    for me, other in ((u, v), (v, u)):
        for conn in geom.find_connections(me, parts, catalog):
            if conn["uid"] == other \
                    and conn.get("connection") in RIGID:
                return True
        mp = by_uid[me]
        if geom._entry(catalog,
                       mp["part_id"]).get("category") not in ("axle",
                                                              "pin"):
            continue
        axis = geom.axis_world(mp, catalog)
        if axis is None:
            continue
        line_pt = np.asarray(mp["position_mm"], dtype=float)
        line_dir = _unit(axis)
        for _i, hp in _hole_centres(by_uid[other], catalog):
            rel = hp - line_pt
            dev = rel - float(np.dot(rel, line_dir)) * line_dir
            if float(np.linalg.norm(dev)) <= COAX_LATERAL_MM:
                return True
    return False


def _toothed_pair(u: str, v: str, catalog: dict,
                  by_uid: dict) -> bool:
    """Both parts toothed (rolling/mesh contact owns the pair)."""
    cats = {geom._entry(catalog, by_uid[w]["part_id"]).get("category")
            for w in (u, v)}
    return cats <= {"gear", "turntable", "rack"} and bool(cats)


def _contain_confirmed(world_a: np.ndarray,
                         world_b: np.ndarray
                         ) -> tuple[int, float, int]:
    """(n_inside, max_confirmed_depth_mm, n_confirmed).

    Only stencil-confirmed vertices count toward the depth verdict;
    unconfirmed hole-threading misfires are reported, never verdicts.
    """
    sa, sb = _sample_verts(world_a), _sample_verts(world_b)
    ina = _points_in_mesh(sa, world_b)
    inb = _points_in_mesh(sb, world_a)
    n = int(ina.sum() + inb.sum())
    depth = 0.0
    n_conf = 0
    if int(ina.sum()):
        keep = _confirm_inside(sa[ina], world_b)
        n_conf += int(keep.sum())
        if int(keep.sum()):
            depth = max(depth, float(_point_tri_depth(sa[ina][keep],
                                                      world_b).max()))
    if int(inb.sum()):
        keep = _confirm_inside(sb[inb], world_a)
        n_conf += int(keep.sum())
        if int(keep.sum()):
            depth = max(depth, float(_point_tri_depth(sb[inb][keep],
                                                      world_a).max()))
    return n, round(depth, 3), n_conf


def _contain_depth(world_a: np.ndarray,
                   world_b: np.ndarray) -> tuple[int, float]:
    """(n_inside, max_depth_mm) over both containment directions."""
    sa, sb = _sample_verts(world_a), _sample_verts(world_b)
    ina = _points_in_mesh(sa, world_b)
    inb = _points_in_mesh(sb, world_a)
    n = int(ina.sum() + inb.sum())
    depth = 0.0
    if int(ina.sum()):
        depth = max(depth, float(_point_tri_depth(sa[ina],
                                                  world_b).max()))
    if int(inb.sum()):
        depth = max(depth, float(_point_tri_depth(sb[inb],
                                                  world_a).max()))
    return n, round(depth, 3)


# Confirmation stencil: the centre plus +-0.6mm probes on 3 axes.
# A truly buried vertex keeps its neighbours inside; a ray that
# threaded a pin hole or open seam (open LDraw soup) flips when the
# origin jitters. Confirmed = centre + at least 3 of 6 probes.
CONFIRM_EPS_MM = 0.6


def _confirm_inside(pts: np.ndarray, tris: np.ndarray) -> np.ndarray:
    """Confirm inside-vertices against hole-threading misfires."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return np.zeros(0, dtype=bool)
    votes = _points_in_mesh(pts, tris).astype(np.int64)
    for axis in range(3):
        for sgn in (-1.0, 1.0):
            probe = np.asarray(pts)
            probe[:, axis] += sgn * CONFIRM_EPS_MM
            votes += _points_in_mesh(probe, tris).astype(np.int64)
    return votes >= 4


def _meshes_intersect(world_a: np.ndarray,
                      world_b: np.ndarray) -> tuple[bool, float]:
    """Exact-ish mesh penetration test + min vertex gap (mm).

    True when a sampled vertex of either mesh lies inside the other
    (majority-vote ray cast). Grazing without containment is not
    penetration: reported via the gap, never a verdict.
    """
    return _meshes_contain(world_a, world_b), _mesh_gap(world_a,
                                                        world_b)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cosv = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosv))))


def load_div_groups(div_path: str, parts: list[dict]) -> dict:
    """Adopt D0..D6 membership; verify exact partition of parts."""
    doc = json.load(open(div_path, encoding="utf-8"))
    by_uid = {p["uid"]: p for p in parts}
    members = {}
    seen: set[str] = set()
    issues = []
    for link in doc.get("links", []):
        if link["id"] in CHAIN:
            members[link["id"]] = list(link.get("members", []))
    for gid in CHAIN:
        for uid in members.get(gid, []):
            if uid not in by_uid:
                issues.append(f"member {uid} of {gid} missing from parts")
            if uid in seen:
                issues.append(f"member {uid} in two groups")
            seen.add(uid)
    orphans = sorted(set(by_uid) - seen)
    if orphans:
        issues.append(f"{len(orphans)} parts outside all groups")
    return {"members": members, "issues": issues, "orphans": orphans}


def _rotated_member(part: dict, axis_pt: np.ndarray,
                    axis_dir: np.ndarray, angle_deg: float) -> dict:
    """Copy of part rigidly rotated about the axis (positions+matrix)."""
    theta = math.radians(angle_deg)
    k = _unit(axis_dir)
    kx, ky, kz = (float(v) for v in k)
    kmat = np.array([[kx * kx, kx * ky, kx * kz],
                     [ky * kx, ky * ky, ky * kz],
                     [kz * kx, kz * ky, kz * kz]])
    cross = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    rot = (np.eye(3) * math.cos(theta) + (1.0 - math.cos(theta)) * kmat
           + math.sin(theta) * cross)
    out = dict(part)
    out["position_mm"] = list(rot @ (np.asarray(part["position_mm"],
                                                dtype=float) - axis_pt)
                              + axis_pt)
    out["rotation"] = (rot @ np.asarray(part["rotation"],
                                        dtype=float)).tolist()
    return out


def pin_anchors(up_uids: set, dn_uids: set, parts: list[dict],
                catalog: dict) -> list[dict]:
    """Axle/pin parts piercing members of both groups (C3 pin bundle)."""
    by_uid = {p["uid"]: p for p in parts}
    out = []
    for part in parts:
        entry = geom._entry(catalog, part["part_id"])
        if entry.get("category") not in ("axle", "pin"):
            continue
        axis = geom.axis_world(part, catalog)
        if axis is None:
            continue
        direction = _unit(axis)
        span = (geom._axle_span_mm(part, catalog)
                if entry.get("category") == "axle" else 60.0)
        hits_up = sorted(u for u in up_uids
                         if geom._line_hits(part, axis, by_uid[u],
                                            catalog, span))
        hits_dn = sorted(u for u in dn_uids
                         if geom._line_hits(part, axis, by_uid[u],
                                            catalog, span))
        if not (hits_up and hits_dn):
            continue
        # C2: a motor drive shaft is transmission, never a joint.
        shaft = any(c.get("connection") == "motor_shaft"
                    for c in geom.find_connections(part["uid"], parts,
                                                   catalog))
        supports = [geom._pos(part)]
        for u in hits_up + hits_dn:
            holes = geom._entry(catalog,
                                by_uid[u]["part_id"]).get("holes") or []
            for hole in holes:
                if not isinstance(hole, dict):
                    continue
                try:
                    c = geom.hole_world(by_uid[u], hole)
                except Exception:
                    continue
                rel = c - geom._pos(part)
                lateral = rel - float(np.dot(rel, direction)) * direction
                if float(np.linalg.norm(lateral)) \
                        <= geom.AXLE_LINE_TOL_MM:
                    supports.append(c)
        # Per-side fit: clamped (axle-hole) both sides + split groups
        # means the div boundary cut a rigid coupling (C0). A bush
        # bore is a bearing (free) on the axle side: the axle spins
        # in the bush, so bush fits never count as clamps here.
        # (Tier-1 rule; geometry.py itself is untouched.)
        clamped = []
        for side, hits in (("up", hits_up), ("dn", hits_dn)):
            kinds = set()
            for u in hits:
                if geom._entry(catalog,
                               by_uid[u]["part_id"]).get("category") \
                        == "bush":
                    kinds.add("axle_in_round_hole")
                else:
                    kinds.add(geom._classify_axle_fit(
                        by_uid[u], catalog, geom._pos(part), direction))
            clamped.append("axle_in_axle_hole" in kinds)
        uniq: dict[tuple, np.ndarray] = {}
        for p in supports:
            key = tuple(round(float(v), 1) for v in np.asarray(p).ravel())
            uniq.setdefault(key, np.asarray(p, dtype=float))
        pts = list(uniq.values())
        if len(pts) >= 2:
            arr = np.asarray(pts)
            center = arr.mean(axis=0)
            _, _, vh = np.linalg.svd(arr - center)
            fit_dir = _unit(vh[0])
            lateral = arr - center - np.outer((arr - center) @ fit_dir,
                                              fit_dir)
            err = round(float(np.linalg.norm(lateral, axis=1).max()), 3)
        else:
            center = np.asarray(pts[0])
            fit_dir = direction
            err = 0.0
        out.append({"uid": part["uid"], "direction": fit_dir,
                    "center": center, "fit_error_mm": err,
                    "n_supports": len(pts), "motor_shaft": shaft,
                    "clamped_up": clamped[0], "clamped_dn": clamped[1],
                    "hits_up": hits_up, "hits_dn": hits_dn})
    return out


def classify_pin(anchor: dict) -> tuple[str, str]:
    """shaft / boundary / hinge + sentence (C2, C0)."""
    if anchor["motor_shaft"]:
        return ("shaft",
                f"{anchor['uid']} rides a motor shaft line: transmission, "
                f"not a joint (C2)")
    if anchor["clamped_up"] and anchor["clamped_dn"]:
        return ("boundary",
                f"{anchor['uid']} clamped (axle-hole, bushes excluded) "
                f"on both sides yet div splits its groups: boundary "
                f"suspect (C0), no reassignment")
    return ("hinge", "")


def rack_bundle(up_uids: set, dn_uids: set, parts: list[dict],
                catalog: dict) -> dict:
    """C3 ring-centre fit for curved rack arc bundles (e.g. 24121).

    Arc pieces sharing one ring are either stacked along the axis
    (linear spread -> axis = principal direction) or arranged around
    it (planar spread -> axis = plane normal). The centre is the
    bundle fit -- no single part origin is ever used as a centre.
    """
    by_uid = {p["uid"]: p for p in parts}
    arcs = [by_uid[u] for u in sorted(up_uids | dn_uids)
            if geom._entry(catalog,
                           by_uid[u]["part_id"]).get("category") == "rack"]
    if not arcs:
        return {"ok": False}
    pts = np.asarray([np.asarray(a["position_mm"], dtype=float)
                      for a in arcs])
    if len(pts) < 3:
        return {"ok": False, "note": "fewer than 3 arc pieces"}
    mean = pts.mean(axis=0)
    _, s, vh = np.linalg.svd(pts - mean)
    if s[1] / max(s[0], 1e-9) < 0.05:
        direction = _unit(vh[0])
        spread = "linear"
    elif s[2] / max(s[1], 1e-9) < 0.05:
        direction = _unit(vh[2])
        spread = "planar"
    else:
        return {"ok": False, "class": "boundary",
                "note": "rack arcs fit neither a stack nor a ring"}
    lateral = pts - mean - np.outer((pts - mean) @ direction, direction)
    err = round(float(np.linalg.norm(lateral, axis=1).max()), 3)
    return {"ok": True, "direction": direction, "center": mean,
            "kind": "rack-ring", "spread": spread,
            "uids": [a["uid"] for a in arcs],
            "fit_error_mm": err, "n_supports": len(arcs)}


def _ring_sense(fixed_uids: list, moving_uids: list, rot: dict,
                by_uid: dict, catalog: dict, lib, angle: float) -> dict:
    """Restored-sensitivity (c) for one turntable anchor and angle.

    Returns {"worst_hits": {label: mm}, "log": {...}}. With a mesh
    library the LIVE checks are (1) moving annulus normal stays
    parallel to the fixed normal and (2) the moving centroid stays
    on the fixed axis line. Rim radius / axial spread are
    rigid-motion invariants: toothed rims and ring bands carry a
    STATIC spread (teeth depth, band width), so an absolute epsilon
    on them rejects true joints -- they are logged for rigidity
    evidence only, never verdicts. Angle excess is reported through
    the legacy penalty idiom (VERIFY_LATERAL_MM + 1.0) since `worst`
    is in mm. Without a library the legacy origin-coincidence
    lateral is reported and flagged weak (it is degenerate for
    nested origins).
    """
    out: dict = {"worst_hits": {}, "log": {}}
    if lib is None or not fixed_uids or not moving_uids:
        out["log"]["mode"] = "legacy-weak-no-lib"
        return out
    for fuid in fixed_uids:
        frim = _rim_ldu(lib, by_uid[fuid]["part_id"])
        if frim is None:
            out["log"][fuid] = "no-rim-mesh"
            continue
        fpart = by_uid[fuid]
        fw = _world_pts(fpart["position_mm"], fpart["rotation"],
                        _conv_pts(np.asarray(frim[0])))
        nf, cf, _ = _plane_fit(fw)
        for muid in moving_uids:
            mrim = _rim_ldu(lib, by_uid[muid]["part_id"])
            if mrim is None:
                out["log"][muid] = "no-rim-mesh"
                continue
            pose = rot.get(muid, by_uid[muid])
            mw = _world_pts(pose["position_mm"], pose["rotation"],
                            _conv_pts(np.asarray(mrim[0])))
            cap = max(1, len(mw) // 128)
            samp = mw[::cap]
            nm, cm, _ = _plane_fit(samp)
            ang = min(_angle_deg(nm, nf), 180.0 - _angle_deg(nm, nf))
            rel_c = cm - cf
            centre_lat = float(np.linalg.norm(
                rel_c - float(rel_c @ nf) * nf))
            rel = samp - cf
            axial = rel @ nf
            lateral = np.linalg.norm(
                rel - np.outer(axial, nf), axis=1)
            rm = float(np.median(lateral))
            rad_res = float(np.abs(lateral - rm).max())
            ax_spread = float(
                np.abs(axial - np.median(axial)).max())
            tag = f"{fuid}x{muid}@{angle:g}"
            out["log"][tag] = {
                "normal_deg": round(ang, 2),
                "centre_lateral_mm": round(centre_lat, 3),
                "rim_radius_mm": round(rm, 2),
                "radius_resid_mm": round(rad_res, 3),
                "axial_spread_mm": round(ax_spread, 3)}
            if ang > COAX_ANGLE_DEG:
                out["worst_hits"][f"normal@{tag}"] = (
                    VERIFY_LATERAL_MM + 1.0)
            out["worst_hits"][f"centre@{tag}"] = centre_lat
    return out


def verify_joint(axis_pt, axis_dir, up_uids: set, dn_uids: set,
                 anchors: list[dict], parts: list[dict],
                 catalog: dict, lib=None) -> dict:
    """C5 self-verification by rigid downstream rotation.

    (a) all downstream members move as one rigid body, (b) upstream
    poses are byte-identical, (c) bearing elements stay coaxial.
    The (c) turntable check is restored to full sensitivity: after
    rotation the moving ring's annulus plane normal must stay
    parallel to the fixed ring's normal (<=COAX_ANGLE_DEG) AND the
    moving rim samples must remain on the fixed raceway circle
    (<=VERIFY_LATERAL_MM). Origin-coincidence alone is degenerate
    (any axis through the centre passes it) and is never used.
    """
    by_uid = {p["uid"]: p for p in parts}
    axis_pt = np.asarray(axis_pt, dtype=float)
    axis_dir = _unit(np.asarray(axis_dir, dtype=float))
    snap_up = {u: (list(by_uid[u]["position_mm"]),
                   [list(r) for r in by_uid[u]["rotation"]])
               for u in up_uids}
    dn_list = sorted(dn_uids)
    # (a) coverage + rigidity spot check at +30 deg.
    moved = {u: _rotated_member(by_uid[u], axis_pt, axis_dir, 30.0)
             for u in dn_list}
    rep = dn_list[:20]
    rigid_ok = True
    for i in range(len(rep)):
        for j in range(i + 1, len(rep)):
            a, b = by_uid[rep[i]], by_uid[rep[j]]
            d0 = float(np.linalg.norm(np.asarray(a["position_mm"])
                                      - np.asarray(b["position_mm"])))
            ma, mb = moved[rep[i]], moved[rep[j]]
            d1 = float(np.linalg.norm(np.asarray(ma["position_mm"])
                                      - np.asarray(mb["position_mm"])))
            if abs(d0 - d1) > 1e-6:
                rigid_ok = False
    coverage_ok = set(moved) == set(dn_list)
    # Engaged seats at angle 0: only holes the anchor line
    # pierces count afterwards (other holes legitimately leave).
    engaged: dict[str, list[tuple[int, np.ndarray]]] = {}
    for anc in anchors:
        if anc.get("kind", "pin") != "pin":
            continue
        owner = by_uid[anc["uid"]]
        line_pt = np.asarray(owner["position_mm"], dtype=float)
        line_dir = _unit(np.asarray(owner["rotation"], dtype=float)
                         @ _pin_local_dir(owner, catalog))
        for u in anc["hits_up"] + anc["hits_dn"]:
            keep = []
            for i, hp in _hole_centres(by_uid[u], catalog):
                rel = hp - line_pt
                dev = rel - float(np.dot(rel, line_dir)) * line_dir
                if float(np.linalg.norm(dev)) <= COAX_LATERAL_MM:
                    keep.append((i, hp))
            engaged.setdefault(anc["uid"], []).append((u, keep))
    worst = 0.0
    notes = []
    sense_log: dict = {}
    for angle in VERIFY_DEG:
        rot = {u: _rotated_member(by_uid[u], axis_pt, axis_dir, angle)
               for u in dn_list}
        for anc in anchors:
            if anc.get("kind", "pin") == "pin":
                # Anchor rides with its clamped side; seats are the
                # mating holes on both sides (dn seats rotated, up
                # seats static). Anchors outside the upstream set
                # ride downstream.
                owner = anc["uid"]
                owner_rot = (by_uid[owner] if owner in up_uids
                             else rot.get(owner, by_uid[owner]))
                line_pt = np.asarray(owner_rot["position_mm"], dtype=float)
                line_dir = _unit(np.asarray(
                    owner_rot["rotation"], dtype=float)
                    @ _pin_local_dir(by_uid[owner], catalog))
                for u, keep in engaged.get(anc["uid"], []):
                    member = rot.get(u, by_uid[u])
                    centres = dict(_hole_centres(member, catalog))
                    for i, _hp0 in keep:
                        if i not in centres:
                            continue
                        rel = centres[i] - line_pt
                        dev = rel - float(np.dot(rel, line_dir)) \
                            * line_dir
                        worst = max(worst, float(np.linalg.norm(dev)))
            elif anc.get("kind") == "rack-ring":
                # Per-side ring axes: static upstream fit vs rotated
                # downstream fit. A wrong candidate axis swings the
                # downstream ring (angle or lateral vs upstream).
                up_arcs = [by_uid[u] for u in anc["uids"]
                           if u in up_uids]
                dn_arcs = [rot.get(u, by_uid[u]) for u in anc["uids"]
                           if u in dn_uids]
                fa = _ring_side_axis(up_arcs)
                fb = _ring_side_axis(dn_arcs)
                if fa is None or fb is None:
                    notes.append("rack side with <3 arcs; weak check")
                    continue
                ang = _angle_deg(fa[0], fb[0])
                ang = min(ang, 180.0 - ang)
                rel = fb[1] - fa[1]
                lateral = float(np.linalg.norm(
                    rel - float(rel @ fa[0]) * fa[0]))
                worst = max(worst, lateral)
                if ang > COAX_ANGLE_DEG:
                    worst = max(worst, VERIFY_LATERAL_MM + 1.0)
            else:
                # Turntable anchor: restored-sensitivity (c). The
                # legacy origin-coincidence check is degenerate (any
                # axis through the coincident centre passes), so the
                # moving ring's annulus normal and raceway samples
                # are tested against the fixed ring instead.
                fixed = [v for v in anc["uids"] if v in up_uids]
                moving = [v for v in anc["uids"] if v in dn_uids]
                sense = _ring_sense(fixed, moving, rot, by_uid, catalog,
                                    lib, angle)
                for key, val in sense["worst_hits"].items():
                    worst = max(worst, val)
                sense_log.setdefault(angle, {}).update(sense["log"])
    upstream_ok = all(
        list(by_uid[u]["position_mm"]) == snap_up[u][0]
        and [list(r) for r in by_uid[u]["rotation"]] == snap_up[u][1]
        for u in up_uids)
    verdict = ("pass" if (coverage_ok and rigid_ok and upstream_ok
                          and worst <= VERIFY_LATERAL_MM) else "fail")
    if verdict == "fail":
        if not (coverage_ok and rigid_ok):
            notes.append("downstream did not move as one rigid body")
        if not upstream_ok:
            notes.append("upstream poses changed")
        if worst > VERIFY_LATERAL_MM:
            notes.append(
                f"bearing coaxiality lost after rotation "
                f"({worst:.2f}mm > {VERIFY_LATERAL_MM}mm)")
    return {"verdict": verdict,
            "worst_lateral_mm": round(worst, 3),
            "coverage_ok": coverage_ok, "rigid_ok": rigid_ok,
            "upstream_ok": upstream_ok, "notes": notes,
            "sense": sense_log}


def sweep_penetration(axis_pt, axis_dir, up_uids: set, dn_uids: set,
                      parts: list[dict], lib, catalog=None) -> dict:
    """C5-d non-penetration sweep: rotate dn about the candidate axis.

    Mesh-exact (AABB-vs-AABB proved worthless in a dense LEGO pack:
    60-90mm boxes overlap parts standing 21mm apart). AABBs are only
    the complete broadphase; the verdict is mesh containment
    (sampled vertices inside the other mesh, majority-vote ray
    cast). Pairs already interpenetrating at angle 0 are engaged
    seats (meshed teeth legitimately intersect) and skipped; only
    NEW mesh penetrations during the +/-30 deg sweep reject. Grazing
    without containment is reported via min vertex gaps, never a
    verdict. Mesh-less parts are listed as uncovered, never guessed.
    """
    by_uid = {p["uid"]: p for p in parts}
    axis_pt = np.asarray(axis_pt, dtype=float)
    axis_dir = _unit(np.asarray(axis_dir, dtype=float))
    tris: dict[str, np.ndarray] = {}
    uncovered = []
    for u in sorted(up_uids | dn_uids):
        t = _tris_local(lib, by_uid[u]["part_id"])
        if t is None:
            uncovered.append(u)
            continue
        tris[u] = t
    up_list = sorted(u for u in up_uids if u in tris)
    dn_list = sorted(u for u in dn_uids if u in tris)
    # Full-resolution world meshes feed spheres/reach (conservative
    # bounds); decimated copies feed the narrowphase.
    world0_full = {u: _world_tris(by_uid[u]["position_mm"],
                                  by_uid[u]["rotation"], tris[u])
                   for u in up_list + dn_list}
    stris = {u: _decimate(tris[u], SWEEP_VOXEL_MM)
             for u in up_list + dn_list}
    world0 = {u: _world_tris(by_uid[u]["position_mm"],
                             by_uid[u]["rotation"], stris[u])
              for u in up_list + dn_list}
    # Bounding spheres (sound prefilter: spheres contain meshes, so a
    # positive sphere gap proves no contact without any narrowphase).
    sph0: dict = {}
    for u in up_list + dn_list:
        verts = np.unique(world0_full[u].reshape(-1, 3), axis=0)
        ctr = verts.mean(axis=0)
        sph0[u] = (ctr, float(
            np.linalg.norm(verts - ctr, axis=1).max()))
    boxes0 = {u: _world_pts(by_uid[u]["position_mm"],
                            by_uid[u]["rotation"],
                            _box_local(lib, by_uid[u]["part_id"]))
              for u in up_list + dn_list}
    # Pivot reach per dn part: nothing starting farther from an up
    # mesh than this can touch it during the sweep (sound skip).
    reach = {}
    for v in dn_list:
        verts = np.unique(world0_full[v].reshape(-1, 3), axis=0)
        rel = verts - axis_pt
        lateral = np.linalg.norm(
            rel - np.outer(rel @ axis_dir, axis_dir), axis=1)
        reach[v] = float(lateral.max()) * SWEEP_REACH + SWEEP_SLACK_MM
    engaged = set()
    engaged_mesh = set()
    cand = set()

    def _sphere_gap(cu, ru, cv, rv) -> float:
        return float(np.linalg.norm(cu - cv)) - ru - rv

    toothed = {u: (catalog is not None
                   and geom._entry(catalog,
                                   by_uid[u]["part_id"]).get("teeth")
                   is not None) for u in up_list + dn_list}
    for u in up_list:
        for v in dn_list:
            # Sound skip: spheres contain meshes, so sphere-gap <=
            # mesh-gap; beyond pivot reach the pair can never touch
            # at any sweep angle (no narrowphase needed, ever).
            if _sphere_gap(sph0[u][0], sph0[u][1], sph0[v][0],
                           sph0[v][1]) > reach[v]:
                continue
            cand.add((u, v))
            if not _aabbs_overlap(boxes0[u], boxes0[v]):
                continue
            if _meshes_contain(world0[u], world0[v]):
                engaged.add((u, v))
            elif toothed[u] and toothed[v] and geom.check_gear_mesh(
                    u, v, parts, catalog).get("meshes"):
                # Gear-meshed across the joint: a drive contact, not a
                # structural seat. It must roll during joint motion, so
                # freezing it rigidly would fake a collision; the mesh
                # rule owns this pair, not the sweep.
                engaged_mesh.add((u, v))
    new_hits: dict = {}
    near_miss: dict = {}
    spin_cache: dict = {}
    spun = set()
    for (u, v) in sorted(cand):
        if _spin_free(u, parts, catalog or {}, by_uid, axis_pt,
                       axis_dir, spin_cache) \
                or _spin_free(v, parts, catalog or {}, by_uid,
                               axis_pt, axis_dir, spin_cache):
            spun.add((u, v))
    n_spinning = len(spun)
    cand -= spun
    for angle in SWEEP_DEG:
        rot = {u: _rotated_member(by_uid[u], axis_pt, axis_dir, angle)
               for u in dn_list}
        mvc = {}
        for v in dn_list:
            pv = _rotated_member(
                {"position_mm": [float(x) for x in sph0[v][0]],
                 "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                              [0.0, 0.0, 1.0]]},
                axis_pt, axis_dir, angle)
            mvc[v] = np.asarray(pv["position_mm"], dtype=float)
        moved = {u: _world_tris(rot[u]["position_mm"],
                                rot[u]["rotation"], stris[u])
                 for u in dn_list}
        moved_boxes = {
            u: _world_pts(rot[u]["position_mm"], rot[u]["rotation"],
                          _box_local(lib, by_uid[u]["part_id"]))
            for u in dn_list}
        hits = []
        for (u, v) in sorted(cand - engaged - engaged_mesh):
            if _sphere_gap(sph0[u][0], sph0[u][1], mvc[v],
                           sph0[v][1]) > 0.0:
                continue
            if not _aabbs_overlap(boxes0[u], moved_boxes[v]):
                continue
            inside, gap = _meshes_intersect(world0[u], moved[v])
            if not inside:
                # Graze without containment cannot be a deep stab
                # (plate-edge slicing excepted, documented); the
                # crossing test runs only on containment pairs.
                if gap < 2.0:
                    near_miss.setdefault(str(angle), []).append(
                        (u, v, round(gap, 2), 0, 0.0, 0, 0))
                continue
            cross, n_cross = _tris_cross(world0[u], moved[v])
            n_in, depth, n_conf = _contain_confirmed(world0[u],
                                                    moved[v])
            # Verdict-grade: shell triangles cross AND confirmed
            # depth exceeds fidelity (crossing alone also fires on
            # grazing box faces; depth alone also fires on hollow
            # hovering; both together mean a real stab).
            if cross and n_conf > 0 and depth > PENETRATE_DEPTH_MM:
                if catalog is not None and _cross_mounted(
                        u, v, parts, catalog, by_uid):
                    kind = "mounted"
                elif catalog is not None and _toothed_pair(
                        u, v, catalog, by_uid):
                    kind = "meshing"
                else:
                    kind = "paradox"
                hits.append((u, v, round(gap, 2), n_in, depth, n_conf,
                             n_cross, kind))
            else:
                near_miss.setdefault(str(angle), []).append(
                    (u, v, round(gap, 2), n_in, depth, n_conf, n_cross))
        if hits:
            new_hits[str(angle)] = hits
    return {"verdict": "pass" if not new_hits else "fail",
            "new_penetrations": new_hits, "near_miss_sub_mm": near_miss,
            "n_engaged_at_0": len(engaged),
            "n_engaged_mesh": len(engaged_mesh),
            "n_spinning_skipped": n_spinning,
            "n_candidates": len(cand),
            "n_uncovered": len(uncovered), "uncovered": uncovered}


def _perp_unit(d: np.ndarray) -> np.ndarray:
    """Deterministic unit vector perpendicular to d."""
    d = _unit(np.asarray(d, dtype=float))
    e = np.array([1.0, 0.0, 0.0])
    if abs(float(d @ e)) >= 0.9:
        e = np.array([0.0, 1.0, 0.0])
    return _unit(np.cross(d, e))


def negative_control(axis_pt, axis_dir, up_uids: set, dn_uids: set,
                     anchors: list[dict], parts: list[dict],
                     catalog: dict, lib=None) -> dict:
    """C5-n: the confirmed axis with an intentional 90 deg error.

    Runs the full C5 (abc + restored sense) plus the C5-d sweep on
    the perturbed axis. Both must reject it; if either passes, the
    checks have regressed to a degenerate state and the alarm must
    be reported (never silently kept).
    """
    err_dir = _perp_unit(np.asarray(axis_dir, dtype=float))
    ver = verify_joint(axis_pt, err_dir, up_uids, dn_uids, anchors,
                       parts, catalog, lib)
    sweep = (sweep_penetration(axis_pt, err_dir, up_uids, dn_uids,
                               parts, lib, catalog)
             if lib is not None else {"verdict": "no-lib"})
    rejected = (ver["verdict"] == "fail"
                or sweep.get("verdict") == "fail")
    return {"perturbed_axis": [round(float(v), 4) for v in err_dir],
            "verify_verdict": ver["verdict"],
            "verify_worst_mm": ver["worst_lateral_mm"],
            "sweep_verdict": sweep.get("verdict"),
            "rejected": rejected}


def _ring_side_axis(arcs: list) -> tuple | None:
    """Axis + centre of one side's rack arcs (linear or planar)."""
    if len(arcs) < 3:
        return None
    pts = np.asarray([np.asarray(a["position_mm"], dtype=float)
                      for a in arcs])
    mean = pts.mean(axis=0)
    _, s, vh = np.linalg.svd(pts - mean)
    if s[1] / max(s[0], 1e-9) < 0.05:
        return (_unit(vh[0]), mean)
    if s[2] / max(s[1], 1e-9) < 0.05:
        return (_unit(vh[2]), mean)
    return None


def _pin_local_dir(part: dict, catalog: dict) -> np.ndarray:
    axis = geom._entry(catalog, part["part_id"]).get("axis_local")
    if axis is None:
        return np.array([1.0, 0.0, 0.0])
    return geom._conv_dir(axis)


def _hole_centres(part: dict, catalog: dict) -> list:
    """Indexed world hole centres of a part: [(index, centre)]."""
    holes = geom._entry(catalog, part["part_id"]).get("holes") or []
    out = []
    for i, hole in enumerate(holes):
        if isinstance(hole, dict):
            try:
                out.append((i, geom.hole_world(part, hole)))
            except Exception:
                continue
    return out


def _hole_index(part_id: str, catalog: dict, hole) -> int:
    holes = geom._entry(catalog, part_id).get("holes") or []
    for i, h in enumerate(holes):
        if h is hole:
            return i
    return -1


def nested_anchor(up: dict, dn: dict, parts: list[dict],
                  catalog: dict, lib=None) -> dict:
    """C3'/C3''' bearing from a nested ring pair across the boundary.

    Teeth never disqualify: a turntable/ring gear is a dual-role part
    (bearing + drive input). Bearing = concentric nesting of two
    SYMMETRIC ring parts from different groups within NEST_EPS_MM;
    no through-pin required. Symmetric-ring origins coincide at the
    symmetry centre by construction (verified by the coincidence
    itself); this path never applies to asymmetric arc pieces --
    curved rack arcs go through rack_bundle only.
    C3''': the ring rotation axis is the annulus plane normal (thrust
    bearing definition). The axis is the JOINT plane fit of the
    pooled rim vertices of both halves in world mm; adjacent
    third-party parts (rotors, gears) are never axis authority --
    their catalog axes are diagnostics only, and a side-vs-joint
    disagreement is resolved by the joint refit, never by following
    the third party (C3'' violation). The centre (C6 free DOF) is
    the joint plane centroid. Without a mesh library the legacy
    rotor-side catalog axis is used and flagged as a fallback.
    """
    by_uid = {p["uid"]: p for p in parts}
    a, b = by_uid[up["uid"]], by_uid[dn["uid"]]
    axa = geom.axis_world(a, catalog)
    axb = geom.axis_world(b, catalog)
    ca = np.asarray(a["position_mm"], dtype=float)
    cb = np.asarray(b["position_mm"], dtype=float)
    diagnostics: dict = {
        "nest_dist_mm": round(float(np.linalg.norm(ca - cb)), 3)}
    for uid, ax in ((up["uid"], axa), (dn["uid"], axb)):
        if ax is not None:
            diagnostics.setdefault("catalog_axes", {})[uid] = [
                round(float(v), 4) for v in _unit(ax)]
    nest = float(np.linalg.norm(ca - cb))
    if nest > NEST_EPS_MM:
        return {"ok": False, "class": "info",
                "note": f"ring pair not nested ({nest:.2f}mm)",
                "diagnostics": diagnostics}
    rims = {}
    if lib is not None:
        for uid, part in ((up["uid"], a), (dn["uid"], b)):
            rim = _rim_ldu(lib, part["part_id"])
            if rim is not None:
                local = _conv_pts(np.asarray(rim[0]))
                rims[uid] = (
                    _world_pts(part["position_mm"],
                               part["rotation"], local),
                    rim[1] * LDU_MM)
    if len(rims) == 2:
        wa = rims[up["uid"]][0]
        wb = rims[dn["uid"]][0]
        direction, center, rms = _plane_fit(np.vstack([wa, wb]))
        na, cma, _ = _plane_fit(wa)
        nb, cmb, _ = _plane_fit(wb)
        deva = round(min(_angle_deg(na, direction),
                         180.0 - _angle_deg(na, direction)), 2)
        devb = round(min(_angle_deg(nb, direction),
                         180.0 - _angle_deg(nb, direction)), 2)
        diagnostics["axis_source"] = "annulus-joint"
        diagnostics["side_normal_dev_deg"] = {up["uid"]: deva,
                                              dn["uid"]: devb}
        diagnostics["joint_plane_rms_mm"] = round(rms, 3)
        ring_r = round(float(np.median(
            [rims[up["uid"]][1], rims[dn["uid"]][1]])), 2)
        rel = cmb - cma
        lateral = float(np.linalg.norm(
            rel - float(rel @ direction) * direction))
        if axa is not None:
            ang = round(min(_angle_deg(axa, direction),
                            180.0 - _angle_deg(axa, direction)), 2)
            diagnostics.setdefault("catalog_mismatch_deg",
                                   {})[up["uid"]] = ang
        if axb is not None:
            ang = round(min(_angle_deg(axb, direction),
                            180.0 - _angle_deg(axb, direction)), 2)
            diagnostics.setdefault("catalog_mismatch_deg",
                                   {})[dn["uid"]] = ang
        return {"ok": True, "direction": direction, "center": center,
                "kind": "turntable", "uids": [up["uid"], dn["uid"]],
                "ring_radius": ring_r, "ring_residual_mm": round(rms, 3),
                "halves_lateral_mm": round(lateral, 3),
                "n_features": len(wa) + len(wb),
                "diagnostics": diagnostics}
    if axb is None:
        return {"ok": False, "class": "info",
                "note": "rotor-side catalog axis missing "
                        "(no annulus mesh)",
                "diagnostics": diagnostics}
    diagnostics["axis_source"] = "catalog-fallback"
    direction = _unit(axb)
    if axa is not None:
        # Inter-half axis disagreement (direction follows the dn
        # rotor side); a large value here is a diagnostic (C3''),
        # e.g. a drive-oriented catalog axis on a bearing ring.
        ang = _angle_deg(axa, axb)
        ang = round(min(ang, 180.0 - ang), 2)
        diagnostics["axis_mismatch_deg"] = {up["uid"]: ang,
                                            dn["uid"]: ang}
    center = (ca + cb) / 2.0
    return {"ok": True, "direction": direction, "center": center,
            "kind": "turntable", "uids": [up["uid"], dn["uid"]],
            "ring_radius": None, "ring_residual_mm": None,
            "halves_lateral_mm": round(nest, 3),
            "n_features": 0, "diagnostics": diagnostics}


def _axle_seats(anchor: dict, parts: list[dict],
                catalog: dict) -> tuple[list, list]:
    """Bearing seats of an axle: holed parts pierced by its line.

    Returns ([(uid, hole_index)] upstream, [...downstream]). A line
    merely passing near a part (line-hit) without piercing a catalog
    hole is not a seat.
    """
    by_uid = {p["uid"]: p for p in parts}
    owner = by_uid[anchor["uid"]]
    line_pt = np.asarray(owner["position_mm"], dtype=float)
    line_dir = _unit(np.asarray(owner["rotation"], dtype=float)
                     @ _pin_local_dir(owner, catalog))
    seats = []
    for u in anchor["hits_up"] + anchor["hits_dn"]:
        for i, hp in _hole_centres(by_uid[u], catalog):
            rel = hp - line_pt
            dev = rel - float(np.dot(rel, line_dir)) * line_dir
            if float(np.linalg.norm(dev)) <= COAX_LATERAL_MM:
                seats.append((u, i))
    up_set = set(anchor["hits_up"])
    return ([s for s in seats if s[0] in up_set],
            [s for s in seats if s[0] not in up_set])


def _axle_membership(anchor: dict, center, direction, up: str, dn: str,
                     parts: list[dict], catalog: dict) -> str:
    """Which group a spanning axle belongs to (six-axle C5 rule).

    Ownership follows the bearing seats: an axle whose seats all lie
    upstream stops with the upstream group under C5 rotation about
    the final joint axis (and vice versa). A both-sides-seated axle
    off the joint axis breaks under C5 (departure distances quoted);
    on the joint axis it is C5-invariant and structurally
    indeterminate. No reassignment ever follows (C0).
    """
    by_uid = {p["uid"]: p for p in parts}
    seats_up, seats_dn = _axle_seats(anchor, parts, catalog)
    c = np.asarray(center, dtype=float)
    d = _unit(np.asarray(direction, dtype=float))
    rel = np.asarray(anchor["center"], dtype=float) - c
    lateral = float(np.linalg.norm(rel - float(rel @ d) * d))
    if lateral <= COAX_LATERAL_MM:
        return (f"{anchor['uid']} on the {up}->{dn} axis "
                f"(lateral {lateral:.2f}mm): C5-invariant, owner "
                f"structurally indeterminate")
    if seats_up and not seats_dn:
        return (f"{anchor['uid']} seated only in {up} "
                f"({len(seats_up)} hole(s)): stops with {up} under C5, "
                f"{up}-owned")
    if seats_dn and not seats_up:
        return (f"{anchor['uid']} seated only in {dn} "
                f"({len(seats_dn)} hole(s)): rides with {dn} under C5, "
                f"{dn}-owned")
    if seats_up and seats_dn:
        # Both sides seated off-axis: quote the C5 break as evidence.
        owner = by_uid[anchor["uid"]]
        line_pt = np.asarray(owner["position_mm"], dtype=float)
        line_dir = _unit(np.asarray(owner["rotation"], dtype=float)
                         @ _pin_local_dir(owner, catalog))
        worst_dn, worst_up = 0.0, 0.0
        for angle in VERIFY_DEG:
            rot = {u: _rotated_member(by_uid[u], c, d, angle)
                   for u in anchor["hits_dn"]}
            for u, i in seats_dn:
                centres = dict(_hole_centres(rot[u], catalog))
                if i not in centres:
                    continue
                rel2 = centres[i] - line_pt
                dev = rel2 - float(rel2 @ line_dir) * line_dir
                worst_dn = max(worst_dn, float(np.linalg.norm(dev)))
            axle_rot = _rotated_member(owner, c, d, angle)
            lp = np.asarray(axle_rot["position_mm"], dtype=float)
            ld = _unit(np.asarray(axle_rot["rotation"], dtype=float)
                       @ _pin_local_dir(owner, catalog))
            for u, i in seats_up:
                centres = dict(_hole_centres(by_uid[u], catalog))
                if i not in centres:
                    continue
                rel2 = centres[i] - lp
                dev = rel2 - float(rel2 @ ld) * ld
                worst_up = max(worst_up, float(np.linalg.norm(dev)))
        return (f"{anchor['uid']} seated both sides off-axis: dn seats "
                f"depart {worst_dn:.2f}mm (axle static) / up seats depart "
                f"{worst_up:.2f}mm (axle riding dn) under C5 +-30deg; "
                f"spans the boundary")
    return (f"{anchor['uid']} pierces no catalog hole on either side: "
            f"proximity only, boundary suspect")


def judge_clamped(anchor: dict, center, direction, up_uids: set,
                  dn_uids: set, parts: list[dict],
                  catalog: dict, up_name: str = "up",
                  dn_name: str = "dn", lib=None) -> tuple[str, str]:
    """C5 verdict for an axle clamped (non-bush) on both sides.

    Solo C5 about the final joint axis: pass + coaxial -> hinge
    co-bearing. Fail + gear rigidly on the axle -> drive shaft
    (slew/drive pinion; mesh human-confirmed). Else boundary stands.
    Every verdict carries the C5 seat-side membership (six-axle
    rule): the group holding the bearing seats owns the axle.
    """
    by_uid = {p["uid"]: p for p in parts}
    solo = dict(anchor)
    solo["kind"] = "pin"
    ver = verify_joint(np.asarray(center, dtype=float),
                       np.asarray(direction, dtype=float),
                       up_uids, dn_uids, [solo], parts, catalog, lib)
    c = np.asarray(center, dtype=float)
    d = _unit(np.asarray(direction, dtype=float))
    rel = np.asarray(anchor["center"], dtype=float) - c
    lateral = float(np.linalg.norm(rel - float(rel @ d) * d))
    member = _axle_membership(anchor, center, direction, up_name,
                              dn_name, parts, catalog)
    anchor["membership"] = member
    if ver["verdict"] == "pass" and lateral <= COAX_LATERAL_MM:
        return ("hinge",
                f"{anchor['uid']} C5-pass coaxial co-bearing "
                f"(worst {ver['worst_lateral_mm']}mm). {member}")
    gears = [conn["uid"]
             for conn in geom.find_connections(anchor["uid"], parts,
                                               catalog)
             if conn["connection"] == "axle_in_axle_hole"
             and geom._entry(catalog,
                             by_uid[conn["uid"]]["part_id"]).get(
                                 "category") == "gear"]
    if gears:
        return ("shaft",
                f"{anchor['uid']} carries gear(s) "
                f"{sorted(gears)[:4]} and fails C5 as a bearing "
                f"(worst {ver['worst_lateral_mm']}mm): drive pinion, "
                f"mesh human-confirm. {member}")
    return ("boundary",
            f"{anchor['uid']} clamped both sides, fails C5 "
            f"(worst {ver['worst_lateral_mm']}mm): boundary suspect "
            f"(C0), no reassignment. {member}")


def _proximal_distal(direction: np.ndarray, up_uids: set, dn_uids: set,
                     by_uid: dict) -> np.ndarray:
    """C6 direction sign: proximal -> distal along the chain."""
    up_c = np.mean([np.asarray(by_uid[u]["position_mm"], dtype=float)
                    for u in up_uids], axis=0)
    dn_c = np.mean([np.asarray(by_uid[u]["position_mm"], dtype=float)
                    for u in dn_uids], axis=0)
    if float(np.dot(direction, dn_c - up_c)) < 0:
        return -direction
    return direction


def infer_chain(div_path: str, parts: list[dict],
                catalog: dict, lib=None) -> dict:
    """Tier-1 inference over D0..D6. No GT, no Spark, deterministic.

    `lib` is an LDraw mesh library (Studio tree): with it, ring axes
    use the C3''' annulus joint fit, C5(c) runs at restored
    sensitivity, and the C5-d sweep plus C5-n control execute.
    Without it the legacy catalog-axis path runs and is flagged.
    """
    by_uid = {p["uid"]: p for p in parts}
    div = load_div_groups(div_path, parts)
    members = {g: set(div["members"].get(g, [])) for g in CHAIN}
    joints = []
    suspects = []
    deficits = []
    for k in range(1, 7):
        up, dn = CHAIN[k - 1], CHAIN[k]
        up_uids, dn_uids = members[up], members[dn]
        # Bearing anchors (C3'): nested ring pairs across the
        # boundary. Teeth never disqualify; no through-pin required.
        tt_anchors = []
        for uid in sorted(up_uids):
            if geom._entry(catalog,
                           by_uid[uid]["part_id"]).get("category") \
                    != "turntable":
                continue
            for vid in sorted(dn_uids):
                if geom._entry(catalog,
                               by_uid[vid]["part_id"]).get("category") \
                        != "turntable":
                    continue
                tt_anchors.append(
                    nested_anchor({"uid": uid}, {"uid": vid},
                                  parts, catalog, lib))
        tt_ok = [t for t in tt_anchors if t.get("ok")]
        for t in tt_anchors:
            if not t.get("ok") and t.get("class") != "info":
                suspects.append(
                    {"pair": (up, dn), "class": t.get("class"),
                     "note": t.get("note")})
        pins = pin_anchors(up_uids, dn_uids, parts, catalog)
        pin_hinge = []
        pin_clamped = []
        for anchor in pins:
            cls, note = classify_pin(anchor)
            if cls == "hinge":
                anchor["kind"] = "pin"
                pin_hinge.append(anchor)
            elif cls == "boundary":
                # C5 judges clamped-both axles against the final axis
                # below (hinge co-bearing vs drive shaft vs boundary).
                anchor["kind"] = "pin"
                pin_clamped.append((anchor, note))
            else:
                suspects.append({"pair": (up, dn), "class": cls,
                                 "uid": anchor["uid"], "note": note})
        # C1: exactly one joint per adjacent pair; merge coaxial.
        cands = [{"direction": np.asarray(t["direction"]),
                  "center": np.asarray(t["center"]),
                  "kind": t["kind"], "anchor": t,
                  "err": t.get("halves_lateral_mm", 0.0)} for t in tt_ok]
        cands += [{"direction": np.asarray(a["direction"]),
                   "center": np.asarray(a["center"]),
                   "kind": "pin", "anchor": a,
                   "err": a["fit_error_mm"]} for a in pin_hinge]
        rack = rack_bundle(up_uids, dn_uids, parts, catalog)
        if rack.get("ok"):
            cands.append({"direction": np.asarray(rack["direction"]),
                          "center": np.asarray(rack["center"]),
                          "kind": "rack-ring", "anchor": rack,
                          "err": rack["fit_error_mm"]})
        elif rack.get("class"):
            suspects.append({"pair": (up, dn), "class": rack["class"],
                             "note": rack.get("note")})
        clusters: list[list[int]] = []
        for i, cand in enumerate(cands):
            for cl in clusters:
                j = cl[0]
                da = _unit(cands[j]["direction"])
                db = _unit(cand["direction"])
                ang = _angle_deg(da, db)
                ang = min(ang, 180.0 - ang)
                rel = cand["center"] - cands[j]["center"]
                lateral = float(np.linalg.norm(
                    rel - float(rel @ da) * da))
                if ang <= COAX_ANGLE_DEG and lateral <= COAX_LATERAL_MM:
                    cl.append(i)
                    break
            else:
                clusters.append([i])
        if not clusters:
            # C4': deficit only after the C3' relaxed re-search above
            # (nested rings without pins) also finds nothing.
            deficits.append(
                {"joint": f"J{k}", "pair": (up, dn),
                 "note": "no bearing anchor across the boundary after "
                         "C3' relaxed re-search; boundary suspect (C0), "
                         "no reassignment"})
            for anchor, note in pin_clamped:
                suspects.append({"pair": (up, dn), "class": "boundary",
                                 "uid": anchor["uid"], "note": note})
            continue
        if len(clusters) > 1:
            suspects.append(
                {"pair": (up, dn), "class": "boundary",
                 "note": f"{len(clusters)} non-coaxial anchor clusters; "
                         f"keeping ring-bearing first, rest suspect"})
        # Rings before pins: a turntable/rack ring is the primary
        # rotation bearing; pins are secondary. Then size, then error.
        def _rank(cl):
            kinds = {cands[i]["kind"] for i in cl}
            ring = 1 if kinds & {"turntable", "turntable-no-features",
                                 "rack-ring"} else 0
            err = max(cands[i]["err"] for i in cl)
            return (ring, len(cl), -err)
        best = max(clusters, key=_rank)
        members_c = [cands[i] for i in best]
        # SVD signs are arbitrary (and hash-order dependent): align
        # all member directions to one hemisphere before averaging,
        # or opposite signs cancel into a garbage direction.
        ref = np.asarray(members_c[0]["direction"], dtype=float)
        aligned = []
        for c in members_c:
            d = np.asarray(c["direction"], dtype=float)
            aligned.append(d if float(d @ ref) >= 0 else -d)
        direction = _unit(np.mean(aligned, axis=0))
        center = np.mean([c["center"] for c in members_c], axis=0)
        direction = _proximal_distal(direction, up_uids, dn_uids, by_uid)
        ver = verify_joint(center, direction, up_uids, dn_uids,
                           [c["anchor"] for c in members_c],
                           parts, catalog, lib)
        if ver["verdict"] == "fail":
            suspects.append(
                {"joint": f"J{k}", "pair": (up, dn), "class": "verify",
                 "note": "; ".join(ver["notes"])})
            continue
        hinge_new = []
        for anchor, note in pin_clamped:
            verdict = judge_clamped(anchor, center, direction, up_uids,
                                    dn_uids, parts, catalog, up, dn, lib)
            if verdict[0] == "hinge":
                hinge_new.append(anchor)
            suspects.append({"pair": (up, dn), "class": verdict[0],
                             "uid": anchor["uid"], "note": verdict[1]})
        if hinge_new:
            # Merge coaxial co-bearings, re-verify, revert on fail.
            saved = (direction, center, list(members_c), ver)
            for anchor in hinge_new:
                members_c.append({"direction": np.asarray(
                    anchor["direction"]), "center": np.asarray(
                        anchor["center"]), "kind": "pin", "anchor": anchor,
                    "err": anchor["fit_error_mm"]})
            ref = direction
            ds = [np.asarray(c["direction"], dtype=float)
                  for c in members_c]
            ds = [d if float(d @ ref) >= 0 else -d for d in ds]
            direction = _unit(np.mean(ds, axis=0))
            center = np.mean([c["center"] for c in members_c], axis=0)
            direction = _proximal_distal(direction, up_uids, dn_uids,
                                         by_uid)
            ver = verify_joint(center, direction, up_uids, dn_uids,
                               [c["anchor"] for c in members_c],
                               parts, catalog, lib)
            if ver["verdict"] == "fail":
                fail_notes = list(ver["notes"])
                direction, center, members_c, ver = saved
                suspects.append(
                    {"joint": f"J{k}", "pair": (up, dn),
                     "class": "verify",
                     "note": "merged co-bearing re-verify failed; "
                             "kept unmerged; " + "; ".join(fail_notes)})
        for c in members_c:
            if c["kind"] == "pin" and "membership" not in c["anchor"]:
                c["anchor"]["membership"] = _axle_membership(
                    c["anchor"], center, direction, up, dn, parts,
                    catalog)
        if lib is not None:
            sweep = sweep_penetration(center, direction, up_uids,
                                      dn_uids, parts, lib, catalog)
            ver["penetration"] = sweep
            if sweep["verdict"] == "fail":
                all_hits = [h for hits in
                            sweep["new_penetrations"].values()
                            for h in hits]
                paradox = [h for h in all_hits if h[7] == "paradox"]
                dn_paradox = sorted({h[1] for h in paradox})
                first = all_hits[0]
                summary = (f"{sweep['n_engaged_at_0']} engaged at 0, "
                           f"{sweep.get('n_engaged_mesh', 0)} gear-meshed, "
                           f"{sweep.get('n_spinning_skipped', 0)} "
                           f"spin-skipped, "
                           f"{sweep.get('n_candidates', 0)} in reach, "
                           f"{sweep['n_uncovered']} mesh-less skipped")
                if len(dn_paradox) >= 2:
                    suspects.append(
                        {"joint": f"J{k}", "pair": (up, dn),
                         "class": "penetration",
                         "note": f"C5-d sweep found {len(paradox)} "
                                 f"unmounted structural crossings across "
                                 f"{len(dn_paradox)} dn parts "
                                 f"(e.g. up {first[0]} x dn {first[1]} "
                                 f"gap {first[2]}mm n={first[3]} "
                                 f"depth {first[4]}mm nconf={first[5]} "
                                 f"ncross={first[6]} {first[7]}) at "
                                 f"{sorted(sweep['new_penetrations'])}deg; "
                                 f"systematic paradox, axis rejected; "
                                 f"{summary}"})
                    continue
                for h in all_hits:
                    why = ("unmounted paradox (lone; kept as boundary "
                           "watch)" if h[7] == "paradox"
                           else ("cross-mounted hardware" if h[7]
                                 == "mounted" else "rolling mesh contact"))
                    suspects.append(
                        {"joint": f"J{k}", "pair": (up, dn),
                         "class": "boundary", "uid": h[1],
                         "note": f"C5-d {h[7]} crossing up {h[0]} x "
                                 f"dn {h[1]} (gap {h[2]}mm n={h[3]} depth "
                                 f"{h[4]}mm nconf={h[5]} ncross={h[6]}): "
                                 f"{why} (C0), joint stands on its other "
                                 f"anchors; no reassignment"})
            ctrl = negative_control(
                center, direction, up_uids, dn_uids,
                [c["anchor"] for c in members_c], parts, catalog, lib)
            ver["negative_control"] = ctrl
            if not ctrl["rejected"]:
                suspects.append(
                    {"joint": f"J{k}", "pair": (up, dn),
                     "class": "control",
                     "note": "C5-n REGRESSION ALARM: 90deg-perturbed axis "
                             f"{ctrl['perturbed_axis']} was NOT rejected "
                             f"(verify {ctrl['verify_verdict']}, sweep "
                             f"{ctrl['sweep_verdict']}); checks degenerate"})
        joints.append({
            "id": f"J{k}", "type": "revolute", "parent": up, "child": dn,
            "axis": [round(float(v), 4) for v in direction],
            "point_mm": [round(float(v), 2) for v in center],
            "limits_deg": [-180, 180], "confidence": 0.7,
            "status": "selected",
            "anchor_kind": "+".join(sorted(
                {c["kind"] for c in members_c})),
            "n_anchors": len(members_c),
            "verify": ver,
            "diagnostics": {
                c["anchor"].get("uid", c["kind"]): c["anchor"].get(
                    "diagnostics")
                for c in members_c
                if c["anchor"].get("diagnostics")},
            "reason": (f"chain interface {up}->{dn}; "
                       f"{len(best)} coaxial anchor(s)")})
    links = [{"id": g, "part_count": len(members[g]),
              "members": sorted(members[g]),
              "has_motor": any(
                  geom._entry(catalog, by_uid[u]["part_id"]).get(
                      "category") == "motor" for u in members[g]),
              "has_sensor": any(
                  geom._entry(catalog, by_uid[u]["part_id"]).get(
                      "category") == "sensor" for u in members[g])}
             for g in CHAIN]
    return {"method": "chain-tier1-div", "links": links, "joints": joints,
            "base": {"link": "D0"}, "end_effector": {"link": "D6"},
            "sensors": [], "actuators": [],
            "transmission": {
                "coupling": "q = C · m, q in joint degrees, "
                            "m in motor degrees",
                "C": []},
            "boundary_suspects": suspects, "deficits": deficits,
            "div_issues": div["issues"],
            "review_note": ("Tier-1 div.io chain inference; no GT, no "
                            "Spark; flat-.io Tier 2 frozen")}
