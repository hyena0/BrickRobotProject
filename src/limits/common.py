"""Shared mesh-exact collision oracle for Stage 5A (simulation only).

All predicates here run on private authoring data (part meshes from
the Studio LDraw tree, link segmentation). They never enter the
robot package or the generated code: runtime safety uses the
precomputed limits and inequalities only.

Convention (from stage5_dev): AABB/sphere overlap is a candidate
generator only. The verdict is mesh truth (containment + shell
crossing + confirmed depth), following src/tools/chain_joints.py.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np
import yaml

from src.catalog.review import Library
from src.kin.model import chain_order, load_model
from src.tools import chain_joints as chain
from src.tools import geometry as geom
from src.viz.pose_check import find_cross_link_meshes, link_transforms

STUDIO_LDRAW = "/Applications/Studio 2.0/ldraw"
IO_PATH = "3D models/Robot_6axis_integration_Comb.io"

# Verdict threshold: mesh penetration depth above this (mm) between
# parts of different links counts as a collision.
#
# stage-5a-sim-limits.md writes 1 mm with a Blender-BVH oracle. The
# PC oracle here works on 1 mm-decimated LDraw meshes, where 1.0-1.8
# mm contacts flicker non-monotonically with angle (present at 5 deg,
# gone at 10 deg, back at 15 deg: J2 sweep evidence) — the documented
# open-soup/decimation noise band of src/tools/chain_joints.py, whose
# established verdict grade is cross + confirmed depth > 2.0 mm.
# PENETRATION_MM follows that repo standard; the final boundaries are
# additionally confirmed with exact-mesh Blender BVH (pose_confirm),
# and any residue goes to physical measurement in Stage 5C.
PENETRATION_MM = 2.0


def load_library(io_path: str = IO_PATH) -> Library:
    """Studio LDraw tree closed over the inferred part types."""
    parts = json.load(open("runs/stage1/parts.json", encoding="utf-8"))
    seeds = sorted({p["part_id"] for p in parts})
    lib = Library.from_dir(STUDIO_LDRAW, seeds)
    try:
        lib.merge_missing(Library.from_io(io_path))
    except OSError:
        pass
    return lib


def load_inputs(model_path="models/robot_inferred.json",
                parts_path="runs/stage1/parts.json",
                links_path="runs/stage2-chain-r17/links_joints.json",
                catalog_path="src/catalog/parts.yaml",
                sensors_path="runs/stage5/sensors.json") -> dict:
    """Load the authoring inputs shared by every limits module."""
    doc = load_model(model_path)
    parts = json.load(open(parts_path, encoding="utf-8"))
    links_doc = json.load(open(links_path, encoding="utf-8"))
    catalog = yaml.safe_load(open(catalog_path, encoding="utf-8")) or {}
    sensors = json.load(open(sensors_path, encoding="utf-8"))
    if isinstance(sensors, dict):
        sensors = sensors.get("sensors", [])
    members = {link["id"]: set(link.get("members", []))
               for link in links_doc.get("links", [])}
    link_of = {}
    for lid, uids in members.items():
        for u in uids:
            link_of[u] = lid
    ordered = chain_order(doc)
    link_ids = [ordered[0]["parent"]] + [j["child"] for j in ordered]
    return {"doc": doc, "parts": parts,
            "by_uid": {p["uid"]: p for p in parts},
            "catalog": catalog, "sensors": sensors,
            "members": members, "link_of": link_of,
            "ordered": ordered, "link_ids": link_ids,
            "link_index": {lid: n for n, lid in enumerate(link_ids)}}


def downstream_links(ctx: dict, joint_id: str) -> set:
    """Links at and after the child link of a joint (rigid subtree)."""
    ids = ctx["link_ids"]
    ordered = ctx["ordered"]
    pos = next(n for n, j in enumerate(ordered) if j["id"] == joint_id)
    return set(ids[pos + 1:])


def upstream_links(ctx: dict, joint_id: str) -> set:
    """Links at and before the parent link of a joint."""
    ids = ctx["link_ids"]
    ordered = ctx["ordered"]
    pos = next(n for n, j in enumerate(ordered) if j["id"] == joint_id)
    return set(ids[:pos + 1])


def joint_side_sets(ctx: dict, joint_id: str) -> tuple[set, set]:
    """Member uid sets (upstream, downstream) for a joint."""
    up_l, dn_l = upstream_links(ctx, joint_id), downstream_links(ctx, joint_id)
    members = ctx["members"]
    up = set().union(*(members[l] for l in up_l)) if up_l else set()
    dn = set().union(*(members[l] for l in dn_l)) if dn_l else set()
    return up, dn


class MeshContext:
    """Cached home-world meshes plus static exclusion sets.

    Pair loops are vectorised: cross-link candidate pairs are built
    once as index arrays, and every pose check prefilters them with
    one numpy sphere pass before touching any mesh.
    """

    CACHE_PATH = "runs/stage5a/mesh_cache.npz"

    def __init__(self, ctx: dict, lib, use_cache: bool = True) -> None:
        self.ctx = ctx
        self.lib = lib
        self._kcen_list = None
        self._krad_list = None
        self.dec_c = {}
        by_uid = ctx["by_uid"]
        cached: dict = {}
        if use_cache:
            try:
                with np.load(self.CACHE_PATH, allow_pickle=True) as z:
                    cached = {k: z[k] for k in z.files}
            except OSError:
                cached = {}
        self.full: dict[str, np.ndarray] = {}
        self.dec: dict[str, np.ndarray] = {}
        self.sph: dict[str, tuple[np.ndarray, float]] = {}
        self.box: dict[str, np.ndarray] = {}
        if len(cached) and "kcen" in cached and "decc" in cached \
                and "obbc" in cached:
            uids = [str(u) for u in cached["uids"]]
            kcen = [np.asarray(cached["kcen"][n]) for n in
                    range(len(uids))]
            krad = [np.asarray(cached["krad"][n]) for n in
                    range(len(uids))]
            decc_l = [np.asarray(cached["decc"][n]) for n in
                      range(len(uids))]
            self.obb_c0 = np.asarray(cached["obbc"], dtype=float)
            self.obb_a0 = np.asarray(cached["obba"], dtype=float)
            self.obb_h0 = np.asarray(cached["obbh"], dtype=float)
            for n, u in enumerate(uids):
                self.full[u] = np.asarray(cached["full"][n])
                self.dec[u] = np.asarray(cached["dec"][n])
                self.sph[u] = (np.asarray(cached["ctr"][n]),
                               float(cached["rad"][n]))
                bx = np.asarray(cached["box"][n], dtype=float)
                if not np.isnan(bx).all():
                    self.box[u] = bx
            self._kcen_list = kcen
            self._krad_list = krad
            self.dec_c = {u: d for u, d in zip(uids, decc_l)}
        else:
            for u, part in by_uid.items():
                t = chain._tris_local(lib, part["part_id"])
                if t is None or not len(t):
                    continue
                w = chain._world_tris(part["position_mm"],
                                      part["rotation"], t)
                self.full[u] = w
                self.dec[u] = chain._decimate(w, 1.0)
                verts = np.unique(w.reshape(-1, 3), axis=0)
                ctr = verts.mean(axis=0)
                self.sph[u] = (ctr, float(
                    np.linalg.norm(verts - ctr, axis=1).max()))
                b = chain._box_local(lib, part["part_id"])
                if b is not None:
                    self.box[u] = chain._world_pts(
                        part["position_mm"], part["rotation"], b)
            uids = sorted(self.full)
            kcen, krad = {}, {}
            decc = {}
            obb_c, obb_a, obb_h = {}, {}, {}
            for u in uids:
                decc[u] = chain._decimate(self.full[u], 2.5)
                verts = np.unique(self.full[u].reshape(-1, 3), axis=0)
                # Tight oriented box from PCA (sound broadphase tier:
                # the OBB contains the mesh by construction).
                ctr = verts.mean(axis=0)
                _, _, vt = np.linalg.svd(verts - ctr, full_matrices=False)
                proj = (verts - ctr) @ vt.T
                half = np.abs(proj).max(axis=0) + 0.25
                obb_c[u] = ctr
                obb_a[u] = vt.T
                obb_h[u] = half
                if len(verts) > 4000:
                    step = len(verts) / 4000.0
                    verts = verts[(np.arange(4000) * step).astype(int)]
                k = min(8, len(verts))
                first = int(np.argmax(np.linalg.norm(
                    verts - verts.mean(axis=0), axis=1)))
                idx = [first]
                d = np.linalg.norm(verts - verts[first], axis=1)
                for _ in range(k - 1):
                    idx.append(int(np.argmax(d)))
                    nd = np.linalg.norm(verts - verts[idx[-1]], axis=1)
                    d = np.minimum(d, nd)
                cen = verts[idx]
                asg = np.argmin(
                    np.linalg.norm(
                        verts[:, None, :] - cen[None, :, :], axis=2),
                    axis=1)
                rad = np.array([np.linalg.norm(
                    verts[asg == j] - cen[j], axis=1).max()
                    if (asg == j).any() else 0.0 for j in range(k)])
                pad = np.zeros((8 - k, 3))
                kcen[u] = np.vstack([cen, pad])
                krad[u] = np.concatenate(
                    [rad, np.zeros(8 - k)])
            ensure_dir(self.CACHE_PATH)
            np.savez_compressed(
                self.CACHE_PATH,
                uids=np.array(uids),
                full=np.array([self.full[u] for u in uids],
                              dtype=object),
                dec=np.array([self.dec[u] for u in uids], dtype=object),
                decc=np.array([decc[u] for u in uids], dtype=object),
                obbc=np.array([obb_c[u] for u in uids]),
                obba=np.array([obb_a[u] for u in uids]),
                obbh=np.array([obb_h[u] for u in uids]),
                ctr=np.array([self.sph[u][0] for u in uids]),
                rad=np.array([self.sph[u][1] for u in uids]),
                kcen=np.array([kcen[u] for u in uids]),
                krad=np.array([krad[u] for u in uids]),
                box=np.array([self.box.get(
                    u, np.full((8, 3), np.nan)) for u in uids]))
        self.uids = sorted(self.full)
        if self._kcen_list is None:
            self._kcen_list = [kcen[u] for u in self.uids]
            self._krad_list = [krad[u] for u in self.uids]
            self.dec_c = dict(decc)
            self.obb_c0 = np.array([obb_c[u] for u in self.uids])
            self.obb_a0 = np.array([obb_a[u] for u in self.uids])
            self.obb_h0 = np.array([obb_h[u] for u in self.uids])
        self.kcen0 = np.array(self._kcen_list)
        self.krad0 = np.array(self._krad_list)
        del self._kcen_list
        del self._krad_list
        self._idx = {u: n for n, u in enumerate(self.uids)}
        self._link_pos = {lid: n for n, lid in
                          enumerate(ctx["link_ids"])}
        self._part_link = np.array(
            [self._link_pos.get(ctx["link_of"].get(u), 0)
             for u in self.uids])
        links = [ctx["link_of"].get(u) for u in self.uids]
        ii, kk = np.nonzero(
            np.array(links)[:, None] != np.array(links)[None, :])
        keep = ii < kk
        self.pair_idx = np.stack([ii[keep], kk[keep]], axis=1)
        self.ctr0 = np.array([self.sph[u][0] for u in self.uids])
        self.rad = np.array([self.sph[u][1] for u in self.uids])
        boxes = []
        for u in self.uids:
            boxes.append(self.box.get(u, np.full((8, 3), np.nan)))
        self.box0 = np.array(boxes)
        lo0 = np.nanmin(np.where(np.isnan(self.box0), np.inf, self.box0),
                        axis=1)
        hi0 = np.nanmax(np.where(np.isnan(self.box0), -np.inf, self.box0),
                        axis=1)
        pi_all, pk_all = self.pair_idx[:, 0], self.pair_idx[:, 1]
        sep = np.maximum(lo0[pi_all] - hi0[pk_all],
                         lo0[pk_all] - hi0[pi_all])
        self.homegap = sep.max(axis=1).astype(np.float32)
        self.homegap[~np.isfinite(self.homegap)] = float("-inf")
        # Static exclusions from the home pose. Sphere, then AABB,
        # then containment: the narrowphase runs only on overlapping
        # candidates, never blindly over all cross-link pairs. The
        # result is cached on disk (inputs are static authoring data).
        import json as _json
        excl_path = "runs/stage5a/exclusions.json"
        self.engaged = set()
        loaded = False
        if use_cache:
            try:
                saved = _json.load(open(excl_path, encoding="utf-8"))
                self.engaged = {tuple(p) for p in saved["engaged"]}
                self.gear = {tuple(p) for p in saved["gear"]}
                self.baseline = {tuple(p) for p in saved["baseline"]}
                self.bearing = {jid: set(v) for jid, v in
                                saved["bearing"].items()}
                loaded = True
            except OSError:
                loaded = False
        if not loaded:
            self.engaged = set()
            pi, pk = self.pair_idx[:, 0], self.pair_idx[:, 1]
            gap = np.linalg.norm(self.ctr0[pi] - self.ctr0[pk], axis=1) \
                - self.rad[pi] - self.rad[pk]
            cand = np.nonzero(gap <= 0.0)[0]
            for n in cand:
                a, b = self.uids[pi[n]], self.uids[pk[n]]
                if a in self.box and b in self.box and \
                        not chain._aabbs_overlap(self.box[a],
                                                 self.box[b]):
                    continue
                if chain._meshes_contain(self.full[a], self.full[b]):
                    self.engaged.add(tuple(sorted([a, b])))
        if not loaded:
            # Designed gear meshes across links (drive contacts, not
            # structural seats): excluded up front like pose_check.
            self.gear = set()
            parts, catalog = ctx["parts"], ctx["catalog"]
            try:
                for pr in find_cross_link_meshes(parts, catalog,
                                                 ctx["link_of"]):
                    self.gear.add(tuple(sorted([pr["a"], pr["b"]])))
            except KeyError:
                pass
            # Bearing parts of every joint: turntable rings plus the
            # axle/pin members crossing the interface (they stay
            # coaxial by design; verify_joint owns them, not the sweep).
            self.bearing = {}
            members = ctx["members"]
            for j in ctx["ordered"]:
                keep = set()
                for u in members.get(j["parent"], set()) | \
                        members.get(j["child"], set()):
                    e = geom._entry(catalog,
                                    by_uid[u]["part_id"])
                    if e.get("category") == "turntable":
                        keep.add(u)
                axis = np.asarray(j["axis"], dtype=float)
                axis = axis / np.linalg.norm(axis)
                origin = np.asarray(j["point_mm"], dtype=float)
                span = 60.0
                for u in members.get(j["parent"], set()) | \
                        members.get(j["child"], set()):
                    part = by_uid[u]
                    e = geom._entry(catalog, part["part_id"])
                    if e.get("category") not in ("axle", "pin", "motor"):
                        continue
                    wax = geom.axis_world(part, catalog)
                    if wax is None:
                        continue
                    direction = wax / np.linalg.norm(wax)
                    try:
                        if geom._line_hits(
                                {"position_mm": list(origin),
                                 "rotation": np.eye(3).tolist(),
                                 "part_id": part["part_id"]},
                                direction, part, catalog, span):
                            keep.add(u)
                    except Exception:
                        continue
                self.bearing[j["id"]] = keep
            self.baseline = self._home_baseline()
            ensure_dir(excl_path)
            with open(excl_path, "w", encoding="utf-8") as fh:
                _json.dump({"engaged": sorted(self.engaged),
                            "gear": sorted(self.gear),
                            "baseline": sorted(self.baseline),
                            "bearing": {jid: sorted(v) for jid, v in
                                        self.bearing.items()}},
                           fh)

    def excluded(self, joint_id: str | None = None) -> set[tuple[str, str]]:
        """Pairs never judged: home-engaged, gear meshes, bearings,
        plus the zero-pose CAD-interference baseline (stage5_dev:
        zero-pose BVH spot checks separate CAD interference from
        motion-induced contact)."""
        out = set(self.engaged) | set(self.gear) | set(self.baseline)
        if joint_id is not None:
            bear = self.bearing.get(joint_id, set())
            for u in bear:
                for v in self.full:
                    if u != v:
                        out.add(tuple(sorted([u, v])))
        return out

    def _home_baseline(self, depth_mm: float = PENETRATION_MM) -> set:
        """Mesh-verdict contacts already present at the zero pose."""
        base: set[tuple[str, str]] = set()
        pi, pk = self.pair_idx[:, 0], self.pair_idx[:, 1]
        link_of = self.ctx["link_of"]
        excl = set(self.engaged) | set(self.gear)
        for n in range(len(pi)):
            i, k = int(pi[n]), int(pk[n])
            a, b = self.uids[i], self.uids[k]
            key = tuple(sorted([a, b]))
            if key in excl:
                continue
            if np.linalg.norm(self.ctr0[i] - self.ctr0[k]) > \
                    self.rad[i] + self.rad[k]:
                continue
            if a in self.box and b in self.box and \
                    not chain._aabbs_overlap(self.box[a], self.box[b]):
                continue
            inside, _gap = chain._meshes_intersect(self.full[a],
                                                   self.full[b])
            if not inside:
                continue
            cross, _n = chain._tris_cross(self.full[a], self.full[b])
            _n_in, depth, n_conf = chain._contain_confirmed(
                self.full[a], self.full[b])
            if cross and n_conf > 0 and depth > depth_mm:
                base.add(key)
        return base


def aabb_may_collide(wa_box: np.ndarray, wb_box: np.ndarray) -> bool:
    """AABB overlap predicate for the broadphase tier.

    Overlap (any positive amount) is NECESSARY for mesh contact, so
    this never rejects a true collision. It must not take a depth
    threshold: a thin deep stab (needle into a plate) has tiny
    min-axis AABB overlap with large mesh depth, and a depth test
    here silently drops it. Depth is judged only by the exact
    verdict below.
    """
    pen = np.minimum(wa_box.max(axis=0), wb_box.max(axis=0)) \
        - np.maximum(wa_box.min(axis=0), wb_box.min(axis=0))
    return bool((pen > -1e-6).all())


def _verdict(wa: np.ndarray, wb: np.ndarray,
             depth_mm: float = PENETRATION_MM) -> float | None:
    """Mesh verdict for one pair: penetration depth or None.

    Negative containment already rules out a verdict (confirmed
    depth needs inside vertices), so crossing and confirmation run
    only on containment-positive pairs. Mirrors the sweep_penetration
    verdict grade (cross + confirmed depth) without its gap report.
    """
    if not chain._meshes_contain(wa, wb):
        return None
    cross, _n = chain._tris_cross(wa, wb)
    if not cross:
        return None
    _n_in, depth, n_conf = chain._contain_confirmed(wa, wb)
    if n_conf > 0 and depth > depth_mm:
        return float(depth)
    return None


def link_rotation_angles(frames: dict, link_ids: list[str]) -> np.ndarray:
    """Per-link rotation magnitude (radians) vs the home pose.

    Pure helper (no meshes): the angle comes from each link frame's
    rotation matrix. Must index the LINK frames, never the per-part
    matrices — confusing the two silently corrupts the motion-gap
    tier below.
    """
    phi = np.zeros(len(link_ids))
    for n, lid in enumerate(link_ids):
        r = np.asarray(frames[lid])[:3, :3]
        c = max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))
        phi[n] = float(np.arccos(c))
    return phi


def world_at_pose(mesh: MeshContext, q_deg: list[float]) -> dict:
    """Link frames for a joint pose (radians inside, degrees outside)."""
    mats = link_transforms(mesh.ctx["doc"], list(q_deg))
    idx = mesh.ctx["link_index"]
    out = {}
    for lid, n in idx.items():
        out[lid] = np.asarray(mats[n], dtype=float)
    return out


def _xform(tris: np.ndarray, mat: np.ndarray) -> np.ndarray:
    r = mat[:3, :3]
    t = mat[:3, 3]
    return tris @ r.T + t


def pose_collisions(mesh: MeshContext, q_deg: list[float],
                    pairs: set[tuple[str, str]] | None = None,
                    exclude: set[tuple[str, str]] | None = None,
                    depth_mm: float = PENETRATION_MM,
                    first_only: bool = False,
                    budget_s: float = 0.0,
                    verdict: str = "mesh") -> list[dict]:
    """Collisions at one pose (cross-link pairs only).

    verdict="mesh" is the exact grade (containment + shell crossing
    + confirmed depth) on 1 mm-decimated meshes. verdict="coarse"
    runs the same grade on 2.5 mm-decimated meshes: a fast proposal
    for ray marching (less selective, never the final word).
    Validation, safe-pose screening and boundary bisection always
    use "mesh".

    Broadphase tiers (spheres, K-spheres, motion gap, AABB overlap,
    OBB overlap) are all NECESSARY conditions: they only ever admit
    more pairs to the exact test, never reject a true collision.
    In particular the AABB tier tests overlap only, never depth —
    a thin deep stab has tiny AABB overlap with large mesh depth.

    first_only stops at the first verdict (sweeps and existence
    checks do not need the full list). budget_s caps narrowphase
    wall time: on expiry the pose is reported colliding (a
    conservative TIMEOUT hit). Both only ever reject more poses,
    never admit a collision."""
    """Mesh-verdict collisions at one pose (cross-link pairs only).

    Tier 1 (spheres, then AABB corners) is a sound prefilter:
    both contain their mesh, so overlap is necessary for contact.
    Tier 2 (containment + shell crossing + confirmed depth) is the
    verdict, mirroring sweep_penetration.
    """
    link_of = mesh.ctx["link_of"]
    frames = world_at_pose(mesh, q_deg)
    uids = mesh.uids
    lid_list = [link_of.get(u) for u in uids]
    mats = np.array([frames[l] for l in lid_list])
    ctr = np.einsum("nij,nj->ni", mats[:, :3, :3], mesh.ctr0) \
        + mats[:, :3, 3]
    if pairs is None:
        sel = np.arange(len(mesh.pair_idx))
    else:
        n_all = len(mesh.uids)
        want = np.array([min(mesh._idx[a], mesh._idx[b]) * n_all
                         + max(mesh._idx[a], mesh._idx[b])
                         for (a, b) in pairs
                         if a in mesh._idx and b in mesh._idx],
                        dtype=np.int64)
        keys = mesh.pair_idx[:, 0].astype(np.int64) * n_all \
            + mesh.pair_idx[:, 1].astype(np.int64)
        sel = np.nonzero(np.isin(keys, want))[0]
    excl = exclude or set()
    if len(sel):
        pi = mesh.pair_idx[sel][:, 0]
        pk = mesh.pair_idx[sel][:, 1]
        gap = np.linalg.norm(ctr[pi] - ctr[pk], axis=1) \
            - mesh.rad[pi] - mesh.rad[pk]
        sel = sel[gap <= 0.0]
    if len(sel):
        # K-sphere tier (sound: sphere unions contain their mesh).
        pi = mesh.pair_idx[sel][:, 0]
        pk = mesh.pair_idx[sel][:, 1]
        rot = mats[:, :3, :3]
        tr = mats[:, :3, 3]
        ci = (mesh.kcen0[pi] @ rot[pi].transpose(0, 2, 1)
              + tr[pi][:, None, :])
        ck = (mesh.kcen0[pk] @ rot[pk].transpose(0, 2, 1)
              + tr[pk][:, None, :])
        kgap = (np.linalg.norm(ci[:, :, None, :] - ck[:, None, :, :],
                               axis=3)
                - mesh.krad0[pi][:, :, None]
                - mesh.krad0[pk][:, None, :]).min(axis=(1, 2))
        sel = sel[kgap <= 0.0]
    if len(sel):
        # Motion-gap tier (sound): a pair whose home AABB gap exceeds
        # the max possible approach at this pose cannot reach the
        # verdict depth. Approach <= centroid travel + chord spread
        # from the link-frame rotation (telescoping bound).
        link_ids = mesh.ctx["link_ids"]
        phi = link_rotation_angles(frames, link_ids)
        dd = (np.linalg.norm(ctr - mesh.ctr0, axis=1)
              + 2.0 * mesh.rad
              * np.sin(phi[mesh._part_link] / 2.0))
        pi = mesh.pair_idx[sel][:, 0]
        pk = mesh.pair_idx[sel][:, 1]
        reach = dd[pi] + dd[pk] + depth_mm
        sel = sel[mesh.homegap[sel] <= reach]
    box_all = mesh.box0 @ mats[:, :3, :3].transpose(0, 2, 1) \
        + mats[:, None, :3, 3]
    has_box = ~np.isnan(box_all[:, 0, 0])
    if len(sel):
        # Overlap only (no depth test): overlap is necessary for
        # contact, while a depth test here drops thin deep stabs.
        pi = mesh.pair_idx[sel][:, 0]
        pk = mesh.pair_idx[sel][:, 1]
        both = has_box[pi] & has_box[pk]
        pen = np.minimum(box_all[pi].max(axis=1),
                         box_all[pk].max(axis=1)) \
            - np.maximum(box_all[pi].min(axis=1),
                         box_all[pk].min(axis=1))
        keep = ~both
        keep[both] = (pen[both] > -1e-6).all(axis=1)
        sel = sel[keep]
    if len(sel):
        # OBB tier (sound: PCA boxes contain their meshes). SAT over
        # 15 axes, vectorised; much tighter than AABBs for beams.
        rot = mats[:, :3, :3]
        tr = mats[:, :3, 3]
        pi = mesh.pair_idx[sel][:, 0]
        pk = mesh.pair_idx[sel][:, 1]
        aw = rot[pi] @ mesh.obb_a0[pi]
        bw = rot[pk] @ mesh.obb_a0[pk]
        ca = np.einsum("sij,sj->si", rot[pi], mesh.obb_c0[pi]) \
            + tr[pi]
        cb = np.einsum("sij,sj->si", rot[pk], mesh.obb_c0[pk]) \
            + tr[pk]
        ha, hb = mesh.obb_h0[pi], mesh.obb_h0[pk]
        cross = np.cross(aw[:, :, None, :], bw[:, None, :, :])
        axes = np.concatenate(
            [aw, bw, cross.reshape(len(sel), 9, 3)], axis=1)
        sep = cb - ca
        proj = np.abs(np.einsum("sni,si->sn", axes, sep))
        ra = (np.abs(np.einsum("sni,sji->snj", axes, aw))
              * ha[:, None, :]).sum(axis=2)
        rb = (np.abs(np.einsum("sni,sji->snj", axes, bw))
              * hb[:, None, :]).sum(axis=2)
        overlap = np.all(proj <= ra + rb + 1e-6, axis=1)
        sel = sel[overlap]
    link = [link_of.get(u) for u in uids]
    if verdict == "coarse":
        src = mesh.dec_c
    elif verdict == "mesh":
        src = mesh.dec
    else:
        raise ValueError(f"unknown verdict {verdict!r}")
    hits = []
    import time as _time
    t_end = (_time.time() + budget_s) if budget_s else 0.0
    for n in sel:
        i, k = int(mesh.pair_idx[n][0]), int(mesh.pair_idx[n][1])
        a, b = uids[i], uids[k]
        key = tuple(sorted([a, b]))
        if key in excl:
            continue
        wa = _xform(src[a], mats[i])
        wb = _xform(src[b], mats[k])
        depth = _verdict(wa, wb, depth_mm)
        if depth is not None:
            hits.append({"a": a, "b": b,
                         "link_a": link[i], "link_b": link[k],
                         "depth_mm": round(depth, 3)})
            if first_only:
                break
        if t_end and _time.time() > t_end:
            hits.append({"a": "TIMEOUT", "b": "TIMEOUT",
                         "link_a": link[0], "link_b": link[0],
                         "depth_mm": -1.0,
                         "note": "narrowphase budget expired; "
                                 "pose treated as colliding"})
            break
    hits.sort(key=lambda r: -r["depth_mm"])
    return hits


def table_height_mm(ctx: dict) -> float:
    """Table plane estimate: lowest base-link part bottom, minus half
    brick height. Documented assumption for the 40 mm EE rule."""
    d0 = [ctx["by_uid"][u]["position_mm"][2]
          for u in ctx["members"].get("D0", set())
          if u in ctx["by_uid"]]
    if not d0:
        return min(p["position_mm"][2] for p in ctx["parts"]) - 5.0
    return float(min(d0)) - 5.0


def ensure_dir(path: str) -> str:
    """Create the parent directory of an output path."""
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    return path


def span_pairs(mesh: MeshContext, link_set: set[str]) -> set[tuple[str, str]]:
    """Cross-link pairs touching a moving link set.

    Static-static pairs keep their home verdict, so sweeps and grids
    that move only some links check just these.
    """
    link_of = mesh.ctx["link_of"]
    touch = {u for u in mesh.uids if link_of.get(u) in link_set}
    out = set()
    for n in range(len(mesh.pair_idx)):
        i, k = int(mesh.pair_idx[n][0]), int(mesh.pair_idx[n][1])
        a, b = mesh.uids[i], mesh.uids[k]
        if a in touch or b in touch:
            out.add(tuple(sorted([a, b])))
    return out


_FARM_MESH: MeshContext | None = None


def _farm_init() -> None:
    """Worker init: load the cached authoring context once (fork)."""
    global _FARM_MESH
    ctx = load_inputs()
    lib = None
    _FARM_MESH = MeshContext(ctx, lib, use_cache=True)


def _resolve_local(mesh: MeshContext, args: tuple) -> list[dict]:
    """Singleton evaluation on the bound main-process mesh, with the
    same exact TIMEOUT resolution as the pool path."""
    q, pairs, exclude, depth, first_only, budget, verdict = args
    hits = pose_collisions(mesh, list(q),
                           pairs=set(map(tuple, pairs)) if pairs else None,
                           exclude=set(map(tuple, exclude)),
                           depth_mm=depth, first_only=first_only,
                           budget_s=budget, verdict=verdict)
    if budget and hits and all(h.get("a") == "TIMEOUT" for h in hits):
        hits = pose_collisions(mesh, list(q),
                               pairs=set(map(tuple, pairs))
                               if pairs else None,
                               exclude=set(map(tuple, exclude)),
                               depth_mm=depth, first_only=first_only,
                               budget_s=0.0, verdict=verdict)
    return hits


def _farm_pose(args: tuple) -> list[dict]:
    """One pose check inside a farm worker."""
    q, pairs, exclude, depth, first_only, budget, verdict = args
    assert _FARM_MESH is not None
    return pose_collisions(_FARM_MESH, list(q),
                           pairs=set(map(tuple, pairs)) if pairs else None,
                           exclude=set(map(tuple, exclude)),
                           depth_mm=depth, first_only=first_only,
                           budget_s=budget, verdict=verdict)


class PoseFarm:
    """Multiprocessing pose checker (fork; Linux/macOS).

    Workers share the cached meshes copy-on-write. The pool is
    persistent across map calls (worker init loads ~40 MB of caches,
    so per-call pools would dominate). Used for sweeps, grids and
    validation batches that need thousands of pose checks.
    """

    def __init__(self, jobs: int = 0) -> None:
        import multiprocessing as mp
        import os as _os
        self.jobs = jobs or _os.cpu_count() or 4
        self._mp = mp.get_context("fork")
        self._pool = None
        self._local = None
        if self.jobs > 1:
            import atexit as _atexit
            _atexit.register(self.close)

    def _ensure(self):
        if self._pool is None and self.jobs > 1:
            self._pool = self._mp.Pool(processes=self.jobs,
                                       initializer=_farm_init)
        return self._pool

    def bind(self, mesh: MeshContext) -> None:
        """Bind the main-process mesh for singleton evaluations."""
        self._local = mesh

    def close(self) -> None:
        """Shut down the worker pool (also runs at exit)."""
        pool, self._pool = self._pool, None
        if pool is not None:
            try:
                pool.close()
                pool.join()
            except Exception:
                pass

    def map(self, q_list: list[list[float]],
            pairs: set[tuple[str, str]] | None = None,
            exclude: set[tuple[str, str]] | None = None,
            depth_mm: float = PENETRATION_MM,
            chunksize: int = 0,
            first_only: bool = False,
            budget_s: float = 0.0,
            verdict: str = "mesh") -> list[list[dict]]:
        """pose_collisions over q_list, order preserved."""
        serial = sorted(pairs) if pairs else None
        excl = sorted(exclude) if exclude else []
        args = [(q, serial, excl, depth_mm, first_only, budget_s, verdict)
                for q in q_list]
        if self.jobs <= 1 or len(q_list) == 1:
            if getattr(self, "_local", None) is not None:
                return [_resolve_local(self._local, a) for a in args]
            _farm_init()
            return [_farm_pose(a) for a in args]
        pool = self._ensure()
        if chunksize <= 0:
            chunksize = max(1, len(args) // (self.jobs * 8))
        out = pool.map(_farm_pose, args, chunksize=chunksize)
        if budget_s:
            # TIMEOUT pseudo-hits are load-dependent flakiness, not
            # verdicts: resolve each exactly (no budget) so sweeps
            # and fits stay deterministic.
            pending = [n for n, hits in enumerate(out)
                       if hits and all(h.get("a") == "TIMEOUT"
                                       for h in hits)]
            if pending:
                exact = [(args[n][0], args[n][1], args[n][2],
                          args[n][3], args[n][4], 0.0, args[n][6])
                         for n in pending]
                for n, hits in zip(pending, pool.map(
                        _farm_pose, exact, chunksize=1)):
                    out[n] = hits
        return out
