"""Blender pose validation for the inferred robot (Stage 5.4+).

Samples 10 joint poses (7 random in motor space, 3 near range ends),
converts them with q = C m, articulates the real part meshes in
Blender from the Stage 2 link segmentation, and auto-judges every
pose:

(a) support hole-to-axis distance stays within 0.5 mm,
(b) no new inter-link part overlaps versus the zero pose,
(c) cross-link gear-pair centre distances are maintained,
(d) Blender end-effector position matches FK.

Poses violating soft limits are discarded and resampled. Outputs
(Stage 5.4 evidence, all git-ignored):

- assets/private/renders/poses/pose{01..10}_{front,side,top}.png
- runs/stage5/poses10.json (joint + motor angles for physical replay)
- runs/stage5/pose_check_report.json (per-pose numbers and failures)

Usage::

    python -m src.viz.pose_check --out-batch runs/stage5/pose_batch.json

The driver (this file) never imports bpy, so unit tests run without
Blender. All Blender work happens in src/viz/pose_render_inner.py,
launched headless with the repository Blender.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess

import numpy as np
import yaml

from src.codegen.generate import SOFT_MARGIN_DEG
from src.kin.fk import fk_model
from src.kin.model import chain_order, load_model
from src.kin.motors import motors_to_joints
from src.tools import geometry as geom
from src.tools.geometry import check_gear_mesh

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
BLENDER_BIN = os.environ.get(
    "BLENDER_BIN", "/Applications/Blender.app/Contents/MacOS/Blender")
BASE_BLEND = "runs/stage2-chain-r17/model_chain_r17.blend"
PARTS_JSON = "runs/stage1/parts.json"
LINKS_JSON = "runs/stage2-chain-r17/links_joints.json"
CATALOG_YAML = "src/catalog/parts.yaml"

N_RANDOM = 7
N_EDGE = 3
MOTOR_SAMPLE_DEG = 150.0
EDGE_FRACTION = (0.90, 0.98)
MAX_RESAMPLE = 500
SUPPORT_TOL_MM = 0.5
GEAR_DRIFT_TOL_MM = 0.5
EE_TOL_MM = 0.5
OVERLAP_DEPTH_MM = 0.2
SEPARATION_MM = 2.0
POSE_SEED = 17


def soft_limits(doc: dict) -> list[list[float]]:
    """Model limits narrowed by the Stage 5 soft margin."""
    return [[j["limits_deg"][0] + SOFT_MARGIN_DEG,
             j["limits_deg"][1] - SOFT_MARGIN_DEG]
            for j in chain_order(doc)]


def within_limits(q: list[float], lims: list[list[float]]) -> bool:
    return all(lo - 1e-9 <= v <= hi + 1e-9
               for v, (lo, hi) in zip(q, lims))


def sample_motor(rng: random.Random, n: int) -> list[float]:
    return [float(rng.uniform(-MOTOR_SAMPLE_DEG, MOTOR_SAMPLE_DEG))
            for _ in range(n)]


def q_of_m(m: list[float], c: list[list[float]],
           motor_ids: list[str]) -> list[float]:
    return motors_to_joints({mid: v for mid, v in zip(motor_ids, m)},
                            c, motor_ids)


def sample_poses(doc: dict, seed: int = POSE_SEED) -> list[dict]:
    """10 poses: 7 random motor-space + 3 near soft-range ends.

    Every pose satisfies soft limits; violators are discarded and
    resampled. Returns [{id, m_deg, q_deg}].
    """
    rng = random.Random(seed)
    c = doc["transmission"]["C"]
    motor_ids = [a["id"] for a in doc["actuators"]]
    lims = soft_limits(doc)
    poses: list[dict] = []

    def take(m: list[float], pid: str) -> bool:
        q = q_of_m(m, c, motor_ids)
        if not within_limits(q, lims):
            return False
        poses.append({"id": pid, "m_deg": list(m), "q_deg": list(q)})
        return True

    tries = 0
    while len(poses) < N_RANDOM and tries < MAX_RESAMPLE:
        tries += 1
        take(sample_motor(rng, len(motor_ids)),
             "pose%02d" % (len(poses) + 1))
    if len(poses) < N_RANDOM:
        raise RuntimeError("motor-space sampling kept violating soft limits")

    joints = [j["id"] for j in chain_order(doc)]
    edge_joints = rng.sample(range(len(joints)),
                             min(N_EDGE, len(joints)))
    for k, ji in enumerate(edge_joints):
        placed = False
        for _ in range(MAX_RESAMPLE):
            m = sample_motor(rng, len(motor_ids))
            q = q_of_m(m, c, motor_ids)
            if abs(q[ji]) < 1e-6:
                continue
            lo, hi = lims[ji]
            frac = rng.uniform(*EDGE_FRACTION)
            target = (hi if q[ji] > 0 else lo) * frac
            s = target / q[ji]
            m2 = [v * s for v in m]
            if take(m2, "pose%02d" % (len(poses) + 1)):
                placed = True
                break
        if not placed:
            raise RuntimeError(f"no near-end pose found for joint {ji}")
    return poses


def mat_axis_angle(axis, angle_rad: float) -> np.ndarray:
    """4x4 rotation about a unit axis through the origin."""
    x, y, z = (float(v) for v in axis)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    t = 1.0 - c
    r = np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                  [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                  [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])
    m = np.eye(4)
    m[:3, :3] = r
    return m


def link_transforms(doc: dict, q_deg: list[float]) -> list[np.ndarray]:
    """World transform of every link (D0..Dn) for joint angles."""
    ordered = chain_order(doc)
    links = [ordered[0]["parent"]] + [j["child"] for j in ordered]
    t_mats = [np.eye(4)]
    cur = np.eye(4)
    for j, qd in zip(ordered, q_deg):
        axis = np.asarray(j["axis"], dtype=float)
        axis = axis / np.linalg.norm(axis)
        pt = np.asarray(j["point_mm"], dtype=float)
        r = mat_axis_angle(axis, math.radians(qd))
        step = np.eye(4)
        step[:3, 3] = pt
        step = step @ r
        step[:3, 3] -= (r[:3, :3] @ pt)
        cur = cur @ step
        t_mats.append(cur.copy())
    return t_mats


def lateral_to_axis(pt, axis_pt, axis_dir: np.ndarray) -> float:
    rel = np.asarray(pt, dtype=float) - np.asarray(axis_pt, dtype=float)
    d = np.asarray(axis_dir, dtype=float)
    d = d / np.linalg.norm(d)
    return float(np.linalg.norm(rel - float(np.dot(rel, d)) * d))


def collect_supports(parts: list[dict], catalog: dict,
                     members_parent: set, members_child: set) -> list[dict]:
    """Support points with owning part uids for one joint interface.

    Mirrors src/tools/joints.py evidence (crossing shafts + hole
    centres on the shaft line + turntable axes), keeping uid
    attribution so violations name the part.
    """
    by_uid = {p["uid"]: p for p in parts}
    out = []
    for part in parts:
        entry = geom._entry(catalog, part["part_id"])
        if entry.get("category") not in ("axle", "pin", "motor"):
            continue
        axis = geom.axis_world(part, catalog)
        if axis is None:
            continue
        direction = axis / np.linalg.norm(axis)
        span = (geom._axle_span_mm(part, catalog)
                if entry.get("category") == "axle" else 60.0)
        hits_a = [u for u in members_parent
                  if geom._line_hits(part, axis, by_uid[u], catalog, span)]
        hits_b = [u for u in members_child
                  if geom._line_hits(part, axis, by_uid[u], catalog, span)]
        if not (hits_a and hits_b):
            continue
        origin = geom._pos(part)
        out.append({"uid": part["uid"], "point_mm": [float(v) for v in
                                                     np.asarray(origin).ravel()]})
        for u in hits_a + hits_b:
            holes = geom._entry(catalog, by_uid[u]["part_id"]).get("holes") or []
            for hole in holes:
                if not isinstance(hole, dict):
                    continue
                try:
                    c = geom.hole_world(by_uid[u], hole)
                except Exception:
                    continue
                if lateral_to_axis(c, origin, direction) <= \
                        geom.AXLE_LINE_TOL_MM:
                    out.append({"uid": u, "point_mm": [float(v) for v in
                                                       np.asarray(c).ravel()]})
    for u in members_parent | members_child:
        part = by_uid[u]
        if geom._entry(catalog, part["part_id"]).get("category") == \
                "turntable":
            axis = geom.axis_world(part, catalog)
            if axis is not None:
                out.append({"uid": u, "point_mm": [float(v) for v in
                                                   np.asarray(geom._pos(part)).ravel()]})
    return out


def check_supports_pose(doc: dict, supports: dict, link_index: dict,
                        t_mats: list[np.ndarray],
                        q_deg: list[float]) -> list[dict]:
    """Check (a): axis-support holes stay on the joint axis.

    Only supports within SUPPORT_TOL_MM of the joint axis at the zero
    pose constrain the axis ("axis supports"); the rest are interface
    structure tracked by drift. A gated support leaving the axis at a
    pose means wrong link attribution or a wrong axis.
    """
    ordered = chain_order(doc)
    rows = []
    for i, j in enumerate(ordered):
        parent_t = t_mats[link_index[j["parent"]]]
        axis = np.asarray(j["axis"], dtype=float)
        axis = axis / np.linalg.norm(axis)
        pt = np.asarray(j["point_mm"], dtype=float)
        w_axis = (parent_t[:3, :3] @ axis)
        w_axis = w_axis / np.linalg.norm(w_axis)
        w_pt = (parent_t[:3, :3] @ pt) + parent_t[:3, 3]
        worst, bad, drift, n_axis = 0.0, [], 0.0, 0
        for s in supports[j["id"]]:
            link = link_index[s["link"]]
            p0 = np.asarray(s["point_mm"])
            lat0 = lateral_to_axis(p0, pt, axis)
            wp = (t_mats[link][:3, :3] @ p0) + t_mats[link][:3, 3]
            lat = lateral_to_axis(wp, w_pt, w_axis)
            if lat0 <= SUPPORT_TOL_MM:
                n_axis += 1
                worst = max(worst, lat)
                if lat > SUPPORT_TOL_MM:
                    bad.append({"uid": s["uid"],
                                "lateral_mm": round(lat, 3),
                                "zero_lateral_mm": round(lat0, 3)})
            else:
                drift = max(drift, abs(lat - lat0))
        rows.append({"joint": j["id"], "worst_lateral_mm": round(worst, 3),
                     "n_supports": len(supports[j["id"]]),
                     "n_axis_supports": n_axis,
                     "max_drift_mm": round(drift, 4),
                     "violations": bad,
                     "ok": not bad})
    return rows


def find_cross_link_meshes(parts: list[dict], catalog: dict,
                           link_of: dict) -> list[dict]:
    """Gear pairs meshing across two links at the zero pose."""
    def is_gear(pid: str) -> bool:
        e = catalog.get(pid) or {}
        return e.get("teeth") is not None and e.get("category") in (
            "gear", "turntable")
    gears = [p for p in parts if is_gear(p["part_id"])]
    by_uid = {p["uid"]: p for p in parts}
    pairs = []
    for i in range(len(gears)):
        for k in range(i + 1, len(gears)):
            a, b = gears[i], gears[k]
            if link_of.get(a["uid"]) == link_of.get(b["uid"]):
                continue
            pa = np.asarray(a["position_mm"], dtype=float)
            pb = np.asarray(b["position_mm"], dtype=float)
            if float(np.linalg.norm(pa - pb)) > 120.0:
                continue
            try:
                res = check_gear_mesh(a["uid"], b["uid"], parts, catalog)
            except KeyError:
                continue
            if res.get("meshes") and res.get("kind") != "compound":
                pairs.append({"a": a["uid"], "b": b["uid"],
                              "kind": res.get("kind"),
                              "nominal_mm": float(res["centre_mm"]),
                              "link_a": link_of.get(a["uid"]),
                              "link_b": link_of.get(b["uid"])})
    pairs.sort(key=lambda r: (r["a"], r["b"]))
    return pairs


def check_gears_pose(pairs: list[dict], by_uid: dict, link_index: dict,
                     t_mats: list[np.ndarray]) -> list[dict]:
    """Check (c): cross-link gear centre-distance drift at a pose."""
    rows = []
    for pr in pairs:
        out = dict(pr)
        ok = True
        for tag, key in (("a", "link_a"), ("b", "link_b")):
            uid = pr[tag]
            t = t_mats[link_index[pr[key]]]
            p0 = np.asarray(by_uid[uid]["position_mm"], dtype=float)
            out[f"pos_{tag}_mm"] = [round(float(v), 3) for v in
                                    (t[:3, :3] @ p0 + t[:3, 3])]
        d = float(np.linalg.norm(np.asarray(out["pos_a_mm"]) -
                                 np.asarray(out["pos_b_mm"])))
        out["drift_mm"] = round(d - pr["nominal_mm"], 3)
        out["ok"] = abs(out["drift_mm"]) <= GEAR_DRIFT_TOL_MM
        rows.append(out)
    return rows


def pair_penetration(a_min, a_max, b_min, b_max) -> float:
    """Signed AABB penetration (mm): >0 overlap depth, <0 gap width."""
    pen = np.minimum(np.asarray(a_max), np.asarray(b_max)) - \
        np.maximum(np.asarray(a_min), np.asarray(b_min))
    return float(np.min(pen))


def contact_analysis(z_aabbs: dict, p_aabbs: dict,
                     link_of: dict) -> dict:
    """Check (b): motion-induced contacts, separated from AABB noise.

    A dense pack overlaps thousands of AABB pairs even at zero, so a
    plain absent-at-zero test is noise. A pair is a *contact* only
    when it clearly interpenetrates at the pose (depth > 0.2 mm)
    after being clearly separated at zero (gap > 2.0 mm). Pairs in
    the ambiguity band are counted, not failed.
    """
    uids = sorted(p_aabbs)
    lo0 = np.array([z_aabbs[u][0] for u in uids])
    hi0 = np.array([z_aabbs[u][1] for u in uids])
    lo1 = np.array([p_aabbs[u][0] for u in uids])
    hi1 = np.array([p_aabbs[u][1] for u in uids])
    links = np.array([link_of.get(u) for u in uids])
    iu = np.triu_indices(len(uids), k=1)
    cross = (links[:, None] != links[None, :])[iu]
    pen0 = np.min(np.minimum(hi0[iu[0]], hi0[iu[1]]) -
                  np.maximum(lo0[iu[0]], lo0[iu[1]]), axis=1)
    pen1 = np.min(np.minimum(hi1[iu[0]], hi1[iu[1]]) -
                  np.maximum(lo1[iu[0]], lo1[iu[1]]), axis=1)
    is_contact = cross & (pen1 > OVERLAP_DEPTH_MM) & \
        (pen0 < -SEPARATION_MM)
    is_adjacent = cross & (pen1 > OVERLAP_DEPTH_MM) & \
        (pen0 >= -SEPARATION_MM) & (pen0 <= OVERLAP_DEPTH_MM)
    contacts = []
    for k in np.flatnonzero(is_contact):
        a, b = uids[iu[0][k]], uids[iu[1][k]]
        contacts.append({"a": a, "b": b,
                         "link_a": link_of.get(a),
                         "link_b": link_of.get(b),
                         "pose_depth_mm": round(float(pen1[k]), 2),
                         "zero_gap_mm": round(float(-pen0[k]), 2)})
    contacts.sort(key=lambda r: -r["pose_depth_mm"])
    return {"contacts": contacts[:200],
            "n_contacts": int(np.sum(is_contact)),
            "n_adjacent": int(np.sum(is_adjacent)),
            "n_baseline": int(np.sum(cross & (pen0 > OVERLAP_DEPTH_MM)))}


def collision_range_summary(report: dict) -> list[dict]:
    """Per-joint angle ranges among colliding vs clean poses.

    Mesh-confirmed contacts define the verdict when present; the AABB
    candidate list is the fallback for runs without confirmation.
    """
    key = ("mesh_contacts" if any("mesh_contacts" in p
                                  for p in report["poses"])
           else "contacts")
    rows = []
    joint_ids = [j["joint"] for j in report["poses"][0]["supports"]]
    for n, jid in enumerate(joint_ids):
        hit = [p["q_deg"][n] for p in report["poses"]
               if p.get(key)]
        clean = [p["q_deg"][n] for p in report["poses"]
                 if not p.get(key)]
        rows.append({"joint": jid,
                     "colliding_q_deg": [round(v, 1) for v in hit],
                     "clean_min_deg": (round(min(clean), 1)
                                       if clean else None),
                     "clean_max_deg": (round(max(clean), 1)
                                       if clean else None)})
    return rows


def _launch(script: str, args: list[str]) -> None:
    cmd = [BLENDER_BIN, "-b", "--factory-startup", "--python-exit-code",
           "1", "-P", os.path.join(REPO, "src", "viz", script),
           "--"] + args
    print("launch:", " ".join(cmd))
    proc = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO)
    print(proc.stdout[-3000:])
    if proc.returncode != 0:
        print(proc.stderr[-3000:])
        raise RuntimeError(f"Blender run failed: {script}")


def run_blender(batch_path: str, results_path: str, renders_dir: str,
                quick: bool) -> None:
    args = ["--batch", batch_path, "--out", results_path,
            "--renders", renders_dir]
    if quick:
        args.append("--quick")
    _launch("pose_render_inner.py", args)


def run_confirm(batch_path: str, candidates_path: str,
                out_path: str) -> None:
    _launch("pose_confirm_inner.py",
            ["--batch", batch_path, "--candidates", candidates_path,
             "--out", out_path])


def extract_candidates(report: dict, pairs: list[dict],
                       top_n: int = 80) -> dict:
    """Top AABB contacts per pose for BVH confirmation.

    Known gear-mesh pairs are designed to interleave and are excluded
    up front; the rest need exact mesh truth.
    """
    known = set()
    for pr in pairs:
        known.add(tuple(sorted([pr["a"], pr["b"]])))
    out = {}
    for p in report["poses"]:
        cs = sorted(p.get("contacts", []),
                    key=lambda c: -c["pose_depth_mm"])
        sel = [[c["a"], c["b"]] for c in cs
               if tuple(sorted([c["a"], c["b"]])) not in known][:top_n]
        out[p["id"]] = sel
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Blender pose validation")
    ap.add_argument("--model", default="models/robot_inferred.json")
    ap.add_argument("--parts", default=PARTS_JSON)
    ap.add_argument("--links", default=LINKS_JSON)
    ap.add_argument("--catalog", default=CATALOG_YAML)
    ap.add_argument("--out-batch", default="runs/stage5/pose_batch.json")
    ap.add_argument("--results", default="runs/stage5/pose_render.json")
    ap.add_argument("--poses10", default="runs/stage5/poses10.json")
    ap.add_argument("--report", default="runs/stage5/pose_check_report.json")
    ap.add_argument("--renders", default="assets/private/renders/poses")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--skip-blender", action="store_true",
                    help="numeric checks only (no renders)")
    ap.add_argument("--blend-only", action="store_true",
                    help="merge existing batch+results without re-rendering")
    ap.add_argument("--confirm-only", action="store_true",
                    help="merge existing batch+results+confirm "
                    "without any Blender run")
    ap.add_argument("--candidates",
                    default="runs/stage5/pose_candidates.json")
    ap.add_argument("--confirm", default="runs/stage5/pose_confirm.json")
    ap.add_argument("--no-confirm", action="store_true",
                    help="skip the BVH confirmation pass (AABB only)")
    args = ap.parse_args()
    os.chdir(REPO)

    doc = load_model(args.model)
    parts = json.load(open(args.parts, encoding="utf-8"))
    links_doc = json.load(open(args.links, encoding="utf-8"))
    catalog = yaml.safe_load(open(args.catalog, encoding="utf-8")) or {}
    by_uid = {p["uid"]: p for p in parts}
    members = {link["id"]: set(link.get("members", []))
               for link in links_doc.get("links", [])}
    link_of = {}
    for lid, uids in members.items():
        for u in uids:
            link_of[u] = lid
    ordered = chain_order(doc)
    link_ids = [ordered[0]["parent"]] + [j["child"] for j in ordered]
    link_index = {lid: n for n, lid in enumerate(link_ids)}

    poses = sample_poses(doc)
    print(f"sampled {len(poses)} poses "
          f"({N_RANDOM} random + {N_EDGE} near-end)")

    supports = {}
    for j in ordered:
        sup = collect_supports(parts, catalog, members[j["parent"]],
                               members[j["child"]])
        for s in sup:
            s["link"] = link_of.get(s["uid"])
        supports[j["id"]] = sup
        print(f"{j['id']}: {len(sup)} support points")
    pairs = find_cross_link_meshes(parts, catalog, link_of)
    print(f"cross-link gear meshes at zero: {len(pairs)}")

    batch = {"repo": REPO, "base_blend": BASE_BLEND,
             "link_ids": link_ids,
             "joints": [{"id": j["id"], "axis": j["axis"],
                         "point_mm": j["point_mm"]} for j in ordered],
             "members": {lid: sorted(uids) for lid, uids in
                         members.items()},
             "ee_mm": doc.get("end_effector", {}).get("point_mm"),
             "poses": poses}
    os.makedirs(os.path.dirname(args.out_batch) or ".", exist_ok=True)
    json.dump(batch, open(args.out_batch, "w"), indent=1)

    blender_out = None
    if args.blend_only or args.confirm_only:
        batch = json.load(open(args.out_batch, encoding="utf-8"))
        poses = batch["poses"]
        blender_out = json.load(open(args.results, encoding="utf-8"))
    elif not args.skip_blender:
        os.makedirs(args.renders, exist_ok=True)
        run_blender(args.out_batch, args.results, args.renders, args.quick)
        blender_out = json.load(open(args.results, encoding="utf-8"))

    pose_rows = []
    for pose in poses:
        t_mats = link_transforms(doc, pose["q_deg"])
        row = {"id": pose["id"], "q_deg": pose["q_deg"],
               "m_deg": pose["m_deg"],
               "supports": check_supports_pose(doc, supports, link_index,
                                               t_mats, pose["q_deg"]),
               "gears": check_gears_pose(pairs, by_uid, link_index,
                                         t_mats)}
        row["support_ok"] = all(r["ok"] for r in row["supports"])
        row["gears_ok"] = all(r["ok"] for r in row["gears"])
        if blender_out is not None:
            b = blender_out["poses"][pose["id"]]
            row["ee_blender_mm"] = b["ee_mm"]
            row["ee_fk_mm"] = fk_model(pose["q_deg"], doc)["pos_mm"]
            row["ee_err_mm"] = round(float(np.linalg.norm(
                np.asarray(row["ee_blender_mm"]) -
                np.asarray(row["ee_fk_mm"]))), 3)
            row["ee_ok"] = row["ee_err_mm"] <= EE_TOL_MM
            z_aabbs = blender_out["zero_aabbs_mm"]
            p_aabbs = b["aabbs_mm"]
            contact = contact_analysis(z_aabbs, p_aabbs, link_of)
            row["contacts"] = contact["contacts"]
            row["n_contacts"] = contact["n_contacts"]
            row["n_adjacent"] = contact["n_adjacent"]
            row["n_baseline"] = contact["n_baseline"]
            row["overlap_ok"] = contact["n_contacts"] == 0
        pose_rows.append(row)

    report = {"poses": pose_rows,
              "tolerances_mm": {"support": SUPPORT_TOL_MM,
                                "gear_drift": GEAR_DRIFT_TOL_MM,
                                "ee": EE_TOL_MM,
                                "overlap_depth": OVERLAP_DEPTH_MM,
                                "separation": SEPARATION_MM},
              "n_fail_support": sum(1 for r in pose_rows
                                   if not r["support_ok"]),
              "n_fail_gears": sum(1 for r in pose_rows
                                  if not r["gears_ok"])}
    if blender_out is not None and not args.no_confirm:
        if args.confirm_only:
            confirm = json.load(open(args.confirm, encoding="utf-8"))
        else:
            candidates = extract_candidates(
                {"poses": pose_rows}, pairs)
            json.dump(candidates, open(args.candidates, "w"), indent=1)
            run_confirm(args.out_batch, args.candidates, args.confirm)
            confirm = json.load(open(args.confirm, encoding="utf-8"))
        mesh_index = {}
        for r in pose_rows:
            mesh_index[r["id"]] = {
                tuple(sorted([c["a"], c["b"]])): c
                for c in r.get("contacts", [])}
        for r in pose_rows:
            mesh_hits = []
            for a, b in confirm.get(r["id"], []):
                rec = dict(mesh_index[r["id"]].get(
                    tuple(sorted([a, b])), {"a": a, "b": b,
                                            "link_a": link_of.get(a),
                                            "link_b": link_of.get(b)}))
                mesh_hits.append(rec)
            mesh_hits.sort(key=lambda c: -c.get("pose_depth_mm", 0.0))
            r["mesh_contacts"] = mesh_hits
            r["n_mesh_contacts"] = len(mesh_hits)
            r["overlap_ok"] = not mesh_hits
    if blender_out is not None:
        report["n_fail_overlap"] = sum(
            1 for r in pose_rows if not r.get("overlap_ok", True))
        report["n_fail_ee"] = sum(
            1 for r in pose_rows if not r.get("ee_ok", True))
        report["range_summary"] = collision_range_summary(report)
    json.dump(report, open(args.report, "w"), indent=1)

    mesh_by_id = {r["id"]: r.get("n_mesh_contacts") for r in pose_rows}
    poses10 = [{"id": p["id"],
                "q_deg": {j["id"]: round(v, 3) for j, v in
                          zip(ordered, p["q_deg"])},
                "m_deg": {mid: round(v, 3) for mid, v in
                          zip([a["id"] for a in doc["actuators"]],
                              p["m_deg"])},
                "mesh_contacts": mesh_by_id.get(p["id"])} for p in poses]
    json.dump(poses10, open(args.poses10, "w"), indent=1)
    print(f"wrote {args.poses10} + {args.report}")

    failed = [r["id"] for r in pose_rows
              if not (r["support_ok"] and r["gears_ok"] and
                      r.get("overlap_ok", True) and r.get("ee_ok", True))]
    print(f"FAILED poses ({len(failed)}/{len(pose_rows)}): {failed}")
    for r in pose_rows:
        if r["id"] in failed:
            bad_s = [v["uid"] for s in r["supports"]
                     for v in s["violations"]]
            bad_g = [(g["a"], g["b"]) for g in r["gears"]
                     if not g["ok"]]
            mesh = [(c["a"], c["b"], c["link_a"], c["link_b"])
                    for c in r.get("mesh_contacts", [])[:10]]
            print(f"  {r['id']}: supports={sorted(set(bad_s))} "
                  f"gears={bad_g} "
                  f"mesh_contacts={r.get('n_mesh_contacts', 'n/a')} "
                  f"(aabb={r.get('n_contacts', 'n/a')}, "
                  f"adjacent={r.get('n_adjacent', 'n/a')}, "
                  f"baseline={r.get('n_baseline', 'n/a')}) "
                  f"ee_err={r.get('ee_err_mm', 'n/a')}")
            for c in mesh:
                print(f"    mesh-contact {c[0]}({c[2]}) x {c[1]}({c[3]})")


if __name__ == "__main__":
    main()
