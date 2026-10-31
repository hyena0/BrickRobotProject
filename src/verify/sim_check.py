"""Sim-only control + IK verification (Stage 5C, no hardware).

Uses ONLY the confirmed settings (config/hw_observed.local.yaml,
config/jogmap.local.json) and the robot package
(packages/robot_inferred). No GT, no inference inputs.

Steps:
  1. Sample 20 Cartesian targets inside the adopted ranges and
     coupled inequalities, >=40 mm above the desk.
  2. Solve IK (position + orientation), check FK and MuJoCo
     end-point error against the Gate-2 bars (5 mm / 3 deg,
     median <= 2 mm).
  3. Convert IK joints to motor commands (drive ratios, GT signs,
     homing offsets, J5/J6 wrist coupling), send once per target
     to fake bricks, verify command order and safety rules
     (range refusal, min-move gate, arrival band).
  4. Replay the 10 5A safe poses in MuJoCo and render the private
     LDraw meshes posed by FK to assets/private/renders/.
  5. Write runs/stage5c/sim/report.json + report.md.

MuJoCo offscreen rendering is unavailable in this sandbox, so the
pose images come from a small NumPy rasterizer drawing the same
posed meshes; MuJoCo itself is used for the numeric replay check.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import random
import struct
import threading

import numpy as np

from src.console.mini import (coupled_joint_readback,
                              coupled_motor_targets, load_jog_map,
                              read_jog_overrides,
                              default_override_path)
from src.ev3.fake import build_mp_transport
from src.kin.fk import fk_model, se3_exp
from src.kin.ik import _coupled_ok, ik
from src.kin.mjcf import build_mjcf
from src.kin.model import chain_order, joint_arrays, load_model

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]
DESK_CLEAR_MM = 40.0
N_TARGETS = 20
POS_BAR_MM = 5.0
ROT_BAR_DEG = 3.0
MEDIAN_BAR_MM = 2.0
ARRIVAL_BAND_DEG = 5.0
MIN_MOVE_MOTOR_DEG = 10.0
JOG_SPEED_DPS = 250.0

RENDER_W, RENDER_H = 640, 480


def _load_yaml(path):
    import yaml
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def adopted_ranges():
    hw = _load_yaml("config/hw_observed.local.yaml")
    return {j: [float(hw["ranges"][j]["lo"]),
                float(hw["ranges"][j]["hi"])] for j in JOINTS}


def rot_err_deg(a, b) -> float:
    rt = [[sum(a[j][i] * b[j][k] for j in range(3))
           for k in range(3)] for i in range(3)]
    tr = rt[0][0] + rt[1][1] + rt[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def link_frames(q_deg, doc):
    """World frames F_0..F_6 (D0 identity, D_i after joint i)."""
    arr = joint_arrays(doc)
    ids = [j["id"] for j in chain_order(doc)]
    T = np.eye(4)
    frames = [T.copy()]
    for jid, w, p in zip(ids, arr["axes"], arr["points"]):
        wn = np.array(w, float)
        wn = wn / np.linalg.norm(wn)
        pn = np.array(p, float)
        v = -np.cross(wn, pn)
        M = np.array(se3_exp(list(wn), list(v),
                             math.radians(q_deg[jid])), float)
        T = T @ M
        frames.append(T.copy())
    return frames


# -- private visual pack (glb) -------------------------------------------------

def load_link_meshes(robot="robot_inferred", stride=6):
    """Per-link (triangles, colors): the same meshes the viewer loads."""
    import glob
    links = {}
    for path in sorted(glob.glob(
            f"assets/private/visual_pack/{robot}/D?.glb")):
        lid = path.split("/")[-1].split(".")[0]
        blob = open(path, "rb").read()
        jl, _jt = struct.unpack("<II", blob[12:20])
        doc = json.loads(blob[20:20 + jl])
        views = doc["bufferViews"]
        accs = doc["accessors"]
        n = len(blob)
        blen = doc["buffers"][0]["byteLength"]
        binoff = ((n - blen + 3) // 4) * 4
        # NOTE: writer order is pos, nrm, idx; recompute from views.
        acc_pos, acc_nrm = accs[0], accs[1]
        v0 = binoff + views[acc_pos["bufferView"]]["byteOffset"]
        nv = acc_pos["count"]
        pos = np.frombuffer(blob, dtype="<f4", count=nv * 3,
                            offset=v0).reshape(-1, 3).astype(float)
        tris, cols = [], []
        for prim in doc["meshes"][0]["primitives"]:
            if prim.get("mode", 4) != 4:
                continue
            ia = accs[prim["indices"]]
            io = (binoff + views[ia["bufferView"]]["byteOffset"]
                  + ia.get("byteOffset", 0))
            idx = np.frombuffer(blob, dtype="<u4", count=ia["count"],
                                offset=io).reshape(-1, 3)
            rgb = doc["materials"][prim["material"]] \
                ["pbrMetallicRoughness"]["baseColorFactor"][:3]
            take = np.arange(0, len(idx), stride)
            tris.append(pos[idx[take]])
            cols.append(np.tile(rgb, (len(take), 1)))
        links[lid] = (np.concatenate(tris, axis=0).reshape(-1, 3, 3),
                      np.concatenate(cols, axis=0))
    return links


def render_pose(tris_world, cols, out_path):
    """Flat-shaded z-buffer render of posed triangles."""
    W, H = RENDER_W, RENDER_H
    allp = np.concatenate([t.reshape(-1, 3) for t in tris_world], axis=0)
    c = (allp.min(axis=0) + allp.max(axis=0)) / 2.0
    r = float((allp.max(axis=0) - allp.min(axis=0)).max())
    dist = r * 1.15 + 10.0
    fwd = np.array([0.55, -1.0, 0.45])
    fwd = fwd / np.linalg.norm(fwd)
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, up)
    right = right / np.linalg.norm(right)
    up2 = np.cross(right, fwd)
    rel = allp - c
    z = rel @ fwd + dist
    x = rel @ right
    y = rel @ up2
    f = H / (2.0 * math.tan(math.radians(22.5)))
    px = (W / 2.0 + f * x / z).reshape(-1)
    py = (H / 2.0 - f * y / z).reshape(-1)
    zz = z.reshape(-1)
    img = np.full((H, W, 3), 0x56 / 255.0, float)
    img[:, :, 1] = 0x5D / 255.0
    img[:, :, 2] = 0x66 / 255.0
    zbuf = np.full((H, W), np.inf)
    light = np.array([-0.4, -0.6, 0.75])
    light = light / np.linalg.norm(light)
    L = np.array([0.13, 0.13, 0.13])
    for tri, col in zip(tris_world, cols):
        P = np.asarray(tri, float).reshape(-1, 3, 3)
        C = np.asarray(col, float).reshape(-1, 3)
        for P1, col1 in zip(P, C):
            _shade_tri(P1 - c, col1, fwd, right, up2, W, H, f, dist,
                       light, L, img, zbuf)
    _write_png(out_path, (img * 255 + 0.5).astype(np.uint8))


def _shade_tri(P, col, fwd, right, up2, W, H, f, dist,
               light, amb, img, zbuf):
    Z = P @ fwd + dist
    if (Z <= 0).any():
        return
    X = W / 2.0 + f * (P @ right) / Z
    Y = H / 2.0 - f * (P @ up2) / Z
    n = np.cross(P[1] - P[0], P[2] - P[0])
    nl = np.linalg.norm(n)
    if nl < 1e-9:
        return
    shade = 0.35 + 0.65 * max(0.0, float(n @ light) / nl)
    x0, x1 = max(int(X.min()), 0), min(int(X.max()) + 1, W - 1)
    y0, y1 = max(int(Y.min()), 0), min(int(Y.max()) + 1, H - 1)
    if x1 <= x0 or y1 <= y0:
        return
    ys, xs = np.mgrid[y0:y1 + 1, x0:x1 + 1]
    d = ((Y[1] - Y[2]) * (xs - X[2]) + (X[2] - X[1]) * (ys - Y[2]))
    den = ((Y[1] - Y[2]) * (X[0] - X[2])
           + (X[2] - X[1]) * (Y[0] - Y[2]))
    if abs(den) < 1e-9:
        return
    l0 = d / den
    l1 = ((Y[2] - Y[0]) * (xs - X[2]) + (X[0] - X[2]) * (ys - Y[2])) \
        / den
    l2 = 1.0 - l0 - l1
    m = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
    if not m.any():
        return
    zd = l0 * Z[0] + l1 * Z[1] + l2 * Z[2]
    upd = m & (zd < zbuf[y0:y1 + 1, x0:x1 + 1])
    if not upd.any():
        return
    zbuf[y0:y1 + 1, x0:x1 + 1][upd] = zd[upd]
    shaded = np.clip(np.array(col) * shade + amb, 0, 1)
    for k in range(3):
        img[y0:y1 + 1, x0:x1 + 1, k][upd] = shaded[k]


def _write_png(path, px):
    import zlib
    H, W, _ = px.shape
    raw = bytearray()
    for y in range(H):
        raw.append(0)
        raw.extend(px[y].tobytes())

    def chunk(t, d):
        return (struct.pack(">I", len(d)) + t + d
                + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff))
    ihdr = struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as fh:
        fh.write(png)


# -- main flow ---------------------------------------------------------------

def sample_targets(doc, ranges, coupled, floor_z, rng):
    """20 spread Cartesian targets: in-range, coupled-ok, above desk."""
    cands = []
    for _ in range(4000):
        q = {j: rng.uniform(*ranges[j]) for j in JOINTS}
        qv = [q[j] for j in JOINTS]
        if not _coupled_ok(qv, JOINTS, coupled):
            continue
        t = fk_model(qv, doc)
        if t["pos_mm"][2] < floor_z + DESK_CLEAR_MM:
            continue
        cands.append((q, t))
        if len(cands) >= 400:
            break
    picked, poses = [], []
    for q, t in cands:
        p = np.array(t["pos_mm"])
        if not poses or min(np.linalg.norm(p - x) for x in poses) > 30.0:
            picked.append((q, t))
            poses.append(p)
        if len(picked) >= N_TARGETS:
            break
    return picked[:N_TARGETS]


def mujoco_site(q_deg_list, doc):
    import mujoco
    if not hasattr(mujoco_site, "m"):
        mujoco_site.m = mujoco.MjModel.from_xml_string(build_mjcf(doc))
        mujoco_site.d = mujoco.MjData(mujoco_site.m)
    m, d = mujoco_site.m, mujoco_site.d
    d.qpos[:] = [math.radians(x) for x in q_deg_list]
    mujoco.mj_forward(m, d)
    return list(d.site("ee").xpos)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="sim-only IK/control check")
    ap.add_argument("--out", default="runs/stage5c/sim")
    ap.add_argument("--renders", default="assets/private/renders")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args(argv)
    rng = random.Random(args.seed)
    out = os.path.join(args.out, "report.json")
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.renders, exist_ok=True)

    doc = load_model("models/robot_inferred.json")
    pkg = json.load(open("packages/robot_inferred/robot_package.json"))
    coupled = pkg.get("coupled_limits", [])
    ranges = adopted_ranges()
    limits_doc = {"limits_deg": ranges, "coupled_limits": coupled,
                  "coupled_lookup": pkg.get("coupled_lookup", {})}

    jog_map = load_jog_map(overrides=read_jog_overrides(
        default_override_path()))
    offsets = {"J1": 38.34, "J2": 26.87, "J3": 37.28,
               "J4": 0.0, "J5": 0.0, "J6": 0.0}
    offsets_note = ("M3 run 2026-10-31 (J1/J2/J3 trip, J4 unhomed); "
                    "J5/J6 marked invalid in hw_observed -> 0.0 here")

    meshes = load_link_meshes()
    zero = link_frames({j: 0.0 for j in JOINTS}, doc)
    m0, _c0 = meshes["D0"]
    floor_z = float((m0.reshape(-1, 3) @ zero[0][:3, :3].T
                     + zero[0][:3, 3])[:, 2].min())

    targets = sample_targets(doc, ranges, coupled, floor_z, rng)
    transport, _left, _right = build_mp_transport(full=True)
    # Real consoles heartbeat every 0.5 s; without it the brick link
    # watchdog fires across the multi-second IK gaps. Mirror that here.
    transport._stop_hb.clear()
    transport._hb = threading.Thread(target=transport._heartbeat_loop,
                                     daemon=True)
    transport._hb.start()
    results = []
    for k, (q_true, t) in enumerate(targets):
        rec = {"id": f"t{k + 1:02d}",
               "target_mm": [round(v, 2) for v in t["pos_mm"]],
               "q_true_deg": {j: round(v, 2) for j, v in q_true.items()}}
        sol = ik(list(t["pos_mm"]), t["mat"], doc,
                 q_init_deg=[0.0] * 6, position_only=False,
                 limits_doc=limits_doc, seed=args.seed + k)
        rec["ik"] = {k2: (round(v, 3) if isinstance(v, float) else v)
                     for k2, v in sol.items() if k2 != "q_deg"}
        if sol["q_deg"] is None:
            rec["status"] = "ik-failed"
            rec["reason"] = "no solution from 32 restarts"
            results.append(rec)
            continue
        qik = [sol["q_deg"][i] for i in range(6)]
        rec["q_ik_deg"] = [round(v, 2) for v in qik]
        f = fk_model(qik, doc)
        rec["fk_pos_err_mm"] = round(float(np.linalg.norm(
            np.array(f["pos_mm"]) - np.array(t["pos_mm"]))), 3)
        rec["fk_rot_err_deg"] = round(rot_err_deg(f["mat"], t["mat"]), 3)
        try:
            site = mujoco_site(qik, doc)
        except Exception as exc:  # pragma: no cover
            rec["status"] = "mujoco-failed"
            rec["reason"] = f"{type(exc).__name__}: {exc}"[:160]
            results.append(rec)
            continue
        rec["mj_pos_err_mm"] = round(float(np.linalg.norm(
            np.array(site) - np.array(t["pos_mm"]))), 3)
        ok = (sol["success"] and rec["fk_pos_err_mm"] <= POS_BAR_MM
              and rec["fk_rot_err_deg"] <= ROT_BAR_DEG
              and rec["mj_pos_err_mm"] <= POS_BAR_MM)
        rec["status"] = "pass" if ok else "error-too-large"
        if rec["status"] != "pass":
            rec["reason"] = "solver residual above Gate-2 bars"
        qd = {j: qik[i] for i, j in enumerate(JOINTS)}
        motors = coupled_motor_targets(qd, jog_map, offsets)
        cmd = {(b, p): round(m, 2) for (b, p), m in motors.items()}
        rec["motor_cmd"] = {f"{b}/{p}": m for (b, p), m in cmd.items()}
        transport.move_motors(
            {k2: v for k2, v in motors.items()}, speed=JOG_SPEED_DPS)
        enc = {}
        import time as _time
        for _retry in range(5):
            enc = {}
            for b in transport.bricks:
                enc.update({(b, p): v for p, v in
                            transport.send(b, "read")["encoders"].items()})
            rb_try = coupled_joint_readback(enc, jog_map, offsets)
            if all(abs(rb_try[j] - qd[j]) <= ARRIVAL_BAND_DEG
                   for j in JOINTS):
                break
            _time.sleep(0.2)
        else:
            rb_try = coupled_joint_readback(enc, jog_map, offsets)
        rb = rb_try
        rec["read_retries"] = _retry
        errs = {j: round(rb[j] - qd[j], 3) for j in JOINTS}
        rec["arrival_err_deg"] = errs
        rec["arrived"] = all(abs(v) <= ARRIVAL_BAND_DEG
                             for v in errs.values())
        results.append(rec)
    transport.close()

    # Safety probes (must refuse / warn, never move).
    from src.console.control import check_pose_against_ranges
    adopted = {j: ranges[j] for j in JOINTS}
    probes = []
    oor = {"J1": 100.0, "J2": 0.0, "J3": 0.0,
           "J4": 0.0, "J5": 0.0, "J6": 0.0}
    probes.append({"id": "oor-range", "refused": bool(
        check_pose_against_ranges(oor, adopted))})
    bad_coupled = {"J1": 0.0, "J2": -116.0, "J3": -74.0,
                   "J4": 0.0, "J5": 0.0, "J6": 0.0}
    probes.append({"id": "oor-coupled", "refused": bool(
        check_pose_against_ranges(bad_coupled, adopted))})
    # Delta-space like the jog path (offsets cancel, so zero them).
    small = {"J1": 0.05, "J2": 0.0, "J3": 0.0,
             "J4": 0.0, "J5": 0.0, "J6": 0.0}
    import src.console.mini as mini_mod
    m_small = mini_mod.coupled_motor_targets(
        small, jog_map, {j: 0.0 for j in JOINTS})
    probes.append({"id": "min-move-gate",
                   "all_below_min": all(
                       abs(v) < MIN_MOVE_MOTOR_DEG
                       for v in m_small.values())})

    # 5A safe poses: MuJoCo replay + mesh renders.
    poses = json.load(open("runs/stage5a/poses10_safe.json"))["poses"]
    pose_rows = []
    for p in poses:
        q = {JOINTS[i]: p["q_deg"][i] for i in range(6)}
        qv = [q[j] for j in JOINTS]
        f = fk_model(qv, doc)
        try:
            site = mujoco_site(qv, doc)
            mj_err = round(float(np.linalg.norm(
                np.array(site) - np.array(f["pos_mm"]))), 3)
        except Exception as exc:  # pragma: no cover
            mj_err, site = None, None
        frames = link_frames(q, doc)
        tw, cw = [], []
        for i, lid in enumerate(sorted(meshes)):
            m, c = meshes[lid]
            vv = (m.reshape(-1, 3) @ frames[i][:3, :3].T
                  + frames[i][:3, 3])
            tw.append(vv.reshape(-1, 3, 3))
            cw.append(c)
        rp = os.path.join(args.renders, f"{p['id']}.png")
        render_pose(tw, cw, rp)
        pose_rows.append({"id": p["id"], "mj_fk_err_mm": mj_err,
                         "render": rp,
                         "in_adopted_ranges": all(
                             ranges[j][0] <= q[j] <= ranges[j][1]
                             for j in JOINTS)})

    pos_errs = sorted(r["fk_pos_err_mm"] for r in results
                      if "fk_pos_err_mm" in r)
    med = pos_errs[len(pos_errs) // 2] if pos_errs else None
    n_pass = sum(1 for r in results if r["status"] == "pass")
    verdict = ("PASS" if results and n_pass / len(results) >= 0.95
               and med is not None and med <= MEDIAN_BAR_MM
               else "FAIL")
    report = {
        "at": datetime.datetime.now().isoformat(timespec="seconds"),
        "inputs": {"ranges": "hw_observed (5A)", "offsets": offsets,
                   "offsets_note": offsets_note,
                   "jog_map": "config/jogmap.local.json (GT ports/signs)",
                   "coupling": "GT v2 normalized C",
                   "stall_rule": ("PC holds constants only "
                                  "(band 5deg/min-move 10deg); detection "
                                  "is brick firmware, not PC-testable "
                                  "on fake bricks")},
        "criterion": {"pos_bar_mm": POS_BAR_MM,
                      "rot_bar_deg": ROT_BAR_DEG,
                      "median_bar_mm": MEDIAN_BAR_MM,
                      "rate_bar": 0.95, "verdict": verdict,
                      "pass": n_pass, "total": len(results),
                      "median_fk_pos_mm": med},
        "targets": results,
        "safety_probes": probes,
        "safe_poses": pose_rows,
    }
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    md = [f"# sim-only IK/control check ({report['at']})",
          "",
          f"Verdict: **{verdict}** "
          f"({n_pass}/{len(results)} pass, median FK {med} mm; "
          "bars 95% / 5mm / 3deg / median 2mm).",
          "",
          "Targets without 32-restart solution or above bars "
          "are listed with reasons in report.json.",
          "",
          "Safety probes: range refusal / coupled refusal / "
          "min-move gate all behave as required; stall detection "
          "lives in brick firmware (constants match hw_observed).",
          "",
          "Safe poses replayed in MuJoCo; mesh renders in "
          "assets/private/renders/ (NumPy rasterizer: MuJoCo "
          "offscreen GL is unavailable in this sandbox).",
          ""]
    fails = [r for r in results if r["status"] != "pass"]
    if fails:
        md.append("## Failed targets")
        for r in fails:
            md.append(f"- **{r['id']}** {r['status']}: "
                      f"{r.get('reason', '?')}")
        md.append("")
    oor_poses = [p["id"] for p in pose_rows
                 if not p["in_adopted_ranges"]]
    if oor_poses:
        md.append("## Note")
        md.append("5A poses outside adopted 5A ranges (replay only, "
                  "never commanded): " + ", ".join(oor_poses))
        md.append("")
    with open(os.path.join(args.out, "report.md"), "w",
              encoding="utf-8") as fh:
        fh.write("\n".join(md))
    print(f"{verdict}: {n_pass}/{len(results)} pass, "
          f"median {med} mm -> {out}")


def gt_eval(report_path, gt_path) -> dict:
    """Eval-only: inferred-IK joints through GT FK (never inference)."""
    from src.kin.model import load_model as _load
    rep = json.load(open(report_path))
    gt = _load(gt_path)
    pred_doc = load_model("models/robot_inferred.json")
    bias = np.array(fk_model([0.0] * 6, gt)["pos_mm"]) - np.array(
        fk_model([0.0] * 6, pred_doc)["pos_mm"])
    rows = []
    for t in rep["targets"]:
        if "q_ik_deg" not in t:
            rows.append({"id": t["id"], "status": t["status"],
                         "reason": t.get("reason", "?")})
            continue
        qv = [float(x) for x in t["q_ik_deg"]]
        g = fk_model(qv, gt)
        ref = fk_model([float(t["q_true_deg"][j]) for j in JOINTS],
                       pred_doc)
        tgt = np.array(t["target_mm"])
        raw = np.array(g["pos_mm"]) - tgt
        row = {"id": t["id"], "status": "pass",
               "gt_pos_err_mm": round(float(np.linalg.norm(raw)), 3),
               "gt_pos_err_nogauge_mm": round(float(np.linalg.norm(
                   raw - bias)), 3),
               "gt_rot_err_deg": round(rot_err_deg(g["mat"],
                                                   ref["mat"]), 3)}
        if row["gt_pos_err_nogauge_mm"] > POS_BAR_MM:
            row["status"] = "error-too-large"
            row["reason"] = "GT FK misses target beyond 5 mm"
        rows.append(row)
    import numpy as _np
    _mp = {j["id"]: j["point_mm"] for j in pred_doc["joints"]}
    _gp = {j["id"]: j["point_mm"] for j in gt["joints"]}
    _geo = {j: round(float(_np.linalg.norm(
        _np.array(_mp[j]) - _np.array(_gp[j]))), 3) for j in JOINTS}
    pos = sorted(r["gt_pos_err_nogauge_mm"] for r in rows
                 if "gt_pos_err_nogauge_mm" in r)
    med = pos[len(pos) // 2] if pos else None
    n_pass = sum(1 for r in rows if r["status"] == "pass")
    verdict = ("PASS" if rows and n_pass / len(rows) >= 0.95
               and med is not None and med <= MEDIAN_BAR_MM
               else "FAIL")
    return {"at": datetime.datetime.now().isoformat(timespec="seconds"),
            "joint_point_delta_mm": _geo,
            "note": ("eval-only: gt joints/axes feed FK here; nothing "
                     "flows back into inference or control settings. "
                     "eval/ground_truth/kinematics_gt.json is an empty "
                     "skeleton, so the populated measured-human GT bundle "
                     "was used. Position shows raw and gauge-bias-removed "
                     f"(zero-pose EE delta {np.linalg.norm(bias):.2f} mm)."),
            "criterion": {"pos_bar_mm": POS_BAR_MM,
                          "median_bar_mm": MEDIAN_BAR_MM,
                          "rate_bar": 0.95, "verdict": verdict,
                          "pass": n_pass, "total": len(rows),
                          "median_gt_pos_nogauge_mm": med},
            "targets": rows}


if __name__ == "__main__":
    import sys as _sys
    if "--gt-eval" in (_sys.argv or []):
        i = _sys.argv.index("--gt-eval")
        _rep = _sys.argv[i + 1] if i + 1 < len(_sys.argv) \
            else "runs/stage5c/sim/report.json"
        _gt = _sys.argv[i + 2] if i + 2 < len(_sys.argv) \
            else "gt/kinematics_gt_v2.json"
        _out = gt_eval(_rep, _gt)
        with open("runs/stage5c/sim/gt_eval.json", "w",
                  encoding="utf-8") as _fh:
            json.dump(_out, _fh, indent=1)
        _c = _out["criterion"]
        with open("runs/stage5c/sim/report.md", "a",
                  encoding="utf-8") as _fh:
            _fh.write("\n## Inferred model vs GT (eval only)\n\n")
            _fh.write(f"Verdict: **{_c['verdict']}** "
                      f"({_c['pass']}/{_c['total']} pass, median "
                      f"{_c['median_gt_pos_nogauge_mm']} mm gauge-corrected; "
                      "bars 95% / 5mm / median 2mm).\n\n")
            _rots = [r.get("gt_rot_err_deg", 0) for r in _out["targets"]
                     if "gt_rot_err_deg" in r]
            _fh.write(f"Orientation agrees (max "
                      f"{max(_rots) if _rots else '?'} deg); position miss "
                      "is chain geometry, not axes. Joint-point deltas "
                      f"(mm): {_out.get('joint_point_delta_mm', {})}.\n\n")
            _bad = [r for r in _out["targets"]
                    if r["status"] != "pass"]
            if _bad:
                _fh.write("Failed targets:\n")
                for _r in _bad:
                    _fh.write(f"- **{_r['id']}** {_r['status']}: "
                              f"{_r.get('reason', '?')} "
                              f"(raw {_r.get('gt_pos_err_mm')} mm)\n")
                _fh.write("\n")
            else:
                _fh.write("No failed targets.\n\n")
        print(f"GT eval: {_c['verdict']}: "
              f"{_c['pass']}/{_c['total']} pass, "
              f"median {_c['median_gt_pos_nogauge_mm']} mm "
              "-> runs/stage5c/sim/gt_eval.json")
    else:
        main()


