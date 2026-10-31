"""Gizmo-path IK check (Stage 5C, no hardware).

Calls the same server endpoint the gizmo calls
(POST /api/ik via _Handler.dispatch_api -- the HTTP layer is
unavailable in this sandbox, so the dispatch layer is the
closest callable seam; raw socket transport is not exercised).
Same start pose for all targets. Evidence skeletons go to
runs/stage5c/ik_check/ (front/side PNG, inferred + GT overlay).
"""

from __future__ import annotations

import datetime
import json
import math
import os
import random
import struct
import zlib

import numpy as np

from src.console import control as ctl
from src.console.mini import (default_override_path, load_jog_map,
                              read_jog_overrides)
from src.console.control import stage_table
from src.ev3.fake import build_mp_transport
from src.kin.fk import fk_model
from src.kin.model import chain_order, joint_arrays, load_model

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]
Q0 = [0.0] * 6
N_RANDOM = 50
POS_BAR_MM = 5.0
MOVE_BAR_DEG = 2.0
NEED_BASE_JOINTS = 2  # of J1..J3 moving >= 2 deg
OUT_DIR = "runs/stage5c/ik_check"


def _se3(w, v, th):
    w = np.array(w, float)
    w = w / np.linalg.norm(w)
    v = np.array(v, float)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]],
                  [-w[1], w[0], 0]])
    R = (np.eye(3) + math.sin(th) * K
         + (1 - math.cos(th)) * (K @ K))
    # Closed form t = (I-R)(w x v) + w(w.v)th.
    wxv = np.cross(w, v)
    t = (np.eye(3) - R) @ wxv + w * float(w @ v) * th
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def frames_of(q_deg, axes, points):
    T = np.eye(4)
    frames = [T.copy()]
    joints = []
    for qi, w, p in zip(q_deg, axes, points):
        wn = np.array(w, float)
        wn = wn / np.linalg.norm(wn)
        joints.append((T @ np.array([*p, 1.0]))[:3])
        T = T @ _se3(wn, -np.cross(wn, np.array(p, float)),
                     math.radians(qi))
        frames.append(T.copy())
    return frames, joints


def skel_points(q_deg, doc):
    arr = joint_arrays(doc)
    frames, joints = frames_of(q_deg, arr["axes"], arr["points"])
    ee = (frames[-1] @ np.array([*doc["end_effector"]["point_mm"],
                                 1.0]))[:3]
    base = np.array([0.0, 0.0, 0.0])
    return [base] + joints + [ee]


def draw_skel(path, sk_inf, sk_gt, tgt, title, sk_start=None):
    W, H = 520, 420
    img = np.full((H, W, 3), 0x33 / 255.0, float)
    for view, ox in (("front", 0), ("side", 260)):
        if view == "front":
            P = [(x, z) for x, y, z in sk_inf]
            G = [(x, z) for x, y, z in sk_gt]
            Tp = (tgt[0], tgt[2])
        else:
            P = [(y, z) for x, y, z in sk_inf]
            G = [(y, z) for x, y, z in sk_gt]
            Tp = (tgt[1], tgt[2])
        SP = ([(x, z) for x, y, z in sk_start] if view == "front"
              else [(y, z) for x, y, z in sk_start]) if sk_start else []
        allx = [p[0] for p in P + G + SP + [Tp]]
        ally = [p[1] for p in P + G + SP + [Tp]]
        lo_x, hi_x = min(allx) - 30, max(allx) + 30
        lo_y, hi_y = min(ally) - 30, max(ally) + 30
        sc = min(250 / max(hi_x - lo_x, 1e-6),
                 380 / max(hi_y - lo_y, 1e-6))

        def px(p):
            return (int(ox + 5 + (p[0] - lo_x) * sc),
                    int(H - 5 - (p[1] - lo_y) * sc))

        def seg(a, b, col, dash=False):
            n = max(int(math.hypot(b[0] - a[0], b[1] - a[1]) * sc
                        / 4) + 1, 2)
            for k in range(n):
                if dash and (k // 3) % 2:
                    continue
                f = k / (n - 1)
                x = int(a[0] + (b[0] - a[0]) * f)
                y = int(a[1] + (b[1] - a[1]) * f)
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        xx, yy = x + dx, y + dy
                        if 0 <= xx < W and 0 <= yy < H:
                            img[yy, xx] = col

        def dot(p, col, r=3):
            x, y = px(p)
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < W and 0 <= yy < H:
                        img[yy, xx] = col
        PG = [px(p) for p in P]
        GG = [px(p) for p in G]
        if sk_start is not None:
            S = [(x, z) if view == "front" else (y, z)
                 for x, y, z in sk_start]
            SG = [px(p) for p in S]
            for a, b in zip(SG[:-1], SG[1:]):
                seg(a, b, (0.55, 0.55, 0.55), dash=True)
        for a, b in zip(GG[:-1], GG[1:]):
            seg(a, b, (1.0, 0.55, 0.1), dash=True)
        for a, b in zip(PG[:-1], PG[1:]):
            seg(a, b, (0.3, 0.6, 1.0))
        dot(PG[-1], (0.2, 1.0, 0.3), 4)
        dot(Tp, (1.0, 0.2, 0.2), 4)
        for sx in (-1, 1):
            for sy in (-1, 1):
                dot((Tp[0] + sx * 6, Tp[1] + sy * 6), (1.0, 0.2, 0.2),
                    1)
    _png(path, img)


def _png(path, img):
    H, W, _ = img.shape
    raw = bytearray()
    for y in range(H):
        raw.append(0)
        raw.extend((np.clip(img[y], 0, 1) * 255 + 0.5).astype(
            np.uint8).tobytes())

    def chunk(t, d):
        return (struct.pack(">I", len(d)) + t + d
        + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff))
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", W, H, 8, 2, 0,
                                       0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as fh:
        fh.write(png)


def _handler():
    h = ctl._Handler.__new__(ctl._Handler)
    transport, _, _ = build_mp_transport(full=True)
    h.transport = transport
    h.jog_map = load_jog_map(overrides=read_jog_overrides(
        default_override_path()))
    h.jog_overrides = {}
    import tempfile
    tmp = tempfile.mkdtemp(prefix="ikcheck")
    h.override_path = os.path.join(tmp, "jogmap.local.json")
    h.direction_path = os.path.join(tmp, "direction.local.json")
    h.model_doc = load_model("models/robot_inferred.json")
    h.zeros = {}
    h.console_path = os.path.join(tmp, "console.local.json")
    h.private_visuals = None
    return h


def adopted_ranges(handler):
    return {r["joint"]: r["adopted"]
            for r in ctl.ranges_payload(handler)["joints"]}


def main(argv=None) -> None:
    import argparse
    ap = argparse.ArgumentParser(description="gizmo-path IK check")
    ap.add_argument("--out", default=OUT_DIR)
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args(argv)
    rng = random.Random(args.seed)
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(os.path.join(args.out, "failures"), exist_ok=True)

    doc = load_model("models/robot_inferred.json")
    gt = load_model("gt/kinematics_gt_v2.json")
    h = _handler()
    adopted = adopted_ranges(h)
    TABLE = stage_table()
    from src.kin.ik import _coupled_ok
    coupled = __import__("json").load(
        open("packages/robot_inferred/robot_package.json")
    ).get("coupled_limits", [])

    def ok_q(q):
        if any(not adopted[j][0] <= q[i] <= adopted[j][1]
               for i, j in enumerate(JOINTS)):
            return False
        return _coupled_ok(q, JOINTS, coupled)

    # Reachable pool: in-range FK targets.
    pool = []
    for _ in range(6000):
        q = [rng.uniform(*adopted[j]) for j in JOINTS]
        if ok_q(q):
            pool.append((q, fk_model(q, doc)["pos_mm"]))
    # 50 spread random + 10 body-near + 5 unreachable.
    picked, pts = [], []
    for q, p in pool:
        if not pts or min(float(np.linalg.norm(np.array(p) - x))
                         for x in pts) > 45.0:
            picked.append(("r%02d" % (len(picked) + 1), q, p))
            pts.append(np.array(p))
        if len(picked) >= N_RANDOM:
            break
    by_x = sorted(pool, key=lambda t: t[1][0])
    by_y = sorted(pool, key=lambda t: abs(t[1][1]))
    by_z = sorted(pool, key=lambda t: t[1][2])
    near = ([("front%d" % (i + 1),) + by_x[-(i + 1)] for i in range(3)]
            + [("side%d" % (i + 1),) + by_y[-(i + 1)]
               for i in range(3)]
            + [("top%d" % (i + 1),) + by_z[-(i + 1)] for i in range(2)]
            + [("low%d" % (i + 1),) + by_z[i] for i in range(2)])
    reach = float(max(np.linalg.norm(p) for _, p in pool))
    c = np.mean([p for _, p in pool], axis=0)
    far = [("far%d" % (i + 1), None,
            (c + (np.array(p) - c) / max(np.linalg.norm(
                np.array(p) - c), 1e-6) * (reach + 150.0)).tolist())
           for i, (_, p) in enumerate(pool[:3])]
    floor_z = min(p[2] for _, p in pool)
    low = [("under%d" % (i + 1), None,
            [float(c[0]), float(c[1]), floor_z - 60.0 - 20.0 * i])
           for i in range(2)]

    rows = []
    for tid, q_true, tgt in picked + near + far + low:
        kind = ("unreachable" if tid.startswith(("far", "under"))
                else "reachable")
        # EXACT gizmo call: pos/mat/q_init/position_only.
        status, res = h.dispatch_api("/api/ik", {
            "pos": [float(v) for v in tgt], "mat": None,
            "q_init": list(Q0), "position_only": True})
        row = {"id": tid, "kind": kind,
               "target_mm": [round(v, 1) for v in tgt],
               "target_stage_mm": [
                   round(tgt[0] - TABLE["center_x_mm"], 1),
                   round(tgt[1] - TABLE["center_y_mm"], 1),
                   round(tgt[2] - TABLE["table_z_mm"], 1)]}
        if status != 200 or not res.get("ok"):
            row.update(status="api-error",
                       reason=str(res)[:120])
            rows.append(row)
            continue
        r = res["ik"]
        q = [float(v) for v in r["q_deg"]]
        row["q_deg"] = [round(v, 2) for v in q]
        row["dq"] = [round(a - b, 2) for a, b in zip(q, Q0)]
        row["success"] = bool(r["success"])
        # Independent TCP check (FK, not the solver's number).
        f = fk_model(q, doc)
        row["tcp_err_mm"] = round(float(np.linalg.norm(
            np.array(f["pos_mm"]) - np.array(tgt))), 2)
        row["solver_err_mm"] = round(float(r["pos_err_mm"]), 2)
        row["base_moved"] = [j for j in JOINTS[:3]
                             if abs(row["dq"][JOINTS.index(j)])
                             >= MOVE_BAR_DEG]
        row["n_moved"] = sum(1 for d in row["dq"]
                             if abs(d) >= MOVE_BAR_DEG)
        row["at_limit"] = [
            j for i, j in enumerate(JOINTS)
            if abs(q[i] - adopted[j][0]) < 0.5
            or abs(q[i] - adopted[j][1]) < 0.5]
        if kind == "reachable":
            row["pass"] = (row["tcp_err_mm"] <= POS_BAR_MM
                           and len(row["base_moved"]) >=
                           NEED_BASE_JOINTS)
            row["reason"] = ("ok" if row["pass"] else
                             "err>5mm" if row["tcp_err_mm"]
                             > POS_BAR_MM else "base joints static")
        else:
            row["pass"] = (not row["success"]
                           and row["n_moved"] >= NEED_BASE_JOINTS)
            row["reason"] = ("unreachable+full-arm" if row["pass"]
                             else "no-unreachable-flag"
                             if row["success"]
                             else "arm static")
        row["status"] = "pass" if row["pass"] else "FAIL"
        sk1 = skel_points(q, doc)
        skg = skel_points(q, gt)
        sk0 = skel_points(Q0, doc)
        sub = "failures" if row["status"] == "FAIL" else "."
        draw_skel(os.path.join(args.out, sub, f"{tid}.png"),
                  sk1, skg, tgt, tid, sk_start=sk0)
        rows.append(row)
    h.transport.close()

    fails = [r for r in rows if r.get("status") == "FAIL"]
    n_reach = sum(1 for r in rows if r.get("kind") == "reachable")
    p_reach = sum(1 for r in rows if r.get("kind") == "reachable"
                  and r.get("pass"))
    report = {
        "at": datetime.datetime.now().isoformat(timespec="seconds"),
        "stage": TABLE,
        "criterion": {"tcp_bar_mm": POS_BAR_MM,
                      "base_bar": f">={NEED_BASE_JOINTS} of J1-J3 "
                      f"move >={MOVE_BAR_DEG}deg",
                      "start": Q0, "endpoint": "/api/ik "
                      "(position_only, mat None)"},
        "pass_reachable": [p_reach, n_reach],
        "pass_unreachable": [sum(1 for r in rows
                                 if r.get("kind") == "unreachable"
                                 and r.get("pass")),
                             sum(1 for r in rows
                                 if r.get("kind") == "unreachable")],
        "targets": rows,
    }
    with open(os.path.join(args.out, "report.json"), "w",
              encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    lines = ["| id | kind | tcp_err_mm | dq J1..J6 | pass |",
             "|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['id']} | {r.get('kind')} | "
                     f"{r.get('tcp_err_mm', '?')} | "
                     f"{r.get('dq', '?')} | "
                     f"{r.get('status', r.get('pass'))} |")
    with open(os.path.join(args.out, "table.md"), "w",
              encoding="utf-8") as fh:
        fh.write("# ik_check\n\n" + "\n".join(lines) + "\n")
    print(f"reachable {p_reach}/{n_reach}, "
          f"unreachable {report['pass_unreachable'][0]}/"
          f"{report['pass_unreachable'][1]}, "
          f"fails: {[r['id'] for r in fails]}")


if __name__ == "__main__":
    main()

