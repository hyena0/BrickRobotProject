"""6.1 expert demos: point_at with the gripper-tip TCP.

Rest -> via (x, y, 120) -> point (x, y, 60) -> hold 1s -> rest, 10 Hz,
50 steps. Position IK with J6 pinned (addendum 3: J6 fixed), seeds from
the nearest 5C mat point. Every step is checked with the shared
console range function; success = hold tip within 30 mm horizontal of
the block center, tip z in [40, 90] above the pedestal.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess

from src.console.control import check_pose_against_ranges
from src.kin.ik import ik
from src.kin.model import load_model

from src.vla.scene import (JOINTS, REST_J6, adopted_ranges,
                           jitter_cam, load_camera, nearest_mat_seed,
                           render_frame, sample_scene, tip_doc, tip_world,
                           write_png)

HOLD = {"via_z": 120.0, "point_z": 60.0, "hz": 10}
N_DEMOS = 100


def split_legs(D1, D2, total=30):
    """Split approach steps between rest->via and via->point.

    Proportional to joint travel (equalizes per-step speed), at least
    3 steps per leg, sums exactly to ``total``.
    """
    if D1 + D2 <= 0:
        return total - 3, 3
    n1 = min(total - 3, max(3, round(total * D1 / (D1 + D2))))
    return n1, total - n1


def ik_accepted(sol) -> bool:
    """Usable IK solution: exact TCP *and* the solver's success flag.

    The solver returns best-effort q_deg even when coupled limits fail
    (success False); those must never become demos.
    """
    return (sol is not None and sol.get("q_deg") is not None
            and bool(sol.get("success", False))
            and sol.get("pos_err_mm", 1e9) <= 5.0)


def solve_xy(x, y, z, rest_q, seed_q, tdoc, lim_pin):
    tag = abs(hash((round(x, 1), round(y, 1), round(z, 1)))) % 10000
    # Stage 1: local solve from the continuity seed (nearest branch).
    sol = ik([x, y, z], None, tdoc, q_init_deg=seed_q,
             position_only=True, limits_doc=lim_pin, seed=tag,
             max_restarts=0)
    if sol["q_deg"] is not None and sol["pos_err_mm"] <= 5.0:
        sol["seed_used"] = "continuity"
        return sol
    # Stage 2: global multi-start (any branch; may need longer legs).
    sol = ik([x, y, z], None, tdoc, q_init_deg=seed_q,
             position_only=True, limits_doc=lim_pin, seed=tag,
             max_restarts=48)
    sol["seed_used"] = "global"
    return sol


def interp(a, b, n):
    return [[ai + (bi - ai) * (k + 1) / n for ai, bi in zip(a, b)]
            for k in range(n)]


def run_demo(idx, scene, rest_q, tdoc, seed_q):
    """J6 pin: REST_J6 when it admits a valid solution, else the
    nearest mat seed's J6 (a proven 5C neighborhood for that slot).

    One global J6 pin admits no coupled-valid solution on some slots
    (verified: pin 60 fails (-120,150); pin -20 fails (-120,230)).
    The pin stays fixed through the episode (addendum 3).
    """
    from src.vla.scene import limits_doc
    from src.vla.scene import REST_J6 as _R6
    tb = next(b for b in scene["blocks"]
              if b["colour"] == scene["target"])

    def attempt(pin, via_seed):
        lim = limits_doc(pin_j6=pin)
        via = solve_xy(tb["x"], tb["y"], HOLD["via_z"], rest_q,
                       via_seed, tdoc, lim)
        qv = ([float(v) for v in via["q_deg"]]
              if via["q_deg"] is not None else via_seed)
        pt = solve_xy(tb["x"], tb["y"], HOLD["point_z"], rest_q, qv,
                      tdoc, lim)
        return via, pt

    via, pt = attempt(_R6, rest_q)
    pin_used = _R6
    if not (ik_accepted(via) and ik_accepted(pt)):
        via, pt = attempt(seed_q[5], rest_q)
        pin_used = seed_q[5]
    rec = {"target_block": tb, "via_err_mm": via["pos_err_mm"],
           "point_err_mm": pt["pos_err_mm"],
           "via_seed": via.get("seed_used"), "pt_seed": pt.get("seed_used"),
           "j6_pin": pin_used,
           "via_ok": ik_accepted(via), "point_ok": ik_accepted(pt)}
    if not (rec["via_ok"] and rec["point_ok"]):
        rec["status"] = "ik-failed"
        return rec, None
    q_via = [float(v) for v in via["q_deg"]]
    q_pt = [float(v) for v in pt["q_deg"]]
    # Episode: rest 5 (0.5 s), approach 30 (3 s), hold 15 (1.5 s).
    # No fast return leg; the episode ends holding the point.
    def dist(a, b):
        return max(abs(x - y) for x, y in zip(a, b))
    n1, n2 = split_legs(dist(rest_q, q_via), dist(q_via, q_pt))
    traj = ([rest_q] * 5 + interp(rest_q, q_via, n1)
            + interp(q_via, q_pt, n2) + [q_pt] * 15)
    assert len(traj) == 50
    rec["phases"] = {"lead": 5, "n1": n1, "n2": n2, "hold": 15}
    hold = traj[35:50]
    max_step = max(max(abs(b - a) for a, b in zip(t0, t1))
                   for t0, t1 in zip(traj[:-1], traj[1:]))
    rec["max_step_deg"] = round(max_step, 3)
    adopted = adopted_ranges()
    viol = 0
    for q in traj:
        qd = {j: v for j, v in zip(JOINTS, q)}
        if check_pose_against_ranges(qd, adopted):
            viol += 1
    rec["range_violations"] = viol
    errs, zs = [], []
    for q in hold:
        tip, _ = tip_world(q, tdoc)
        errs.append(math.hypot(tip[0] - tb["x"], tip[1] - tb["y"]))
        zs.append(float(tip[2]))
    rec["hold_h_err_mm"] = round(sum(errs) / len(errs), 2)
    rec["hold_max_h_err_mm"] = round(max(errs), 2)
    rec["hold_z_min"] = round(min(zs), 2)
    rec["hold_z_max"] = round(max(zs), 2)
    # Gate: task success + zero range violations + every step <=5 deg
    # (no fast return leg anymore; the episode ends holding).
    rec["smooth_5deg"] = bool(max_step <= 5.0)
    ok = (rec["hold_max_h_err_mm"] <= 30.0
          and rec["hold_z_min"] >= 40.0 and rec["hold_z_max"] <= 90.0
          and viol == 0 and rec["smooth_5deg"])
    rec["status"] = "pass" if ok else "task-failed"
    return rec, traj


def run_61(out="runs/vla/6.1", seed=61, n=N_DEMOS, render_n=3):
    os.makedirs(out, exist_ok=True)
    rng = random.Random(seed)
    cam = load_camera()
    doc = load_model("models/robot_inferred.json")
    tdoc = tip_doc(doc)
    rep60 = json.load(open("runs/vla/6.0/report.json", encoding="utf-8"))
    rest_q = [float(v) for v in rep60["rest_q_deg"]]
    assert abs(rest_q[5] - REST_J6) < 1e-9, "rest J6 must equal REST_J6"
    import src.vla.scene as S
    mat_pts = S.load_slot_seeds()
    mat_seeds = [p["seed"] for p in mat_pts]
    order = ["red", "blue", "green", "yellow"]
    rows = []
    for i in range(n):
        scene = sample_scene(rng)
        scene["target"] = order[i % 4]
        if scene["target"] not in [b["colour"] for b in scene["blocks"]]:
            scene["blocks"][rng.randrange(len(scene["blocks"]))][
                "colour"] = scene["target"]
        tb = next(b for b in scene["blocks"]
                  if b["colour"] == scene["target"])
        seed_q = nearest_mat_seed(tb["x"], tb["y"], mat_seeds, mat_pts)
        rec, traj = run_demo(i, scene, rest_q, tdoc, seed_q)
        rec.update({"id": f"demo_{i:03d}", "scene": scene,
                    "target": scene["target"]})
        rows.append((rec, traj, scene))
    for i, (rec, traj, scene) in enumerate(rows[:render_n]):
        if traj is None:
            continue
        ddir = os.path.join(out, f"demo_{i:02d}_frames")
        os.makedirs(ddir, exist_ok=True)
        cami = jitter_cam(cam, random.Random(seed + 1000 + i))
        for k, q in enumerate(traj):
            img = render_frame(scene, q, doc, cami, None)
            write_png(os.path.join(ddir, f"f{k:02d}.png"), img)
        mp4 = os.path.join(out, f"demo_{i:02d}.mp4")
        subprocess.run(["ffmpeg", "-y", "-v", "error", "-framerate", "10",
                        "-i", os.path.join(ddir, "f%02d.png"),
                        "-pix_fmt", "yuv420p", mp4], check=True)
        rec["video"] = mp4
    ok = [r for r, _, _ in rows if r["status"] == "pass"]
    by_colour = {}
    for c in order:
        sub = [r for r, _, _ in rows if r["target"] == c]
        by_colour[c] = (f"{sum(1 for r in sub if r['status'] == 'pass')}"
                        f"/{len(sub)}")
    share = {c: round(100 * sum(1 for _, _, s in rows if s["target"] == c)
                      / n, 1) for c in order}
    viol = sum(r.get("range_violations", 0) for r, _, _ in rows)
    report = {
        "seed": seed, "n": n, "rest_q_deg": [round(v, 2) for v in rest_q],
        "gate": {"success": f"{len(ok)}/{n}",
                 "pass": len(ok) / n >= 0.95 if n else False,
                 "colour_share_pct": share,
                 "colour_pass": by_colour,
                 "range_violations": viol},
        "demos": [{k: r[k] for k in r if k != "scene"} for r, _, _ in rows],
    }
    json.dump(report, open(os.path.join(out, "report.json"), "w"), indent=1)
    md = [f"# 6.1 expert demos ({len(ok)}/{n} pass)",
          "",
          f"Target share: {share}. Range violations: {viol}.",
          "",
          "First-frame gate (6.0 rest pose) reused for all demos.",
          ""]
    with open(os.path.join(out, "report.md"), "w",
              encoding="utf-8") as fh:
        fh.write("\n".join(md))
    print(f"6.1: {len(ok)}/{n} pass, violations {viol}, share {share}")
    print("PASS" if report["gate"]["pass"] and viol == 0 else "FAIL",
          "->", out)
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description="6.1 expert demos")
    ap.add_argument("--out", default="runs/vla/6.1")
    ap.add_argument("--seed", type=int, default=61)
    ap.add_argument("--n", type=int, default=N_DEMOS)
    args = ap.parse_args(argv)
    run_61(args.out, args.seed, args.n)


if __name__ == "__main__":
    main()
