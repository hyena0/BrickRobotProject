"""Convert 6.1-style demos to a LeRobot v3 dataset (verified: lerobot 0.6.1).

Regenerates episodes (own seed, same gates as 6.1), renders every frame,
and writes data/chunk-*.parquet + videos + meta via LeRobotDataset.
Run with an env that has lerobot (project .venv does not)::

    /tmp/lerobot-check/bin/python src/vla/to_lerobot.py --n 20
    # or on Modal (lerobot[smolvla]==0.6.1 in the image)

Trial default: 20 episodes (addendum 6.2).
"""

from __future__ import annotations

import argparse
import os
import random
import sys

sys.path.insert(0, ".")

import numpy as np

TASK_TEMPLATES = ("Point to the {colour} block.",
                  "Where is the {colour} one?",
                  "Show me the {colour} brick.")

CAM_KEY = "observation.images.top"


def summarize(recs):
    """B.5 pre-check table content from per-episode recs (pure)."""
    import statistics
    ok = [r for r in recs if r.get("status") == "pass"]
    n = len(recs)
    viol = sum(r.get("range_violations", 0) for r in recs)
    ms = [r.get("max_step_deg", 0) for r in ok]
    herr = [r.get("hold_max_h_err_mm", 0) for r in ok]
    share = {}
    for r in ok:
        key = r.get("target", "?")
        share[key] = share.get(key, 0) + 1
    return {
        "episodes": n, "pass": f"{len(ok)}/{n}",
        "pass_rate": round(len(ok) / n, 4) if n else 0.0,
        "range_violations": viol,
        "colour_share": {k: round(100 * v / len(ok), 1) if ok else 0.0
                         for k, v in sorted(share.items())},
        "max_step_med": round(statistics.median(ms), 2) if ms else None,
        "max_step_max": round(max(ms), 2) if ms else None,
        "hold_herr_max_mm": round(max(herr), 2) if herr else None,
    }


def build_episode(i, rng, rest_q, doc, tdoc, cam, mat_seeds, mat_pts):
    """One gated episode: (ep, rec, reason)."""
    import src.vla.demo as D
    import src.vla.scene as S
    scene = S.sample_scene(rng)
    order = ["red", "blue", "green", "yellow"]
    scene["target"] = order[i % 4]
    if scene["target"] not in [b["colour"] for b in scene["blocks"]]:
        scene["blocks"][rng.randrange(len(scene["blocks"]))][
            "colour"] = scene["target"]
    tb = next(b for b in scene["blocks"]
              if b["colour"] == scene["target"])
    seed_q = S.nearest_mat_seed(tb["x"], tb["y"], mat_seeds, mat_pts)
    rec, traj = D.run_demo(i, scene, rest_q, tdoc, seed_q)
    if traj is None or rec["status"] != "pass":
        return None, rec, rec.get("status", "no-traj")
    cami = S.jitter_cam(cam, random.Random(10000 + i))
    rec["target"] = scene["target"]
    gate = S.frame_checks(scene, traj[0], doc, cami)
    if not (gate["blocks_visible"] and gate["arm_out"]):
        rec = dict(rec, status="first-frame-failed", gate=gate)
        return None, rec, "first-frame-failed"
    return {"scene": scene, "traj": traj, "cam": cami,
            "task": TASK_TEMPLATES[i % 3].format(colour=scene["target"])}, \
        rec, None


def convert(out, n=20, seed=62, use_videos=True):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import src.vla.scene as S
    from src.kin.model import load_model

    rng = random.Random(seed)
    cam = S.load_camera()
    doc = load_model("models/robot_inferred.json")
    tdoc = S.tip_doc(doc)
    rep60 = __import__("json").load(
        open("runs/vla/6.0/report.json", encoding="utf-8"))
    rest_q = [float(v) for v in rep60["rest_q_deg"]]
    mat_pts = S.load_slot_seeds()
    mat_seeds = [p["seed"] for p in mat_pts]

    features = {
        CAM_KEY: {"dtype": "video", "shape": (256, 256, 3),
                  "names": ["height", "width", "channel"]},
        "observation.state": {"dtype": "float32", "shape": (6,),
                              "names": None},
        "action": {"dtype": "float32", "shape": (6,), "names": None},
    }
    ds = LeRobotDataset.create(
        repo_id="local/brickrobot-point", fps=10, features=features,
        root=out, robot_type="brickrobot", use_videos=use_videos)
    made, skipped, recs = 0, [], []
    i = 0
    while made < n:
        ep, rec, reason = build_episode(i, rng, rest_q, doc, tdoc, cam,
                                       mat_seeds, mat_pts)
        i += 1
        recs.append(rec if rec is not None else {"status": reason,
                                                "id": f"src_{i - 1}"})
        if ep is None:
            skipped.append((i - 1, reason))
            if len(skipped) > n:
                raise RuntimeError(f"too many skipped: {skipped}")
            continue
        traj = ep["traj"]
        for t in range(50):
            img = S.render_frame(ep["scene"], traj[t], doc, ep["cam"],
                                 None)
            q = np.array(traj[t], dtype=np.float32)
            qn = np.array(traj[min(t + 1, 49)], dtype=np.float32)
            ds.add_frame({CAM_KEY: img, "observation.state": q,
                          "action": qn, "task": ep["task"]})
        ds.save_episode()
        made += 1
        print(f"episode {made}/{n} ok (src {i - 1})", flush=True)
    ds.finalize()
    report = summarize(recs)
    report["out"] = out
    report["skipped"] = skipped
    __import__("json").dump(report, open(
        os.path.join(out, "report.json"), "w"), indent=1)
    print(f"wrote {out}: {report['pass']} pass, "
          f"violations {report['range_violations']}")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="6.1 demos -> LeRobot")
    ap.add_argument("--out", default="runs/vla/lerobot-trial")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--seed", type=int, default=62)
    ap.add_argument("--no-video", action="store_true")
    args = ap.parse_args(argv)
    convert(args.out, args.n, args.seed, use_videos=not args.no_video)


if __name__ == "__main__":
    main()
