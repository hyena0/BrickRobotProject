"""Stage 9B dataset builder: virtual recipes + LeRobot conversion.

Two stages share one recipe file so local and Modal runs agree exactly:

1. gen_recipes (local, no lerobot needed): 900 gated virtual episodes
   (scene + 50-step IK traj + jittered camera + task string + seed),
   plus report.json, 9 first-frames, grid_3x3.png and a real-vs-virtual
   side-by-side. Recipes are deterministic in (seed, inputs).

2. convert9b (needs lerobot: runs on Modal via trial_modal --which build9b):
   renders every frame with scene.render9b (measured palette + DR) and
   appends the real train batches: the scene is static while pointing, so
   all 50 frames repeat the real tray photo (physically faithful, and it
   matches the first-frame eval protocol). Each real episode is repeated
   REAL_REPEAT times with fresh photometric DR per repeat, raising the
   real share of the mix. The same batch never appears in train and test.

Real-image rationale (first-frame-only real data): at test time the
policy sees one real photo as its first observation; training on
real-photo episodes whose actions come from IK on the matched slots
teaches appearance->action without a fake sim/real temporal seam.
"""

import argparse
import json
import os
import random
import sys

sys.path.insert(0, ".")

CAM_KEY = "observation.images.top"
REAL_REPEAT = 20
#: 1h budget sizing (measured 2026-11-02, local Mac): 20 eps = 26 s
#: (render 18 + add/save 8, h264 ultrafast), i.e. ~1.3 s/ep.
#: 400 virtual + 300 real repeats ~= 12 min end to end.
N_VIRTUAL = 400


def stamp_outdir(base="runs/stage9/lerobot9b"):
    """Fresh timestamped output dir (no intermediate folders)."""
    import datetime
    return base + "-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")


def fast_encoder():
    """CPU-friendly encoder: h264 ultrafast crf23 instead of SVT-AV1.

    The stock rgb_encoder_defaults (libsvtav1) is what timed out the
    Modal CPU build at 3600 s. Training-side decoding (torchcodec/pyav)
    reads h264 mp4s the same way; only bytes-on-disk change.
    """
    from lerobot.configs.video import RGBEncoderConfig
    return RGBEncoderConfig(vcodec="h264", preset="ultrafast", crf="23")


def plan_recipes(n=N_VIRTUAL, seed=9000):
    """Gated virtual episodes, in memory (no IO: the dataset dir must be
    created fresh by LeRobotDataset.create afterwards)."""
    import src.vla.scene as S
    from src.kin.model import load_model
    from src.vla.to_lerobot import build_episode

    rng = random.Random(seed)
    cam = S.load_camera()
    doc = load_model("models/robot_inferred.json")
    tdoc = S.tip_doc(doc)
    rep60 = json.load(open("runs/vla/6.0/report.json", encoding="utf-8"))
    rest_q = [float(v) for v in rep60["rest_q_deg"]]
    mat_pts = S.load_slot_seeds()
    mat_seeds = [p["seed"] for p in mat_pts]

    recs, made, skipped = [], 0, []
    i = 0
    while made < n:
        ep, rec, reason = build_episode(i, rng, rest_q, doc, tdoc, cam,
                                       mat_seeds, mat_pts)
        base = dict(rec) if rec is not None else {"status": reason,
                                                 "id": f"src_{i}"}
        base["src"] = i
        recs.append(base)
        if ep is None:
            skipped.append((i, reason))
            if len(skipped) > n:
                raise RuntimeError(f"too many skipped: {skipped[:5]}")
            i += 1
            continue
        # 9B: fresh camera-jitter stream (20000+i, not the 6.x 10000+i).
        cami = S.jitter_cam(cam, random.Random(20000 + i))
        recs[-1].update({
            "status": "pass", "seed_ep": 900000 + i,
            "scene": ep["scene"], "traj": ep["traj"],
            "cam": cami, "task": ep["task"],
        })
        made += 1
        i += 1
    return recs, skipped


def write_first_frames(outdir, recs, render_n=9):
    """First-frame pngs + 3x3 grid (render9b look the policy trains on)."""
    import numpy as np
    from PIL import Image
    import src.vla.scene as S
    from src.kin.model import load_model
    S.PALETTE9B = dict(S.PALETTE9B_MEASURED)
    S.PAPER9B = S.PAPER9B_MEASURED
    doc = load_model("models/robot_inferred.json")
    made_r = [r for r in recs if r.get("status") == "pass"][:render_n]
    for k, r in enumerate(made_r):
        img = S.render9b(r["scene"], r["traj"][0], doc, r["cam"],
                         r["seed_ep"] * 1000)
        S.write_png(os.path.join(outdir, f"frame_{k:02d}.png"), img)
        r["frame_png"] = f"frame_{k:02d}.png"
    tiles = [np.asarray(Image.open(
        os.path.join(outdir, f"frame_{k:02d}.png")).convert("RGB"))
        for k in range(len(made_r))]
    grid = np.zeros((3 * 256, 3 * 256, 3), np.uint8)
    for k, t in enumerate(tiles):
        grid[(k // 3) * 256:(k // 3 + 1) * 256,
             (k % 3) * 256:(k % 3 + 1) * 256] = t
    Image.fromarray(grid).save(os.path.join(outdir, "grid_3x3.png"))


def gen_recipes(outdir, n=N_VIRTUAL, seed=9000, render_n=9):
    """Legacy entry: plan + write recipes/report/frames (not for use as
    a LeRobot root; the dataset dir must stay fresh for create())."""
    from src.vla.to_lerobot import summarize
    os.makedirs(outdir, exist_ok=True)
    recs, skipped = plan_recipes(n, seed)
    report = summarize([r for r in recs if r.get("status") == "pass"])
    report["out"] = outdir
    report["skipped"] = skipped
    report["cam_jitter_stream"] = "20000+i"
    write_first_frames(outdir, recs, render_n)
    json.dump(recs, open(os.path.join(outdir, "recipes.json"), "w"))
    json.dump(report, open(os.path.join(outdir, "report.json"), "w"),
              indent=1)
    print(f"9B recipes: {report['pass']} pass, "
          f"violations {report['range_violations']} -> {outdir}")
    return outdir


def side_by_side(real_png, out_png, slots, rest_q=None):
    """Real tray photo next to a render9b of the same slot layout."""
    import numpy as np
    from PIL import Image
    import src.vla.scene as S
    from src.kin.model import load_model
    doc = load_model("models/robot_inferred.json")
    if rest_q is None:
        rep60 = json.load(open("runs/vla/6.0/report.json",
                              encoding="utf-8"))
        rest_q = [float(v) for v in rep60["rest_q_deg"]]
    xy_of = {v: k for k, v in S.SLOT_IDS.items()}
    scene = {"blocks": [{"colour": c, "x": float(xy_of[s][0]),
                         "y": float(xy_of[s][1]), "yaw_deg": 0.0,
                         "slot": s} for s, c in slots.items()],
             "target": next(iter(slots.values()))}
    S.PALETTE9B = dict(S.PALETTE9B_MEASURED)
    S.PAPER9B = S.PAPER9B_MEASURED
    virt = S.render9b(scene, rest_q, doc, S.load_camera(), 424242)
    real = np.asarray(Image.open(real_png).convert("RGB"))
    assert real.shape == (256, 256, 3), real.shape
    combo = np.zeros((256, 512, 3), np.uint8)
    combo[:, :256] = real
    combo[:, 256:] = virt
    Image.fromarray(combo).save(out_png)
    print("wrote", out_png)


def load_real_train(real_root):
    """Train batches from the collector manifest (test excluded)."""
    import numpy as np
    from PIL import Image
    man = os.path.join(real_root, "manifest.jsonl")
    eps = []
    if not os.path.exists(man):
        return eps, []
    with open(man, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rec = json.loads(line)
                # Train only: "test" and "holdout" batches never enter
                # the dataset, no matter how many are collected later.
                if rec.get("split") == "train":
                    eps.append(rec)
    out = []
    for rec in eps:
        photo = np.asarray(Image.open(
            os.path.join(real_root, rec["photo"])).convert("RGB"))
        tj = json.load(open(os.path.join(real_root, rec["traj_file"]),
                            encoding="utf-8"))
        for s in tj["samples"]:
            out.append({"task": s["task"], "photo": photo,
                        "traj": s["traj"], "batch": rec["batch"]})
    return out, [r["batch"] for r in eps]


def convert9b(out, recipes_path, real_root, repeat=REAL_REPEAT,
              use_videos=True, encoder=None, progress_every=25):
    """Recipes + real train batches -> LeRobot dataset (needs lerobot)."""
    import time
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import numpy as np
    import src.vla.scene as S
    from src.kin.model import load_model

    if isinstance(recipes_path, str):
        recs = json.load(open(recipes_path, encoding="utf-8"))
    else:
        recs = recipes_path
    valid = [r for r in recs if r.get("status") == "pass"
             and "traj" in r]
    real_eps, real_batches = load_real_train(real_root)
    doc = load_model("models/robot_inferred.json")
    S.PALETTE9B = dict(S.PALETTE9B_MEASURED)
    S.PAPER9B = S.PAPER9B_MEASURED

    features = {
        CAM_KEY: {"dtype": "video", "shape": (256, 256, 3),
                  "names": ["height", "width", "channel"]},
        "observation.state": {"dtype": "float32", "shape": (6,),
                              "names": None},
        "action": {"dtype": "float32", "shape": (6,), "names": None},
    }
    ds = LeRobotDataset.create(
        repo_id="local/brickrobot-point9b", fps=10, features=features,
        root=out, robot_type="brickrobot", use_videos=use_videos,
        rgb_encoder=encoder or fast_encoder())
    total = len(valid) + repeat * len(real_eps)
    done, t0 = 0, time.time()

    def tick():
        if progress_every and done % progress_every == 0:
            el = time.time() - t0
            eta = el / done * (total - done) if done else 0
            print(f"convert9b {done}/{total} eps "
                  f"elapsed {el:.0f}s ETA {eta:.0f}s", flush=True)

    n_v = n_r = 0
    for r in valid:
        cam = r["cam"]
        for t in range(50):
            img = S.render9b(r["scene"], r["traj"][t], doc, cam,
                             r["seed_ep"] * 1000 + t)
            q = np.array(r["traj"][t], dtype=np.float32)
            qn = np.array(r["traj"][min(t + 1, 49)], dtype=np.float32)
            ds.add_frame({CAM_KEY: img, "observation.state": q,
                          "action": qn, "task": r["task"]})
        ds.save_episode()
        n_v += 1
        done += 1
        tick()
    for rep in range(repeat):
        for e in real_eps:
            for t in range(50):
                img = S.render9b(e["photo"], None, None, None,
                                 77000000 + e["batch"] * 1000 + rep,
                                 photo_dr=True)
                q = np.array(e["traj"][t], dtype=np.float32)
                qn = np.array(e["traj"][min(t + 1, 49)], dtype=np.float32)
                ds.add_frame({CAM_KEY: img, "observation.state": q,
                              "action": qn, "task": e["task"]})
            ds.save_episode()
            n_r += 1
            done += 1
            tick()
    ds.finalize()
    report = {"virtual_episodes": n_v, "real_episodes": n_r,
              "real_batches": real_batches, "repeat": repeat,
              "total": n_v + n_r, "out": out}
    json.dump(report, open(os.path.join(out, "report9b.json"), "w"),
              indent=1)
    print(f"wrote {out}: {n_v} virtual + {n_r} real "
          f"({len(real_batches)} batches x{repeat})")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="9B recipes + conversion")
    ap.add_argument("--outdir", default=None,
                    help="default: fresh timestamped runs/stage9/lerobot9b-*")
    ap.add_argument("--n", type=int, default=N_VIRTUAL)
    ap.add_argument("--seed", type=int, default=9000)
    ap.add_argument("--real-root", default="runs/stage9/real")
    ap.add_argument("--repeat", type=int, default=REAL_REPEAT)
    ap.add_argument("--recipes-only", action="store_true")
    args = ap.parse_args(argv)
    outdir = args.outdir or stamp_outdir()
    if args.recipes_only:
        gen_recipes(outdir, args.n, args.seed)
        print("9B done ->", outdir)
        return 0
    # Single-folder flow (no intermediate dirs): plan in memory, create
    # the dataset dir fresh, then write recipes/report/frames next to it.
    from src.vla.to_lerobot import summarize
    recs, skipped = plan_recipes(args.n, args.seed)
    convert9b(outdir, recs, args.real_root, args.repeat)
    report = summarize([r for r in recs if r.get("status") == "pass"])
    report["out"] = outdir
    report["skipped"] = skipped
    report["cam_jitter_stream"] = "20000+i"
    write_first_frames(outdir, recs)
    json.dump(recs, open(os.path.join(outdir, "recipes.json"), "w"))
    json.dump(report, open(os.path.join(outdir, "report.json"), "w"),
              indent=1)
    print(f"9B recipes: {report['pass']} pass, "
          f"violations {report['range_violations']}")
    print("9B done ->", outdir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
