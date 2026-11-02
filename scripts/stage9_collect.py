"""Stage 9B real-photo collector: one batch (arrangement) -> photo + IK answers.

Usage:
  .venv/bin/python scripts/stage9_collect.py \
      --batch '{"S1": "yellow", "S2": "red", "S5": "green"}' [--split auto|train|test]

What it does, per the 9B spec:
- Captures one full-res webcam still, crops the tray (paper) area as the
  largest square inside the detected paper bbox (falls back to the
  honest center-square crop), resizes to 256x256. The camera faces the
  paper, so the robot can never enter the crop.
- Writes one instruction sample per distinct colour present (up to 4):
  task string (rotated templates, same source as to_lerobot) + the
  50-step IK answer trajectory from demo.run_demo on the matching slot.
- Split rule: --split auto assigns every 4th batch to test
  (batches 4,8,12,16,20 -> 5 test of 20), so one batch never lands on
  both sides. Test batches are never used for training.

Layout on disk (runs/ only, never committed):
  runs/stage9/real/batch_NN.full.png   full-res still
  runs/stage9/real/batch_NN.png        256x256 tray crop (the sample image)
  runs/stage9/real/{train,test}/batch_NN_traj.json
  runs/stage9/real/manifest.jsonl      one record per batch
"""

import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, ".")
import numpy as np
from PIL import Image

REAL_ROOT = os.path.join("runs", "stage9", "real")
IMG_SIZE = 256
COLOURS = ("red", "blue", "green", "yellow")


def capture_full(path):
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-f", "avfoundation",
                    "-framerate", "30", "-i", "0",
                    "-frames:v", "1", path], check=True)


def tray_crop(full_path, out_path):
    """Largest square inside the paper bbox -> 256x256.

    Paper (~150 gray) separates cleanly from the desk (~65 gray);
    threshold 110 finds it. Falls back to the center square when the
    paper is not confidently detected. Returns the crop box.
    """
    img = Image.open(full_path).convert("RGB")
    a = np.asarray(img).astype(np.int32)
    W, H = img.size
    paper = a.mean(axis=2) > 110
    ys, xs = np.where(paper)
    if len(xs) > 0.5 * W * H:
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        bw, bh = x1 - x0, y1 - y0
        side = min(bw, bh)
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        sx = min(max(cx - side // 2, 0), W - side)
        sy = min(max(cy - side // 2, 0), H - side)
    else:  # fallback: honest center square
        side = min(W, H)
        sx, sy = (W - side) // 2, (H - side) // 2
    box = (sx, sy, sx + side, sy + side)
    img.crop(box).resize((IMG_SIZE, IMG_SIZE)).save(out_path)
    return {"box": list(box), "full_size": [W, H]}


def assign_split(no):
    """Every 4th batch goes to test; one batch never lands on both sides."""
    return "test" if no % 4 == 0 else "train"


#: Splits that must never enter training. "test" is the original
#: held-out set (batches 4,8,12,16,20); "holdout" is for extra test
#: batches shot later (different lighting etc.). Builders select only
#: split == "train"; eval tools select test/holdout explicitly.
NEVER_TRAIN = ("test", "holdout")


def next_batch_no():
    n = 0
    man = os.path.join(REAL_ROOT, "manifest.jsonl")
    if os.path.exists(man):
        with open(man, encoding="utf-8") as fh:
            for line in fh:
                if line.strip():
                    n += 1
    return n + 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="9B real batch collector")
    ap.add_argument("--batch", required=True,
                    help="JSON slot->colour, e.g. '{\"S1\": \"yellow\"}'")
    ap.add_argument("--split", default="auto",
                    choices=("auto", "train", "test", "holdout"),
                    help="holdout = extra test batches, never training")
    ap.add_argument("--outdir", default=REAL_ROOT)
    args = ap.parse_args(argv)

    batch = json.loads(args.batch)
    assert set(batch) <= {"S1", "S2", "S3", "S4", "S5", "S6"}, batch
    assert set(batch.values()) <= set(COLOURS), batch.values()
    assert len(batch) >= 1, "place at least one block"

    import src.vla.demo as D
    import src.vla.scene as S
    from src.kin.model import load_model
    from src.vla.to_lerobot import TASK_TEMPLATES

    no = next_batch_no()
    split = args.split
    if split == "auto":
        split = assign_split(no)

    os.makedirs(args.outdir, exist_ok=True)
    os.makedirs(os.path.join(args.outdir, split), exist_ok=True)
    full = os.path.join(args.outdir, f"batch_{no:02d}.full.png")
    photo = os.path.join(args.outdir, f"batch_{no:02d}.png")
    capture_full(full)
    crop = tray_crop(full, photo)

    doc = load_model("models/robot_inferred.json")
    tdoc = S.tip_doc(doc)
    rep60 = json.load(open("runs/vla/6.0/report.json", encoding="utf-8"))
    rest_q = [float(v) for v in rep60["rest_q_deg"]]
    mat_pts = S.load_slot_seeds()
    mat_seeds = [p["seed"] for p in mat_pts]
    xy_of = {v: k for k, v in S.SLOT_IDS.items()}

    samples, skipped = [], []
    present = sorted(set(batch.values()), key=list(COLOURS).index)
    for si, colour in enumerate(present[:4]):
        scene = {"blocks": [], "target": colour}
        for sid, col in batch.items():
            x, y = xy_of[sid]
            scene["blocks"].append({"colour": col, "x": float(x),
                                   "y": float(y), "yaw_deg": 0.0,
                                   "slot": sid})
        tb = next(b for b in scene["blocks"]
                  if b["colour"] == colour)
        seed_q = S.nearest_mat_seed(tb["x"], tb["y"], mat_seeds, mat_pts)
        rec, traj = D.run_demo(f"real{no}{colour}", scene, rest_q,
                              tdoc, seed_q)
        if traj is None or rec.get("status") != "pass":
            skipped.append({"colour": colour,
                            "reason": (rec or {}).get("status", "no-traj")})
            continue
        samples.append({
            "colour": colour,
            "task": TASK_TEMPLATES[si % 3].format(colour=colour),
            "slot": tb["slot"],
            "traj": [[round(v, 3) for v in q] for q in traj],
            "hold_max_h_err_mm": rec["hold_max_h_err_mm"],
        })
    if not samples:
        print(json.dumps({"ok": False, "batch": no,
                          "reason": "all-ik-failed",
                          "skipped": skipped}, ensure_ascii=False))
        return 1
    tpath = os.path.join(args.outdir, split,
                         f"batch_{no:02d}_traj.json")
    json.dump({"batch": no, "split": split, "slots": batch,
               "samples": samples, "skipped": skipped},
              open(tpath, "w", encoding="utf-8"), indent=1)
    rec = {"batch": no, "split": split, "slots": batch,
           "photo": os.path.relpath(photo, args.outdir),
           "full": os.path.relpath(full, args.outdir),
           "crop": crop, "traj_file": os.path.relpath(tpath, args.outdir),
           "colours": [s["colour"] for s in samples],
           "skipped": skipped}
    with open(os.path.join(args.outdir, "manifest.jsonl"), "a",
              encoding="utf-8") as fh:
        fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
    print(json.dumps({"ok": True, **rec}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
