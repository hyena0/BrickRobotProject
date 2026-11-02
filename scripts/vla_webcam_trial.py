"""Stage 9 B.4 webcam trial: real photo -> VLA hold pose -> chosen slot.

Usage:
  .venv/bin/python scripts/vla_webcam_trial.py --target red --out runs/stage9/webcam/trial_01.png

Captures one 256x256 frame, runs the local checkpoint, FKs the final
predicted joints to an (x, y) point and picks the nearest S1..S6 slot.
Appends the trial record to runs/stage9/webcam/trials.jsonl.
"""
import argparse
import datetime
import json
import subprocess
import sys

sys.path.insert(0, ".")

from PIL import Image

from src.kin.fk import fk_model
from src.kin.model import load_model
from src.console.control import tip_doc
from src.vla import twin as T


def capture(path):
    # Full-resolution still, then a CENTER SQUARE crop resized to
    # 256: never a non-uniform -s scale, which would squash the
    # 16:9 frame into a square and distort every block.
    raw = path + ".full.png"
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-f", "avfoundation",
                    "-framerate", "30", "-i", "0",
                    "-frames:v", "1", raw],
                   check=True)
    from PIL import Image
    img = Image.open(raw).convert("RGB")
    side = min(img.size)
    left = (img.size[0] - side) // 2
    top = (img.size[1] - side) // 2
    img.crop((left, top, left + side, top + side)).resize(
        (256, 256)).save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--slots", default=None,
                    help="JSON like '{\"S1\": \"red\"}' (for records only)")
    args = ap.parse_args()

    capture(args.out)
    img = Image.open(args.out).convert("RGB")
    import numpy as np
    arr = np.array(img)
    assert arr.shape == (256, 256, 3), arr.shape

    from src.vla import local_infer as LI
    doc = load_model("models/robot_inferred.json")
    rest = list(T.REST_Q)
    out = LI.predict_chunk("runs/vla/ckpt-last", arr, rest,
                           "Point to the %s block." % args.target)
    assert len(out["actions"]) == 50, out.get("n_steps")
    q = [float(v) for v in out["actions"][-1]]
    tcp = fk_model(q, tip_doc(doc))["pos_mm"]

    # SLOT_IDS maps (x, y) -> sid
    best, best_d = None, 1e18
    for (sx, sy), sid in \
            __import__("src.vla.scene", fromlist=["SLOT_IDS"]).SLOT_IDS.items():
        d = (tcp[0] - sx) ** 2 + (tcp[1] - sy) ** 2
        if d < best_d:
            best, best_d = sid, d
    import math
    rec = {"time": datetime.datetime.now().isoformat(timespec="seconds"),
           "target": args.target, "slots": args.slots,
           "chosen": best, "dist_mm": round(math.sqrt(best_d), 1),
           "tcp_mm": [round(v, 1) for v in tcp],
           "infer_s": out["infer_s"], "photo": args.out,
           "state_source": out.get("state_source"),
           "n_steps": out.get("n_steps")}
    print(json.dumps(rec, ensure_ascii=False))
    with open("runs/stage9/webcam/trials.jsonl", "a") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
