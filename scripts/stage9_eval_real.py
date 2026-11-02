"""Stage 9B real-test eval: before/after ckpt on the 5 held-out batches.

Usage:
  .venv/bin/python scripts/stage9_eval_real.py \
      --a runs/vla/ckpt-last --b runs/vla/ckpt-ft9b-3000

For every sample of every test-split batch: run the checkpoint's chunk
open-loop from the real tray photo (rest pose start, same protocol as
the webcam trials), FK the final joints, score against the batch's slot
map (success: tip within 30mm of the target slot, z 40-90; wrong_block:
within 30mm of another occupied slot). Random baseline picks uniformly
among the batch's occupied slots (analytic 1/K plus a simulated run).
Prints a comparison table; failures are listed, not sent anywhere.
"""

import argparse
import json
import os
import random
import sys

sys.path.insert(0, ".")
import numpy as np
from PIL import Image


def load_test_samples(real_root, splits=("test",)):
    man = os.path.join(real_root, "manifest.jsonl")
    out = []
    with open(man, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if rec.get("split") not in splits:
                continue
            photo = np.asarray(Image.open(
                os.path.join(real_root, rec["photo"])).convert("RGB"))
            tj = json.load(open(os.path.join(real_root, rec["traj_file"]),
                                encoding="utf-8"))
            out.append({"batch": rec["batch"], "photo": photo,
                        "slots": rec["slots"], "samples": tj["samples"]})
    return out


def score_tip(tip, slots_xy, target_slot):
    import math
    dists = {s: math.hypot(tip[0] - xy[0], tip[1] - xy[1])
             for s, xy in slots_xy.items()}
    herr = dists[target_slot]
    ok_z = 40.0 <= tip[2] <= 90.0
    success = herr <= 30.0 and ok_z
    wrong = (not success and any(
        s != target_slot and d <= 30.0 for s, d in dists.items()))
    return success, wrong, herr


def run_ckpt(label, ckpt, batches, rest_q):
    from src.kin.fk import fk_model
    from src.kin.model import load_model
    from src.console.control import tip_doc
    from src.vla import local_infer as LI
    import src.vla.scene as S
    doc = load_model("models/robot_inferred.json")
    xy_of = {v: k for k, v in S.SLOT_IDS.items()}
    rows, fails = [], []
    for b in batches:
        slots_xy = {s: xy_of[s] for s in b["slots"]}
        for s in b["samples"]:
            out = LI.predict_chunk(ckpt, b["photo"], list(rest_q),
                                   s["task"])
            assert len(out["actions"]) == 50, out.get("n_steps")
            q = [float(v) for v in out["actions"][-1]]
            tip = fk_model(q, tip_doc(doc))["pos_mm"]
            ok, wrong, herr = score_tip(tip, slots_xy, s["slot"])
            rows.append({"batch": b["batch"], "target": s["colour"],
                         "slot": s["slot"], "success": ok,
                         "wrong_block": wrong,
                         "herr_mm": round(herr, 1),
                         "state_source": out.get("state_source")})
            if not ok:
                fails.append({"ckpt": label, "batch": b["batch"],
                              "target": s["colour"],
                              "herr_mm": round(herr, 1),
                              "wrong_block": wrong})
    n = len(rows)
    return {
        "ckpt": label, "n": n,
        "success_rate": round(sum(r["success"] for r in rows) / n, 3),
        "wrong_block_rate": round(
            sum(r["wrong_block"] for r in rows) / n, 3),
        "failures": fails,
    }


def run_random(batches, seed=6464):
    rng = random.Random(seed)
    rows = []
    for b in batches:
        occ = sorted(b["slots"])
        for s in b["samples"]:
            pick = rng.choice(occ)
            rows.append({"success": pick == s["slot"],
                         "wrong_block": pick != s["slot"]})
    n = len(rows)
    return {"ckpt": "random", "n": n,
            "success_rate": round(sum(r["success"] for r in rows) / n, 3),
            "wrong_block_rate": round(
                sum(r["wrong_block"] for r in rows) / n, 3)}


def write_report(res, path):
    """Write the eval dict to path (same bytes as stdout)."""
    text = json.dumps(res, ensure_ascii=False, indent=1)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text + "\n")
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description="9B real-test before/after")
    ap.add_argument("--a", default="runs/vla/ckpt-last")
    ap.add_argument("--b", required=True)
    ap.add_argument("--real-root", default="runs/stage9/real")
    ap.add_argument("--splits", default="test",
                    help="comma list, e.g. test or test,holdout")
    ap.add_argument("--out", default=None,
                    help="also save the printed JSON to this path")
    args = ap.parse_args(argv)
    splits = tuple(s.strip() for s in args.splits.split(",") if s.strip())
    for s in splits:
        assert s in ("test", "holdout", "train"), s
    batches = load_test_samples(args.real_root, splits)
    assert batches, "no batches for splits %s yet" % (splits,)
    rep60 = json.load(open("runs/vla/6.0/report.json", encoding="utf-8"))
    rest_q = [float(v) for v in rep60["rest_q_deg"]]
    res = {"batches": [b["batch"] for b in batches],
           "splits": list(splits),
           "n_samples": sum(len(b["samples"]) for b in batches)}
    res["before"] = run_ckpt("before", args.a, batches, rest_q)
    res["after"] = run_ckpt("after", args.b, batches, rest_q)
    res["random"] = run_random(batches)
    print(json.dumps(res, ensure_ascii=False, indent=1))
    if args.out:
        write_report(res, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
