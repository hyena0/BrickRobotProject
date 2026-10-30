"""Collision-free safe pose set for Stage 5C replay (Stage 5A.5).

Five small poses (<= 30 % of each operating range) plus five large
ones. Every pose and its path from home is verified with the mesh
oracle (collision-free, inside limits and inequalities) and the end
effector stays >= 40 mm above the table estimate. Renders go to
assets/private/renders/poses_safe/ via the Stage 5 Blender pipeline
(private, git-ignored).
"""

from __future__ import annotations

import argparse
import json
import random

from src.kin.fk import fk_model
from src.kin.model import chain_order, load_model
from src.limits.common import (MeshContext, ensure_dir, load_inputs,
                               load_library, pose_collisions,
                               table_height_mm)
from src.limits.enforce import check_pose, is_path_safe

EE_CLEAR_MM = 40.0


def _q_to_m(q, c, motor_ids):
    import numpy as np
    cc = np.asarray(c, dtype=float)
    m, *_ = np.linalg.lstsq(cc, np.asarray(q, dtype=float), rcond=None)
    return [float(v) for v in m]


def candidates(names: list[str], boxes: dict, seed: int, small: bool,
                 count: int) -> list[list[float]]:
    """Resample-free candidate generation inside the box."""
    rng = random.Random(seed)
    out = []
    for _ in range(count):
        frac = 0.30 if small else 1.0
        q = []
        for jid in names:
            lo, hi = boxes[jid]
            span = (hi - lo) * frac / 2.0
            mid = (hi + lo) / 2.0
            q.append(round(rng.uniform(mid - span, mid + span), 2))
        out.append(q)
    return out


def main() -> None:
    from .common import PoseFarm
    ap = argparse.ArgumentParser(description="Sample safe poses")
    ap.add_argument("--model", default="models/robot_inferred.json")
    ap.add_argument("--limits", default="runs/stage5a/limits.json")
    ap.add_argument("--out", default="runs/stage5a/poses10_safe.json")
    ap.add_argument("--batch", default="runs/stage5a/pose_batch_safe.json")
    ap.add_argument("--seed", type=int, default=51)
    ap.add_argument("--jobs", type=int, default=0)
    ap.add_argument("--try-n", type=int, default=300)
    args = ap.parse_args()
    doc = load_model(args.model)
    ctx = load_inputs(model_path=args.model)
    lib = load_library()
    mesh = MeshContext(ctx, lib)
    farm = PoseFarm(jobs=args.jobs)
    farm.bind(mesh)
    names = [j["id"] for j in ctx["ordered"]]
    limits_doc = json.load(open(args.limits, encoding="utf-8"))
    boxes = limits_doc["limits_deg"]
    table_z = table_height_mm(ctx)
    c = doc["transmission"]["C"]
    motor_ids = [a["id"] for a in doc["actuators"]]
    exclude = set(mesh.excluded())
    for jid in names:
        exclude |= set(mesh.excluded(jid))
    poses = []
    for small, need in ((True, 5), (False, 5)):
        found: list[dict] = []
        seed = args.seed + (0 if small else 1000)
        for _round in range(6):
            if len(found) >= need:
                break
            qs = candidates(names, boxes, seed + _round * 77, small,
                            args.try_n)
            qs = [q for q in qs
                  if not check_pose(q, names, limits_doc)
                  and is_path_safe([0.0] * len(names), q, names,
                                   limits_doc)["safe"]]
            if not qs:
                continue
            results = farm.map(qs, pairs=None, exclude=exclude,
                               first_only=True, budget_s=60.0)
            for q, hits in zip(qs, results):
                if hits:
                    continue
                ee = fk_model(q, doc)["pos_mm"]
                if ee[2] < table_z + EE_CLEAR_MM:
                    continue
                m = _q_to_m(q, c, motor_ids)
                found.append({"q_deg": q,
                              "m_deg": [round(v, 3) for v in m],
                              "ee_mm": [round(v, 2) for v in ee],
                              "table_z_mm": round(table_z, 1)})
                if len(found) >= need:
                    break
        if len(found) < need:
            raise RuntimeError(f"only {len(found)}/{need} "
                               f"{'small' if small else 'large'} poses")
        for row in found:
            row["id"] = "safe%02d" % (len(poses) + 1)
            row["size"] = "small" if small else "large"
            poses.append(row)
            print(f"{row['id']} ({row['size']}): q={row['q_deg']} "
                  f"ee_z={row['ee_mm'][2]}", flush=True)
    ordered = chain_order(doc)
    batch = {"repo": ".",
             "base_blend": "runs/stage2-chain-r17/model_chain_r17.blend",
             "link_ids": ctx["link_ids"],
             "joints": [{"id": j["id"], "axis": j["axis"],
                         "point_mm": j["point_mm"]} for j in ordered],
             "members": {lid: sorted(uids) for lid, uids in
                         ctx["members"].items()},
             "ee_mm": doc.get("end_effector", {}).get("point_mm"),
             "poses": [{"id": p["id"], "q_deg": p["q_deg"],
                        "m_deg": p["m_deg"]} for p in poses]}
    ensure_dir(args.out)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump({"poses": poses,
                   "method": ("mesh-oracle verified: limits, path "
                              "from home, collision-free, EE >= 40 mm "
                              "above the table estimate"),
                   "table_z_mm": round(table_height_mm(ctx), 1)},
                  fh, indent=2)
    with open(args.batch, "w", encoding="utf-8") as fh:
        json.dump(batch, fh, indent=1)
    print(f"poses={len(poses)} -> {args.out} + {args.batch}")


if __name__ == "__main__":
    main()
