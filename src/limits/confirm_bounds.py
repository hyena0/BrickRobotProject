"""Blender-BVH confirmation of Stage 5A boundary verdicts.

The PC oracle works on decimated LDraw meshes; stage5_dev requires
BVH mesh truth for verdicts that stand. This driver articulates the
exact part meshes in Blender at the recorded sensor/collision
boundary poses and BVH-checks the PC oracle's hit pairs
(src/viz/pose_confirm_inner.py via pose_check.run_confirm).

Poses that Blender confirms stay boundaries; poses it clears are
flagged for re-sweep (5C measures them physically either way).
"""

from __future__ import annotations

import argparse
import json

from src.kin.model import chain_order, load_model
from src.limits.common import (MeshContext, ensure_dir, load_inputs,
                               load_library, pose_collisions)
from src.limits.safe_poses import _q_to_m
from src.viz.pose_check import run_confirm


def boundary_poses(ctx: dict, sens_path: str,
                   coll_path: str) -> list[dict]:
    """Boundary poses from the 5A.1/5A.2 records."""
    sens = json.load(open(sens_path, encoding="utf-8"))["bounds"]
    coll = {r["joint"]: r for r in
            json.load(open(coll_path, encoding="utf-8"))["bounds"]}
    names = [j["id"] for j in ctx["ordered"]]
    out = []
    for row in sens:
        if row.get("status") != "ok":
            continue
        q = [0.0] * len(names)
        q[names.index(row["joint"])] = row["boundary_deg"]
        out.append({"id": f"sens-{row['joint']}", "q_deg": q,
                    "kind": "sensor"})
    for jid, rec in coll.items():
        for tag in ("pos", "neg"):
            side = rec.get(tag, {})
            if side.get("status") != "ok":
                continue
            q = [0.0] * len(names)
            q[names.index(jid)] = side["collision_deg"]
            out.append({"id": f"coll-{jid}-{tag}", "q_deg": q,
                        "kind": "collision"})
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="BVH-confirm boundaries")
    ap.add_argument("--sensors", default="runs/stage5a/sensor_bounds.json")
    ap.add_argument("--collisions",
                    default="runs/stage5a/collision_bounds.json")
    ap.add_argument("--batch",
                    default="runs/stage5a/confirm_batch.json")
    ap.add_argument("--candidates",
                    default="runs/stage5a/confirm_candidates.json")
    ap.add_argument("--out", default="runs/stage5a/confirm_bounds.json")
    args = ap.parse_args()
    doc = load_model("models/robot_inferred.json")
    ctx = load_inputs()
    lib = load_library()
    mesh = MeshContext(ctx, lib)
    poses = boundary_poses(ctx, args.sensors, args.collisions)
    ordered = chain_order(doc)
    c = doc["transmission"]["C"]
    motor_ids = [a["id"] for a in doc["actuators"]]
    batch_poses = []
    candidates: dict[str, list] = {}
    for p in poses:
        m = _q_to_m(p["q_deg"], c, motor_ids)
        batch_poses.append({"id": p["id"], "q_deg": p["q_deg"],
                            "m_deg": [round(v, 3) for v in m]})
        hits = pose_collisions(mesh, p["q_deg"],
                               exclude=mesh.excluded())[:8]
        candidates[p["id"]] = [[h["a"], h["b"]] for h in hits]
        print(f"{p['id']} q={p['q_deg']}: oracle hits="
              f"{[(h['a'], h['b'], h['depth_mm']) for h in hits[:4]]}",
              flush=True)
    batch = {"repo": ".",
             "base_blend": "runs/stage2-chain-r17/model_chain_r17.blend",
             "link_ids": ctx["link_ids"],
             "joints": [{"id": j["id"], "axis": j["axis"],
                         "point_mm": j["point_mm"]} for j in ordered],
             "members": {lid: sorted(uids) for lid, uids in
                         ctx["members"].items()},
             "ee_mm": doc.get("end_effector", {}).get("point_mm"),
             "poses": batch_poses}
    ensure_dir(args.batch)
    json.dump(batch, open(args.batch, "w"), indent=1)
    json.dump(candidates, open(args.candidates, "w"), indent=1)
    run_confirm(args.batch, args.candidates, args.out)
    confirm = json.load(open(args.out, encoding="utf-8"))
    n_conf = sum(len(v) for v in confirm.values())
    print(f"Blender-confirmed pairs: {n_conf} "
          f"over {len(batch_poses)} poses -> {args.out}")


if __name__ == "__main__":
    main()
