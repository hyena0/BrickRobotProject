"""Joint limits from self-collision (Stage 5A.2, simulation only).

For each joint, sweep away from home one side at a time until the
mesh oracle reports a new cross-link penetration, then back off by
5 degrees. Exclusions (home-engaged seats, designed gear meshes,
bearings, zero-pose CAD baseline) come from the shared MeshContext.
Coarse scans run on the process farm; refinement is bisection in
the main process.
"""

from __future__ import annotations

import argparse
import json

from .common import (MeshContext, PoseFarm, downstream_links, ensure_dir,
                     joint_side_sets, load_inputs, load_library,
                     pose_collisions, upstream_links)

MARGIN_DEG = 5.0


def joint_cross_pairs(mesh: MeshContext, joint_id: str) -> set:
    """Upstream-vs-downstream member pairs for one joint.

    Same-side pairs keep their home verdict (rigid motion), so only
    these can newly collide during the sweep.
    """
    up, dn = joint_side_sets(mesh.ctx, joint_id)
    up &= set(mesh.full)
    dn &= set(mesh.full)
    return {tuple(sorted([a, b])) for a in up for b in dn}


def sweep_side(farm: PoseFarm, mesh: MeshContext, joint_id: str,
               pairs: set, exclude: set, direction: float,
               coarse_deg: float = 5.0, fine_deg: float = 1.0,
               span_deg: float = 180.0) -> dict:
    """First colliding angle on one side via batched coarse + bisect."""
    ctx = mesh.ctx
    ordered = ctx["ordered"]
    jpos = next(n for n, j in enumerate(ordered) if j["id"] == joint_id)
    n_steps = int(span_deg / coarse_deg)
    angles = [round(direction * s * coarse_deg, 3)
              for s in range(1, n_steps + 1)]
    hit_at = None
    for start in range(0, len(angles), 6):
        chunk = angles[start:start + 6]
        qs = []
        for a in chunk:
            q = [0.0] * len(ordered)
            q[jpos] = a
            qs.append(q)
        results = farm.map(qs, pairs=pairs, exclude=exclude,
                           first_only=True, budget_s=60.0)
        hit_at = next((a for a, hits in zip(chunk, results) if hits),
                      None)
        if hit_at is not None:
            break
    if hit_at is None:
        return {"collision_deg": None, "boundary_deg": None,
                "status": "clean"}
    prev = round(hit_at - direction * coarse_deg, 3)
    # Bisect between the last clean angle and the first colliding
    # one. For the negative direction the clean side is ABOVE, so
    # the known-colliding bound (lo) moves up and the answer is lo;
    # mirroring them reports the coarse grid point instead.
    lo, hi = (prev, hit_at) if direction > 0 else (hit_at, prev)
    while hi - lo > fine_deg + 1e-9:
        mid = round((lo + hi) / 2.0, 3)
        q = [0.0] * len(ordered)
        q[jpos] = mid
        # Via the farm so TIMEOUT pseudo-hits auto-resolve exactly;
        # a timeout-corrupted bisection misplaces the boundary.
        hit = bool(farm.map([q], pairs=pairs, exclude=exclude,
                            first_only=True, budget_s=120.0)[0])
        if direction > 0:
            if hit:
                hi = mid
            else:
                lo = mid
        else:
            if hit:
                lo = mid
            else:
                hi = mid
    first = hi if direction > 0 else lo
    mag = max(abs(first) - MARGIN_DEG, 0.0)
    bound = round((1.0 if first >= 0 else -1.0) * mag if mag else 0.0, 3)
    return {"collision_deg": first, "boundary_deg": bound,
            "status": "ok"}


def main() -> None:
    ap = argparse.ArgumentParser(description="Collision boundary sweep")
    ap.add_argument("--out", default="runs/stage5a/collision_bounds.json")
    ap.add_argument("--coarse", type=float, default=5.0)
    ap.add_argument("--span", type=float, default=180.0)
    ap.add_argument("--jobs", type=int, default=0)
    args = ap.parse_args()
    ctx = load_inputs()
    lib = load_library()
    mesh = MeshContext(ctx, lib)
    farm = PoseFarm(jobs=args.jobs)
    farm.bind(mesh)
    rows = []
    for j in ctx["ordered"]:
        pairs = joint_cross_pairs(mesh, j["id"])
        entry: dict = {"joint": j["id"], "n_pairs": len(pairs)}
        for direction, tag in ((+1.0, "pos"), (-1.0, "neg")):
            res = sweep_side(farm, mesh, j["id"], pairs,
                             mesh.excluded(j["id"]), direction,
                             coarse_deg=args.coarse, span_deg=args.span)
            entry[tag] = res
            print(f"{j['id']}[{tag}]: {res}", flush=True)
        rows.append(entry)
    ensure_dir(args.out)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump({"bounds": rows,
                   "method": ("single-joint mesh sweep per side to first "
                              "cross-link penetration (depth > 2.0 mm, "
                              "home-engaged/gear/bearing/baseline "
                              "excluded), boundary = first hit minus "
                              "5 deg"),
                   "margin_deg": MARGIN_DEG,
                   "coarse_deg": args.coarse,
                   "fine_deg": 1.0}, fh, indent=2)
    print(f"joints={len(rows)} -> {args.out}")


if __name__ == "__main__":
    main()
