"""Joint limits from sensor geometry (Stage 5A.1, simulation only)."""

from __future__ import annotations

import argparse
import json

from .common import (MeshContext, ensure_dir, joint_side_sets, load_inputs,
                     load_library, pose_collisions)


def sensor_sides(ctx: dict) -> dict:
    """Map joint id -> sensor records from the Stage 2.7 association."""
    out: dict[str, list[dict]] = {}
    for s in ctx["sensors"]:
        if s.get("joint"):
            out.setdefault(s["joint"], []).append(s)
    return out


def sweep_angles(farm, mesh: MeshContext, joint_id: str,
                 pairs: set, exclude: set, direction: float,
                 step_deg: float, span_deg: float) -> float | None:
    """Batched scan for the first contact angle on one side."""
    ctx = mesh.ctx
    ordered = ctx["ordered"]
    jpos = next(n for n, j in enumerate(ordered) if j["id"] == joint_id)
    n_steps = int(span_deg / step_deg)
    angles = [round(direction * s * step_deg, 3)
              for s in range(1, n_steps + 1)]
    for start in range(0, len(angles), 6):
        chunk = angles[start:start + 6]
        qs = []
        for a in chunk:
            q = [0.0] * len(ordered)
            q[jpos] = a
            qs.append(q)
        results = farm.map(qs, pairs=pairs, exclude=exclude,
                           first_only=True, budget_s=60.0)
        hit = next((a for a, hits in zip(chunk, results) if hits), None)
        if hit is not None:
            return hit
    return None


def sweep_sensor(farm, mesh: MeshContext, joint_id: str, sensor_uid: str,
                 step_deg: float = 2.0,
                 span_deg: float = 180.0) -> dict:
    """First mesh contact between a sensor and the opposite side.

    The presser set is every meshed part across the joint from the
    sensor mount link; the active region is proxied by the sensor
    mesh itself (button-tip and view-cone geometry is unresolved in
    the catalog, so whole-mesh contact is the conservative stand-in).
    Returns the first contact angle per sweep direction plus the
    homing direction (the side that presses).
    """
    ctx = mesh.ctx
    ordered = ctx["ordered"]
    jpos = next(n for n, j in enumerate(ordered) if j["id"] == joint_id)
    mount = ctx["link_of"].get(sensor_uid)
    up, dn = joint_side_sets(ctx, joint_id)
    up_l = {ctx["link_ids"][n] for n in range(jpos + 1)}
    sensor_up = mount in up_l
    pressers = (dn if sensor_up else up) - {sensor_uid}
    pressers = {u for u in pressers if u in mesh.full}
    if sensor_uid not in mesh.full or not pressers:
        return {"joint": joint_id, "sensor": sensor_uid,
                "status": "unverified",
                "note": "sensor or presser meshes missing from library"}
    pairs = {tuple(sorted([sensor_uid, u])) for u in pressers}
    exclude = mesh.excluded(joint_id)
    result: dict = {"joint": joint_id, "sensor": sensor_uid,
                    "mount_link": mount,
                    "sensor_side": "upstream" if sensor_up else "downstream",
                    "n_pressers": len(pressers)}
    result["boundary_+pos"] = sweep_angles(
        farm, mesh, joint_id, pairs, exclude, +1.0, step_deg, span_deg)
    result["boundary_+neg"] = sweep_angles(
        farm, mesh, joint_id, pairs, exclude, -1.0, step_deg, span_deg)
    pos_b = result.get("boundary_+pos")
    neg_b = result.get("boundary_+neg")
    if pos_b is None and neg_b is None:
        result["status"] = "unverified"
        result["note"] = ("no sensor contact over the full sweep; "
                          "physical check in Stage 5C decides")
        result["homing_direction"] = "unknown"
    elif pos_b is not None and neg_b is not None:
        result["status"] = "ambiguous"
        result["note"] = ("contact on both sides; association needs "
                          "physical review in Stage 5C")
        result["homing_direction"] = "unknown"
    else:
        result["status"] = "ok"
        result["homing_direction"] = "+" if pos_b is not None else "-"
        result["boundary_deg"] = pos_b if pos_b is not None else neg_b
    return result


def main() -> None:
    from .common import PoseFarm
    ap = argparse.ArgumentParser(description="Sensor boundary sweep")
    ap.add_argument("--out", default="runs/stage5a/sensor_bounds.json")
    ap.add_argument("--step", type=float, default=2.0,
                    help="coarse sweep step; refined to 0.5 deg")
    ap.add_argument("--jobs", type=int, default=0)
    args = ap.parse_args()
    ctx = load_inputs()
    lib = load_library()
    mesh = MeshContext(ctx, lib)
    farm = PoseFarm(jobs=args.jobs)
    farm.bind(mesh)
    by_joint = sensor_sides(ctx)
    rows = []
    for jid in [j["id"] for j in ctx["ordered"]]:
        for sens in by_joint.get(jid, []):
            row = sweep_sensor(farm, mesh, jid, sens["uid"],
                               step_deg=args.step)
            # Refine a found boundary to 0.5 deg.
            if row.get("status") == "ok":
                fine = sweep_sensor(farm, mesh, jid, sens["uid"],
                                    step_deg=0.5,
                                    span_deg=abs(row["boundary_deg"]) + 1.0)
                key = ("boundary_+pos"
                       if row["homing_direction"] == "+" else "boundary_+neg")
                if fine.get(key) is not None:
                    row["boundary_deg"] = fine[key]
            row["uid"] = sens["uid"]
            row["brick"] = sens.get("brick")
            row["port"] = sens.get("port")
            row["colour_role"] = ("reference" if
                                  str(sens.get("uid")) == "p000489" and
                                  row.get("status") != "ok"
                                  else "limit")
            rows.append(row)
            print(f"{jid}/{sens['uid']}: {row.get('status')} "
                  f"{row.get('boundary_deg')}", flush=True)
    ensure_dir(args.out)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump({"bounds": rows,
                   "method": ("touch: first sensor-vs-opposite-side mesh "
                              "contact per direction (whole-mesh proxy "
                              "for the button tip); colour: same sweep, "
                              "reference unless it blocks motion"),
                   "step_deg": args.step,
                   "refine_deg": 0.5}, fh, indent=2)
    print(f"sensors={len(rows)} ok="
          f"{sum(1 for r in rows if r.get('status') == 'ok')} "
          f"-> {args.out}")


if __name__ == "__main__":
    main()
