"""IK evaluation against ground truth (Stage 4.5).

Samples random q_true within ground-truth limits, takes the pose
from the MuJoCo ground-truth model, solves IK with the inferred
model, and measures the end-effector error back in the MuJoCo
ground-truth model. Prints aggregates only, never ground-truth
values. The `gt/` path is accepted only as `--gt`.
"""

from __future__ import annotations

import argparse
import json
import math
import random

from src.kin.ik import (SUCCESS_POS_MM, SUCCESS_ROT_DEG, ik)
from src.kin.mjcf import build_mjcf
from src.kin.model import chain_order, ee_point, joint_arrays, load_model
from src.verify.conventions import (align_ee_gauge, apply_signs,
                                    sign_map)

MEDIAN_POS_MM = 2.0
SUCCESS_RATE = 0.95


def _rot_err_deg(r_pred, r_tgt) -> float:
    rt = [[sum(r_pred[j][i] * r_tgt[j][k] for j in range(3))
           for k in range(3)] for i in range(3)]
    tr = rt[0][0] + rt[1][1] + rt[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def _quantile(xs: list[float], q: float) -> float:
    s = sorted(xs)
    if not s:
        return float("nan")
    pos = (len(s) - 1) * q
    lo, hi = math.floor(pos), math.ceil(pos)
    return s[lo] + (s[hi] - s[lo]) * (pos - lo)


def evaluate(pred_doc: dict, gt_doc: dict, n: int = 500,
             seed: int = 0, restarts: int = 8,
             position_only: bool = False, align_signs: bool = True,
             align_ee: bool = True) -> dict:
    """Run the Stage 4.5 protocol; returns summary metrics only.

    With align_signs, the reference side is rewritten with
    s_i = sign(dot) per joint (identical physics, pred conventions);
    sampled angles are mapped by the same signs. With align_ee, the
    evaluated EE is shifted along its own roll axis to the reference
    EE level (gauge). Both report what they removed, never values.
    """
    import mujoco

    signs = {j["id"]: 1 for j in chain_order(pred_doc)}
    if align_signs:
        signs = sign_map(pred_doc, gt_doc)
    gt_aligned = apply_signs(gt_doc, signs)
    offset = 0.0
    pred_eval = pred_doc
    if align_ee:
        pred_eval, offset = align_ee_gauge(pred_doc, gt_aligned)

    gt_xml = build_mjcf(gt_aligned)
    gt_arr = joint_arrays(gt_aligned)
    gt_model = mujoco.MjModel.from_xml_string(gt_xml)
    gt_data = mujoco.MjData(gt_model)
    ee_id = mujoco.mj_name2id(gt_model, mujoco.mjtObj.mjOBJ_SITE,
                              "ee")
    if ee_id < 0:
        raise ValueError("ground-truth MJCF has no ee site")

    pred_arr = joint_arrays(pred_eval)
    rng = random.Random(seed)
    pos_errs, rot_errs, ok = [], [], 0
    for _ in range(n):
        q_true = [rng.uniform(lo, hi) for lo, hi in gt_arr["limits"]]
        q_aligned = [signs[gt_arr["ids"][i]] * q_true[i]
                     for i in range(len(q_true))]
        gt_data.qpos[:] = q_aligned
        mujoco.mj_forward(gt_model, gt_data)
        pos_t = [float(x) for x in gt_data.site(ee_id).xpos]
        xmat = gt_data.site(ee_id).xmat
        mat_t = [[float(xmat[i * 3 + j]) for j in range(3)]
                 for i in range(3)]
        sol = ik(pos_t, mat_t, pred_eval, n_restarts=restarts,
                 seed=rng.randrange(1 << 30),
                 position_only=position_only)
        q_pred = [math.radians(x) for x in sol["q_deg"]]
        gt_data.qpos[:] = q_pred
        mujoco.mj_forward(gt_model, gt_data)
        pos_p = [float(x) for x in gt_data.site(ee_id).xpos]
        xmat_p = gt_data.site(ee_id).xmat
        mat_p = [[float(xmat_p[i * 3 + j]) for j in range(3)]
                 for i in range(3)]
        pe = math.sqrt(sum((pos_t[i] - pos_p[i]) ** 2
                           for i in range(3)))
        re = _rot_err_deg(mat_p, mat_t)
        pos_errs.append(pe)
        rot_errs.append(re)
        if pe <= SUCCESS_POS_MM and (position_only or
                                     re <= SUCCESS_ROT_DEG):
            ok += 1
    rate = ok / n
    med = _quantile(pos_errs, 0.5)
    p95 = _quantile(pos_errs, 0.95)
    gate = rate >= SUCCESS_RATE and med <= MEDIAN_POS_MM
    return {"n": n, "success_rate": rate,
            "median_pos_mm": med, "p95_pos_mm": p95,
            "median_rot_deg": _quantile(rot_errs, 0.5),
            "p95_rot_deg": _quantile(rot_errs, 0.95),
            "gate_pass": bool(gate),
            "signs": {k: signs[k] for k in gt_arr["ids"]},
            "ee_roll_gauge_mm": offset,
            "align_signs": align_signs, "align_ee": align_ee}


def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 4.5 IK evaluation")
    ap.add_argument("--pred", required=True, help="inferred model JSON")
    ap.add_argument("--gt", required=True,
                    help="ground-truth model JSON (CLI only)")
    ap.add_argument("--n", type=int, default=500)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--restarts", type=int, default=8)
    ap.add_argument("--position-only", action="store_true")
    ap.add_argument("--no-sign-align", action="store_true",
                    help="disable per-joint sign mapping (ablation)")
    ap.add_argument("--no-ee-align", action="store_true",
                    help="disable EE roll-gauge alignment (ablation)")
    ap.add_argument("--out", default=None,
                    help="optional JSON summary path")
    args = ap.parse_args()
    try:
        pred_doc = load_model(args.pred)
        gt_doc = load_model(args.gt)
        _ = chain_order(gt_doc), ee_point(gt_doc)
    except (OSError, ValueError) as exc:
        print(f"model unreadable ({exc}); cannot score")
        raise SystemExit(2)
    if len(joint_arrays(pred_doc)["ids"]) != 6 or \
            len(joint_arrays(gt_doc)["ids"]) != 6:
        print("both models need 6 joints; cannot score")
        raise SystemExit(2)
    res = evaluate(pred_doc, gt_doc, n=args.n, seed=args.seed,
                   restarts=args.restarts,
                   position_only=args.position_only,
                   align_signs=not args.no_sign_align,
                   align_ee=not args.no_ee_align)
    print("signs: " + " ".join(f"{k}={v:+d}"
                               for k, v in res["signs"].items()))
    print(f"ee roll gauge removed: {res['ee_roll_gauge_mm']:+.2f}mm")
    print(f"samples: {res['n']} success: "
          f"{res['success_rate'] * 100:.1f}% "
          f"(threshold {SUCCESS_RATE * 100:.0f}%)")
    print(f"position err: median={res['median_pos_mm']:.2f}mm "
          f"(threshold {MEDIAN_POS_MM:.1f}mm) "
          f"p95={res['p95_pos_mm']:.2f}mm")
    print(f"orientation err: median={res['median_rot_deg']:.2f}deg "
          f"p95={res['p95_rot_deg']:.2f}deg")
    print(f"Gate 2: {'PASS' if res['gate_pass'] else 'FAIL'}")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1)
        print(f"wrote {args.out}")
    raise SystemExit(0 if res["gate_pass"] else 1)


if __name__ == "__main__":
    main()
