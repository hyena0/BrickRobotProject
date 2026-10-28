"""Transmission error metrics (Stage 3.3).

Compares a predicted transmission (C, ratios) with ground truth.
Fails loudly (exit 2) when ground truth is pending, like the Stage 2
verifiers. Never prints ground-truth values, only errors.
"""

from __future__ import annotations

import argparse
import json
import math


def max_abs_diff(pred_C: list[list[float]],
                 gt_C: list[list[float]]) -> float | None:
    if not gt_C or not gt_C[0]:
        return None
    if len(pred_C) != len(gt_C) or len(pred_C[0]) != len(gt_C[0]):
        return math.inf
    return max(abs(p - g) for prow, grow in zip(pred_C, gt_C)
               for p, g in zip(prow, grow))


def ratio_errors(pred: dict, gt: dict) -> list[dict]:
    """Per-joint ratio error (%) and sign check, matched by position."""
    rows = []
    gt_C, pred_C = gt.get("C", []), pred.get("C", [])
    n = min(len(gt_C), len(pred_C))
    for i in range(n):
        grow, prow = gt_C[i], pred_C[i]
        m = min(len(grow), len(prow))
        for k in range(m):
            g, p = grow[k], prow[k]
            err = None if g == 0 else abs(p - g) / abs(g) * 100.0
            rows.append({"joint": i, "motor": k,
                         "ratio_err_pct": err,
                         "sign_ok": (p >= 0) == (g >= 0)})
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description="Score transmission")
    ap.add_argument("--pred", required=True,
                    help="predicted links_joints_public.json (transmission.C)")
    ap.add_argument("--gt", required=True,
                    help="eval/ground_truth/kinematics_gt.json")
    args = ap.parse_args()
    pred = json.load(open(args.pred, encoding="utf-8"))
    gt = json.load(open(args.gt, encoding="utf-8"))
    pred_T = pred.get("transmission", {})
    gt_T = gt.get("transmission", {})
    pred_C, gt_C = pred_T.get("C", []), gt_T.get("C", [])
    if not gt_C or not gt_C[0]:
        print("ground-truth transmission pending; cannot score")
        raise SystemExit(2)
    diff = max_abs_diff(pred_C, gt_C)
    print(f"C max abs diff: {diff:.4f} "
          f"{'PASS' if diff is not None and diff <= 0.02 else 'FAIL'} "
          f"(threshold 0.02)")
    ok_all = True
    for row in ratio_errors(pred_T, gt_T):
        err = row["ratio_err_pct"]
        ok = (err is not None and err <= 1.0 and row["sign_ok"])
        ok_all = ok_all and ok
        err_s = "n/a" if err is None else f"{err:.3f}%"
        print(f"joint{row['joint']}/motor{row['motor']}: "
              f"err={err_s} sign_ok={row['sign_ok']} "
              f"{'PASS' if ok else 'FAIL'}")
    if diff is None or diff > 0.02 or not ok_all:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
