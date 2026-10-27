"""Joint error metrics (Stage 2.5).

Greedy joint matching by axis-line distance, then per-joint axis angle
error (degrees) and axis line distance (mm). Gate: joint count exact,
angle error <= 5 deg and axis distance <= 5 mm for every joint.
"""

from __future__ import annotations

import argparse
import json
import math


def _norm(v: list[float]) -> list[float]:
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 0 else [0.0, 0.0, 1.0]


def axis_angle_deg(a: list[float], b: list[float]) -> float:
    na, nb = _norm(a), _norm(b)
    cosv = abs(sum(x * y for x, y in zip(na, nb)))
    cosv = max(-1.0, min(1.0, cosv))
    return math.degrees(math.acos(cosv))


def axis_line_distance(axis_p: list[float], point_p: list[float],
                       axis_g: list[float], point_g: list[float]) -> float:
    """Shortest distance between two 3D lines in mm."""
    u, v = _norm(axis_p), _norm(axis_g)
    w = [pp - pg for pp, pg in zip(point_p, point_g)]
    cross = [u[1] * v[2] - u[2] * v[1],
             u[2] * v[0] - u[0] * v[2],
             u[0] * v[1] - u[1] * v[0]]
    n = math.sqrt(sum(x * x for x in cross))
    if n < 1e-9:  # parallel: point-to-line distance
        proj = sum(w[i] * u[i] for i in range(3))
        perp = [w[i] - proj * u[i] for i in range(3)]
        return math.sqrt(sum(x * x for x in perp))
    nhat = [x / n for x in cross]
    return abs(sum(w[i] * nhat[i] for i in range(3)))


def match_joints(pred: list[dict], gt: list[dict]) -> list[dict]:
    """Greedy match on axis-line distance; each GT joint used once."""
    rows = []
    for i, pjoint in enumerate(pred):
        for j, gjoint in enumerate(gt):
            dist = axis_line_distance(pjoint["axis"], pjoint["point_mm"],
                                      gjoint["axis"], gjoint["point_mm"])
            ang = axis_angle_deg(pjoint["axis"], gjoint["axis"])
            rows.append((dist, ang, i, j))
    rows.sort()
    used_p, used_g, matches = set(), set(), []
    for dist, ang, i, j in rows:
        if i in used_p or j in used_g:
            continue
        used_p.add(i)
        used_g.add(j)
        matches.append({"pred": pred[i].get("id"),
                        "gt": gt[j].get("id"),
                        "axis_angle_deg": round(ang, 3),
                        "axis_distance_mm": round(dist, 3)})
    return matches


def main() -> None:
    ap = argparse.ArgumentParser(description="Score joint axes")
    ap.add_argument("--pred", required=True,
                    help="runs/<id>/links_joints_public.json")
    ap.add_argument("--gt", required=True,
                    help="eval/ground_truth/kinematics_gt.json")
    args = ap.parse_args()
    pred = json.load(open(args.pred, encoding="utf-8"))
    gt = json.load(open(args.gt, encoding="utf-8"))
    pred_joints, gt_joints = pred.get("joints", []), gt.get("joints", [])
    print(f"joint count: pred={len(pred_joints)} gt={len(gt_joints)} "
          f"{'PASS' if len(pred_joints) == len(gt_joints) and gt_joints else 'FAIL'}")
    if not gt_joints:
        print("ground-truth joints pending; cannot score")
        raise SystemExit(2)
    for match in match_joints(pred_joints, gt_joints):
        ok = (match["axis_angle_deg"] <= 5.0
              and match["axis_distance_mm"] <= 5.0)
        print(f"{match['pred']}~{match['gt']}: "
              f"angle={match['axis_angle_deg']}deg "
              f"dist={match['axis_distance_mm']}mm "
              f"{'PASS' if ok else 'FAIL'}")


if __name__ == "__main__":
    main()
