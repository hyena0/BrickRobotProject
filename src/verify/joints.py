"""Joint error metrics (Stage 2.5).

Scores predicted joint axes against `gt/gt_joints.json`, whose schema
is joints.{J1..J6} with `point` (mm, extraction frame) and `axis`
(unit vector). The scoring rules below implement that file's embedded
`grading` text verbatim:

- position: distance from the pred point to the GT axis line
  (point-along-axis is unobservable gauge);
- axis: |dot(pred_axis, gt_axis)| >= cos(tolerance_deg)
  (sign-equivalent direction).

Gates: joint IDs exactly J1..J6, lateral distance <= 5 mm and axis
tolerance 5 deg for every joint. The `gt/` path is accepted only as
the `--gt` CLI argument; nothing else in `src/` may read it.
"""

from __future__ import annotations

import argparse
import json
import math

GT_IDS = ["J1", "J2", "J3", "J4", "J5", "J6"]
POS_TOL_MM = 5.0
AXIS_TOL_DEG = 5.0


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


def _vec3(name: str, value) -> list[float]:
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or not all(isinstance(x, (int, float)) for x in value)):
        raise ValueError(f"GT joint entry {name!r} needs 3 numbers")
    return [float(x) for x in value]


def load_gt_joints(path: str) -> dict:
    """Load `gt/gt_joints.json` (joints.{J1..J6}.point/axis).

    Raises ValueError when any of J1..J6 is missing or malformed.
    """
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    joints = doc.get("joints", {}) if isinstance(doc, dict) else {}
    out = {}
    for jid in GT_IDS:
        spec = joints.get(jid, {})
        point = _vec3(f"{jid}.point", spec.get("point"))
        axis = _vec3(f"{jid}.axis", spec.get("axis"))
        if math.sqrt(sum(x * x for x in axis)) == 0.0:
            raise ValueError(f"GT joint {jid} axis is zero")
        out[jid] = {"point": point, "axis": _norm(axis)}
    return out


def point_to_axis_line(point: list[float], axis_g: list[float],
                       point_g: list[float]) -> float:
    """Perpendicular distance from a point to the GT axis line (mm)."""
    u = _norm(axis_g)
    w = [p - g for p, g in zip(point, point_g)]
    proj = sum(w[i] * u[i] for i in range(3))
    perp = [w[i] - proj * u[i] for i in range(3)]
    return math.sqrt(sum(x * x for x in perp))


def cosine_abs(a: list[float], b: list[float]) -> float:
    na, nb = _norm(a), _norm(b)
    cosv = abs(sum(x * y for x, y in zip(na, nb)))
    return max(-1.0, min(1.0, cosv))


def score_joint(pred_axis: list[float], pred_point: list[float],
                gt: dict, pos_tol_mm: float = POS_TOL_MM,
                tol_deg: float = AXIS_TOL_DEG) -> dict:
    """Score one joint per the embedded grading text."""
    pos = point_to_axis_line(pred_point, gt["axis"], gt["point"])
    cosv = cosine_abs(pred_axis, gt["axis"])
    angle = math.degrees(math.acos(cosv))
    ok = pos <= pos_tol_mm and cosv >= math.cos(math.radians(tol_deg))
    return {"pos_mm": round(pos, 3), "cos_abs": round(cosv, 5),
            "angle_deg": round(angle, 3), "pass": bool(ok)}


def main() -> None:
    ap = argparse.ArgumentParser(description="Score joint axes")
    ap.add_argument("--pred", required=True,
                    help="runs/<id>/links_joints_public.json")
    ap.add_argument("--gt", required=True,
                    help="gt/gt_joints.json (joints.J1..J6 point/axis)")
    ap.add_argument("--pos-tol-mm", type=float, default=POS_TOL_MM)
    ap.add_argument("--axis-tol-deg", type=float, default=AXIS_TOL_DEG)
    args = ap.parse_args()
    pred = json.load(open(args.pred, encoding="utf-8"))
    try:
        gt = load_gt_joints(args.gt)
    except (OSError, ValueError) as exc:
        print(f"ground-truth joints unreadable ({exc}); cannot score")
        raise SystemExit(2)
    pred_joints = {j.get("id"): j for j in pred.get("joints", [])}
    print(f"joint count: pred={len(pred_joints)} gt={len(gt)} "
          f"{'PASS' if sorted(pred_joints) == GT_IDS else 'FAIL'}")
    all_ok = sorted(pred_joints) == GT_IDS
    for jid in GT_IDS:
        pj = pred_joints.get(jid)
        if pj is None:
            print(f"{jid}: missing in pred FAIL")
            all_ok = False
            continue
        row = score_joint(pj["axis"], pj["point_mm"], gt[jid],
                          args.pos_tol_mm, args.axis_tol_deg)
        all_ok = all_ok and row["pass"]
        print(f"{jid}: lateral={row['pos_mm']}mm "
              f"angle={row['angle_deg']}deg |cos|={row['cos_abs']} "
              f"{'PASS' if row['pass'] else 'FAIL'}")
    raise SystemExit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
