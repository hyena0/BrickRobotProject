"""Forward kinematics by product of exponentials (Stage 4.2).

Each revolute joint is a space-frame screw (w, v = -w x p) at the
zero pose; T(q) = exp([S1] q1) ... exp([Sn] qn) M, with M the home
pose of the end effector. Matches the nested-body MJCF from
src/kin/mjcf.py exactly (same joint frames).

Prismatic joints (Stage 7 T.3, additive): screw (0, v = axis unit),
exp = pure translation along v. PoE stays exact because earlier
factors carry the translation (e^{S1} e^{S2} with S2 prismatic gives
t1 + R1 v2 q2). q units are native: radians revolute, mm prismatic.
"""

from __future__ import annotations

import argparse
import json
import math

from .model import ee_point, joint_arrays, load_model


def skew(w) -> list[list[float]]:
    return [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]],
            [-w[1], w[0], 0.0]]


def mat_mul(a, b) -> list[list[float]]:
    n = len(a)
    return [[sum(a[i][k] * b[k][j] for k in range(n))
             for j in range(len(b[0]))] for i in range(n)]


def mat_vec(a, v) -> list[float]:
    return [sum(row[k] * v[k] for k in range(len(v))) for row in a]


def se3_exp(w, v, theta) -> list[list[float]]:
    """Closed-form exp of an se(3) matrix (Rodrigues, no scipy)."""
    k = skew(w)
    k2 = mat_mul(k, k)
    s, c = math.sin(theta), math.cos(theta)
    r = [[(1.0 if i == j else 0.0) + s * k[i][j]
          + (1.0 - c) * k2[i][j] for j in range(3)] for i in range(3)]
    wxv = mat_vec(k, v)
    wtv = sum(w[i] * v[i] for i in range(3))
    t = [0.0, 0.0, 0.0]
    for i in range(3):
        t[i] = sum(((1.0 if i == j else 0.0) - r[i][j]) * wxv[j]
                   for j in range(3)) + w[i] * wtv * theta
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]],
            [0.0, 0.0, 0.0, 1.0]]


def fk(q_rad: list[float], axes, points,
       ee: list[float], kinds=None) -> list[list[float]]:
    """Homogeneous end-effector pose for joint values.

    Revolute takes radians, prismatic takes mm (`kinds` entries
    "revolute"/"prismatic"; None means all revolute, as before).
    """
    t: list[list[float]] = [[1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]]
    kinds = kinds or ["revolute"] * len(q_rad)
    for qi, w, p, kind in zip(q_rad, axes, points, kinds):
        if kind == "prismatic":
            tr = [[1.0, 0.0, 0.0, w[0] * qi],
                  [0.0, 1.0, 0.0, w[1] * qi],
                  [0.0, 0.0, 1.0, w[2] * qi],
                  [0.0, 0.0, 0.0, 1.0]]
            t = mat_mul(t, tr)
            continue
        v = [-(w[1] * p[2] - w[2] * p[1]),
             -(w[2] * p[0] - w[0] * p[2]),
             -(w[0] * p[1] - w[1] * p[0])]
        t = mat_mul(t, se3_exp(w, v, qi))
    m = [[1.0, 0.0, 0.0, ee[0]], [0.0, 1.0, 0.0, ee[1]],
         [0.0, 0.0, 1.0, ee[2]], [0.0, 0.0, 0.0, 1.0]]
    return mat_mul(t, m)


def fk_model(q_deg: list[float], doc: dict) -> dict:
    """FK from a model dict; revolute in degrees, prismatic in mm."""
    arr = joint_arrays(doc)
    kinds = arr.get("kinds", ["revolute"] * len(q_deg))
    q = [x if k == "prismatic" else math.radians(x)
         for x, k in zip(q_deg, kinds)]
    t = fk(q, arr["axes"], arr["points"], ee_point(doc), kinds)
    return {"pos_mm": [t[0][3], t[1][3], t[2][3]],
            "mat": [row[:3] for row in t[:3]], "T": t}


def main() -> None:
    ap = argparse.ArgumentParser(description="FK pose for joint angles")
    ap.add_argument("--model", required=True)
    ap.add_argument("--q", required=True,
                    help="comma-separated joint angles in degrees")
    args = ap.parse_args()
    q = [float(x) for x in args.q.split(",")]
    out = fk_model(q, load_model(args.model))
    print(json.dumps({"pos_mm": out["pos_mm"], "mat": out["mat"]}))


if __name__ == "__main__":
    main()
