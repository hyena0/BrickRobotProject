"""Inverse kinematics: damped least squares (Stage 4.3).

Numeric solver only (no spherical wrist: the last three R17 axes do
not meet at one point, see axes_intersect probe in LOG). Interface:
ik(target, q_init, position_only) -> (q_deg, success, pos_err_mm,
rot_err_deg). Damped least squares with joint-limit clamping and
random restarts.
"""

from __future__ import annotations

import math
import random

from .fk import fk, mat_vec, skew
from .model import ee_point, joint_arrays

SUCCESS_POS_MM = 5.0
SUCCESS_ROT_DEG = 3.0
N_RESTARTS = 8
MAX_ITERS = 200
DAMPING = 0.05
ROT_WEIGHT = 100.0  # mm-equivalent scale for orientation rows
STEP_MAX = 0.25  # max radians per joint per iteration


def _transpose(a):
    return [list(row) for row in zip(*a)]


def _rot_log(r) -> list[float]:
    """Axis-angle vector (radians) of a rotation matrix."""
    tr = r[0][0] + r[1][1] + r[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-9:
        return [0.0, 0.0, 0.0]
    s = 2.0 * math.sin(th)
    return [(r[2][1] - r[1][2]) / s * th,
            (r[0][2] - r[2][0]) / s * th,
            (r[1][0] - r[0][1]) / s * th]


def _solve(a, b):
    """Dense linear solve by Gaussian elimination (small systems)."""
    n = len(b)
    m = [row[:] + [val] for row, val in zip(a, b)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[piv] = m[piv], m[col]
        if abs(m[col][col]) < 1e-12:
            continue
        pivv = m[col][col]
        for k in range(col, n + 1):
            m[col][k] /= pivv
        for row in range(n):
            if row != col and m[row][col] != 0.0:
                f = m[row][col]
                for k in range(col, n + 1):
                    m[row][k] -= f * m[col][k]
    return [m[i][n] for i in range(n)]


def _chain_frames(q, axes, points):
    """Prefix transforms and world joint frames along the chain."""
    r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    t = [0.0, 0.0, 0.0]
    frames = []
    for qi, w, p in zip(q, axes, points):
        k = skew(w)
        k2 = [[sum(k[i][m] * k[m][j] for m in range(3))
               for j in range(3)] for i in range(3)]
        s, c = math.sin(qi), math.cos(qi)
        dr = [[(1.0 if i == j else 0.0) + s * k[i][j]
               + (1.0 - c) * k2[i][j] for j in range(3)]
              for i in range(3)]
        # Rotation about the line through p: dt = (I - dR)(p).
        pv = [p[i] - sum(dr[i][j] * p[j] for j in range(3))
              for i in range(3)]
        nw = mat_vec(r, w)
        npp = [sum(r[i][j] * p[j] for j in range(3)) + t[i]
               for i in range(3)]
        frames.append((nw, npp))
        # Compose: (R, t) <- (R, t) * (dR, pv).
        t = [sum(r[i][j] * pv[j] for j in range(3)) + t[i]
             for i in range(3)]
        r = [[sum(r[i][j] * dr[j][k] for j in range(3))
              for k in range(3)] for i in range(3)]
    return frames


def _dls_step(q, axes, points, ee, pos_t, rot_t, position_only,
              damping=DAMPING):
    t = fk(q, axes, points, ee)
    pos = [t[0][3], t[1][3], t[2][3]]
    rot = [row[:3] for row in t[:3]]
    ep = [pos_t[i] - pos[i] for i in range(3)]
    if position_only:
        e = ep
        rows = 3
    else:
        # Space-frame error rotation (matches the space Jacobian).
        rt = [[sum(rot_t[i][j] * rot[k][j] for j in range(3))
               for k in range(3)] for i in range(3)]
        er = [x * ROT_WEIGHT for x in _rot_log(rt)]
        e = ep + er
        rows = 6
    frames = _chain_frames(q, axes, points)
    n = len(q)
    jcols = []
    for w, pp in frames:
        col = [w[1] * (pos[2] - pp[2]) - w[2] * (pos[1] - pp[1]),
               w[2] * (pos[0] - pp[0]) - w[0] * (pos[2] - pp[2]),
               w[0] * (pos[1] - pp[1]) - w[1] * (pos[0] - pp[0])]
        if not position_only:
            col += [w[i] * ROT_WEIGHT for i in range(3)]
        jcols.append(col)
    jt = _transpose(jcols)  # rows x n
    # A = J^T J + damping^2 I ; solve A dq = J^T e.
    a = [[sum(jt[k][i] * jt[k][j] for k in range(rows))
          + (damping * damping if i == j else 0.0)
          for j in range(n)] for i in range(n)]
    g = [sum(jt[k][i] * e[k] for k in range(rows)) for i in range(n)]
    dq = _solve(a, g)
    mx = max(abs(x) for x in dq) or 1.0
    if mx > STEP_MAX:
        dq = [x * STEP_MAX / mx for x in dq]
    return [q[i] + dq[i] for i in range(n)], ep, e


def _errors(q, axes, points, ee, pos_t, rot_t):
    t = fk(q, axes, points, ee)
    pos = [t[0][3], t[1][3], t[2][3]]
    rot = [row[:3] for row in t[:3]]
    pe = math.sqrt(sum((pos_t[i] - pos[i]) ** 2 for i in range(3)))
    rt = [[sum(rot[j][i] * rot_t[j][k] for j in range(3))
           for k in range(3)] for i in range(3)]
    lv = _rot_log(rt)
    re = math.degrees(math.sqrt(sum(x * x for x in lv)))
    return pe, re


def ik(target_pos, target_mat, doc, q_init_deg=None,
       position_only=False, n_restarts=N_RESTARTS,
       max_restarts=32, seed=0) -> dict:
    """Solve IK. Target pose: mm position + 3x3 rotation matrix.

    Returns {"q_deg", "success", "pos_err_mm", "rot_err_deg"}.
    Success uses the Gate 2 thresholds (5 mm / 3 deg). Seeds are
    added in batches until success or max_restarts is reached.
    """
    arr = joint_arrays(doc)
    axes, points = arr["axes"], arr["points"]
    ee = ee_point(doc)
    lims = arr["limits"]
    rng = random.Random(seed)
    seeds = []
    if q_init_deg is not None:
        seeds.append([math.radians(x) for x in q_init_deg])
    seeds.append([0.0] * len(axes))
    def run_seed(s, damping, iters):
        q = [min(max(v, lo), hi) for v, (lo, hi) in zip(s, lims)]
        for it in range(iters):
            q, ep, _ = _dls_step(q, axes, points, ee, target_pos,
                                 target_mat, position_only,
                                 damping=damping)
            q = [min(max(v, lo), hi)
                 for v, (lo, hi) in zip(q, lims)]
            if it % 10 == 9 or sum(x * x for x in ep) < 1e-6:
                pe, re = _errors(q, axes, points, ee, target_pos,
                                 target_mat)
                if pe < 0.1 and (position_only or re < 0.1):
                    break
        pe, re = _errors(q, axes, points, ee, target_pos, target_mat)
        return ((pe, re), q, pe, re)

    def successful(cand) -> bool:
        _, _, pe, re = cand
        return pe <= SUCCESS_POS_MM and (position_only or
                                         re <= SUCCESS_ROT_DEG)

    best = None
    randoms_used = 0
    pending = list(seeds)
    while True:
        take = min(n_restarts, max_restarts - randoms_used)
        if take > 0:
            pending += [[rng.uniform(lo, hi) for lo, hi in lims]
                        for _ in range(take)]
            randoms_used += take
        for s in pending:
            cand = run_seed(s, DAMPING, MAX_ITERS)
            if best is None or cand[0] < best[0]:
                best = cand
        pending = []
        if best is not None and successful(best):
            break
        if randoms_used >= max_restarts:
            break
    if not position_only and best is not None:
        # Refinement: perturb the winner with low damping to
        # escape orientation local minima near the target.
        wq = best[1]
        for k in range(4):
            pert = [v + rng.uniform(-0.05, 0.05) for v in wq]
            pert = [min(max(v, lo), hi)
                    for v, (lo, hi) in zip(pert, lims)]
            cand = run_seed(pert, DAMPING / 3.0, MAX_ITERS)
            if cand[0] < best[0]:
                best = cand
    (_, q, pe, re) = best
    ok = pe <= SUCCESS_POS_MM and (position_only or re <= SUCCESS_ROT_DEG)
    return {"q_deg": [math.degrees(x) for x in q], "success": bool(ok),
            "pos_err_mm": pe, "rot_err_deg": re}
