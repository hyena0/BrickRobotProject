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
STEP_MAX_PRISM_MM = 5.0  # max mm per prismatic joint per iteration


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


def _chain_frames(q, axes, points, kinds=None):
    """Prefix transforms and world joint frames along the chain.

    Prismatic joints translate downstream along their (rotated) axis
    and keep orientation; their frame column is the direction itself.
    """
    r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    t = [0.0, 0.0, 0.0]
    frames = []
    kinds = kinds or ["revolute"] * len(q)
    for qi, w, p, kind in zip(q, axes, points, kinds):
        if kind == "prismatic":
            nw = mat_vec(r, w)
            npp = [sum(r[i][j] * p[j] for j in range(3)) + t[i]
                   for i in range(3)]
            frames.append((nw, npp, kind))
            t = [t[i] + sum(r[i][j] * w[j] for j in range(3)) * qi
                 for i in range(3)]
            continue
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
        frames.append((nw, npp, "revolute"))
        # Compose: (R, t) <- (R, t) * (dR, pv).
        t = [sum(r[i][j] * pv[j] for j in range(3)) + t[i]
             for i in range(3)]
        r = [[sum(r[i][j] * dr[j][k] for j in range(3))
              for k in range(3)] for i in range(3)]
    return frames


def _dls_step(q, axes, points, ee, pos_t, rot_t, position_only,
              damping=DAMPING, kinds=None):
    kinds = kinds or ["revolute"] * len(q)
    t = fk(q, axes, points, ee, kinds)
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
    frames = _chain_frames(q, axes, points, kinds)
    n = len(q)
    jcols = []
    for w, pp, kind in frames:
        if kind == "prismatic":
            col = list(w)
            if not position_only:
                col += [0.0, 0.0, 0.0]
        else:
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
    caps = [STEP_MAX_PRISM_MM if k == "prismatic" else STEP_MAX
            for k in kinds]
    mx = max(abs(x) / c for x, c in zip(dq, caps)) or 1.0
    if mx > 1.0:
        dq = [x / mx for x in dq]
    return [q[i] + dq[i] for i in range(n)], ep, e


def _errors(q, axes, points, ee, pos_t, rot_t, kinds=None):
    t = fk(q, axes, points, ee, kinds)
    pos = [t[0][3], t[1][3], t[2][3]]
    rot = [row[:3] for row in t[:3]]
    pe = math.sqrt(sum((pos_t[i] - pos[i]) ** 2 for i in range(3)))
    if rot_t is None:
        return pe, 0.0
    rt = [[sum(rot[j][i] * rot_t[j][k] for j in range(3))
           for k in range(3)] for i in range(3)]
    lv = _rot_log(rt)
    re = math.degrees(math.sqrt(sum(x * x for x in lv)))
    return pe, re


def _coupled_ok(q_deg, joint_ids, coupled, lookup=None) -> bool:
    """Stage 5A box-external check: linear inequalities hold.

    Lookup tables stay implemented but switched off
    (USE_COUPLED_LOOKUP): only limits_deg and coupled_limits gate.
    """
    from src.limits.enforce import USE_COUPLED_LOOKUP, check_lookup
    pos = {jid: v for jid, v in zip(joint_ids, q_deg)}
    for d in coupled or []:
        ja, jb = d["joints"]
        val = d["a"][0] * pos[ja] + d["a"][1] * pos[jb] + d["b"]
        if d["op"] == ">=" and val < -1e-9:
            return False
        if d["op"] == "<=" and val > 1e-9:
            return False
    if USE_COUPLED_LOOKUP and lookup and not check_lookup(
            q_deg, joint_ids, lookup):
        return False
    return True


def ik(target_pos, target_mat, doc, q_init_deg=None,
       position_only=False, n_restarts=N_RESTARTS,
       max_restarts=32, seed=0, limits_doc=None,
       extra_starts=None) -> dict:
    """Solve IK. Target pose: mm position + 3x3 rotation matrix.

    Returns {"q", "q_deg", "success", "pos_err_mm", "rot_err_deg"}.
    "q" is native units (degrees revolute, mm prismatic); "q_deg"
    matches "q" for revolute-only docs (backward compatible).
    Success uses the Gate 2 thresholds (5 mm / 3 deg). Seeds are
    added in batches until success or max_restarts is reached.

    With limits_doc (Stage 5A runs/stage5a/limits.json shape:
    limits_deg per joint plus coupled_limits/coupled_lookup),
    seeds are drawn inside the operating ranges and
    coupled-violating solutions are rejected, so success implies
    a limit-safe pose. Prismatic joints read limits_mm instead.
    """
    arr = joint_arrays(doc)
    axes, points = arr["axes"], arr["points"]
    kinds = arr.get("kinds", ["revolute"] * len(axes))
    ee = ee_point(doc)
    joint_ids = arr["ids"]
    lims = arr["limits"]
    coupled = None
    lookup = None
    if limits_doc:
        lims = []
        for jid, kind in zip(joint_ids, kinds):
            key = "limits_mm" if kind == "prismatic" else "limits_deg"
            lo, hi = limits_doc.get(key, {}).get(jid, [-180.0, 180.0])
            lims.append([float(lo), float(hi)] if kind == "prismatic"
                        else [math.radians(lo), math.radians(hi)])
        coupled = limits_doc.get("coupled_limits", [])
        lookup = limits_doc.get("coupled_lookup", {})
    rng = random.Random(seed)
    # Position-only solving cannot move joints whose axis passes
    # through the TCP (zero position Jacobian column, e.g. J6 when
    # the TCP sits on its axis): they are free variables that only
    # add random spins from restart seeds. Freeze them at the
    # incoming pose so a gizmo drag moves the arm, not the wrist.
    # Prismatic columns never vanish (unit direction), so only
    # revolute joints are considered for locking.
    locked = []
    if position_only:
        for i, (w, p, kind) in enumerate(zip(axes, points, kinds)):
            if kind == "prismatic":
                continue
            lx, ly, lz = (ee[0] - p[0], ee[1] - p[1], ee[2] - p[2])
            cx, cy, cz = (w[1] * lz - w[2] * ly,
                          w[2] * lx - w[0] * lz,
                          w[0] * ly - w[1] * lx)
            if math.sqrt(cx * cx + cy * cy + cz * cz) < 1.0:
                locked.append(i)
    seeds = []
    if q_init_deg is not None:
        seeds.append([x if k == "prismatic" else math.radians(x)
                      for x, k in zip(q_init_deg, kinds)])
    # Multi-start: extra candidate poses (safe poses, previous
    # solution) run through the same DLS; the best (min TCP error)
    # wins below. Deterministic order, no threads (GIL-bound).
    for extra in extra_starts or []:
        try:
            seeds.append([float(x) if k == "prismatic"
                          else math.radians(float(x))
                          for x, k in zip(extra, kinds)])
        except (TypeError, ValueError):
            continue
    seeds.append([0.0] * len(axes))
    # Native-unit noise scales (radians revolute, mm prismatic).
    near = [20.0 if k == "prismatic" else 0.35 for k in kinds]
    kick_amp = [30.0 if k == "prismatic" else 0.5 for k in kinds]
    fine_amp = [3.0 if k == "prismatic" else 0.05 for k in kinds]

    def run_seed(s, damping, iters):
        q = [min(max(v, lo), hi) for v, (lo, hi) in zip(s, lims)]
        # Locked joints stay at their own seed value (each start
        # keeps its wrist), not at a global first-seed value.
        seed_hold = [q[i] for i in locked]
        for it in range(iters):
            q, ep, _ = _dls_step(q, axes, points, ee, target_pos,
                                 target_mat, position_only,
                                 damping=damping, kinds=kinds)
            q = [min(max(v, lo), hi)
                 for v, (lo, hi) in zip(q, lims)]
            for k, i in enumerate(locked):
                q[i] = seed_hold[k]
            if it % 10 == 9 or sum(x * x for x in ep) < 1e-6:
                pe, re = _errors(q, axes, points, ee, target_pos,
                                 target_mat, kinds)
                if pe < 0.1 and (position_only or re < 0.1):
                    break
        pe, re = _errors(q, axes, points, ee, target_pos, target_mat,
                         kinds)
        return ((pe, re), q, pe, re)

    def successful(cand) -> bool:
        _, q, pe, re = cand
        if not (pe <= SUCCESS_POS_MM and (position_only or
                                          re <= SUCCESS_ROT_DEG)):
            return False
        if (coupled or lookup) and not _coupled_ok(
                [math.degrees(v) for v in q], joint_ids, coupled,
                lookup):
            return False
        return True

    best = None
    randoms_used = 0
    pending = list(seeds)
    while True:
        take = min(n_restarts, max_restarts - randoms_used)
        if take > 0:
            pending += [[rng.uniform(lo, hi) for lo, hi in lims]
                        for _ in range(take)]
            randoms_used += take
            # Gizmo-realistic seeds: the drag starts from the current
            # pose, so also try its neighborhood (deterministic).
            if q_init_deg is not None:
                base = [x if k == "prismatic" else math.radians(x)
                        for x, k in zip(q_init_deg, kinds)]
                for _ in range(min(4, take)):
                    pending.append(
                        [min(max(b + rng.uniform(-a, a), lo), hi)
                         for b, (lo, hi), a in zip(base, lims, near)])
        for s in pending:
            cand = run_seed(s, DAMPING, MAX_ITERS)
            if best is None or cand[0] < best[0]:
                best = cand
        pending = []
        if best is not None and successful(best):
            break
        if randoms_used >= max_restarts:
            break
    if best is not None and not successful(best):
        # Basin hopping: the DLS sticks at joint limits while a
        # better basin exists. Kick the winner hard a few times,
        # keep the best. Applies to position and orientation both;
        # locked joints keep their own seed value inside run_seed.
        for _ in range(6):
            kick = [v + rng.uniform(-a, a)
                    for v, a in zip(best[1], kick_amp)]
            kick = [min(max(v, lo), hi)
                    for v, (lo, hi) in zip(kick, lims)]
            for i in locked:
                kick[i] = best[1][i]
            cand = run_seed(kick, DAMPING, MAX_ITERS)
            if cand[0] < best[0]:
                best = cand
            if successful(best):
                break
    if not position_only and best is not None:
        # Refinement: perturb the winner with low damping to
        # escape orientation local minima near the target.
        wq = best[1]
        for k in range(4):
            pert = [v + rng.uniform(-a, a)
                    for v, a in zip(wq, fine_amp)]
            pert = [min(max(v, lo), hi)
                    for v, (lo, hi) in zip(pert, lims)]
            cand = run_seed(pert, DAMPING / 3.0, MAX_ITERS)
            if cand[0] < best[0]:
                best = cand
    (_, q, pe, re) = best
    ok = pe <= SUCCESS_POS_MM and (position_only or re <= SUCCESS_ROT_DEG)
    q_out = [x if k == "prismatic" else math.degrees(x)
             for x, k in zip(q, kinds)]
    coupled_ok = _coupled_ok(q_out, joint_ids, coupled, lookup)
    if (coupled or lookup) and not coupled_ok:
        ok = False
    return {"q": q_out, "q_deg": q_out, "success": bool(ok),
            "pos_err_mm": pe, "rot_err_deg": re,
            "coupled_ok": bool(coupled_ok)}


def gantry_closed_form(target_pos: list[float], doc: dict) -> dict:
    """Closed-form IK for all-prismatic (gantry) docs.

    With no revolute joints the EE moves as ee0 + sum(axis_i * q_i);
    orthonormal axes give q_i = dot(target - ee0, axis_i) directly.
    Returns the same keys as ik() (native mm units). Raises for
    mixed/revolute docs or non-orthonormal axes.
    """
    arr = joint_arrays(doc)
    kinds = arr.get("kinds", [])
    if not kinds or any(k != "prismatic" for k in kinds):
        raise ValueError("gantry_closed_form needs all-prismatic docs")
    axes = arr["axes"]
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            dot = sum(a * b for a, b in zip(axes[i], axes[j]))
            if abs(dot) > 1e-6:
                raise ValueError("gantry axes must be orthonormal")
    ee0 = ee_point(doc)
    q = [sum((target_pos[k] - ee0[k]) * ax[k] for k in range(3))
         for ax in axes]
    lims = arr["limits"]
    inside = all(lo - 1e-9 <= v <= hi + 1e-9
                 for v, (lo, hi) in zip(q, lims))
    qc = [min(max(v, lo), hi) for v, (lo, hi) in zip(q, lims)]
    t = fk(qc, axes, arr["points"], ee0,
           ["prismatic"] * len(axes))
    pos = [t[0][3], t[1][3], t[2][3]]
    pe = math.sqrt(sum((target_pos[i] - pos[i]) ** 2 for i in range(3)))
    return {"q": qc, "q_deg": qc, "success": bool(inside and pe <= 5.0),
            "pos_err_mm": pe, "rot_err_deg": 0.0, "coupled_ok": True}
