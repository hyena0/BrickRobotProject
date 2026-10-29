"""Joint/motor-space conversion (Stage 4.4).

q = C m (degrees both sides) plus human-provided zero offsets:
m_cmd = m + offset. Joint limits are checked in joint space before
conversion. Zero offsets default to 0; a human fills
config/zero.local.yaml (git-ignored, motor uid -> offset degrees).
"""

from __future__ import annotations

import math


def load_zeros(path: str | None) -> dict:
    """Load motor zero offsets; missing file means all zeros."""
    if not path:
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
    except OSError:
        return {}
    zeros = doc.get("zeros", doc)
    return {k: float(v) for k, v in zeros.items()
            if isinstance(v, (int, float))}


def check_limits(q_deg: list[float],
                 limits_deg: list[list[float]]) -> list[str]:
    """Joint-space limit violations before conversion."""
    bad = []
    for i, (v, (lo, hi)) in enumerate(zip(q_deg, limits_deg)):
        if not (lo - 1e-9 <= v <= hi + 1e-9):
            bad.append(f"joint{i}: {v:.3f}deg outside "
                       f"[{lo:.1f}, {hi:.1f}]")
    return bad


def _lstsq(a: list[list[float]],
           b: list[float]) -> tuple[list[float], float]:
    """Least-squares solve via normal equations (small systems)."""
    n = len(a[0])
    ata = [[sum(a[k][i] * a[k][j] for k in range(len(a)))
            for j in range(n)] for i in range(n)]
    atb = [sum(a[k][i] * b[k] for k in range(len(a)))
           for i in range(n)]
    # Gaussian elimination with partial pivoting.
    m = [row[:] + [val] for row, val in zip(ata, atb)]
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
    x = [m[i][n] for i in range(n)]
    res = math.sqrt(sum((sum(a[k][j] * x[j] for j in range(n))
                             - b[k]) ** 2 for k in range(len(a))))
    return x, res


def joints_to_motors(q_deg: list[float], c: list[list[float]],
                     motor_ids: list[str],
                     limits_deg: list[list[float]],
                     zeros: dict | None = None) -> dict:
    """Joint angles -> motor commands. Raises on limit violation."""
    bad = check_limits(q_deg, limits_deg)
    if bad:
        raise ValueError("joint limit violation: " + "; ".join(bad))
    m, residual = _lstsq(c, list(q_deg))
    zeros = zeros or {}
    cmds = {mid: m[i] + float(zeros.get(mid, 0.0))
            for i, mid in enumerate(motor_ids)}
    return {"motors_deg": cmds, "coupling_residual_deg": residual}


def motors_to_joints(motors_deg: dict, c: list[list[float]],
                     motor_ids: list[str],
                     zeros: dict | None = None) -> list[float]:
    """Motor commands -> joint angles (forward, offsets removed)."""
    zeros = zeros or {}
    m = [float(motors_deg[mid]) - float(zeros.get(mid, 0.0))
         for mid in motor_ids]
    return [sum(c[i][j] * m[j] for j in range(len(m)))
            for i in range(len(c))]
