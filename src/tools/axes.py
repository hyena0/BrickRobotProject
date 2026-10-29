"""Common-axis intersection test (Stage 4.3).

Checks whether a set of revolute joint axes meets at one point
(spherical-wrist condition for closed-form IK). Least-squares fit:
minimize the sum of squared perpendicular distances from a center
candidate to each axis line. Parallel or skew axes give a large
residual and report no intersection.
"""

from __future__ import annotations

import math


def _norm(v: list[float]) -> list[float]:
    n = math.sqrt(sum(x * x for x in v))
    if n == 0.0:
        raise ValueError("zero axis vector")
    return [x / n for x in v]


def axes_intersect(axes: list[list[float]],
                   points: list[list[float]],
                   tol_mm: float = 2.0) -> dict:
    """Fit one common point to N axis lines.

    Returns {"center": [x, y, z], "residuals_mm": [...],
    "max_residual_mm": float, "intersect": bool}. Residuals are
    perpendicular distances from the fitted center to each line.
    """
    if len(axes) != len(points) or not axes:
        raise ValueError("axes and points need the same nonzero length")
    units = [_norm(a) for a in axes]
    # Solve (sum(I - w w^T)) c = sum((I - w w^T) p) by Gaussian
    # elimination with partial pivoting (3x3, may be singular).
    a = [[0.0] * 3 for _ in range(3)]
    b = [0.0] * 3
    for w, p in zip(units, points):
        for i in range(3):
            for j in range(3):
                m = (1.0 if i == j else 0.0) - w[i] * w[j]
                a[i][j] += m
            b[i] += sum(((1.0 if i == j else 0.0) - w[i] * w[j])
                             * p[j] for j in range(3))
    center = _solve_3x3(a, b)
    residuals = []
    for w, p in zip(units, points):
        d = [center[i] - p[i] for i in range(3)]
        proj = sum(d[i] * w[i] for i in range(3))
        perp = [d[i] - proj * w[i] for i in range(3)]
        residuals.append(math.sqrt(sum(x * x for x in perp)))
    worst = max(residuals)
    return {"center": center, "residuals_mm": residuals,
            "max_residual_mm": worst,
            "intersect": worst <= tol_mm}


def _solve_3x3(a: list[list[float]], b: list[float]) -> list[float]:
    m = [row[:] + [val] for row, val in zip(a, b)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        m[col], m[piv] = m[piv], m[col]
        if abs(m[col][col]) < 1e-12:
            continue
        for row in range(3):
            if row != col and m[row][col] != 0.0:
                f = m[row][col] / m[col][col]
                for k in range(col, 4):
                    m[row][k] -= f * m[col][k]
    return [(m[i][3] / m[i][i] if abs(m[i][i]) > 1e-12 else 0.0)
            for i in range(3)]
