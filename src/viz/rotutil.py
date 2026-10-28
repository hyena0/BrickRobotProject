"""Rotation helpers shared by viz scripts (Stage 2.8).

Pure Python (no bpy dependency) so unit tests can import this module.
"""

from __future__ import annotations

import math


def orthonormalize_rotation(rot):
    """Project a near-rotation 3x3 (row-major nested lists) to SO(3).

    Studio rounds .io matrices (det-1 ~ 1e-4). At gimbal-singular poses
    (Y ~ -90 deg) Blender's object euler decomposition amplifies that
    defect into degree-scale spins on save/reload. Gram-Schmidt on the
    columns keeps the correction ~1e-4 rad and makes the stored euler
    round-trip exactly.
    """
    x = [rot[0][0], rot[1][0], rot[2][0]]
    y = [rot[0][1], rot[1][1], rot[2][1]]
    nx = math.sqrt(sum(v * v for v in x))
    xn = [v / nx for v in x]
    d = sum(a * b for a, b in zip(y, xn))
    y = [a - d * b for a, b in zip(y, xn)]
    ny = math.sqrt(sum(v * v for v in y))
    yn = [v / ny for v in y]
    z = [xn[1] * yn[2] - xn[2] * yn[1],
         xn[2] * yn[0] - xn[0] * yn[2],
         xn[0] * yn[1] - xn[1] * yn[0]]
    return ((xn[0], yn[0], z[0]),
            (xn[1], yn[1], z[1]),
            (xn[2], yn[2], z[2]))
