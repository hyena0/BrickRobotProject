"""Viz rotation helper tests (Stage 2.8).

Covers src.viz.rotutil.orthonormalize_rotation: the Blender object euler
decomposition amplifies Studio rounding noise (det-1 ~ 1e-4) into
degree-scale spins at gimbal-singular poses, so the build script projects
every part rotation to SO(3) before writing it.
"""

from __future__ import annotations

import math

from src.viz.rotutil import orthonormalize_rotation


def _cols(m):
    return [[m[r][c] for r in range(3)] for c in range(3)]


def _angle(a, b):
    tr = sum(a[r][c] * b[r][c] for r in range(3) for c in range(3))
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    return math.degrees(math.acos(c))


# Real p000506 rotation: Y ~ -90 deg singular pose, det-1 ~ 8e-5.
SINGULAR = [[-3.2e-05, -0.70403, 0.710204],
            [-3.3e-05, -0.710253, -0.704049],
            [0.999987, -4.5e-05, 1e-06]]


def test_output_is_orthonormal_with_det_plus_one():
    out = orthonormalize_rotation(SINGULAR)
    cx, cy, cz = _cols(out)
    for c in (cx, cy, cz):
        assert math.isclose(math.sqrt(sum(v * v for v in c)),
                            1.0, abs_tol=1e-12)
    assert math.isclose(sum(a * b for a, b in zip(cx, cy)),
                        0.0, abs_tol=1e-12)
    assert math.isclose(sum(a * b for a, b in zip(cx, cz)),
                        0.0, abs_tol=1e-12)
    assert math.isclose(sum(a * b for a, b in zip(cy, cz)),
                        0.0, abs_tol=1e-12)
    det = (out[0][0] * (out[1][1] * out[2][2] - out[1][2] * out[2][1])
           - out[0][1] * (out[1][0] * out[2][2] - out[1][2] * out[2][0])
           + out[0][2] * (out[1][0] * out[2][1] - out[1][1] * out[2][0]))
    assert math.isclose(det, 1.0, abs_tol=1e-12)


def test_correction_is_tiny_for_singular_pose():
    # The projection must move the input by ~1e-4 rad, not degrees.
    out = orthonormalize_rotation(SINGULAR)
    assert _angle(SINGULAR, out) < 0.01


def test_identity_stays_identity():
    ident = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    out = orthonormalize_rotation(ident)
    assert _angle(ident, out) < 1e-9
