"""LDraw mesh tessellation (pure Python, no bpy).

Shared by viz (metres) and Tier-1 joint inference (LDU annulus fits).
Moved verbatim from src/viz/build_blend.py; behaviour identical at
the default scale.
"""

from __future__ import annotations

import numpy as np


def collect_faces(lib, name, matrix, offset, out_verts, out_faces,
                  depth=0, seen=frozenset(), xform=None):
    """Append transformed triangles/quads of a part into the buffers.

    `xform` maps an LDraw (x, y, z) vertex to output units; None keeps
    LDraw units.
    """
    key = name.lower()
    if depth > 8 or key in seen:
        return
    rng = lib.blocks.get(key)
    if not rng:
        return
    seen = seen | {key}
    lines = lib.lines
    for line in lines[rng[0] + 1:rng[1]]:
        s = line.strip()
        if s.startswith("0 FILE") or s.startswith("0 NOFILE"):
            break
        if s.startswith("0 "):
            continue
        t = s.split()
        if not t or t[0] not in ("1", "3", "4") or len(t) < 8:
            continue
        if t[0] == "1" and len(t) != 15:
            continue
        try:
            # t[1] is the colour (skipped); the part filename, if any,
            # is the last token (skipped by the t[2:14] slice).
            nums = [float(x) for x in t[2:14]]
        except (ValueError, IndexError):
            continue
        if t[0] == "1":
            pos = nums[0:3]
            rot = [nums[3:6], nums[6:9], nums[9:12]]
            child_m = [[sum(matrix[i][k] * rot[k][j] for k in range(3))
                        for j in range(3)] for i in range(3)]
            child_o = [offset[i] + sum(matrix[i][k] * pos[k]
                                       for k in range(3))
                       for i in range(3)]
            collect_faces(lib, t[14], child_m, child_o,
                          out_verts, out_faces, depth + 1, seen,
                          xform=xform)
        else:
            n = 3 if t[0] == "3" else 4
            if len(nums) < n * 3:
                continue
            idx = []
            for j in range(n):
                v = nums[j * 3:j * 3 + 3]
                w = [offset[i] + sum(matrix[i][k] * v[k]
                                     for k in range(3))
                     for i in range(3)]
                if xform is not None:
                    w = xform(*w)
                key_v = (round(w[0], 6), round(w[1], 6), round(w[2], 6))
                if key_v not in out_verts:
                    out_verts[key_v] = len(out_verts)
                idx.append(out_verts[key_v])
            out_faces.append(tuple(idx))


def type_vertices(lib, pid, xform=None) -> np.ndarray:
    """Vertex soup of one part type (type-local frame) as Nx3 array."""
    verts: dict = {}
    faces: list = []
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    collect_faces(lib, pid, ident, [0, 0, 0], verts, faces, xform=xform)
    return np.array(sorted(verts, key=verts.get), dtype=float)


def type_tris(lib, pid) -> tuple[np.ndarray, list]:
    """Triangulated mesh of one part type (type-local LDraw frame).

    Returns (verts Nx3 array, tris list of 3-tuples); quads split as
    (0, 1, 2) + (0, 2, 3). Deterministic vertex order.
    """
    verts: dict = {}
    faces: list = []
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    collect_faces(lib, pid, ident, [0, 0, 0], verts, faces)
    varr = np.array(sorted(verts, key=verts.get), dtype=float)
    tris = []
    for f in faces:
        if len(f) == 3:
            tris.append((f[0], f[1], f[2]))
        elif len(f) == 4:
            tris.append((f[0], f[1], f[2]))
            tris.append((f[0], f[2], f[3]))
    return varr, tris
