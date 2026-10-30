"""Mesh-measured part bounds from the Studio library (Stage 5D).

Wraps the public catalog mesh API (``src.catalog.review.Library``,
``src.catalog.mesh.type_vertices``) to answer size questions the
hole catalog cannot: motor casing extents, beam end margins past the
last hole, pin lengths. All bounds are honest mesh measurements, not
assumptions; parts whose mesh is missing report ``None`` so callers
fail loudly instead of guessing.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np

STUDIO_LDRAW = "/Applications/Studio 2.0/ldraw"
LDU_TO_MM = 0.4


@lru_cache(maxsize=256)
def _library(root: str, part_id: str):
    from src.catalog.review import Library

    return Library.from_dir(root, [part_id])


@lru_cache(maxsize=1024)
def _box_cached(root: str, part_id: str):
    from src.catalog.mesh import type_vertices

    lib = _library(root, part_id)
    try:
        verts = np.asarray(type_vertices(lib, part_id), dtype=float)
    except Exception:
        return None
    if verts.shape[0] == 0:
        return None
    return verts


def local_box_mm(part_id: str, root: str = STUDIO_LDRAW,
                 ) -> tuple[list[float], list[float]] | None:
    """Converted-local mm AABB ``(lo, hi)`` of a part mesh, or None.

    Type-local LDraw LDU vertices are remapped with the parser point
    map ``(x, y, z) -> (x, z, -y)`` into the converted local frame
    (mm, Z up) shared by parsed parts.
    """
    verts = _box_cached(root, part_id.lower())
    if verts is None:
        return None
    conv = np.stack([verts[:, 0], verts[:, 2], -verts[:, 1]],
                    axis=1) * LDU_TO_MM
    lo, hi = conv.min(axis=0), conv.max(axis=0)
    return ([float(v) for v in lo], [float(v) for v in hi])


def world_box_mm(part: dict, root: str = STUDIO_LDRAW,
                 ) -> tuple[list[float], list[float]] | None:
    """World-frame mm AABB of a placed part, or None without mesh."""
    box = local_box_mm(part["part_id"], root)
    if box is None:
        return None
    lo, hi = (np.asarray(v, dtype=float) for v in box)
    corners = np.stack(np.meshgrid(*zip(lo, hi)), axis=-1).reshape(-1, 3)
    world = (np.asarray(part["position_mm"], dtype=float)
             + corners @ np.asarray(part["rotation"], dtype=float).T)
    wlo, whi = world.min(axis=0), world.max(axis=0)
    return ([float(v) for v in wlo], [float(v) for v in whi])


def library_present(root: str = STUDIO_LDRAW) -> bool:
    return (Path(root) / "parts").is_dir()
