"""LDraw I/O for the 5D generator (Stage 5D.4).

Reads module geometry through ``src.parse`` (mm, Z up) and writes
assembled robots as flat LDraw type-1 lines. The writer is the exact
inverse of the ``docs/schema.md`` conversion, then snapped to integer
LDU (the stud grid).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from src.parse import parse_file
from src.parse.ldr import LDU_TO_MM

# Point remap C: LDraw (x, y, z) -> mm/Z-up (x, z, -y), up to scale.
_C = np.array([[1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0],
               [0.0, -1.0, 0.0]])
_C_INV = _C.T


def mm_to_ldu_point(position_mm) -> list[float]:
    """Inverse of the schema.md point map (mm Z-up -> LDU)."""
    x, y, z = (float(v) for v in position_mm)
    return [x / LDU_TO_MM, -z / LDU_TO_MM, y / LDU_TO_MM]


def rotation_to_ldraw(rotation) -> list[float]:
    """Inverse of the schema.md rotation map: L = C^-1 @ R @ C."""
    r = np.asarray(rotation, dtype=float)
    mat = _C_INV @ r @ _C
    return [float(v) for v in mat.reshape(-1)]


def _snap_number(value: float, tol: float = 1e-6) -> int | float:
    rounded = round(value)
    if abs(value - rounded) <= tol:
        return rounded
    return value


def _fmt_number(value: int | float) -> str:
    if isinstance(value, float):
        text = f"{value:.6f}".rstrip("0").rstrip(".")
        return text if text not in ("-0", "") else "0"
    return str(value)


def format_type1(part: dict, colour: str | None = None) -> tuple[str, float]:
    """Format one part as a type-1 line. Returns (line, snap_mm)."""
    ldu = mm_to_ldu_point(part["position_mm"])
    snapped = [int(round(v)) for v in ldu]
    snap_mm = max(abs(a - b) for a, b in zip(ldu, snapped)) * LDU_TO_MM
    nums = [_snap_number(v) for v in rotation_to_ldraw(part["rotation"])]
    fields = [str(colour or part.get("colour") or 16)]
    fields += [_fmt_number(v) for v in snapped]
    fields += [_fmt_number(v) for v in nums]
    fields.append(part["part_id"])
    return "1 " + " ".join(fields), snap_mm


def read_module_geometry(module_dir: str | Path, module: dict) -> list[dict]:
    """Parse a module's LDraw file (module-local mm, Z up)."""
    return parse_file(str(Path(module_dir) / module["geometry"]))


def write_robot_ldraw(parts: list[dict], path: str | Path,
                      header: str = "5D generated robot") -> dict:
    """Write flat type-1 lines. Returns {n_parts, max_snap_mm}."""
    lines = [f"0 {header}", "0 Name: robot.ldr",
             "0 Author: 5D generator"]
    max_snap = 0.0
    for part in parts:
        line, snap_mm = format_type1(part)
        lines.append(line)
        max_snap = max(max_snap, snap_mm)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"n_parts": len(parts), "max_snap_mm": max_snap}
