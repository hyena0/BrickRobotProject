"""Mount-hole interface core (Stage 5D.1/5D.4 shared).

Canonical hole model for beams: catalog ``holes`` mix tunnel centres
``(0, 0, z)`` with face mouths (``y = +-5`` / ``+-10``). Pins and
axles travel the tunnel, so every interface computation here uses
hole CENTRES ``(0, 0, z)`` (one per distinct ``z`` with ``x == 0``,
exact integers), never mouths. Centre kinds come from their mouths
(``axle`` wins when mixed: the cross bore governs the fit).

Two conventions are fixed here and shared by the template emitter,
the module validator and the assembler:

* mount pins run along the mount frame +X (transverse to the chain,
  whose +Z points base-to-tip);
* beam hole tunnels pierce the converted-local Z axis (proved per
  part from mouth offsets when mouths exist; the documented default
  otherwise — every sampled beam agrees, and the final renders show
  pins threading holes as a backstop).
"""

from __future__ import annotations

import math

LDU_MM = 0.4
GRID_LDU = 10.0  # half-grid all mount holes must sit on
MATCH_LDU = 0.1  # hole coincidence tolerance


def conv_ldu_to_mm(point_ldu) -> list[float]:
    """Catalog LDU into converted-local mm (mirrors the parser map)."""
    x, y, z = (float(v) for v in point_ldu)
    return [x * LDU_MM, z * LDU_MM, -y * LDU_MM]


def hole_centres(entry: dict) -> list[dict]:
    """Hole tunnel centres of a part (catalog LDU, exact integers).

    Returns ``[{z, kinds}]`` sorted by ``z``: one entry per distinct
    ``z`` among holes with ``x == 0``. ``kinds`` is the sorted set of
    catalog kinds seen at that ``z``.
    """
    by_z: dict[float, set[str]] = {}
    for hole in entry.get("holes") or []:
        if not isinstance(hole, dict):
            continue
        pos = hole.get("pos") or [0, 0, 0]
        if float(pos[0]) != 0.0:
            continue
        by_z.setdefault(float(pos[2]), set()).add(
            hole.get("kind", "round"))
    return [{"z": z, "kinds": sorted(by_z[z])} for z in sorted(by_z)]


def centre_kind(centre: dict) -> str:
    """Fitting kind of a centre (``axle`` wins when mixed)."""
    kinds = centre.get("kinds") or ["round"]
    return "axle" if "axle" in kinds else kinds[0]


def hole_dir_converted(entry: dict) -> tuple[int, int, int]:
    """Tunnel direction in the converted local frame.

    Derived from mouth offsets when the part has mouths (offset axis
    in the catalog frame, remapped); the documented +Z default when
    only centres are catalogued. Raises when mouths disagree.
    """
    centres = {c["z"] for c in hole_centres(entry)}
    axes = set()
    for hole in entry.get("holes") or []:
        if not isinstance(hole, dict):
            continue
        pos = [float(v) for v in (hole.get("pos") or [0, 0, 0])]
        if pos[0] != 0.0 or pos[2] not in centres:
            continue
        if pos[1] == 0.0:
            continue
        axes.add(1)  # catalog Y offset
        if pos[0] != 0.0:  # unreachable (x filtered) but explicit
            axes.add(0)
    if not axes:
        return (0, 0, 1)  # documented default (verified pattern)
    if axes != {1}:
        raise ValueError(f"mouth offsets disagree: {sorted(axes)}")
    # Catalog +Y remaps to converted -Z; direction is sign-free here.
    return (0, 0, 1)


def row_axis_converted(entry: dict) -> int:
    """Long-spread axis (0/1/2) of hole centres in converted mm."""
    pts = [conv_ldu_to_mm([0, 0, c["z"]]) for c in hole_centres(entry)]
    if len(pts) < 2:
        raise ValueError("need >= 2 hole centres for a row axis")
    spread = [max(p[ax] for p in pts) - min(p[ax] for p in pts)
              for ax in range(3)]
    return int(max(range(3), key=lambda ax: spread[ax]))


def list_axial_beams(catalog: dict,
                     inventory: dict | None = None) -> list[dict]:
    """Straight beams with hole-centre runs (stock-aware, sorted).

    Returns ``[{part_id, runs, max_span_mm}]`` for beam parts with
    >= 2 centres, sorted by (max span, part_id). Runs are straight
    by construction (consecutive centres along the row); bent beam
    ends are gated later by mesh-exact BVH, not here.
    """
    stocked = None
    if inventory is not None:
        stocked = {pid: row.get("total", 0)
                   for pid, row in (inventory.get("parts") or {}).items()}
    out = []
    for part_id in sorted(catalog):
        entry = catalog[part_id] or {}
        if entry.get("category") != "beam":
            continue
        if len(hole_centres(entry)) < 2:
            continue
        if stocked is not None and stocked.get(part_id, 0) <= 0:
            continue
        runs = usable_runs(entry)
        out.append({"part_id": part_id, "runs": runs,
                    "max_span_mm": max(r["span_mm"] for r in runs)})
    out.sort(key=lambda b: (b["max_span_mm"], b["part_id"]))
    return out


def usable_runs(entry: dict) -> list[dict]:
    """Maximal consecutive-8mm centre runs (part-local converted mm).

    Returns ``[{start_mm, span_mm, n, kinds}]`` sorted by start: each
    run is a maximal run of centres spaced exactly 8 mm. ``kinds``
    lists the fitting kind per centre in run order.
    """
    centres = sorted(hole_centres(entry), key=lambda c: c["z"])
    runs: list[dict] = []
    cur: list[dict] = []
    for centre in centres:
        if cur and abs((centre["z"] - cur[-1]["z"]) * LDU_MM - 8.0) \
                > 1e-9:
            runs.append(cur)
            cur = []
        cur.append(centre)
    if cur:
        runs.append(cur)
    out = []
    for run in runs:
        start = run[0]["z"] * LDU_MM
        span = (run[-1]["z"] - run[0]["z"]) * LDU_MM
        out.append({"start_mm": start, "span_mm": span, "n": len(run),
                    "kinds": [centre_kind(c) for c in run]})
    return out


def _as_int_ldu(value: float, what: str) -> int:
    got = value / LDU_MM
    if abs(got - round(got)) > 1e-6:
        raise ValueError(f"{what}={value} is not on the LDU grid")
    return int(round(got))


def _rotation_to_int(rotation) -> list[list[int]]:
    """Converted-frame rotation into exact integers (must be signed
    permutation, else loud)."""
    ints = [[int(round(float(v))) for v in row] for row in rotation]
    for row, row_i in zip(rotation, ints):
        for got, want in zip(row, row_i):
            if abs(float(got) - want) > 1e-6 or want not in (-1, 0, 1):
                raise ValueError(
                    f"rotation {rotation} is not a signed permutation")
    return ints


def _apply_int(mat, vec):
    return [mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
            mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
            mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2]]


def solve_beam_origin(entry: dict, rotation, points_mm: list[list[float]]
                      ) -> dict:
    """Beam origin (world LDU ints) covering ``points_mm`` with a run.

    ``rotation`` is the beam world rotation (converted frame, must be
    a signed permutation). Every point must coincide EXACTLY with a
    mapped run centre (integer converted-LDU arithmetic throughout);
    the points need only be a collinear subset of one run (integer
    multiples of the 20-LDU step), so a long beam may span a gap
    whose middle holes stay empty. Returns ``{origin_ldu, run_ldu,
    kinds}`` (``run_ldu`` parallels ``points_mm`` order); raises when
    no run covers.
    """
    r = _rotation_to_int(rotation)
    want = [[_as_int_ldu(v, f"point[{i}][{ax}]")
             for ax, v in enumerate(p)]
            for i, p in enumerate(points_mm)]
    if len({tuple(p) for p in want}) != len(want):
        raise ValueError(f"duplicate points in {points_mm}")
    step = _apply_int(r, [0, 20, 0])  # world LDU per run step
    norm2 = sum(s * s for s in step)
    keys = [sum(p[i] * step[i] for i in range(3)) / norm2
            for p in want]
    order = sorted(range(len(want)), key=lambda i: keys[i])
    ordered = [want[i] for i in order]
    # Collinear subset: every hop an integer multiple of one step.
    hops: list[int] = []
    for j in range(1, len(ordered)):
        diff = [ordered[j][i] - ordered[j - 1][i] for i in range(3)]
        mult = None
        for m in (1, -1):
            if diff == [m * s for s in step]:
                mult = m
                break
        if mult is None:
            for m in range(2, 21):
                if diff == [m * s for s in step]:
                    mult = m
                    break
                if diff == [-m * s for s in step]:
                    mult = -m
                    break
        if mult is None:
            raise ValueError(f"points {points_mm} are not collinear "
                             "run steps")
        hops.append(mult)
    if any(h < 0 for h in hops) and any(h > 0 for h in hops):
        raise ValueError(f"points {points_mm} backtrack along the run")
    forward = not hops or hops[0] > 0
    # Row offsets of each point from the first, in 20-LDU steps.
    offsets = [0]
    for h in hops:
        offsets.append(offsets[-1] + abs(h))
    centres = hole_centres(entry)
    if not centres:
        raise ValueError("beam has no hole centres")
    # Maximal consecutive-20-LDU runs over centre z values.
    runs: list[list[float]] = []
    cur = [centres[0]["z"]]
    for centre in centres[1:]:
        if abs(centre["z"] - cur[-1] - 20.0) < 1e-9:
            cur.append(centre["z"])
        else:
            runs.append(cur)
            cur = [centre["z"]]
    runs.append(cur)
    need = offsets[-1] + 1 if offsets else 1
    for run in runs:
        if len(run) < need:
            continue
        for k in range(len(run) - need + 1):
            window = [run[k + o] for o in offsets]
            if not forward:
                window = list(reversed(window))
            origin = [ordered[0][i]
                      - _apply_int(r, [0, window[0], 0])[i]
                      for i in range(3)]
            ok = True
            for point, z in zip(ordered, window):
                mapped = _apply_int(r, [0, z, 0])
                placed = [mapped[i] + origin[i] for i in range(3)]
                if placed != point:
                    ok = False
                    break
            if ok:
                kinds = [centre_kind(
                    next(c for c in centres if c["z"] == z))
                    for z in window]
                # Return run entries in the caller's point order.
                kinds = [kinds[order.index(i)]
                         for i in range(len(points_mm))]
                return {"origin_ldu": origin, "run_ldu": window,
                        "kinds": kinds}
    raise ValueError("no run covers "
                     f"{[[float(v) * LDU_MM for v in p] for p in want]}")


def verify_hole_line(entry: dict, origin_mm, rotation,
                     station_mm, line_dir, tol_mm: float = 0.01) -> None:
    """Require a hole centre within ``tol_mm`` of a line (loud)."""
    rot = [[float(v) for v in row] for row in rotation]
    org = [float(v) for v in origin_mm]
    sta = [float(v) for v in station_mm]
    direction = [float(v) for v in line_dir]
    norm = math.sqrt(sum(v * v for v in direction))
    direction = [v / norm for v in direction]
    best = None
    for centre in hole_centres(entry):
        local = conv_ldu_to_mm([0, 0, centre["z"]])
        world = [org[i] + sum(rot[i][k] * local[k] for k in range(3))
                 for i in range(3)]
        rel = [world[i] - sta[i] for i in range(3)]
        along = sum(rel[i] * direction[i] for i in range(3))
        lateral = math.sqrt(max(
            0.0, sum(v * v for v in rel) - along * along))
        if best is None or lateral < best[0]:
            best = (lateral, world)
    if best is None:
        raise ValueError("part has no hole centres")
    if best[0] > tol_mm:
        raise ValueError(f"no hole centre near {station_mm} along "
                         f"{line_dir} (nearest {best[1]} at "
                         f"{best[0]:.3f} mm)")


def end_margin_mm(part_id: str, entry: dict,
                  root: str | None = None) -> dict:
    """Mesh-measured beam margins past the end hole centres.

    Returns ``{lo_mm, hi_mm}`` along the row axis (converted local
    mm). Raises when the mesh is missing (loud, never guessed).
    """
    from src.gen.meshbox import STUDIO_LDRAW, local_box_mm

    box = local_box_mm(part_id, root=root or STUDIO_LDRAW)
    if box is None:
        raise ValueError(f"{part_id}: mesh missing, cannot measure end "
                         "margins")
    (lo, hi), centres = box, hole_centres(entry)
    if not centres:
        raise ValueError(f"{part_id}: no hole centres")
    ax = row_axis_converted(entry)
    first = conv_ldu_to_mm([0, 0, centres[0]["z"]])[ax]
    last = conv_ldu_to_mm([0, 0, centres[-1]["z"]])[ax]
    return {"lo_mm": round(first - lo[ax], 3),
            "hi_mm": round(hi[ax] - last, 3)}
