"""Stage 5D joint-template generator: base_yaw, pitch, wrist_roll.

Builds parametric joint modules (module.json + module.ldr geometry) for
the ``yaw -> pitch -> pitch -> roll`` arm_4dof configuration (no gripper).
Output passes :mod:`src.gen.modules` validation unchanged. Templates read
part geometry (holes, axes, teeth, axle lengths) from the public catalog
(``src/catalog/parts.yaml``) at generation time and check the BOM against
the owned stock (``config/inventory.local.yaml``) when an inventory is
given; they never reference external robot files or part layouts.

Layouts (module-local millimetres, Z up; canonical axes: yaw/roll +Z,
pitch +Y)::

    base_yaw   H=52  motor-12T -> idler-20T -> out-12T    ratio 1.0
    pitch      H=64  motor-12T -> lay(20T+16T) -> out-16T ratio 0.6
    wrist_roll H=52  motor-16T -> idler-16T -> out-16T    ratio 1.0

All meshes are parallel-axis external spur/double-bevel meshes with
centre distances ``0.5 * (t1 + t2)`` mm on the 8 mm hole grid, so every
gear-train sign is known-positive and every shaft threads round
(free-spinning) beam holes. Output shafts stop short of the output
mount so the mount connector pin never touches shaft material.

Mount conventions (validated, not silently repaired):

* Mount rotations must be identity (axis-aligned chain). Rotated mounts
  raise; relocation is supported through translation only.
* Mount positions must sit on the LDU grid and share X/Y (vertical
  stack); the input/output height must equal the template H.
* Each interface carries 2 mount holes on real mount beams (x=+12,
  beside the gear train): input stubs hang 16/8 below the input
  origin, output stubs rise 0/8 above the output origin. Pins run
  along the mount frame +X (see :mod:`src.gen.mounts`).
* ``base_yaw``/``wrist_roll`` inputs sit at local z=0;
  ``pitch`` inputs sit at local z=-4 (canonical).

Connection rule: a downstream module connects to an upstream module
when the two mount *orientations* match (see :func:`mounts_compatible`).
Positions live in different module frames, so only rotations compare;
the assembler bridges the positional gap with beams.

Known simplifications (documented for the Studio refinement pass):

* Motors are shaft-coupled but have no casing fasteners in the BOM
  (motor bodies carry no catalog holes). The base-yaw medium motor
  rests its shaft face on the top plate; the pitch large motor hangs
  on its axle bridge with a 115 mm tail behind the module.
* Beam/module junctions are idealized as coincident holes (the beam
  hole and the mount hole share one point); a physical build offsets
  the beam one module sideways along the pin axis. Full bracket
  detailing is deferred.
* Module subfiles are rigid (stator and rotor frozen together):
  joint motion is modelled as rigid-subtree rotation for clearance
  checks only. A physical rotor split (turntable/axle engagement)
  is deferred, with the mount ``axle`` hole type reserved for it.
* Gears cantilever past their bearing plates; motor bodies are placed
  by shaft-line constraints only (casings mesh-measured, fasteners
  deferred).
* Shafts overhang 4 mm past each end for retainer space; axle picks
  are the shortest stocked axles covering the margined span.
"""

from __future__ import annotations

import math
from pathlib import Path

from src.gen import ldraw as ldraw_io
from src.gen import mounts as _mounts
from src.gen.inventory import load_catalog_merged
from src.gen.modules import validate_module

LDU_MM = 0.4
GRID_MM = 8.0
CARDINALS = ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
             (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
             (0.0, 0.0, 1.0), (0.0, 0.0, -1.0))
COLOUR_BODY = "7"
COLOUR_MOTOR = "72"


# ------------------------------------------------------------------ math


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b) -> list[float]:
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def _mat_vec(m, v) -> list[float]:
    return [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]]


def _mat_mat(a, b) -> list[list[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mat_det(m) -> float:
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def _is_cardinal(vec) -> tuple[float, float, float] | None:
    norm = math.sqrt(sum(v * v for v in vec))
    if norm <= 0:
        return None
    unit = [v / norm for v in vec]
    for cand in CARDINALS:
        if _dot(unit, cand) >= math.cos(math.radians(1.0)):
            return cand
    return None


def _proper_rot_mapping(local_axes: dict[int, tuple[float, float, float]]
                        ) -> list[list[float]]:
    """Signed-permutation rotation honouring local->world axis pairs.

    ``local_axes`` maps a local basis index (0/1/2) to the world cardinal
    it must image to. Remaining columns complete deterministically to a
    right-handed basis. Raises if the constraints conflict.
    """
    cols: dict[int, list[float]] = {}
    for local, world in local_axes.items():
        cols[local] = [float(v) for v in world]
    used = {tuple(v) for v in cols.values()}
    if len(used) != len(cols):
        raise ValueError(f"conflicting axis mapping {local_axes}")
    free_locals = [i for i in range(3) if i not in cols]
    free_worlds = [list(c) for c in CARDINALS if tuple(c) not in used
                   and tuple(-v for v in c) not in used]
    if free_locals:
        first = free_locals[0]
        others = [i for i in range(3) if i != first]
        if all(i in cols for i in others):
            a = cols[others[0]]
            b = cols[others[1]]
            order = sorted(others)
            c = _cross(a, b)
            if order != others:
                c = [-v for v in c]
            cols[first] = c
            free_locals = free_locals[1:]
        else:
            for local in list(free_locals):
                pick = None
                for cand in free_worlds:
                    if all(abs(_dot(cand, cols[i])) < 0.5
                           for i in cols):
                        pick = cand
                        break
                if pick is None:
                    raise ValueError(
                        f"cannot complete axis mapping {local_axes}")
                cols[local] = pick
                used.add(tuple(pick))
                free_worlds = [c for c in free_worlds
                               if tuple(c) != tuple(pick)
                               and tuple(-v for v in c) != tuple(pick)]
                free_locals = [i for i in free_locals if i != local]
    mat = [[cols[c][r] for c in range(3)] for r in range(3)]
    det = _mat_det(mat)
    if abs(det + 1.0) <= 1e-9:
        free = [i for i in range(3) if i not in local_axes]
        if not free:
            raise ValueError(f"mapping {local_axes} is not a proper "
                             "rotation (det=-1)")
        cols[free[-1]] = [-v for v in cols[free[-1]]]
        mat = [[cols[c][r] for c in range(3)] for r in range(3)]
        det = _mat_det(mat)
    if abs(det - 1.0) > 1e-9:
        raise ValueError(f"mapping {local_axes} is not a proper rotation"
                         f" (det={det})")
    return mat


def _perm_for_axis(src, dst) -> list[list[float]]:
    """Proper rotation taking cardinal ``src`` to cardinal ``dst``."""
    for perm in (((0, 1, 2), 1), ((1, 2, 0), 1), ((2, 0, 1), 1),
                 ((0, 2, 1), -1), ((2, 1, 0), -1), ((1, 0, 2), -1)):
        order, psign = perm
        for signs in ((1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1),
                      (1, 1, -1), (1, -1, 1), (-1, 1, 1), (-1, -1, -1)):
            if psign * signs[0] * signs[1] * signs[2] != 1:
                continue
            cols = [[0.0, 0.0, 0.0] for _ in range(3)]
            for local, world in enumerate(order):
                cols[local][world] = float(signs[local])
            if all(abs(cols[i][j]) <= 1.0 for i in range(3)
                   for j in range(3)):
                image = [sum(cols[k][r] * src[k] for k in range(3))
                         for r in range(3)]
                if all(abs(image[r] - dst[r]) < 1e-9 for r in range(3)):
                    return [[cols[c][r] for c in range(3)]
                            for r in range(3)]
    raise ValueError(f"no proper rotation maps {src} to {dst}")


def _rotation_angle_deg(a, b) -> float:
    """Angle between two rotation matrices in degrees."""
    bt = [[b[j][i] for j in range(3)] for i in range(3)]
    rel = _mat_mat(a, bt)
    trace = rel[0][0] + rel[1][1] + rel[2][2]
    cosang = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cosang))


# ------------------------------------------------------------ catalog


def _conv_dir(local) -> tuple[float, float, float]:
    """Catalog LDraw-frame vector into the converted local frame.

    Mirrors ``src.tools.geometry._conv_dir``: (x, y, z) -> (x, z, -y).
    Beam hole rows run along converted +Y; gear/motor axes convert
    from LDraw +Z to converted +Y.
    """
    x, y, z = (float(v) for v in local)
    return (x, z, -y)


def _holes_mm(entry: dict) -> list[tuple[str, tuple[float, float, float]]]:
    """Catalog holes in converted local mm (mirrors ``hole_world``)."""
    out = []
    for hole in entry.get("holes") or []:
        pos = tuple(round(v * LDU_MM, 6) for v in _conv_dir(hole["pos"]))
        out.append((hole.get("kind", "round"), pos))
    return out


def _as_int_ldu(value: float) -> int:
    q = value / LDU_MM
    if abs(q - round(q)) > 1e-6:
        raise ValueError(f"{value} is not on the LDU grid")
    return int(round(q))


def _motor_part_id(catalog: dict, kind: str) -> str:
    want = kind.strip().lower()
    if want not in ("large", "medium"):
        raise ValueError(f"motor kind must be Large or Medium, got {kind!r}")
    found = [pid for pid, entry in catalog.items()
             if (entry.get("category") == "motor"
                 and str(entry.get("motor") or "").lower() == want)]
    if len(found) != 1:
        raise ValueError(f"expected one {want} motor in catalog, "
                         f"found {found}")
    return found[0]


def _teeth(catalog: dict, part_id: str) -> int:
    teeth = (catalog.get(part_id) or {}).get("teeth")
    if not teeth:
        raise ValueError(f"{part_id} has no teeth in catalog")
    return int(teeth)


def _gear_axis_ok(catalog: dict, part_id: str) -> None:
    axis = (catalog.get(part_id) or {}).get("axis_local") or [0, 0, 1]
    if [round(v) for v in axis] != [0, 0, 1]:
        raise ValueError(f"{part_id} gear axis is not local +Z: {axis}")


def _axle_half_span_mm(entry: dict) -> float | None:
    studs = entry.get("axle_length")
    if studs is None:
        return None
    return float(studs) * GRID_MM / 2.0 + 3.0


def _stock_of(inventory: dict | None, part_id: str) -> int | None:
    if inventory is None:
        return None
    row = (inventory.get("parts") or {}).get(part_id) or {}
    return int(row.get("total", 0))


def _pick_axle(catalog: dict, inventory: dict | None,
               min_half_span: float) -> str:
    cands = []
    for pid, entry in catalog.items():
        half = _axle_half_span_mm(entry)
        if half is None or half + 1e-9 < min_half_span:
            continue
        stock = _stock_of(inventory, pid)
        if stock is not None and stock <= 0:
            continue
        cands.append((half, -(stock or 0), pid))
    if not cands:
        raise ValueError(f"no axle covers half-span {min_half_span} mm"
                         + (" with stock" if inventory else ""))
    cands.sort()
    return cands[0][2]


def _pick_pin(catalog: dict, inventory: dict | None) -> str:
    cands = []
    for pid, entry in catalog.items():
        if entry.get("category") != "pin" or not entry.get("friction"):
            continue
        stock = _stock_of(inventory, pid)
        if stock is not None and stock <= 0:
            continue
        cands.append((-(stock or 0), pid))
    if not cands:
        raise ValueError("no friction pin available"
                         + (" with stock" if inventory else ""))
    cands.sort()
    return cands[0][1]


def _part_axis_local(catalog: dict, part_id: str) -> tuple[float, ...]:
    """Part axis in the converted local frame (mirrors ``axis_world``)."""
    entry = catalog.get(part_id) or {}
    axis = entry.get("axis_local")
    if axis is None:
        if entry.get("category") in ("motor", "gear"):
            axis = (0.0, 0.0, 1.0)
        else:
            raise ValueError(f"{part_id} has no axis_local in catalog")
    snapped = _is_cardinal(_conv_dir(axis))
    if snapped is None:
        raise ValueError(f"{part_id} axis {axis} is not cardinal")
    return snapped


def _rot_for_axis(catalog: dict, part_id: str,
                  world_dir) -> list[list[float]]:
    local = _part_axis_local(catalog, part_id)
    idx = max(range(3), key=lambda i: abs(local[i]))
    return _proper_rot_mapping({idx: (float(world_dir[0]),
                                      float(world_dir[1]),
                                      float(world_dir[2]))})


# --------------------------------------------------------------- frame


def _check_mount(name: str, mount: dict, height_from: dict | None,
                 height: float) -> list[float]:
    try:
        pos = [float(v) for v in mount["position_mm"]]
        rot = [[float(v) for v in row] for row in mount["rotation"]]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{name} mount is malformed: {exc}") from exc
    ident = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    if _rotation_angle_deg(rot, ident) > 1e-6:
        raise ValueError(f"{name} mount must be axis-aligned (identity "
                         "rotation); rotated mounts are not supported")
    for value in pos:
        _as_int_ldu(value)
    if height_from is not None:
        base = [float(v) for v in height_from["position_mm"]]
        if abs(pos[0] - base[0]) > 1e-6 or abs(pos[1] - base[1]) > 1e-6:
            raise ValueError(f"{name} mount must share X/Y with the input "
                             "mount (vertical stack)")
        if abs((pos[2] - base[2]) - height) > 1e-6:
            raise ValueError(f"{name} mount height must be {height} mm "
                             "above the input mount")
    return pos


def _check_ratio(template: str, ratio: float, product: float) -> None:
    err = abs(float(ratio) - product)
    if err / max(product, 1e-12) > 0.01 or err > 0.02:
        raise ValueError(f"{template} supports ratio {product:.6f}, "
                         f"got {ratio}")


def _check_axis(template: str, axis) -> tuple[float, float, float]:
    snapped = _is_cardinal(axis)
    if snapped is None:
        raise ValueError(f"{template} axis must be a cardinal direction, "
                         f"got {axis}")
    return snapped


def mounts_compatible(out_mount: dict, in_mount: dict,
                      tol_deg: float = 1.0) -> tuple[bool, str]:
    """Whether an output frame accepts a downstream input frame.

    Only mount *orientations* compare: the two frames live in different
    module-local frames, so positions are meaningless here. The assembler
    bridges the positional gap with beams. Returns ``(ok, reason)``.
    """
    try:
        rout = [[float(v) for v in row] for row in out_mount["rotation"]]
        rin = [[float(v) for v in row] for row in in_mount["rotation"]]
    except (KeyError, TypeError, ValueError) as exc:
        return False, f"malformed mount: {exc}"
    angle = _rotation_angle_deg(rout, rin)
    if angle <= tol_deg:
        return True, f"orientations match ({angle:.3f} deg)"
    return False, (f"orientation mismatch ({angle:.3f} deg "
                   f"> {tol_deg} deg)")


# ------------------------------------------------------------- placing


class _Placer:
    def __init__(self) -> None:
        self.parts: list[dict] = []
        self.count = 0

    def add(self, part_id: str, pos, rot, colour: str = COLOUR_BODY
            ) -> dict:
        self.count += 1
        part = {"uid": f"t{self.count:06d}", "part_id": part_id,
                "position_mm": [float(v) for v in pos],
                "rotation": [[float(v) for v in row] for row in rot],
                "colour": colour}
        self.parts.append(part)
        return part


def _verify_stations(template: str, part_id: str, entry: dict,
                     origin, rot, stations: list[tuple[float, ...]],
                     shaft_dir=(0.0, 0.0, 1.0),
                     need_round: bool = True) -> None:
    """Verify shaft lines through ``stations`` thread holes in the part.

    Uses lateral distance from the shaft line (mirroring the geometry
    ``_line_hits`` rule), not point distance: end-hole side entries sit
    off the shaft point but on its line.
    """
    world = []
    for kind, local in _holes_mm(entry):
        point = [origin[i] + (rot[i][0] * local[0] + rot[i][1] * local[1]
                              + rot[i][2] * local[2]) for i in range(3)]
        world.append((kind, point))
    missing = []
    for station in stations:
        def lateral(item) -> float:
            rel = [item[1][i] - station[i] for i in range(3)]
            along = _dot(rel, shaft_dir)
            return math.sqrt(max(0.0, _dot(rel, rel) - along * along))
        best = min(world, key=lateral)
        kind, point = best
        if lateral(best) > 2.0:
            missing.append(f"{station} (nearest hole {point})")
        elif need_round and kind != "round":
            missing.append(f"{station} (hole kind {kind}, need round)")
    if missing:
        raise ValueError(f"{template}: {part_id} at {origin} misses "
                         f"stations: {'; '.join(missing)}")


def _beam_offset_covering(holes: list[tuple[str, tuple[float, ...]]],
                          stations: list[float]) -> float:
    """Offset along the beam row covering 1-D ``stations`` (mm).

    Converted beam holes run along local Y (index 1).
    """
    coords = sorted({round(p[1], 6) for _, p in holes})
    targets = sorted(stations)
    for hole in coords:
        offset = targets[0] - hole
        if abs(offset / GRID_MM - round(offset / GRID_MM)) > 1e-6:
            continue
        if all(any(abs(h + offset - t) <= 2.0 for h in coords)
               for t in targets):
            return round(offset, 6)
    raise ValueError(f"no beam offset covers {stations} from {coords}")


def _mesh_ok(template: str, catalog: dict, gear_a: dict, gear_b: dict
             ) -> None:
    ta = _teeth(catalog, gear_a["part_id"])
    tb = _teeth(catalog, gear_b["part_id"])
    expect = 0.5 * (ta + tb)
    got = math.dist(gear_a["position_mm"], gear_b["position_mm"])
    if abs(got - expect) > 0.8:
        raise ValueError(f"{template}: {gear_a['part_id']} x "
                         f"{gear_b['part_id']} mesh at {got:.3f} mm, "
                         f"expected {expect:.3f} mm")


def _no_false_mesh(template: str, parts: list[dict], catalog: dict,
                   intended: set[tuple[str, str]]) -> None:
    """Fail loudly if any unintended gear pair meshes.

    The mesh detector works on 3D centre distances, so gears in nearby
    planes can false-positive; a spurious edge would corrupt the
    inferred ratio path. Intended pairs are ``(uid, uid)`` both ways.
    """
    from src.tools.geometry import check_gear_mesh

    gears = [p for p in parts
             if (catalog.get(p["part_id"]) or {}).get("teeth")]
    for i in range(len(gears)):
        for j in range(i + 1, len(gears)):
            pair = (gears[i]["uid"], gears[j]["uid"])
            if pair in intended or pair[::-1] in intended:
                continue
            res = check_gear_mesh(pair[0], pair[1], parts, catalog)
            if res.get("meshes") and res.get("kind") != "compound":
                raise ValueError(
                    f"{template}: spurious mesh {gears[i]['part_id']} x "
                    f"{gears[j]['part_id']} ({res.get('kind')})")


# ------------------------------------------------------------ builders

_IDENT = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _beam_rot(row_dir) -> list[list[float]]:
    """Beam rotation running the hole row (local +Y) along ``row_dir``."""
    return _proper_rot_mapping({1: (float(row_dir[0]), float(row_dir[1]),
                                   float(row_dir[2]))})


_BEAM_ROW_X = _beam_rot((1.0, 0.0, 0.0))
_BEAM_ROW_Z = _beam_rot((0.0, 0.0, 1.0))


def _mount_rot() -> list[list[float]]:
    """Mount-beam rotation: row along +Y, hole tunnels along +X."""
    return _proper_rot_mapping({1: (0.0, 1.0, 0.0),
                                2: (1.0, 0.0, 0.0)})


_MOUNT_ROT = _mount_rot()

MOUNT_X_MM = 12.0  # mount beams ride beside the gear train, never over it
MOUNT_IN_DZ = (-16.0, -8.0)  # input stub offsets from the input origin
MOUNT_OUT_DZ = (0.0, 8.0)  # output stub offsets from the output origin
MOUNT_THIN_X_MM = 8.5  # mount beams must be thin across the chain


def _pick_mount_centre(catalog: dict, part_id: str) -> tuple[float, str]:
    """Mount hole centre (converted mm) of a mount beam part.

    Prefers round centres on the 8 mm grid (nearest the part origin);
    axle centres only when no round grid centre exists. Raises
    (loud) when the part cannot serve as a mount beam.
    """
    entry = catalog.get(part_id) or {}
    if _mounts.row_axis_converted(entry) != 1:
        raise ValueError(f"{part_id}: mount beams need a Y row, got "
                         f"axis {_mounts.row_axis_converted(entry)}")
    if _mounts.hole_dir_converted(entry) != (0, 0, 1):
        raise ValueError(f"{part_id}: mount holes must pierce local Z")
    box = None
    try:
        from src.gen.meshbox import local_box_mm

        box = local_box_mm(part_id)
    except Exception:
        box = None
    if box is not None:
        thin = box[1][0] - box[0][0]
        if thin > MOUNT_THIN_X_MM:
            raise ValueError(f"{part_id}: mount beam {thin:.1f} mm "
                             "across the chain (wide beams collide)")
    cands = []
    for centre in _mounts.hole_centres(entry):
        mm = centre["z"] * LDU_MM
        if abs(mm / GRID_MM - round(mm / GRID_MM)) > 1e-9:
            continue
        kind = _mounts.centre_kind(centre)
        cands.append((0 if kind == "round" else 1, abs(mm), mm, kind))
    if not cands:
        raise ValueError(f"{part_id}: no 8 mm-grid hole centre")
    cands.sort()
    return cands[0][2], cands[0][3]


def _add_mounts(placer: _Placer, template: str, catalog: dict,
                mount_beam: str, in_pos, out_pos) -> list[dict]:
    """Bolt 4 mount beams on; return mount-hole rows (module mm).

    Two holes per interface at x=+12: input stubs hang below the
    input origin, output stubs rise from the output origin. Each
    hole row is ``{pos_mm, kind}`` with ``kind`` in {round, axle}.
    """
    centre_mm, kind = _pick_mount_centre(catalog, mount_beam)
    entry = catalog[mount_beam]
    holes = []
    for base, offsets in ((in_pos, MOUNT_IN_DZ), (out_pos, MOUNT_OUT_DZ)):
        for dz in offsets:
            station = [base[0] + MOUNT_X_MM, base[1], base[2] + dz]
            origin = [station[0] - _MOUNT_ROT[0][1] * centre_mm,
                      station[1] - _MOUNT_ROT[1][1] * centre_mm,
                      station[2] - _MOUNT_ROT[2][1] * centre_mm]
            _mounts.verify_hole_line(entry, origin, _MOUNT_ROT,
                                     station, (1.0, 0.0, 0.0))
            placer.add(mount_beam, origin, _MOUNT_ROT)
            holes.append({"pos_mm": station, "kind": kind})
    return holes


def _mount_block(mount: dict, holes: list[dict], perm, shift) -> dict:
    rows = []
    for hole in holes:
        moved = [a + b for a, b in zip(_mat_vec(perm, hole["pos_mm"]),
                                       shift)]
        ldu = [v / LDU_MM for v in moved]
        ints = [int(round(v)) for v in ldu]
        if any(abs(a - b) > 1e-6 for a, b in zip(ldu, ints)):
            raise ValueError(f"mount hole {moved} is off the LDU grid")
        rows.append({"pos_ldu": ints,
                     "type": "axle" if hole["kind"] == "axle" else "pin"})
    rows.sort(key=lambda r: (r["pos_ldu"][2], r["pos_ldu"][0]))
    rot = [[float(v) for v in row] for row in mount["rotation"]]
    pos = [float(v) for v in mount["position_mm"]]
    return {"frame": [[rot[i][0], rot[i][1], rot[i][2], pos[i]]
                      for i in range(3)],
            "holes": rows}


def _finish(template: str, placer: _Placer, joint_point, joint_axis,
            motor_kind: str, ratio: float, range_deg: list[float],
            input_mount: dict, output_mount: dict,
            in_pos, mount_holes: list[dict] | None = None,
            perm=None, shift=None) -> tuple[dict, list[dict]]:
    parts = placer.parts
    xs = [p["position_mm"][0] for p in parts]
    ys = [p["position_mm"][1] for p in parts]
    zs = [p["position_mm"][2] for p in parts]
    lo = [min(xs) - 20.0, min(ys) - 20.0, min(zs) - 20.0]
    hi = [max(xs) + 20.0, max(ys) + 20.0, max(zs) + 20.0]
    for label, mount in (("input", input_mount), ("output", output_mount)):
        pos = mount["position_mm"]
        if any(p < a - 8.0 or p > b + 8.0
               for p, a, b in zip(pos, lo, hi)):
            raise ValueError(f"{template}: {label} mount {pos} is "
                             "detached from the generated frame")
    bom: dict[str, int] = {}
    for part in parts:
        bom[part["part_id"]] = bom.get(part["part_id"], 0) + 1
    module = {
        "name": template,
        "template": template,
        "frame": "module local, Z up, mm",
        "joint": {"axis": [float(v) for v in joint_axis],
                  "point_mm": [float(v) for v in joint_point],
                  "range_deg": [float(range_deg[0]), float(range_deg[1])],
                  "joint_deg_per_motor_deg": float(ratio)},
        "motor": {"kind": motor_kind.strip().lower()},
        "input_mount": input_mount,
        "output_mount": output_mount,
        "size_mm": [round(hi[i] - lo[i], 3) for i in range(3)],
        "bom": bom,
        "geometry": "module.ldr",
    }
    if mount_holes is not None:
        if perm is None or shift is None:
            raise ValueError("mount holes need the frame transform")
        module["mount_in"] = _mount_block(
            input_mount, mount_holes[:2], perm, shift)
        module["mount_out"] = _mount_block(
            output_mount, mount_holes[2:], perm, shift)
    return module, parts


def _apply_frame(parts: list[dict], joint_point, perm,
                 shift) -> tuple[list[dict], list[float]]:
    for part in parts:
        part["position_mm"] = [a + b for a, b in
                       zip(_mat_vec(perm, part["position_mm"]), shift)]
        part["rotation"] = _mat_mat(perm, part["rotation"])
    return parts, [a + b for a, b in zip(_mat_vec(perm, joint_point),
                                        shift)]


def base_yaw(axis, input_mount: dict, output_mount: dict,
             ratio: float, motor_kind: str,
             catalog: dict | None = None,
             inventory: dict | None = None,
             mount_beam: str | None = None,
             ) -> tuple[dict, list[dict]]:
    """Yaw joint module: vertical output shaft, H=52, ratio 1.0.

    Single-stage idler train (12T -> 20T -> 12T); the idler offsets
    the motor from the mount centre for connector clearance.
    ``mount_beam`` bolts the 4 mount beams on (None skips mounts).
    """
    template = "base_yaw"
    catalog = catalog if catalog is not None else load_catalog_merged()
    in_pos = _check_mount("input", input_mount, None, 0.0)
    _check_mount("output", output_mount, input_mount, 52.0)
    want = _check_axis(template, axis)
    motor_part = _motor_part_id(catalog, motor_kind)
    pinion, idler, out = "32270.dat", "32269.dat", "32270.dat"
    for pid in (pinion, idler, out):
        _gear_axis_ok(catalog, pid)
    product = (_teeth(catalog, pinion) / _teeth(catalog, idler)
               * _teeth(catalog, idler) / _teeth(catalog, out))
    _check_ratio(template, ratio, product)
    plates = "40490.dat"
    plate_entry = catalog[plates]
    plate_holes = _holes_mm(plate_entry)
    ox = _beam_offset_covering(plate_holes, [-32.0, -16.0, 0.0])
    # Half-spans include a 4 mm retainer overhang past each end.
    axle_m = _pick_axle(catalog, inventory, 10.0)
    axle_i = _pick_axle(catalog, inventory, 16.0)
    axle_o = _pick_axle(catalog, inventory, 16.0)
    gear_rot = _rot_for_axis(catalog, pinion, (0.0, 0.0, 1.0))
    motor_rot = _rot_for_axis(catalog, motor_part, (0.0, 0.0, 1.0))
    placer = _Placer()
    for z in (4.0, 28.0):
        origin = [ox, 0.0, z]
        _verify_stations(template, plates, plate_entry, origin,
                         _BEAM_ROW_X,
                         [(-32.0, 0.0, z), (-16.0, 0.0, z), (0.0, 0.0, z)])
        placer.add(plates, origin, _BEAM_ROW_X)
    placer.add(motor_part, [-32.0, 0.0, 32.0], motor_rot, COLOUR_MOTOR)
    gear_p = placer.add(pinion, [-32.0, 0.0, 16.0], gear_rot)
    gear_i = placer.add(idler, [-16.0, 0.0, 16.0],
                        _rot_for_axis(catalog, idler, (0.0, 0.0, 1.0)))
    gear_o = placer.add(out, [0.0, 0.0, 16.0], gear_rot)
    _mesh_ok(template, catalog, gear_p, gear_i)
    _mesh_ok(template, catalog, gear_i, gear_o)
    _no_false_mesh(template, placer.parts, catalog,
                   {(gear_p["uid"], gear_i["uid"]),
                    (gear_i["uid"], gear_o["uid"])})
    placer.add(axle_m, [-32.0, 0.0, 22.0],
               _rot_for_axis(catalog, axle_m, (0.0, 0.0, 1.0)))
    placer.add(axle_i, [-16.0, 0.0, 16.0],
               _rot_for_axis(catalog, axle_i, (0.0, 0.0, 1.0)))
    placer.add(axle_o, [0.0, 0.0, 16.0],
               _rot_for_axis(catalog, axle_o, (0.0, 0.0, 1.0)))
    mount_holes = None
    if mount_beam is not None:
        mount_holes = _add_mounts(placer, template, catalog, mount_beam,
                                  [0.0, 0.0, 0.0], [0.0, 0.0, 52.0])
    perm = _perm_for_axis((0.0, 0.0, 1.0), want)
    joint_axis = _mat_vec(perm, [0.0, 0.0, 1.0])
    parts, joint_point = _apply_frame(placer.parts, [0.0, 0.0, 16.0],
                                      perm, in_pos)
    return _finish(template, placer, joint_point, joint_axis, motor_kind,
                   ratio, [-150.0, 150.0], input_mount, output_mount,
                   in_pos, mount_holes, perm, in_pos)


def pitch(axis, input_mount: dict, output_mount: dict,
          ratio: float, motor_kind: str,
          catalog: dict | None = None,
          inventory: dict | None = None,
          mount_beam: str | None = None) -> tuple[dict, list[dict]]:
    """Pitch joint module: transverse output shaft, H=64, ratio 0.6.

    Two-stage compound train (12T -> 20T, then 16T -> 16T); 0.6 is
    the deepest reduction available from rigid-bore gears on the
    8 mm hole grid within stock.
    ``mount_beam`` bolts the 4 mount beams on (None skips mounts).
    """
    template = "pitch"
    catalog = catalog if catalog is not None else load_catalog_merged()
    in_pos = _check_mount("input", input_mount, None, 0.0)
    _check_mount("output", output_mount, input_mount, 64.0)
    want = _check_axis(template, axis)
    motor_part = _motor_part_id(catalog, motor_kind)
    pinion, gear_b, gear_c, out = ("32270.dat", "32269.dat",
                                   "94925.dat", "94925.dat")
    for pid in (pinion, gear_b, gear_c, out):
        _gear_axis_ok(catalog, pid)
    product = (_teeth(catalog, pinion) / _teeth(catalog, gear_b)
               * _teeth(catalog, gear_c) / _teeth(catalog, out))
    _check_ratio(template, ratio, product)
    plates = "40490.dat"
    plate_entry = catalog[plates]
    oz = _beam_offset_covering(_holes_mm(plate_entry),
                               [0.0, 16.0, 32.0, 48.0])
    axle_m = _pick_axle(catalog, inventory, 25.0)
    axle_l = _pick_axle(catalog, inventory, 28.0)
    axle_o = _pick_axle(catalog, inventory, 28.0)
    placer = _Placer()
    for y in (-12.0, 12.0):
        origin = [0.0, y, oz]
        _verify_stations(template, plates, plate_entry, origin,
                         _BEAM_ROW_Z,
                         [(0.0, y, z) for z in (0.0, 16.0, 32.0, 48.0)],
                         shaft_dir=(0.0, 1.0, 0.0))
        placer.add(plates, origin, _BEAM_ROW_Z)
    motor_rot = _rot_for_axis(catalog, motor_part, (0.0, 1.0, 0.0))
    # The large motor rides 32 back from the pinion axis (axle
    # bridge): its fat body then clears the downstream mount sweep
    # in Y (mesh-verified 0.8 mm; tails reach y=-128 module-local).
    placer.add(motor_part, [0.0, -32.0, 16.0], motor_rot, COLOUR_MOTOR)
    gear_rot = _rot_for_axis(catalog, pinion, (0.0, 1.0, 0.0))
    gear_p = placer.add(pinion, [0.0, 24.0, 16.0], gear_rot)
    gear_i = placer.add(gear_b, [0.0, 24.0, 32.0],
                        _rot_for_axis(catalog, gear_b, (0.0, 1.0, 0.0)))
    gear_j = placer.add(gear_c, [0.0, 36.0, 32.0],
                        _rot_for_axis(catalog, gear_c, (0.0, 1.0, 0.0)))
    gear_o = placer.add(out, [0.0, 36.0, 48.0],
                        _rot_for_axis(catalog, out, (0.0, 1.0, 0.0)))
    _mesh_ok(template, catalog, gear_p, gear_i)
    _mesh_ok(template, catalog, gear_j, gear_o)
    _no_false_mesh(template, placer.parts, catalog,
                   {(gear_p["uid"], gear_i["uid"]),
                    (gear_j["uid"], gear_o["uid"])})
    placer.add(axle_m, [0.0, 4.0, 16.0],
               _rot_for_axis(catalog, axle_m, (0.0, 1.0, 0.0)))
    placer.add(axle_l, [0.0, 12.0, 32.0],
               _rot_for_axis(catalog, axle_l, (0.0, 1.0, 0.0)))
    placer.add(axle_o, [0.0, 12.0, 48.0],
               _rot_for_axis(catalog, axle_o, (0.0, 1.0, 0.0)))
    mount_holes = None
    if mount_beam is not None:
        mount_holes = _add_mounts(placer, template, catalog, mount_beam,
                                  [0.0, 0.0, -4.0], [0.0, 0.0, 60.0])
    perm = _perm_for_axis((0.0, 1.0, 0.0), want)
    joint_axis = _mat_vec(perm, [0.0, 1.0, 0.0])
    shift = [in_pos[0], in_pos[1], in_pos[2] + 4.0]
    parts, joint_point = _apply_frame(placer.parts, [0.0, 0.0, 48.0],
                                      perm, shift)
    return _finish(template, placer, joint_point, joint_axis, motor_kind,
                   ratio, [-120.0, 120.0], input_mount, output_mount,
                   in_pos, mount_holes, perm, shift)


def wrist_roll(axis, input_mount: dict, output_mount: dict,
               ratio: float, motor_kind: str,
               catalog: dict | None = None,
               inventory: dict | None = None,
               mount_beam: str | None = None) -> tuple[dict, list[dict]]:
    """Wrist-roll module: coaxial output shaft, H=52, ratio 1.0.

    ``mount_beam`` bolts the 4 mount beams on (None skips mounts).
    """
    template = "wrist_roll"
    catalog = catalog if catalog is not None else load_catalog_merged()
    in_pos = _check_mount("input", input_mount, None, 0.0)
    _check_mount("output", output_mount, input_mount, 52.0)
    want = _check_axis(template, axis)
    motor_part = _motor_part_id(catalog, motor_kind)
    gear = "94925.dat"
    _gear_axis_ok(catalog, gear)
    product = 1.0
    _check_ratio(template, ratio, product)
    plates = "32524.dat"
    plate_entry = catalog[plates]
    ox = _beam_offset_covering(_holes_mm(plate_entry), [0.0, 16.0, 32.0])
    # Half-spans include a 4 mm retainer overhang past each end.
    axle_m = _pick_axle(catalog, inventory, 10.0)
    axle_i = _pick_axle(catalog, inventory, 16.0)
    axle_o = _pick_axle(catalog, inventory, 16.0)
    gear_rot = _rot_for_axis(catalog, gear, (0.0, 0.0, 1.0))
    motor_rot = _rot_for_axis(catalog, motor_part, (0.0, 0.0, 1.0))
    placer = _Placer()
    for z in (4.0, 28.0):
        origin = [ox, 0.0, z]
        _verify_stations(template, plates, plate_entry, origin,
                         _BEAM_ROW_X,
                         [(x, 0.0, z) for x in (0.0, 16.0, 32.0)])
        placer.add(plates, origin, _BEAM_ROW_X)
    placer.add(motor_part, [32.0, 0.0, 32.0], motor_rot, COLOUR_MOTOR)
    gear_p = placer.add(gear, [32.0, 0.0, 16.0], gear_rot)
    gear_i = placer.add(gear, [16.0, 0.0, 16.0], gear_rot)
    gear_o = placer.add(gear, [0.0, 0.0, 16.0], gear_rot)
    _mesh_ok(template, catalog, gear_p, gear_i)
    _mesh_ok(template, catalog, gear_i, gear_o)
    _no_false_mesh(template, placer.parts, catalog,
                   {(gear_p["uid"], gear_i["uid"]),
                    (gear_i["uid"], gear_o["uid"])})
    placer.add(axle_m, [32.0, 0.0, 22.0],
               _rot_for_axis(catalog, axle_m, (0.0, 0.0, 1.0)))
    placer.add(axle_i, [16.0, 0.0, 16.0],
               _rot_for_axis(catalog, axle_i, (0.0, 0.0, 1.0)))
    placer.add(axle_o, [0.0, 0.0, 16.0],
               _rot_for_axis(catalog, axle_o, (0.0, 0.0, 1.0)))
    mount_holes = None
    if mount_beam is not None:
        mount_holes = _add_mounts(placer, template, catalog, mount_beam,
                                  [0.0, 0.0, 0.0], [0.0, 0.0, 52.0])
    perm = _perm_for_axis((0.0, 0.0, 1.0), want)
    joint_axis = _mat_vec(perm, [0.0, 0.0, 1.0])
    parts, joint_point = _apply_frame(placer.parts, [0.0, 0.0, 16.0],
                                      perm, in_pos)
    return _finish(template, placer, joint_point, joint_axis, motor_kind,
                   ratio, [-180.0, 180.0], input_mount, output_mount,
                   in_pos, mount_holes, perm, in_pos)


TEMPLATES = {"base_yaw": base_yaw, "pitch": pitch,
             "wrist_roll": wrist_roll}


# ---------------------------------------------------------------- IO


def write_module_files(modules_root: str | Path, name: str, module: dict,
                       parts: list[dict], catalog: dict | None = None,
                       inventory: dict | None = None,
                       inventory_path: str | Path | None = None) -> Path:
    """Write module.json + module.ldr and validate the result.

    Loads the real inventory from ``inventory_path`` (default
    ``config/inventory.local.yaml``) unless ``inventory`` is given;
    a missing file skips the stock check. Raises on validation errors.
    """
    catalog = catalog if catalog is not None else load_catalog_merged()
    if inventory is None and inventory_path is None:
        default = Path("config/inventory.local.yaml")
        if default.exists():
            inventory_path = default
    if inventory is None and inventory_path is not None:
        import yaml

        with open(inventory_path, encoding="utf-8") as fh:
            inventory = yaml.safe_load(fh) or {}
    out_dir = Path(modules_root) / name
    out_dir.mkdir(parents=True, exist_ok=True)
    import json

    with open(out_dir / "module.json", "w", encoding="utf-8") as fh:
        json.dump(module, fh, indent=2, sort_keys=True)
        fh.write("\n")
    ldraw_io.write_robot_ldraw(parts, out_dir / "module.ldr")
    errors = validate_module(module, out_dir, catalog, inventory)
    if errors:
        raise ValueError(f"{name}: " + "; ".join(errors))
    return out_dir


